#include <string>
#include <vector>

#include "check.h"
#include "sofgps/pvdb.h"
#include "sofgps/timing_reporter.h"

using namespace sofgps;

namespace {

struct Out {
  std::vector<std::string> sentences;
};

void emit(const char* s, size_t n, void* ctx) {
  static_cast<Out*>(ctx)->sentences.emplace_back(s, n);
}

}  // namespace

TEST(reporter_pps_sentence_unwrapped) {
  Out out;
  TimingReporter r(100, false, emit, &out);
  r.on_pps(0xFFFFFFF0, 0);
  r.on_pps(0x00000010, 3);  // wrapped: 0x20 ticks later
  CHECK_EQ(out.sentences.size(), 2u);
  CHECK(pvdb::verify(out.sentences[0].data(), out.sentences[0].size()));
  CHECK(out.sentences[0].rfind("$PVDB,SOFPPS,PPS,", 0) == 0);
  unsigned long long a = std::stoull(out.sentences[0].substr(17));
  unsigned long long b = std::stoull(out.sentences[1].substr(17));
  CHECK_EQ(b - a, 0x20u);
  CHECK_EQ(a & 0xFFFFFFFF, 0xFFFFFFF0u);
  CHECK(out.sentences[1].find(",3*") != std::string::npos);  // rx_idle_ms field
  CHECK_EQ(r.pps_count(), 2u);
}

TEST(reporter_sof_applies_divider) {
  Out out;
  TimingReporter r(10, false, emit, &out);
  for (uint64_t f = 95; f < 125; ++f) r.on_sof(static_cast<uint32_t>(f * 80000), f);
  CHECK_EQ(out.sentences.size(), 3u);  // 100, 110, 120
  CHECK(out.sentences[0].rfind("$PVDB,SOFPPS,SOF,", 0) == 0);
  CHECK(out.sentences[0].find(",100*") != std::string::npos);
  CHECK(out.sentences[2].find(",120*") != std::string::npos);
  CHECK_EQ(r.sof_count(), 3u);

  r.set_divider(0);  // clamps to 1
  CHECK_EQ(r.divider(), 1u);
  r.on_sof(1, 125);
  CHECK_EQ(out.sentences.size(), 4u);
}

TEST(reporter_dither_filter_one_per_window) {
  Out out;
  TimingReporter r(100, true, emit, &out);
  for (uint64_t f = 0; f < 1000; ++f) r.on_sof(static_cast<uint32_t>(f * 80000), f);
  CHECK_EQ(out.sentences.size(), 10u);
  SofSchedule sched{100, true};
  bool any_nonzero_offset = false;
  for (const std::string& s : out.sentences) {
    unsigned long long frame = std::stoull(s.substr(s.rfind(',') + 1));
    CHECK(sched.is_target(frame));
    if (frame % 100 != 0) any_nonzero_offset = true;
  }
  CHECK(any_nonzero_offset);
}

TEST(reporter_pps_and_sof_share_one_clock) {
  Out out;
  TimingReporter r(1, false, emit, &out);
  r.on_sof(1000, 5);
  r.on_pps(900, 0);  // captured before the SOF, delivered after
  unsigned long long sof = std::stoull(out.sentences[0].substr(17));
  unsigned long long pps = std::stoull(out.sentences[1].substr(17));
  CHECK_EQ(sof - pps, 100u);
}
