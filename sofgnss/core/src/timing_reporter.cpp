#include "sofgnss/timing_reporter.h"

#include "sofgnss/pvdb.h"

namespace sofgnss {

TimingReporter::TimingReporter(uint32_t divider, bool dither, Emit emit, void* ctx)
    : divider_(divider ? divider : 1), dither_(dither), emit_(emit), ctx_(ctx) {}

void TimingReporter::on_pps(uint32_t tick, uint32_t rx_idle_ms) {
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_pps(s, sizeof s, ticks_.update(tick), rx_idle_ms);
  pps_++;
  emit_(s, n, ctx_);
}

void TimingReporter::on_sof(uint32_t tick, uint64_t frame) {
  uint64_t t = ticks_.update(tick);
  if (!schedule().is_target(frame)) return;
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_sof(s, sizeof s, t, frame);
  sof_++;
  emit_(s, n, ctx_);
}

void TimingReporter::on_cal(uint32_t tick, const char* mode, bool pps_synced, uint32_t n,
                            uint32_t chip_rate, uint32_t length) {
  char s[pvdb::kMaxSentence];
  size_t len =
      pvdb::format_cal(s, sizeof s, ticks_.update(tick), mode, pps_synced, n, chip_rate, length);
  emit_(s, len, ctx_);
}

}  // namespace sofgnss
