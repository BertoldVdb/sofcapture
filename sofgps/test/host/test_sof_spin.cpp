#include <cstdint>
#include <utility>
#include <vector>

#include "check.h"
#include "hal_sim.h"
#include "sofgps/sof_spin_source.h"

using namespace sofgps;

namespace {

struct Event {
  uint32_t tick;
  uint64_t frame;
  uint64_t true_index;  // sim SOF index at detection
};

struct Capture {
  std::vector<Event> events;
};

void sink(uint32_t tick, uint64_t frame, void* ctx) {
  static_cast<Capture*>(ctx)->events.push_back({tick, frame, sim::state().last_detected_index});
}

void run_until(SofSpinSource& src, Capture& c, size_t n, int max_steps = 200000) {
  for (int i = 0; i < max_steps && c.events.size() < n; ++i) src.step();
}

// Ticks must match the true SOF times, frames the true frame numbers.
void check_events(const Capture& c, uint32_t divider, size_t from = 0, size_t to = SIZE_MAX,
                  bool dither = false) {
  const sim::State& s = sim::state();
  SofSchedule sched{divider, dither};
  if (to > c.events.size()) to = c.events.size();
  for (size_t i = from; i < to; ++i) {
    const Event& e = c.events[i];
    CHECK(sched.is_target(e.frame));
    CHECK_EQ(e.frame & 0x7FF, (s.frame_base + e.true_index) & 0x7FF);
    CHECK_EQ(e.tick, static_cast<uint32_t>(sim::sof_time(e.true_index) + s.detect_latency));
    // one tag per window, no gaps
    if (i > from) CHECK_EQ(sched.window_of(e.frame), sched.window_of(c.events[i - 1].frame) + 1);
  }
}

uint64_t us_ticks(uint64_t us) { return us * sim::state().tick_hz / 1000000; }

}  // namespace

TEST(spin_tags_every_nth_frame_exactly) {
  sim::reset();
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  run_until(src, c, 50);
  CHECK_EQ(c.events.size(), 50u);
  check_events(c, 100);
  // the 11-bit frame number wrapped
  CHECK(c.events.back().frame > 2048);
  CHECK(sim::state().spin_ticks / c.events.size() < us_ticks(400));
  CHECK_EQ(src.stats().late, 0u);
}

TEST(spin_dither_one_tag_per_window_exact_ticks) {
  sim::reset();
  sim::state().period = 80000.0 * (1 + 25.3e-6);
  Capture c;
  SofSpinConfig cfg{sim::state().tick_hz, 100};
  cfg.dither = true;
  SofSpinSource src(cfg, sink, &c);
  run_until(src, c, 60);
  CHECK_EQ(c.events.size(), 60u);
  check_events(c, 100, 0, SIZE_MAX, true);
  size_t distinct = 0;
  for (size_t i = 1; i < c.events.size(); ++i) {
    uint32_t off = static_cast<uint32_t>(c.events[i].frame % 100);
    CHECK(off < 50);
    if (off != c.events[i - 1].frame % 100) distinct++;
  }
  CHECK(distinct > 40);
  CHECK_EQ(src.stats().late, 0u);
  CHECK(sim::state().spin_ticks / c.events.size() < us_ticks(500));
}

TEST(spin_dither_divider_change) {
  sim::reset();
  Capture c;
  SofSpinConfig cfg{sim::state().tick_hz, 100};
  cfg.dither = true;
  SofSpinSource src(cfg, sink, &c);
  run_until(src, c, 5);
  src.set_divider(30);
  size_t before = c.events.size();
  run_until(src, c, before + 20);
  check_events(c, 30, before + 1, SIZE_MAX, true);
}

TEST(spin_tracks_host_clock_drift) {
  sim::reset();
  sim::state().period = 80000.0 * (1 + 400e-6);  // +400 ppm
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  run_until(src, c, 50);
  CHECK_EQ(c.events.size(), 50u);
  check_events(c, 100);
  CHECK(sim::state().spin_ticks / c.events.size() < us_ticks(500));
  CHECK(src.stats().period_ticks >= 80031 && src.stats().period_ticks <= 80033);
}

TEST(spin_survives_wakeup_jitter) {
  sim::reset();
  sim::state().sleep_jitter = us_ticks(60);
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  run_until(src, c, 30);
  CHECK_EQ(c.events.size(), 30u);
  check_events(c, 100);
}

TEST(spin_late_wakeup_never_reports_wrong_frame) {
  sim::reset();
  sim::state().sleep_jitter = us_ticks(1500);  // wake more than a frame late
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  for (int i = 0; i < 2000; ++i) src.step();
  check_events(c, 100);
  CHECK(src.stats().late > 0);
}

TEST(spin_divider_change_takes_effect) {
  sim::reset();
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  run_until(src, c, 5);
  src.set_divider(250);
  size_t before = c.events.size();
  run_until(src, c, before + 10);
  for (size_t i = before; i < c.events.size(); ++i) CHECK_EQ(c.events[i].frame % 250, 0u);
  const sim::State& s = sim::state();
  for (const Event& e : c.events) {
    CHECK_EQ(e.tick, static_cast<uint32_t>(sim::sof_time(e.true_index) + s.detect_latency));
  }
}

TEST(spin_large_divider_keeps_unwrap_valid) {
  sim::reset();
  Capture c;
  SofSpinConfig cfg{sim::state().tick_hz, 5000};  // 5 s between tags, > 2048 frames
  SofSpinSource src(cfg, sink, &c);
  run_until(src, c, 6);
  CHECK_EQ(c.events.size(), 6u);
  check_events(c, 5000);
  CHECK_EQ(c.events[1].frame - c.events[0].frame, 5000u);
}

TEST(spin_idles_when_usb_down) {
  sim::reset();
  sim::state().usb_ready = false;
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  for (int i = 0; i < 100; ++i) src.step();
  CHECK(c.events.empty());
  CHECK_EQ(sim::state().spin_ticks, 0u);
  sim::state().usb_ready = true;
  run_until(src, c, 5);
  CHECK_EQ(c.events.size(), 5u);
  check_events(c, 100);
}

TEST(spin_recovers_after_sof_stops) {
  sim::reset();
  Capture c;
  SofSpinSource src(SofSpinConfig{sim::state().tick_hz, 100}, sink, &c);
  run_until(src, c, 5);
  sim::state().sof_running = false;
  for (int i = 0; i < 50; ++i) src.step();
  CHECK_EQ(c.events.size(), 5u);
  CHECK(src.stats().timeouts > 0);
  sim::state().sof_running = true;
  run_until(src, c, 10);
  CHECK_EQ(c.events.size(), 10u);
  // frame count jumps over the outage
  check_events(c, 100, 0, 5);
  check_events(c, 100, 5, 10);
}
