// SOF timestamps for ports without hardware SOF capture: sleep until just
// before the next tagged frame, then spin on the frame number register and
// read the counter when it changes.
#pragma once

#include <atomic>
#include <cstdint>

#include "sofgps/sof_schedule.h"
#include "sofgps/unwrap.h"

namespace sofgps {

struct SofSpinConfig {
  uint32_t tick_hz;
  uint32_t divider;
  bool dither = false;
  uint32_t lead_us = 200;           // wake this early
  uint32_t max_sleep_us = 500000;   // re-read the frame number at least this often
  uint32_t spin_timeout_us = 3000;  // give up spinning after this
  uint32_t idle_sleep_us = 100000;  // pause when USB or SOFs are down
};

struct SofSpinStats {
  uint32_t tags = 0;      // frames reported
  uint32_t late = 0;      // spin ended past the target
  uint32_t timeouts = 0;  // no SOF while spinning
  uint32_t replans = 0;   // woke more than a frame early
  uint32_t period_ticks = 0;  // estimated SOF period
};

class SofSpinSource {
 public:
  // Called from the spin task.
  using Sink = void (*)(uint32_t tick, uint64_t frame, void* ctx);

  SofSpinSource(const SofSpinConfig& cfg, Sink sink, void* ctx);

  void set_divider(uint32_t d) { divider_.store(d ? d : 1); }
  uint32_t divider() const { return divider_.load(); }
  SofSchedule schedule() const { return SofSchedule{divider_.load(), cfg_.dither}; }

  // Run in a loop from a dedicated task.
  void step();

  const SofSpinStats& stats() const { return stats_; }
  // Lets other cores keep their unwrappers current without reading the counter.
  uint32_t last_tick() const { return last_tick_.load(std::memory_order_relaxed); }

 private:
  bool acquire();
  void spin_for(uint64_t target);
  void set_reference(uint64_t tick, uint64_t frame);
  uint64_t us_to_ticks(uint64_t us) const;
  uint64_t ticks_to_us(uint64_t ticks) const;

  SofSpinConfig cfg_;
  std::atomic<uint32_t> divider_;
  std::atomic<uint32_t> last_tick_{0};
  Sink sink_;
  void* ctx_;
  TickUnwrapper ticks_;
  FrameUnwrapper frames_;
  bool have_ref_ = false;
  uint64_t ref_tick_ = 0;
  uint64_t ref_frame_ = 0;
  uint64_t period_q16_;  // ticks per frame, 16.16 fixed point
  SofSpinStats stats_;
};

}  // namespace sofgps
