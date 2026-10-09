// Turns PPS and SOF captures into $PVDB sentences. Use from one task only.
#pragma once

#include <cstddef>
#include <cstdint>

#include "sofgnss/sof_schedule.h"
#include "sofgnss/unwrap.h"

namespace sofgnss {

class TimingReporter {
 public:
  using Emit = void (*)(const char* sentence, size_t len, void* ctx);

  TimingReporter(uint32_t divider, bool dither, Emit emit, void* ctx);

  void on_pps(uint32_t tick, uint32_t rx_idle_ms);
  // Only frames picked by the schedule are reported.
  void on_sof(uint32_t tick, uint64_t frame);
  void on_cal(uint32_t tick, const char* mode, bool pps_synced, uint32_t n);
  // Call every few seconds so the unwrap never misses a wrap.
  void note_tick(uint32_t tick) { ticks_.update(tick); }

  void set_divider(uint32_t d) { divider_ = d ? d : 1; }
  uint32_t divider() const { return divider_; }
  SofSchedule schedule() const { return SofSchedule{divider_, dither_}; }

  uint32_t pps_count() const { return pps_; }
  uint32_t sof_count() const { return sof_; }

 private:
  TickUnwrapper ticks_;
  uint32_t divider_;
  bool dither_;
  Emit emit_;
  void* ctx_;
  uint32_t pps_ = 0;
  uint32_t sof_ = 0;
};

}  // namespace sofgnss
