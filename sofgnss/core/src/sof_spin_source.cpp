#include "sofgnss/sof_spin_source.h"

#include "sofgnss/hal.h"

namespace sofgnss {

SofSpinSource::SofSpinSource(const SofSpinConfig& cfg, Sink sink, void* ctx)
    : cfg_(cfg), divider_(cfg.divider ? cfg.divider : 1), sink_(sink), ctx_(ctx) {
  // Assume 1 kHz until measured.
  period_q16_ = (static_cast<uint64_t>(cfg_.tick_hz) << 16) / 1000;
  stats_.period_ticks = static_cast<uint32_t>(period_q16_ >> 16);
}

uint64_t SofSpinSource::us_to_ticks(uint64_t us) const { return us * cfg_.tick_hz / 1000000; }
uint64_t SofSpinSource::ticks_to_us(uint64_t ticks) const { return ticks * 1000000 / cfg_.tick_hz; }

void SofSpinSource::set_reference(uint64_t tick, uint64_t frame) {
  if (have_ref_ && frame > ref_frame_) {
    uint64_t span_f = frame - ref_frame_;
    uint64_t span_t = tick - ref_tick_;
    // Update the period, ignoring values more than 1% off 1 kHz.
    if (span_f <= 65536 && span_t < (uint64_t{1} << 40)) {
      uint64_t p = (span_t << 16) / span_f;
      uint64_t nominal = (static_cast<uint64_t>(cfg_.tick_hz) << 16) / 1000;
      uint64_t diff = p > nominal ? p - nominal : nominal - p;
      if (diff < nominal / 100) {
        period_q16_ = p;
        stats_.period_ticks = static_cast<uint32_t>(p >> 16);
      }
    }
  }
  ref_tick_ = tick;
  ref_frame_ = frame;
  have_ref_ = true;
}

// Grab any SOF as a starting reference.
bool SofSpinSource::acquire() {
  uint32_t tick;
  uint16_t frame;
  if (!hal::usb_sof_wait_change(static_cast<uint32_t>(us_to_ticks(cfg_.spin_timeout_us)), &tick,
                                &frame)) {
    stats_.timeouts++;
    return false;
  }
  set_reference(ticks_.update(tick), frames_.update(frame));
  return true;
}

void SofSpinSource::step() {
  uint32_t raw_now = hal::tick_now();
  last_tick_.store(raw_now, std::memory_order_relaxed);
  if (!hal::usb_ready()) {
    have_ref_ = false;
    hal::sleep_us(cfg_.idle_sleep_us);
    return;
  }

  uint64_t now = ticks_.update(raw_now);
  uint64_t frame = frames_.update(hal::usb_frame_number());

  if (!have_ref_) {
    if (!acquire()) hal::sleep_us(cfg_.idle_sleep_us);
    return;
  }

  uint64_t target = schedule().next_target_after(frame);
  uint64_t predicted = ref_tick_ + (((target - ref_frame_) * period_q16_) >> 16);
  int64_t wait = static_cast<int64_t>(predicted - now);

  // Wake early enough for wake-up latency plus 500 ppm clock drift.
  int64_t lead = static_cast<int64_t>(us_to_ticks(cfg_.lead_us)) + (wait > 0 ? wait / 2000 : 0);
  int64_t sleep = wait - lead;
  int64_t max_sleep = static_cast<int64_t>(us_to_ticks(cfg_.max_sleep_us));

  if (sleep > max_sleep) {
    // Sleep in chunks so the 11-bit frame unwrap stays valid.
    hal::sleep_us(cfg_.max_sleep_us);
    return;
  }
  if (sleep > 0) hal::sleep_us(static_cast<uint32_t>(ticks_to_us(static_cast<uint64_t>(sleep))));
  spin_for(target);
}

void SofSpinSource::spin_for(uint64_t target) {
  uint32_t timeout = static_cast<uint32_t>(us_to_ticks(cfg_.spin_timeout_us));
  int extra = 0;
  for (;;) {
    uint32_t tick;
    uint16_t f;
    if (!hal::usb_sof_wait_change(timeout, &tick, &f)) {
      stats_.timeouts++;
      have_ref_ = false;
      return;
    }
    uint64_t frame = frames_.update(f);
    uint64_t t64 = ticks_.update(tick);

    if (frame < target) {
      if (target - frame > 1 || ++extra > 2) {
        // Woke too early: use this SOF as the reference and plan again.
        set_reference(t64, frame);
        stats_.replans++;
        return;
      }
      continue;  // the next change is the target
    }

    set_reference(t64, frame);
    if (frame == target) {
      stats_.tags++;
      sink_(tick, frame, ctx_);
    } else {
      stats_.late++;
    }
    return;
  }
}

}  // namespace sofgnss
