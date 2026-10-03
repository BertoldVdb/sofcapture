#include "hal_sim.h"

#include <cmath>

#include "sofgps/hal.h"

namespace sim {

State& state() {
  static State s;
  return s;
}

void reset() { state() = State{}; }

uint64_t sof_time(uint64_t index) {
  return state().origin + static_cast<uint64_t>(std::llround(index * state().period));
}

uint64_t frame_index_at(uint64_t tick) {
  const State& s = state();
  uint64_t i = static_cast<uint64_t>((tick - s.origin) / s.period);
  while (sof_time(i + 1) <= tick) i++;
  while (i > 0 && sof_time(i) > tick) i--;
  return i;
}

}  // namespace sim

namespace sofgps::hal {

void init() {}

uint32_t tick_hz() { return sim::state().tick_hz; }
uint32_t tick_now() { return static_cast<uint32_t>(sim::state().now); }

void pps_capture_start(PpsCallback, void*) {}
void pps_inject() {}
bool cal_init(CalCallback, void*) { return true; }
void cal_set_pattern(const uint8_t*) {}
bool cal_burst() { return true; }
bool cal_burst_square40(uint32_t) { return true; }
bool sof_capture_start(SofCallback, void*) { return false; }

void usb_start() {}
bool usb_ready() { return sim::state().usb_ready; }

uint16_t usb_frame_number() {
  sim::State& s = sim::state();
  return static_cast<uint16_t>((s.frame_base + sim::frame_index_at(s.now)) & 0x7FF);
}

bool usb_sof_wait_change(uint32_t timeout_ticks, uint32_t* tick, uint16_t* frame) {
  sim::State& s = sim::state();
  if (!s.sof_running) {
    s.now += timeout_ticks;
    s.spin_ticks += timeout_ticks;
    return false;
  }
  uint64_t idx = sim::frame_index_at(s.now);
  uint64_t next = sim::sof_time(idx + 1) + s.detect_latency;
  if (next - s.now > timeout_ticks) {
    s.now += timeout_ticks;
    s.spin_ticks += timeout_ticks;
    return false;
  }
  s.spin_ticks += next - s.now;
  s.now = next;
  s.last_detected_index = idx + 1;
  *tick = static_cast<uint32_t>(s.now);
  *frame = static_cast<uint16_t>((s.frame_base + idx + 1) & 0x7FF);
  return true;
}

void cdc_set_callbacks(LineCodingCallback, CdcRxCallback, void*) {}
bool cdc_connected() { return true; }
bool cdc_set_dcd(bool) { return true; }
UsbStats usb_stats() { return UsbStats{true, false, true, 0, 0, 0}; }
size_t cdc_write(const uint8_t*, size_t len) { return len; }
void cdc_write_flush() {}
size_t cdc_write_available() { return 1024; }
size_t cdc_write_capacity() { return 1024; }
size_t cdc_read(uint8_t*, size_t) { return 0; }
size_t cdc_read_available() { return 0; }

void gps_uart_configure(const LineCoding&) {}
size_t gps_uart_wait(uint32_t) { return 0; }
size_t gps_uart_read(uint8_t*, size_t) { return 0; }
size_t gps_uart_rx_pending() { return 0; }
size_t gps_uart_write(const uint8_t*, size_t len) { return len; }
size_t gps_uart_write_available() { return 2048; }

void sleep_us(uint32_t us) {
  sim::State& s = sim::state();
  s.now += static_cast<uint64_t>(us) * s.tick_hz / 1000000 + s.sleep_jitter;
  s.sleep_calls++;
}

uint64_t uptime_us() {
  const sim::State& s = sim::state();
  return s.now * 1000000 / s.tick_hz;
}

void led_set(bool) {}

bool ota_begin(uint32_t) {
  sim::State& s = sim::state();
  if (!s.ota_begin_ok) return false;
  s.ota_active = true;
  s.ota_bytes = 0;
  return true;
}
bool ota_write(const uint8_t*, size_t len) {
  sim::state().ota_bytes += len;
  return true;
}
bool ota_end(const uint8_t*) {
  sim::state().ota_active = false;
  return sim::state().ota_end_ok;
}
void ota_abort() {
  sim::state().ota_active = false;
  sim::state().ota_aborts++;
}
bool ota_pending_verify() { return sim::state().ota_pending; }
void ota_confirm() {
  sim::state().ota_confirms++;
  sim::state().ota_pending = false;
}
void restart() { sim::state().restarts++; }
const char* reset_reason() { return "SIM"; }
void watchdog_register_task() {}
void watchdog_feed() {}
AppInfo app_info() { return AppInfo{"sim", "none", "ota_0", "00"}; }

}  // namespace sofgps::hal
