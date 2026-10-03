// TinyUSB device glue and the SOF frame-number spin.
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_private/usb_phy.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "port_internal.h"
#include "soc/usb_dwc_struct.h"
#include "sofgps/hal.h"
#include "tusb.h"

namespace {

const char* TAG = "usb";
constexpr int kUsbTaskPriority = 5;
constexpr int kUsbTaskCore = 0;

usb_phy_handle_t s_phy;
sofgps::hal::LineCodingCallback s_lc_cb;
sofgps::hal::CdcRxCallback s_rx_cb;
void* s_cb_ctx;
uint32_t s_line_coding_events;
uint32_t s_line_state_events;
uint32_t s_last_baud;
bool s_dcd_wanted;
bool s_dcd_sent;
bool s_dcd_valid;  // s_dcd_sent is what the host has seen
uint32_t s_cpu_hz;

// SERIAL_STATE notification carrying DCD (and DSR, always on).
bool dcd_send() {
  cdc_notify_uart_state_t st = {};
  st.dsr = 1;
  st.dcd = s_dcd_wanted;
  if (!tud_cdc_notify_uart_state(&st)) return false;
  s_dcd_sent = s_dcd_wanted;
  s_dcd_valid = true;
  return true;
}

void usb_task(void*) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
  for (;;) {
    tud_task_ext(1000, false);  // returns every second to feed the watchdog
    esp_task_wdt_reset();
  }
}

}  // namespace

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p) {
  (void)itf;
  s_line_coding_events++;
  s_last_baud = p->bit_rate;
  if (!s_lc_cb) return;
  sofgps::hal::LineCoding lc{p->bit_rate, p->data_bits, p->parity, p->stop_bits};
  s_lc_cb(lc, s_cb_ctx);
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  (void)dtr;
  (void)rts;
  s_line_state_events++;
}

extern "C" void tud_cdc_notify_complete_cb(uint8_t itf) {
  (void)itf;
  if (s_dcd_valid && s_dcd_sent != s_dcd_wanted) dcd_send();
}

extern "C" void tud_mount_cb(void) { s_dcd_valid = false; }

extern "C" void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;
  if (s_rx_cb) s_rx_cb(s_cb_ctx);
}

#if CONFIG_SOFGPS_SOF_DEBUG
namespace sofgps::port {
SofDebug g_sof_debug;
}
#endif

namespace sofgps::hal {

UsbStats usb_stats() {
  UsbStats st;
  st.mounted = tud_mounted();
  st.suspended = tud_suspended();
  st.dtr = tud_cdc_connected();
  st.line_coding_events = s_line_coding_events;
  st.line_state_events = s_line_state_events;
  st.baud = s_last_baud;
  return st;
}

void usb_start() {
  s_cpu_hz = static_cast<uint32_t>(esp_clk_cpu_freq());
  usb_phy_config_t phy = {};
  phy.controller = USB_PHY_CTRL_OTG;
  phy.target = USB_PHY_TARGET_INT;
  phy.otg_mode = USB_OTG_MODE_DEVICE;
  phy.otg_speed = USB_PHY_SPEED_FULL;
  ESP_ERROR_CHECK(usb_new_phy(&phy, &s_phy));

  // TinyUSB by default overwrites the TX FIFO while DTR is low, which cuts
  // messages in half. Turn that off.
  tud_cdc_configure_t cdc_cfg = TUD_CDC_CONFIGURE_DEFAULT();
  cdc_cfg.tx_overwritabe_if_not_connected = 0;
  tud_cdc_configure(&cdc_cfg);

  // The USB interrupt lands on the calling core; keep it off the SOF spin core.
  tusb_rhport_init_t rh = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
  if (!tusb_init(0, &rh)) {
    ESP_LOGE(TAG, "tusb_init failed");
    abort();
  }
  xTaskCreatePinnedToCore(usb_task, "tinyusb", 4096, nullptr, kUsbTaskPriority, nullptr,
                          kUsbTaskCore);
}

bool usb_ready() { return tud_mounted() && !tud_suspended(); }

uint16_t usb_frame_number() { return USB_DWC.dsts_reg.soffn & 0x7FF; }

// Every poll must take the same time, or the timestamps get biased. So the
// timeout uses the cheap CPU cycle counter, not a capture (about 85 cycles).
// One poll takes about 36 cycles (150 ns), which sets the resolution.
bool IRAM_ATTR usb_sof_wait_change(uint32_t timeout_ticks, uint32_t* tick, uint16_t* frame) {
  const uint32_t timeout_cycles = static_cast<uint32_t>(
      static_cast<uint64_t>(timeout_ticks) * s_cpu_hz / tick_hz());
  bool ok = false;
  portENTER_CRITICAL(&port::g_soft_capture_mux);
  const uint32_t f0 = USB_DWC.dsts_reg.soffn;
  const uint32_t start = esp_cpu_get_cycle_count();
#if CONFIG_SOFGPS_SOF_DEBUG
  uint32_t c_last = start, max_gap = 0, max_read = 0, n = 0;
#endif
  for (;;) {
    const uint32_t c0 = esp_cpu_get_cycle_count();
    const uint32_t f = USB_DWC.dsts_reg.soffn;
    if (f != f0) {
#if CONFIG_SOFGPS_SOF_DEBUG
      const uint32_t c1 = esp_cpu_get_cycle_count();
#endif
      *tick = port::soft_capture(port::kSoftCaptureChannel);
      *frame = static_cast<uint16_t>(f & 0x7FF);
#if CONFIG_SOFGPS_SOF_DEBUG
      const uint32_t c2 = esp_cpu_get_cycle_count();
      port::g_sof_debug = {c0 - c_last, c1 - c0, c2 - c1, max_gap, max_read, n, c1 - 3 * *tick};
#endif
      ok = true;
      break;
    }
    if (c0 - start >= timeout_cycles) break;
#if CONFIG_SOFGPS_SOF_DEBUG
    const uint32_t c1 = esp_cpu_get_cycle_count();
    if (c0 - c_last > max_gap) max_gap = c0 - c_last;
    if (c1 - c0 > max_read) max_read = c1 - c0;
    c_last = c0;
    n++;
#endif
  }
  portEXIT_CRITICAL(&port::g_soft_capture_mux);
  return ok;
}

void cdc_set_callbacks(LineCodingCallback lc, CdcRxCallback rx, void* ctx) {
  s_lc_cb = lc;
  s_rx_cb = rx;
  s_cb_ctx = ctx;
}

bool cdc_connected() { return tud_cdc_connected(); }

bool cdc_set_dcd(bool on) {
  s_dcd_wanted = on;
  if (s_dcd_valid && s_dcd_sent == on) return true;
  return dcd_send();
}
size_t cdc_write(const uint8_t* data, size_t len) { return tud_cdc_write(data, len); }
void cdc_write_flush() { tud_cdc_write_flush(); }
size_t cdc_write_available() { return tud_cdc_write_available(); }
size_t cdc_write_capacity() { return CFG_TUD_CDC_TX_BUFSIZE; }
size_t cdc_read(uint8_t* buf, size_t cap) { return tud_cdc_read(buf, cap); }
size_t cdc_read_available() { return tud_cdc_available(); }

}  // namespace sofgps::hal
