// TinyUSB device glue. tud_task() runs in the main loop, so the callbacks
// here never race with the HAL calls.
#include "port.h"
#include "sofgnss/hal.h"
#include "tusb.h"

namespace {

sofgnss::hal::LineCodingCallback s_lc_cb;
sofgnss::hal::CdcRxCallback s_rx_cb;
void* s_cb_ctx;
uint32_t s_line_coding_events;
uint32_t s_line_state_events;
uint32_t s_last_baud;
bool s_dcd_wanted;
bool s_dcd_sent;
bool s_dcd_valid;  // s_dcd_sent is what the host has seen

// If the wanted level changes meanwhile, the notify-complete callback sends again.
bool dcd_send() {
  cdc_notify_uart_state_t st = {};
  st.dsr = 1;
  st.dcd = s_dcd_wanted;
  if (!tud_cdc_notify_uart_state(&st)) return false;
  s_dcd_sent = s_dcd_wanted;
  s_dcd_valid = true;
  return true;
}

}  // namespace

extern "C" {

// Flags the handler so a PPS arriving meanwhile doesn't start a burst (see cal_start).
volatile bool s_usb_isr_active;

static void usb_irq() {
  s_usb_isr_active = true;
  tud_int_handler(0);
  s_usb_isr_active = false;
}

void USB_HP_IRQHandler() { usb_irq(); }
void USB_LP_IRQHandler() { usb_irq(); }
void USBWakeUp_IRQHandler() { usb_irq(); }

uint32_t tusb_time_millis_api() { return static_cast<uint32_t>(sofgnss::hal::uptime_us() / 1000); }

void tud_cdc_line_coding_cb(uint8_t, cdc_line_coding_t const* p) {
  s_line_coding_events++;
  s_last_baud = p->bit_rate;
  if (!s_lc_cb) return;
  sofgnss::hal::LineCoding lc{p->bit_rate, p->data_bits, p->parity, p->stop_bits};
  s_lc_cb(lc, s_cb_ctx);
}

void tud_cdc_line_state_cb(uint8_t, bool dtr, bool) {
  s_line_state_events++;
#if CONFIG_SOFGNSS_REQUIRE_DTR
  // Port closed: drop queued data so the next open starts fresh.
  if (!dtr) tud_cdc_write_clear();
#else
  (void)dtr;
#endif
}

void tud_cdc_notify_complete_cb(uint8_t) {
  if (s_dcd_valid && s_dcd_sent != s_dcd_wanted) dcd_send();
}

void tud_mount_cb() { s_dcd_valid = false; }

void tud_cdc_rx_cb(uint8_t) {
  if (s_rx_cb) s_rx_cb(s_cb_ctx);
}

}  // extern "C"

namespace sofgnss::port {

bool usb_isr_active() { return s_usb_isr_active; }

void usb_detach(uint32_t ms) {
  tud_disconnect();
  delay_ms(ms);
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

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
  RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;  // 48 MHz set up by clock_init()
  (void)RCC->APB1ENR1;
  // After any reset the host must see us leave the bus. The pull-up stays off
  // until TinyUSB connects.
  port::delay_ms(port::kUsbDetachMs);

  // TinyUSB by default overwrites the TX FIFO while DTR is low, which cuts
  // messages in half. Turn that off.
  tud_cdc_configure_t cdc_cfg = TUD_CDC_CONFIGURE_DEFAULT();
  cdc_cfg.tx_overwritabe_if_not_connected = 0;
  tud_cdc_configure(&cdc_cfg);

  NVIC_SetPriority(USB_HP_IRQn, port::kPrioUsb);
  NVIC_SetPriority(USB_LP_IRQn, port::kPrioUsb);
  NVIC_SetPriority(USBWakeUp_IRQn, port::kPrioUsb);
  tusb_rhport_init_t rh = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
  tusb_init(0, &rh);
}

bool usb_ready() { return tud_mounted() && !tud_suspended(); }

uint16_t usb_frame_number() { return USB->FNR & USB_FNR_FN; }

void cdc_set_callbacks(LineCodingCallback lc, CdcRxCallback rx, void* ctx) {
  s_lc_cb = lc;
  s_rx_cb = rx;
  s_cb_ctx = ctx;
}

bool cdc_connected() { return tud_cdc_connected(); }

bool cdc_set_dcd(bool on) {
  s_dcd_wanted = on;
  return (s_dcd_valid && s_dcd_sent == on) || dcd_send();
}

size_t cdc_write(const uint8_t* data, size_t len) { return tud_cdc_write(data, len); }
void cdc_write_flush() { tud_cdc_write_flush(); }
size_t cdc_write_available() { return tud_cdc_write_available(); }
size_t cdc_write_capacity() { return CFG_TUD_CDC_TX_BUFSIZE; }
size_t cdc_read(uint8_t* buf, size_t cap) { return tud_cdc_read(buf, cap); }
size_t cdc_read_available() { return tud_cdc_available(); }

}  // namespace sofgnss::hal
