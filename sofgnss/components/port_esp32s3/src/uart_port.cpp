// GPS UART. Uses whatever line coding the host sets on the CDC port.
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "hal/uart_ll.h"
#include "port_internal.h"
#include "sdkconfig.h"
#include "sofgnss/hal.h"

namespace {

const char* TAG = "uart";
constexpr uart_port_t kUart = static_cast<uart_port_t>(CONFIG_SOFGNSS_GPS_UART_NUM);
constexpr size_t kRxBuf = 4096;
constexpr size_t kTxBuf = 2048;

QueueHandle_t s_events;

void handle_event(const uart_event_t& ev) {
  if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
    ESP_LOGW(TAG, "rx overflow, flushing");
    uart_flush_input(kUart);
    xQueueReset(s_events);
  }
}

}  // namespace

namespace sofgnss::port {

void uart_init() {
  uart_config_t cfg = {};
  cfg.baud_rate = CONFIG_SOFGNSS_GPS_DEFAULT_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(kUart, kRxBuf, kTxBuf, 16, &s_events, 0));
  ESP_ERROR_CHECK(uart_param_config(kUart, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(kUart, CONFIG_SOFGNSS_GPS_TX_GPIO, CONFIG_SOFGNSS_GPS_RX_GPIO,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // Hand over data after 2 idle characters or 64 bytes.
  ESP_ERROR_CHECK(uart_set_rx_timeout(kUart, 2));
  ESP_ERROR_CHECK(uart_set_rx_full_threshold(kUart, 64));
  ESP_LOGI(TAG, "GPS on UART%d TX=GPIO%d RX=GPIO%d, %d baud 8N1", CONFIG_SOFGNSS_GPS_UART_NUM,
           CONFIG_SOFGNSS_GPS_TX_GPIO, CONFIG_SOFGNSS_GPS_RX_GPIO, CONFIG_SOFGNSS_GPS_DEFAULT_BAUD);
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

void gps_uart_configure(const LineCoding& lc) {
  if (lc.baud < 300 || lc.baud > 5000000) {
    ESP_LOGW(TAG, "ignoring baud %lu", (unsigned long)lc.baud);
    return;
  }
  uart_word_length_t bits;
  switch (lc.data_bits) {
    case 5: bits = UART_DATA_5_BITS; break;
    case 6: bits = UART_DATA_6_BITS; break;
    case 7: bits = UART_DATA_7_BITS; break;
    default: bits = UART_DATA_8_BITS; break;
  }
  uart_parity_t parity;
  switch (lc.parity) {
    case 1: parity = UART_PARITY_ODD; break;
    case 2: parity = UART_PARITY_EVEN; break;
    default: parity = UART_PARITY_DISABLE; break;  // mark/space unsupported
  }
  uart_stop_bits_t stop;
  switch (lc.stop_bits) {
    case 1: stop = UART_STOP_BITS_1_5; break;
    case 2: stop = UART_STOP_BITS_2; break;
    default: stop = UART_STOP_BITS_1; break;
  }
  uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
  uart_set_baudrate(kUart, lc.baud);
  uart_set_word_length(kUart, bits);
  uart_set_parity(kUart, parity);
  uart_set_stop_bits(kUart, stop);
  ESP_LOGI(TAG, "line coding %lu baud, %u data, parity %u, stop %u", (unsigned long)lc.baud,
           lc.data_bits, lc.parity, lc.stop_bits);
}

size_t gps_uart_wait(uint32_t timeout_ms) {
  size_t avail = 0;
  uart_get_buffered_data_len(kUart, &avail);
  if (avail == 0) {
    uart_event_t ev;
    if (!xQueueReceive(s_events, &ev, pdMS_TO_TICKS(timeout_ms))) return 0;
    handle_event(ev);
    uart_get_buffered_data_len(kUart, &avail);
    if (avail == 0) return 0;
  }
  // Drain leftover events so the queue can't fill up.
  uart_event_t ev;
  while (xQueueReceive(s_events, &ev, 0)) handle_event(ev);
  return avail;
}

size_t gps_uart_read(uint8_t* buf, size_t cap) {
  size_t avail = 0;
  uart_get_buffered_data_len(kUart, &avail);
  if (avail == 0) return 0;
  int n = uart_read_bytes(kUart, buf, avail < cap ? avail : cap, 0);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

// Read the hardware before the driver buffer, so bytes moving between them are
// counted twice at worst, never missed.
size_t gps_uart_rx_pending() {
  uart_dev_t* hw = UART_LL_GET_HW(kUart);
  size_t n = hw->fsm_status.st_urx_out != 0 ? 1 : 0;  // receiver mid-character
  n += uart_ll_get_rxfifo_len(hw);
  size_t buffered = 0;
  uart_get_buffered_data_len(kUart, &buffered);
  return n + buffered;
}

size_t gps_uart_write_available() {
  size_t free = 0;
  uart_get_tx_buffer_free_size(kUart, &free);
  return free;
}

size_t gps_uart_write(const uint8_t* data, size_t len) {
  size_t space = gps_uart_write_available();
  if (len > space) len = space;
  if (len == 0) return 0;
  int n = uart_write_bytes(kUart, data, len);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

}  // namespace sofgnss::hal
