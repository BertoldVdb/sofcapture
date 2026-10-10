// Hardware layer for the portable core; each port implements all of it.
// "ISR context" callbacks run in an interrupt and return true if a context
// switch is needed on exit (FreeRTOS xHigherPriorityTaskWoken).
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgnss::hal {

// Called once at startup, before anything else.
void init();

// ---- Timestamp counter -----------------------------------------------------
// One free-running 32-bit counter, shared by PPS and SOF. It wraps.
uint32_t tick_hz();
// ESP32-S3: may block a few ms while a SOF spin runs. Best called from the
// SOF spin task.
uint32_t tick_now();

// ---- PPS input -------------------------------------------------------------
// ISR context.
using PpsCallback = bool (*)(uint32_t tick, void* ctx);
void pps_capture_start(PpsCallback cb, void* ctx);
// Test hook: fake a PPS edge now.
void pps_inject();

// ---- SOF capture -----------------------------------------------------------
// Hardware SOF capture (e.g. STM32): the callback gets every SOF, ISR context.
// Returns false if the port has none; SofSpinSource is used instead.
using SofCallback = bool (*)(uint32_t tick, uint16_t frame, void* ctx);
bool sof_capture_start(SofCallback cb, void* ctx);

// ---- Calibration output ------------------------------------------------------
// Called when a burst has ended, with its start tick and its measured length in
// ticks, 0 if not measured (ISR context).
using CalCallback = bool (*)(uint32_t tick, uint32_t length, void* ctx);
bool cal_init(CalCallback cb, void* ctx);
uint32_t cal_chip_rate();  // SPI bits per second
// kCalBytes long. Not during a burst.
void cal_set_pattern(const uint8_t* data);
// Blocks until the burst has been sent.
bool cal_burst();
// Start tick is within about 50 ns of the first edge.
bool cal_burst_square40(uint32_t duration_us);

// ---- USB device (TinyUSB CDC-ACM) ------------------------------------------
void usb_start();
bool usb_ready();             // configured and not suspended
uint16_t usb_frame_number();  // 11 bits, latest SOF

// Spins with interrupts off until the frame number changes; *tick is read
// right after the change. SOF spin task only.
bool usb_sof_wait_change(uint32_t timeout_ticks, uint32_t* tick, uint16_t* frame);

// USB CDC encoding: parity 0=none 1=odd 2=even 3=mark 4=space;
// stop_bits 0=1 1=1.5 2=2.
struct LineCoding {
  uint32_t baud;
  uint8_t data_bits;
  uint8_t parity;
  uint8_t stop_bits;
};

// Called from the USB task.
using LineCodingCallback = void (*)(const LineCoding& lc, void* ctx);
using CdcRxCallback = void (*)(void* ctx);
void cdc_set_callbacks(LineCodingCallback lc, CdcRxCallback rx, void* ctx);

bool cdc_connected();         // DTR set (Linux: port open)
// Sets the DCD line. False if not queued yet; the port retries by itself.
bool cdc_set_dcd(bool on);

// Reported in the $PVDB,SOFPPS,USB sentence.
struct UsbStats {
  bool mounted;
  bool suspended;
  bool dtr;
  uint32_t line_coding_events;  // SET_LINE_CODING count
  uint32_t line_state_events;   // SET_CONTROL_LINE_STATE count
  uint32_t baud;                // from the last line coding
};
UsbStats usb_stats();
size_t cdc_write(const uint8_t* data, size_t len);  // non-blocking, returns bytes taken
void cdc_write_flush();
size_t cdc_write_available();
size_t cdc_write_capacity();  // TX FIFO size
size_t cdc_read(uint8_t* buf, size_t cap);  // non-blocking
size_t cdc_read_available();

// ---- GPS UART --------------------------------------------------------------
void gps_uart_configure(const LineCoding& lc);
// Waits for data; returns bytes buffered, 0 on timeout.
size_t gps_uart_wait(uint32_t timeout_ms);
size_t gps_uart_read(uint8_t* buf, size_t cap);  // non-blocking
// Bytes not read yet, including the FIFO and a byte still on the wire.
// May count a byte twice, never misses one.
size_t gps_uart_rx_pending();
size_t gps_uart_write(const uint8_t* data, size_t len);  // non-blocking, returns bytes taken
size_t gps_uart_write_available();

// ---- Firmware update (A/B application slots) --------------------------------
bool ota_begin(uint32_t size);                   // erases the inactive slot
bool ota_write(const uint8_t* data, size_t len);
bool ota_end(const uint8_t expected_sha256[32]);  // verifies, boots it next time
void ota_abort();
bool ota_pending_verify();  // new image not confirmed yet
void ota_confirm();         // keep this image, no rollback
void restart();
const char* reset_reason();  // POWERON, SW, PANIC, TASK_WDT, ...

// Each long-running task registers once, then feeds every few seconds.
void watchdog_register_task();
void watchdog_feed();

struct AppInfo {
  char version[32];
  char idf[32];
  char partition[16];
  char elf_sha256[65];  // hex
};
AppInfo app_info();

// ---- System ----------------------------------------------------------------
// Not reentrant; only the SOF spin task uses it.
void sleep_us(uint32_t us);
uint64_t uptime_us();  // ISR-safe
void led_set(bool on);  // no-op without an LED

}  // namespace sofgnss::hal
