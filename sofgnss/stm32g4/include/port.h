// Internal to the STM32G4 port, not part of the HAL.
#pragma once

#include <cstdint>

#include "config.h"
#include "sofgnss/hal.h"
#include "stm32g4xx.h"

namespace sofgnss::port {

// SYSCLK and the TIM2 timestamp counter.
constexpr uint32_t kCoreHz = 144000000;

// NVIC priorities (0 = most urgent). TIM2 must win so the SOF capture is
// always paired with the right frame number.
constexpr uint32_t kPrioTim2 = 0;
constexpr uint32_t kPrioUart = 1;
constexpr uint32_t kPrioUsb = 2;
constexpr uint32_t kPrioSysTick = 3;

// ---- Pins (NUCLEO-G474RE) ---------------------------------------------------
struct Pin {
  GPIO_TypeDef* port;
  uint8_t pin;
  uint8_t af;
};
constexpr Pin kGpsTx{GPIOB, 10, 7};     // USART3_TX, Arduino D6
constexpr Pin kGpsRx{GPIOB, 11, 7};     // USART3_RX, CN10 pin 18
constexpr Pin kPps{GPIOA, 1, 1};        // TIM2_CH2, Arduino A1
constexpr Pin kConsoleTx{GPIOA, 2, 12}; // LPUART1_TX, ST-LINK virtual COM port
constexpr Pin kLed{GPIOA, 5, 0};        // LD2, active high
constexpr Pin kCalOut{GPIOA, 7, 5};     // SPI1_MOSI, Arduino D11
constexpr Pin kCalMarker{GPIOA, 4, 5};  // SPI1_NSS, Arduino A2, wired to kCalCapture
constexpr Pin kCalCapture{GPIOA, 9, 10};  // TIM2_CH3, Arduino D8

enum class Pull : uint8_t { None = 0, Up = 1, Down = 2 };
void pin_af(const Pin& p, Pull pull = Pull::None);
void pin_output(const Pin& p);
inline void pin_write(const Pin& p, bool on) { p.port->BSRR = (on ? 1u : 0x10000u) << p.pin; }

void clock_init();
uint32_t hse_hz();  // 0: running from HSI16
bool hse_bypass();  // HSE is an external clock, not a crystal
uint32_t hse_measured(bool bypass);  // against HSI16, 0 if that mode didn't start
void timer_init();
void uart_init();

// The TIM2 counter extended to 64 bits. Any context.
uint64_t ticks64();
// SOF captures lost to overcapture, or dropped because the next SOF came
// before the frame number was read.
uint32_t sof_dropped();

// Calibration bursts. cal_start() runs in the TIM2 interrupt or with
// interrupts off, and returns false while a burst is running. The CAL callback
// runs when a burst has ended, with its captured start tick.
void cal_capture_start();
void cal_started(uint32_t tick);  // from the capture interrupt
uint8_t* cal_buffer();            // kCalBytes, not while busy
bool cal_busy();
bool cal_start();
uint32_t cal_last_ticks();        // start to end of the last burst, 0 if unknown
bool usb_isr_active();            // inside the USB interrupt handler

// How long the GPS line had been quiet at `now_us`, in ms, or 0 if a byte was
// arriving or bytes came since the line last went idle. TIM2 ISR only.
uint32_t gps_line_idle_ms(uint64_t now_us);

// Debug console on the ST-LINK virtual COM port (115200 8N1). Blocking.
void log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Busy wait that keeps the watchdog fed.
void delay_ms(uint32_t ms);

// Drops off the USB bus for `ms`. Shorter than 250 ms is unreliable behind hubs.
constexpr uint32_t kUsbDetachMs = 300;
void usb_detach(uint32_t ms);

}  // namespace sofgnss::port
