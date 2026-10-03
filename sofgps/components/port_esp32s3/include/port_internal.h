// Internal to the ESP32-S3 port, not part of the HAL.
#pragma once

#include <cstdint>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "soc/mcpwm_struct.h"

// Kconfig leaves "n" booleans undefined; treat them as 0.
#ifndef CONFIG_SOFGPS_PPS_RISING_EDGE
#define CONFIG_SOFGPS_PPS_RISING_EDGE 0
#endif
#ifndef CONFIG_SOFGPS_PPS_PULLDOWN
#define CONFIG_SOFGPS_PPS_PULLDOWN 0
#endif
#ifndef CONFIG_SOFGPS_LED_ACTIVE_LOW
#define CONFIG_SOFGPS_LED_ACTIVE_LOW 0
#endif

namespace sofgps::port {

// Capture channels on one timer. The driver assigns them in creation order.
constexpr int kPpsCaptureChannel = 0;
constexpr int kCalCaptureChannel = 1;
constexpr int kSoftCaptureChannel = 2;

// Latch the running capture timer into a channel and return it. The capture
// lands a clock or two after the trigger write, so poll until the register
// moves away from its previous value.
__attribute__((always_inline)) static inline uint32_t soft_capture(int channel) {
  uint32_t prev = MCPWM0.cap_chn[channel].capn_value;
  MCPWM0.cap_chn_cfg[channel].capn_sw = 1;
  uint32_t v;
  do {
    v = MCPWM0.cap_chn[channel].capn_value;
  } while (v == prev);
  return v;
}

void timer_init();
void uart_init();
void sys_init();

extern portMUX_TYPE g_soft_capture_mux;

#if CONFIG_SOFGPS_SOF_DEBUG
// SOF spin diagnostics, in CPU cycles (240 MHz).
struct SofDebug {
  uint32_t gap;       // previous poll to the detecting poll
  uint32_t read;      // detecting register read
  uint32_t capture;   // detecting read to capture done
  uint32_t max_gap;
  uint32_t max_read;
  uint32_t polls;
  uint32_t offset;    // cycle count minus 3 * tick
};
extern SofDebug g_sof_debug;
#endif
struct mcpwm_cap_channel_t* cal_capture_channel();

}  // namespace sofgps::port
