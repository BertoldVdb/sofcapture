// Timestamp counter: the MCPWM capture timer at 80 MHz. Channels: see port_internal.h.
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/mcpwm_ll.h"
#include "port_internal.h"
#include "sdkconfig.h"
#include "sofgps/hal.h"

namespace {

const char* TAG = "timer";

mcpwm_cap_timer_handle_t s_timer;
mcpwm_cap_channel_handle_t s_pps;
mcpwm_cap_channel_handle_t s_cal;
uint32_t s_tick_hz;
sofgps::hal::PpsCallback s_pps_cb;
void* s_pps_ctx;

bool IRAM_ATTR on_pps_capture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t* e,
                              void*) {
  return s_pps_cb(e->cap_value, s_pps_ctx);
}

}  // namespace

namespace sofgps::port {

portMUX_TYPE g_soft_capture_mux = portMUX_INITIALIZER_UNLOCKED;

mcpwm_cap_channel_t* cal_capture_channel() { return s_cal; }

void timer_init() {
  mcpwm_capture_timer_config_t tc = {};
  tc.group_id = 0;
  tc.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  tc.resolution_hz = CONFIG_SOFGPS_TICK_HZ;
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tc, &s_timer));
  ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(s_timer, &s_tick_hz));

  // Must be created first to get channel 0.
  mcpwm_capture_channel_config_t cc = {};
  cc.gpio_num = CONFIG_SOFGPS_PPS_GPIO;
  cc.prescale = 1;
  cc.flags.pos_edge = CONFIG_SOFGPS_PPS_RISING_EDGE;
  cc.flags.neg_edge = !CONFIG_SOFGPS_PPS_RISING_EDGE;
  cc.flags.pull_down = CONFIG_SOFGPS_PPS_PULLDOWN;
  ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_timer, &cc, &s_pps));

  // Calibration burst start. cal_init connects the pad later.
  mcpwm_capture_channel_config_t cal = {};
  cal.gpio_num = -1;
  cal.prescale = 1;
  cal.flags.neg_edge = 1;  // chip select goes low when a burst starts
  ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_timer, &cal, &s_cal));

  mcpwm_ll_capture_enable_channel(&MCPWM0, kSoftCaptureChannel, true);

  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(s_timer));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(s_timer));
  ESP_LOGI(TAG, "capture timer %lu Hz, PPS on GPIO%d (%s edge)", (unsigned long)s_tick_hz,
           CONFIG_SOFGPS_PPS_GPIO, CONFIG_SOFGPS_PPS_RISING_EDGE ? "rising" : "falling");
}

}  // namespace sofgps::port

namespace sofgps::hal {

uint32_t tick_hz() { return s_tick_hz; }

uint32_t tick_now() {
  portENTER_CRITICAL(&port::g_soft_capture_mux);
  uint32_t v = port::soft_capture(port::kSoftCaptureChannel);
  portEXIT_CRITICAL(&port::g_soft_capture_mux);
  return v;
}

void pps_capture_start(PpsCallback cb, void* ctx) {
  s_pps_cb = cb;
  s_pps_ctx = ctx;
  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = on_pps_capture;
  // The interrupt lands on the calling core.
  ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(s_pps, &cbs, nullptr));
  ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_pps));
}

// A software capture fires the same interrupt as a real edge.
void pps_inject() { mcpwm_ll_trigger_soft_capture(&MCPWM0, port::kPpsCaptureChannel); }

// The ESP32-S3 can't route SOF to the capture timer.
bool sof_capture_start(SofCallback, void*) { return false; }

}  // namespace sofgps::hal
