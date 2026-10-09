// Calibration bursts. SPI2 sends the pattern at 40 Mbit/s, clocked from APB like
// the capture timer. Its chip select is looped back to capture channel 1, so
// each burst start is timestamped in hardware.
//
// SQ40 uses LEDC instead. The chip-select pad then becomes a plain GPIO that
// we pull low right as LEDC starts, so the capture still marks the start.
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm_cap.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "hal/ledc_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/ledc_periph.h"
#include "soc/spi_periph.h"
#include "port_internal.h"
#include "sdkconfig.h"
#include "soc/mcpwm_periph.h"
#include "sofgnss/cal_signal.h"
#include "sofgnss/hal.h"

namespace {

const char* TAG = "cal";

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;

spi_device_handle_t s_dev;
bool s_ledc_ok;
portMUX_TYPE s_start_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t* s_buf;
sofgnss::hal::CalCallback s_cb;
void* s_cb_ctx;

// A timer wakes us this long before a SQ40 burst ends; we busy-wait the rest.
constexpr uint32_t kSq40SpinUs = 100;
esp_timer_handle_t s_stop_timer;
SemaphoreHandle_t s_stop_sem;

void IRAM_ATTR on_stop_timer(void*) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_stop_sem, &woken);
  if (woken) esp_timer_isr_dispatch_need_yield();
}

bool stop_timer_init() {
  s_stop_sem = xSemaphoreCreateBinary();
  if (!s_stop_sem) return false;
  esp_timer_create_args_t args = {};
  args.callback = on_stop_timer;
  args.dispatch_method = ESP_TIMER_ISR;
  args.name = "cal_stop";
  return esp_timer_create(&args, &s_stop_timer) == ESP_OK;
}

void wait_until(int64_t stop_at_us) {
  int64_t left = stop_at_us - esp_timer_get_time();
  if (left > static_cast<int64_t>(kSq40SpinUs)) {
    esp_timer_stop(s_stop_timer);   // fails harmlessly if not running
    xSemaphoreTake(s_stop_sem, 0);  // clear a stale wake-up
    if (esp_timer_start_once(s_stop_timer, left - kSq40SpinUs) == ESP_OK) {
      xSemaphoreTake(s_stop_sem, pdMS_TO_TICKS(left / 1000 + 10));
    }
  }
  while (esp_timer_get_time() < stop_at_us) {
  }
}

bool IRAM_ATTR on_cal_capture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t* e,
                              void*) {
  return s_cb ? s_cb(e->cap_value, s_cb_ctx) : false;
}

void route_spi() {
  esp_rom_gpio_connect_out_signal(CONFIG_SOFGNSS_CAL_GPIO, spi_periph_signal[SPI2_HOST].spid_out,
                                  false, false);
  esp_rom_gpio_connect_out_signal(CONFIG_SOFGNSS_CAL_CS_GPIO,
                                  spi_periph_signal[SPI2_HOST].spics_out[0], false, false);
}

bool ledc_init() {
  ledc_timer_config_t tc = {};
  tc.speed_mode = kLedcMode;
  tc.duty_resolution = LEDC_TIMER_1_BIT;
  tc.timer_num = kLedcTimer;
  tc.freq_hz = 40000000;
  tc.clk_cfg = LEDC_USE_APB_CLK;
  if (ledc_timer_config(&tc) != ESP_OK) return false;
  ledc_channel_config_t cc = {};
  cc.gpio_num = CONFIG_SOFGNSS_CAL_GPIO;
  cc.speed_mode = kLedcMode;
  cc.channel = kLedcChannel;
  cc.timer_sel = kLedcTimer;
  cc.duty = 1;  // 50 %
  if (ledc_channel_config(&cc) != ESP_OK) return false;
  ledc_stop(kLedcMode, kLedcChannel, 0);
  return true;
}

// Pull chip select low (captured) and start LEDC back to back.
void IRAM_ATTR square40_start() {
  portENTER_CRITICAL(&s_start_mux);
  gpio_ll_set_level(&GPIO, CONFIG_SOFGNSS_CAL_CS_GPIO, 0);
  ledc_ll_set_sig_out_en(&LEDC, kLedcMode, kLedcChannel, true);
  ledc_ll_set_duty_start(&LEDC, kLedcMode, kLedcChannel, true);
  ledc_ll_ls_channel_update(&LEDC, kLedcMode, kLedcChannel);
  portEXIT_CRITICAL(&s_start_mux);
}

}  // namespace

namespace sofgnss::hal {

bool cal_init(CalCallback cb, void* ctx) {
  s_cb = cb;
  s_cb_ctx = ctx;
  s_buf = static_cast<uint8_t*>(heap_caps_calloc(1, kCalBytes, MALLOC_CAP_DMA));
  if (!s_buf) {
    ESP_LOGE(TAG, "no DMA memory for the burst buffer");
    return false;
  }

  spi_bus_config_t bus = {};
  bus.mosi_io_num = CONFIG_SOFGNSS_CAL_GPIO;
  bus.miso_io_num = -1;
  bus.sclk_io_num = -1;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = kCalBytes;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t dev = {};
  dev.mode = 0;
  dev.clock_source = SPI_CLK_SRC_APB;
  dev.clock_speed_hz = kCalChipRate;
  dev.spics_io_num = CONFIG_SOFGNSS_CAL_CS_GPIO;
  dev.queue_size = 1;
  dev.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_dev));

  int khz = 0;
  spi_device_get_actual_freq(s_dev, &khz);
  if (khz * 1000 != static_cast<int>(kCalChipRate)) {
    ESP_LOGE(TAG, "SPI runs at %d kHz, not %lu", khz, (unsigned long)(kCalChipRate / 1000));
    return false;
  }

  s_ledc_ok = ledc_init() && stop_timer_init();
  if (!s_ledc_ok) ESP_LOGE(TAG, "LEDC 40 MHz setup failed; SQ40 unavailable");
  route_spi();  // ledc_channel_config took over the output pad

  // Sharp edges give harmonics up into the GHz range.
  gpio_set_drive_capability(static_cast<gpio_num_t>(CONFIG_SOFGNSS_CAL_GPIO), GPIO_DRIVE_CAP_3);

  // Loop chip select back into capture channel 1.
  gpio_input_enable(static_cast<gpio_num_t>(CONFIG_SOFGNSS_CAL_CS_GPIO));
  esp_rom_gpio_connect_in_signal(
      CONFIG_SOFGNSS_CAL_CS_GPIO,
      mcpwm_periph_signals.groups[0].captures[port::kCalCaptureChannel].cap_sig, false);

  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = on_cal_capture;
  ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(port::cal_capture_channel(), &cbs,
                                                                 nullptr));
  ESP_ERROR_CHECK(mcpwm_capture_channel_enable(port::cal_capture_channel()));

  ESP_LOGI(TAG, "output GPIO%d, burst-start loopback GPIO%d, %lu bits at %d kHz",
           CONFIG_SOFGNSS_CAL_GPIO, CONFIG_SOFGNSS_CAL_CS_GPIO, (unsigned long)kCalBits, khz);
  return true;
}

void cal_set_pattern(const uint8_t* data) {
  if (s_buf) memcpy(s_buf, data, kCalBytes);
}

bool cal_burst_square40(uint32_t duration_us) {
  if (!s_dev || !s_ledc_ok) return false;
  // Hold the SPI bus while the pads are re-routed.
  if (spi_device_acquire_bus(s_dev, portMAX_DELAY) != ESP_OK) return false;
  gpio_ll_set_level(&GPIO, CONFIG_SOFGNSS_CAL_CS_GPIO, 1);
  esp_rom_gpio_connect_out_signal(CONFIG_SOFGNSS_CAL_CS_GPIO, SIG_GPIO_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(CONFIG_SOFGNSS_CAL_GPIO,
                                  ledc_periph_signal[kLedcMode].sig_out0_idx + kLedcChannel,
                                  false, false);
  square40_start();
  wait_until(esp_timer_get_time() + duration_us);
  ledc_stop(kLedcMode, kLedcChannel, 0);
  gpio_ll_set_level(&GPIO, CONFIG_SOFGNSS_CAL_CS_GPIO, 1);
  route_spi();
  spi_device_release_bus(s_dev);
  return true;
}

bool cal_burst() {
  if (!s_dev) return false;
  spi_transaction_t t = {};
  t.length = kCalBits;
  t.tx_buffer = s_buf;
  esp_err_t err = spi_device_transmit(s_dev, &t);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "burst failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

}  // namespace sofgnss::hal
