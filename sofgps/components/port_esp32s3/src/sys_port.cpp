#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "port_internal.h"
#include "sdkconfig.h"
#include "sofgps/hal.h"

namespace {

esp_timer_handle_t s_sleep_timer;
TaskHandle_t s_sleeper;

void IRAM_ATTR on_sleep_timer(void*) {
  BaseType_t woken = pdFALSE;
  if (s_sleeper) vTaskNotifyGiveFromISR(s_sleeper, &woken);
  if (woken) esp_timer_isr_dispatch_need_yield();
}

}  // namespace

namespace sofgps::port {

void sys_init() {
  esp_timer_create_args_t args = {};
  args.callback = on_sleep_timer;
  args.dispatch_method = ESP_TIMER_ISR;
  args.name = "sleep_us";
  ESP_ERROR_CHECK(esp_timer_create(&args, &s_sleep_timer));

#if CONFIG_SOFGPS_LED_GPIO >= 0
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << CONFIG_SOFGPS_LED_GPIO;
  io.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&io));
  hal::led_set(false);
#endif
}

}  // namespace sofgps::port

namespace sofgps::hal {

void init() {
  port::sys_init();
  port::timer_init();
  port::uart_init();
}

// Short waits busy-loop; longer ones sleep on an ISR-dispatched timer, which
// wakes us within a few microseconds.
void sleep_us(uint32_t us) {
  if (us < 50) {
    esp_rom_delay_us(us);
    return;
  }
  s_sleeper = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdTRUE, 0);  // discard a stale notification
  ESP_ERROR_CHECK(esp_timer_start_once(s_sleep_timer, us));
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(us / 1000 + 10));
}

uint64_t IRAM_ATTR uptime_us() { return static_cast<uint64_t>(esp_timer_get_time()); }

const char* reset_reason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    default: return "UNKNOWN";
  }
}

// ESP-IDF starts the task watchdog at boot; it panics, which reboots.
void watchdog_register_task() { ESP_ERROR_CHECK(esp_task_wdt_add(nullptr)); }
void watchdog_feed() { esp_task_wdt_reset(); }

void led_set(bool on) {
#if CONFIG_SOFGPS_LED_GPIO >= 0
  gpio_set_level(static_cast<gpio_num_t>(CONFIG_SOFGPS_LED_GPIO),
                 on != CONFIG_SOFGPS_LED_ACTIVE_LOW);
#else
  (void)on;
#endif
}

}  // namespace sofgps::hal
