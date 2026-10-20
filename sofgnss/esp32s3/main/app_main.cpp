// ESP32-S3 task wiring. The actual logic lives in core/.
#include <cstdlib>
#include <cstring>

#include <atomic>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sofgnss/cal_signal.h"
#include "sofgnss/frame_parser.h"
#include "sofgnss/hal.h"
#include "sofgnss/msg_queue.h"
#include "sofgnss/ota_receiver.h"
#include "sofgnss/pps_filter.h"
#include "sofgnss/pvdb.h"
#include "sofgnss/sof_spin_source.h"
#include "sofgnss/timing_reporter.h"

#if CONFIG_SOFGNSS_SOF_DEBUG
#include "port_internal.h"
#endif

// Kconfig leaves "n" booleans undefined; treat them as 0.
#ifndef CONFIG_SOFGNSS_SOF_DITHER
#define CONFIG_SOFGNSS_SOF_DITHER 0
#endif

// Any GPS frame plus its 2-byte header must fit in the outbox.
static_assert(CONFIG_SOFGNSS_PARSER_BYTES + 2 <= CONFIG_SOFGNSS_OUTBOX_BYTES,
              "SOFGNSS_PARSER_BYTES must be below SOFGNSS_OUTBOX_BYTES");

using namespace sofgnss;

namespace {

const char* TAG = "sofgnss";

// ---- Outbox ----------------------------------------------------------------

struct Outbox {
  uint8_t storage[CONFIG_SOFGNSS_OUTBOX_BYTES];
  MsgQueue q{storage, sizeof storage};
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  SemaphoreHandle_t avail = nullptr;

  void init() { avail = xSemaphoreCreateBinary(); }

  void push(const uint8_t* data, size_t len) {
    portENTER_CRITICAL(&mux);
    q.push(data, len);
    portEXIT_CRITICAL(&mux);
    xSemaphoreGive(avail);
  }

  size_t pop(uint8_t* out, size_t cap) {
    portENTER_CRITICAL(&mux);
    size_t n = q.pop(out, cap);
    portEXIT_CRITICAL(&mux);
    return n;
  }

  void clear() {
    portENTER_CRITICAL(&mux);
    q.clear();
    portEXIT_CRITICAL(&mux);
  }

  uint32_t dropped() {
    portENTER_CRITICAL(&mux);
    uint32_t d = q.dropped();
    portEXIT_CRITICAL(&mux);
    return d;
  }
};

Outbox s_outbox;

// ---- Timing events ---------------------------------------------------------

enum class EvKind : uint8_t { Pps, Sof, Cal, SetDivider, InfoRequest, VerRequest, Error };

struct TimingEvent {
  EvKind kind;
  uint32_t tick;   // capture tick, or the error code for Error
  uint64_t frame;  // frame, divider, burst number, or PPS uptime (us)
  char tag[pvdb::kMaxTag + 1];
  CalMode cal_mode;
  bool cal_synced;  // burst was started by a PPS
  uint32_t cal_length;  // ticks, 0 if not measured
};

QueueHandle_t s_timing_q;
std::atomic<uint32_t> s_timing_dropped{0};  // events lost to a full queue
SofSpinSource* s_spin;
OtaReceiver* s_ota;
FrameParser* s_parser;
TaskHandle_t s_h2g_task;
TaskHandle_t s_cal_task;

// The *_now values describe the burst in flight; the CAL ISR reads them.
std::atomic<CalMode> s_cal_request{CalMode::Off};
volatile CalMode s_cal_mode_now = CalMode::Off;
volatile bool s_cal_synced_now = false;
volatile uint32_t s_cal_n_now = 0;

// Restart if the host sends no data for this long, to recover a stuck USB link.
constexpr uint32_t kHostSilenceS = 60;
uint32_t s_host_activity_s;  // uptime in seconds

TimingEvent make_event(EvKind kind, uint32_t tick = 0, uint64_t frame = 0) {
  TimingEvent ev{};
  ev.kind = kind;
  ev.tick = tick;
  ev.frame = frame;
  return ev;
}

void post(const TimingEvent& ev) {
  if (xQueueSend(s_timing_q, &ev, 0) != pdTRUE) s_timing_dropped++;
}
void post(EvKind kind, uint32_t tick = 0, uint64_t frame = 0) { post(make_event(kind, tick, frame)); }

// Drops extra edges from ringing or a slow PPS edge.
PpsFilter s_pps_filter;
uint32_t s_tick_hz;  // cached for the ISR

bool IRAM_ATTR on_pps_isr(uint32_t tick, void*) {
  uint64_t now = hal::uptime_us();
  if (!s_pps_filter.accept(tick, now, s_tick_hz)) return false;
  TimingEvent ev = make_event(EvKind::Pps, tick, now);
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(s_timing_q, &ev, &woken) != pdTRUE) s_timing_dropped++;
  if (s_cal_task) vTaskNotifyGiveFromISR(s_cal_task, &woken);  // start a calibration burst
  return woken == pdTRUE;
}

// At the end of a burst, with its start tick.
bool IRAM_ATTR on_cal_isr(uint32_t tick, uint32_t length, void*) {
  TimingEvent ev = make_event(EvKind::Cal, tick, s_cal_n_now);
  ev.cal_mode = s_cal_mode_now;
  ev.cal_synced = s_cal_synced_now;
  ev.cal_length = length;
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(s_timing_q, &ev, &woken) != pdTRUE) s_timing_dropped++;
  return woken == pdTRUE;
}

// One burst per PPS, or one per second if PPS has been gone for 1.5 s.
// Burst starts are timestamped in hardware, so the delay after PPS is measured.
void cal_task(void*) {
  bool ok = hal::cal_init(on_cal_isr, nullptr);
  if (!ok) ESP_LOGE(TAG, "calibration output unavailable");
  CalMode loaded = CalMode::Off;
  uint64_t last_pps = 0, last_burst = 0;
  uint32_t n = 0;
  hal::watchdog_register_task();
  for (;;) {
    bool pps = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0;
    hal::watchdog_feed();
    uint64_t now = hal::uptime_us();
    if (pps) last_pps = now;
    CalMode want = s_cal_request.load();
    if (!ok || want == CalMode::Off) continue;
    if (want != loaded && want != CalMode::Square40) {
      uint8_t* pattern = static_cast<uint8_t*>(malloc(kCalBytes));
      if (!pattern) continue;
      cal_fill(want, pattern);
      hal::cal_set_pattern(pattern);
      free(pattern);
      loaded = want;
    }
    bool free_run = !pps && now - last_pps > 1500000 && now - last_burst >= 1000000;
    if (!pps && !free_run) continue;
    s_cal_mode_now = want;
    s_cal_synced_now = pps;
    s_cal_n_now = n++;
    last_burst = now;
    if (want == CalMode::Square40) {
      hal::cal_burst_square40(cal_burst_us(hal::cal_chip_rate()));
    } else {
      hal::cal_burst();
    }
  }
}

// Only used on ports with hardware SOF capture.
FrameUnwrapper s_hw_frames;
bool IRAM_ATTR on_sof_isr(uint32_t tick, uint16_t frame, void*) {
  TimingEvent ev = make_event(EvKind::Sof, tick, s_hw_frames.update(frame));
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(s_timing_q, &ev, &woken) != pdTRUE) s_timing_dropped++;
  return woken == pdTRUE;
}

void on_sof_spin(uint32_t tick, uint64_t frame, void*) {
  post(EvKind::Sof, tick, frame);
#if CONFIG_SOFGNSS_SOF_DEBUG
  const port::SofDebug& d = port::g_sof_debug;
  char body[128], out[144];
  snprintf(body, sizeof body, "PVDB,SOFPPS,SOFDBG,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
           (unsigned long)tick, (unsigned long)d.gap, (unsigned long)d.read,
           (unsigned long)d.capture, (unsigned long)d.max_gap, (unsigned long)d.max_read,
           (unsigned long)d.polls, (unsigned long)d.offset);
  size_t n = pvdb::format(out, sizeof out, body);
  s_outbox.push(reinterpret_cast<const uint8_t*>(out), n);
#endif
}

void sof_spin_task(void*) {
  hal::watchdog_register_task();
  for (;;) {
    s_spin->step();  // sleeps at most 0.5 s
    hal::watchdog_feed();
  }
}

// ---- GPS -> host -----------------------------------------------------------

void emit_sentence(const char* s, size_t n, void*) {
  s_outbox.push(reinterpret_cast<const uint8_t*>(s), n);
}

// Uptime (ms) of the last valid GPS frame, 0 if none yet.
std::atomic<uint32_t> s_last_good_frame_ms{0};

void on_frame(FrameType, const uint8_t* data, size_t len, void*) {
  s_last_good_frame_ms.store(static_cast<uint32_t>(hal::uptime_us() / 1000) | 1);
  s_outbox.push(data, len);
}

// Status LED: on while valid GPS frames arrive, flipped for 100 ms at each PPS.
constexpr uint64_t kLedPulseUs = 100000;
constexpr uint32_t kSerialOkMs = 2000;

bool serial_ok(uint64_t now_us) {
  uint32_t last = s_last_good_frame_ms.load();
  return last != 0 && static_cast<uint32_t>(now_us / 1000) - last < kSerialOkMs;
}

// GPS bytes read by gps_rx but not yet in the outbox.
std::atomic<uint32_t> s_rx_held{0};
// Current GPS baud rate, for the frame timeout.
std::atomic<uint32_t> s_gps_baud{CONFIG_SOFGNSS_GPS_DEFAULT_BAUD};
// Uptime (ms, rounded up) when the UART driver last handed us GPS bytes.
std::atomic<uint32_t> s_rx_handover_ms{0};

// How long (ms) the GPS line was quiet at the PPS edge, or 0 if GPS bytes are
// still in flight. Call only from the timing task, so gps_rx can't run meanwhile.
uint32_t gps_rx_idle_ms(uint64_t edge_us) {
  if (hal::gps_uart_rx_pending() + s_rx_held.load() != 0) return 0;
  int32_t idle = static_cast<int32_t>(static_cast<uint32_t>(edge_us / 1000) -
                                      s_rx_handover_ms.load());
  return idle > 0 ? static_cast<uint32_t>(idle) : 0;
}

void gps_rx_task(void*) {
  static uint8_t buf[512];
  uint64_t last_rx = 0;
  uint64_t frame_start = 0;  // 0: no frame in progress
  hal::watchdog_register_task();
  for (;;) {
    hal::watchdog_feed();
    size_t avail = hal::gps_uart_wait(100);
    uint64_t now = hal::uptime_us();
    size_t partial = s_parser->partial_len();
    if (avail) {
      s_rx_handover_ms.store(static_cast<uint32_t>((now + 999) / 1000));
      // Count the bytes before reading them so the timing task never misses them.
      s_rx_held.store(partial + (avail < sizeof buf ? avail : sizeof buf));
      size_t n = hal::gps_uart_read(buf, sizeof buf);
      s_parser->feed(buf, n);
      if (n) last_rx = now;
    } else if (s_parser->in_frame() &&
               now - last_rx > static_cast<uint64_t>(CONFIG_SOFGNSS_FRAME_TIMEOUT_MS) * 1000) {
      s_parser->abort_partial();
    }
    if (!s_parser->in_frame()) {
      frame_start = 0;
    } else if (frame_start == 0) {
      frame_start = now;
    } else if (now - frame_start >
               FrameParser::max_frame_us(s_parser->declared_len(), s_gps_baud.load(),
                                         static_cast<uint64_t>(CONFIG_SOFGNSS_FRAME_SLACK_MS) * 1000)) {
      s_parser->abort_partial();
      frame_start = 0;
    }
    s_rx_held.store(s_parser->partial_len());
  }
}

bool host_listening() {
  if (!hal::usb_ready()) return false;
#if CONFIG_SOFGNSS_REQUIRE_DTR
  if (!hal::cdc_connected()) return false;
#endif
  return true;
}

// Writes each message into the CDC FIFO whole so it never gets cut in half.
// Returns false if the host went away or the port dropped the message's start.
bool send_message(const uint8_t* msg, size_t n) {
  uint32_t clears = hal::cdc_tx_clears();
  size_t off = 0;
  while (off < n) {
    size_t want = n - off;
    size_t limit = hal::cdc_write_capacity();
    if (want > limit) want = limit;
    for (;;) {
      if (!host_listening() || hal::cdc_tx_clears() != clears) return false;
      if (hal::cdc_write_available() >= want) break;
      hal::watchdog_feed();
      vTaskDelay(1);
    }
    off += hal::cdc_write(msg + off, want);
    hal::cdc_write_flush();
  }
  return true;
}

void usb_tx_task(void*) {
  static uint8_t msg[CONFIG_SOFGNSS_PARSER_BYTES];
  hal::watchdog_register_task();
  for (;;) {
    hal::watchdog_feed();
    if (!host_listening()) {
      // Nobody listening: drop old data so the next connection starts fresh.
      s_outbox.clear();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    size_t n = s_outbox.pop(msg, sizeof msg);
    if (n == 0) {
      xSemaphoreTake(s_outbox.avail, pdMS_TO_TICKS(100));
      continue;
    }
    if (n > sizeof msg) n = sizeof msg;
    send_message(msg, n);
  }
}

// ---- host -> GPS -----------------------------------------------------------

void on_line_coding(const hal::LineCoding& lc, void*) {
  hal::gps_uart_configure(lc);
  if (lc.baud >= 300 && lc.baud <= 5000000) s_gps_baud.store(lc.baud);  // same range as gps_uart_configure
}

void on_cdc_rx(void*) {
  if (s_h2g_task) xTaskNotifyGive(s_h2g_task);
}

void on_command(const char* fields, size_t len, void*) {
  // A valid command proves this image works: confirm the OTA update.
  hal::ota_confirm();
  if (len > 4 && memcmp(fields, "DIV,", 4) == 0) {
    char* end;
    unsigned long d = strtoul(fields + 4, &end, 10);
    if (end == fields + len && d >= 1 && d <= 1000000) {
      post(EvKind::SetDivider, 0, d);
    } else {
      post(EvKind::Error, 1, 0);
    }
  } else if (len == 4 && memcmp(fields, "INFO", 4) == 0) {
    post(EvKind::InfoRequest, 0, 0);
  } else if (len > 5 && memcmp(fields, "INFO,", 5) == 0) {
    if (!pvdb::valid_tag(fields + 5, len - 5)) {
      post(EvKind::Error, 3, 0);
      return;
    }
    TimingEvent ev = make_event(EvKind::InfoRequest);
    memcpy(ev.tag, fields + 5, len - 5);
    ev.tag[len - 5] = '\0';
    post(ev);
  } else if (len == 3 && memcmp(fields, "VER", 3) == 0) {
    post(EvKind::VerRequest, 0, 0);
  } else if (len == 4 && memcmp(fields, "PING", 4) == 0) {
    // Nothing to do: any host data resets the host watchdog.
  } else if (len > 4 && memcmp(fields, "CAL,", 4) == 0) {
    const char* arg = fields + 4;
    size_t alen = len - 4;
    if (alen == 5 && memcmp(arg, "NOISE", 5) == 0) {
      s_cal_request.store(CalMode::Noise);
    } else if (alen == 4 && memcmp(arg, "SQ20", 4) == 0) {
      s_cal_request.store(CalMode::Square20);
    } else if (alen == 3 && memcmp(arg, "SQ2", 3) == 0) {
      s_cal_request.store(CalMode::Square2);
    } else if (alen == 4 && memcmp(arg, "SQ40", 4) == 0) {
      s_cal_request.store(CalMode::Square40);
    } else if (alen == 3 && memcmp(arg, "OFF", 3) == 0) {
      s_cal_request.store(CalMode::Off);
    } else {
      post(EvKind::Error, 4, 0);
    }
  } else if (len == 6 && memcmp(fields, "SIMPPS", 6) == 0) {
    hal::pps_inject();  // test hook: fake a PPS edge
  } else if (len > 4 && memcmp(fields, "OTA,", 4) == 0) {
    s_ota->start(fields + 4, len - 4);
  } else {
    post(EvKind::Error, 2, 0);
  }
}

// Only whole, valid frames go to the GPS; our own $PVDB commands stay here.
void on_host_frame(FrameType type, const uint8_t* data, size_t len, void*) {
  const char* fields;
  size_t fields_len;
  if (type == FrameType::Nmea && pvdb::command_fields(data, len, &fields, &fields_len)) {
    on_command(fields, fields_len, nullptr);
    return;
  }
  // Wait for room rather than cut a frame.
  while (len) {
    size_t n = hal::gps_uart_write(data, len);
    data += n;
    len -= n;
    if (len) {
      hal::watchdog_feed();
      vTaskDelay(1);
    }
  }
}

// Largest frame the host can send to the GPS.
constexpr size_t kHostFrameBytes = 2048;
// Yield after this many chunks so a long OTA upload can't starve the idle task.
constexpr int kHostChunksPerPass = 16;

void host_to_gps_task(void*) {
  static uint8_t buf[512];
  static uint8_t frame_buf[kHostFrameBytes];
  FrameParser host_frames(frame_buf, sizeof frame_buf, on_host_frame, nullptr);
  uint64_t last_host_byte = 0;
  bool more = false;  // previous pass left data unread
  hal::watchdog_register_task();
  for (;;) {
    if (more) {
      vTaskDelay(1);
    } else {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    }
    hal::watchdog_feed();
    uint64_t now = hal::uptime_us();
    if (host_frames.in_frame() &&
        now - last_host_byte > static_cast<uint64_t>(CONFIG_SOFGNSS_FRAME_TIMEOUT_MS) * 1000) {
      host_frames.abort_partial();
    }
    // Host went away mid-update: what it sends next isn't the rest of this image.
    if (s_ota->receiving() && !host_listening()) s_ota->cancel("OTALINK");
    s_ota->poll(now);
    if (s_ota->finished()) {
      vTaskDelay(pdMS_TO_TICKS(500));  // let OTA,OK go out first
      hal::restart();
    }
    more = false;
    for (int chunk = 0; hal::cdc_read_available(); ++chunk) {
      if (chunk == kHostChunksPerPass) {
        more = true;
        break;
      }
      hal::watchdog_feed();
      last_host_byte = hal::uptime_us();
      s_host_activity_s = static_cast<uint32_t>(last_host_byte / 1000000);
      size_t n = hal::cdc_read(buf, sizeof buf);
      if (n == 0) break;
      size_t taken = 0;
      if (s_ota->receiving()) {
        // OTA image bytes go to flash, not the GPS.
        taken = s_ota->feed(buf, n, last_host_byte);
      }
      if (taken < n) host_frames.feed(buf + taken, n - taken);
    }
  }
}

// ---- Timing task -----------------------------------------------------------

void send_info(const TimingReporter& reporter, const char* tag = nullptr) {
  pvdb::Info info{};
  info.tag = tag;
  info.tick_hz = hal::tick_hz();
  info.divider = reporter.divider();
  info.pps = reporter.pps_count();
  info.sof_tags = reporter.sof_count();
  info.sof_missed = s_timing_dropped.load();
  if (s_spin) info.sof_missed += s_spin->stats().late + s_spin->stats().timeouts;
  info.dropped = s_outbox.dropped();
  info.parse_errors = s_parser->stats().errors();
  info.pps_rejected = s_pps_filter.rejected();
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_info(s, sizeof s, info);
  emit_sentence(s, n, nullptr);

  hal::UsbStats us = hal::usb_stats();
  pvdb::UsbInfo u{us.mounted, us.suspended, us.dtr, us.line_coding_events, us.line_state_events,
                  us.baud};
  n = pvdb::format_usb(s, sizeof s, u);
  emit_sentence(s, n, nullptr);
}

void send_ver() {
  hal::AppInfo app = hal::app_info();
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_ver(s, sizeof s, app.version, app.idf, app.partition,
                              hal::ota_pending_verify(), app.elf_sha256, hal::reset_reason());
  emit_sentence(s, n, nullptr);
}

void send_error(uint32_t code) {
  const char* text = code == 1   ? "BADDIV"
                     : code == 3 ? "BADTAG"
                     : code == 4 ? "BADCAL"
                                 : "UNKNOWN";
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_err(s, sizeof s, text);
  emit_sentence(s, n, nullptr);
}

void timing_task(void*) {
  TimingReporter reporter(CONFIG_SOFGNSS_SOF_DIVIDER, CONFIG_SOFGNSS_SOF_DITHER, emit_sentence,
                          nullptr);
  uint64_t next_housekeeping = 0;
  uint64_t dcd_off_at = 0;
  uint64_t pulse_until = 0;  // end of the LED's PPS pulse
  bool led = false;
  uint32_t info_countdown = 0;
  bool was_listening = false;
  uint32_t tx_clears = 0;
  hal::watchdog_register_task();

  for (;;) {
    hal::watchdog_feed();
    TimingEvent ev;
    // Poll faster while a DCD or LED pulse needs ending.
    if (xQueueReceive(s_timing_q, &ev, pdMS_TO_TICKS(dcd_off_at || pulse_until ? 5 : 100))) {
      switch (ev.kind) {
        case EvKind::Pps:
          reporter.on_pps(ev.tick, gps_rx_idle_ms(ev.frame));
          pulse_until = hal::uptime_us() + kLedPulseUs;
#if CONFIG_SOFGNSS_PPS_DCD
          // Also pulse DCD for LinuxPPS (ldattach PPS). Only ms accurate.
          hal::cdc_set_dcd(true);
          dcd_off_at = hal::uptime_us() + CONFIG_SOFGNSS_DCD_PULSE_MS * 1000ull;
#endif
          break;
        case EvKind::Sof:
          reporter.on_sof(ev.tick, ev.frame);
          break;
        case EvKind::SetDivider:
          reporter.set_divider(static_cast<uint32_t>(ev.frame));
          if (s_spin) s_spin->set_divider(static_cast<uint32_t>(ev.frame));
          ESP_LOGI(TAG, "divider set to %lu", (unsigned long)ev.frame);
          send_info(reporter);
          break;
        case EvKind::InfoRequest:
          send_info(reporter, ev.tag);
          break;
        case EvKind::VerRequest:
          send_ver();
          break;
        case EvKind::Cal:
          reporter.on_cal(ev.tick, cal_mode_name(ev.cal_mode), ev.cal_synced,
                          static_cast<uint32_t>(ev.frame), hal::cal_chip_rate(), ev.cal_length);
          break;
        case EvKind::Error:
          send_error(ev.tick);
          break;
      }
    }

    uint64_t now = hal::uptime_us();
    if (pulse_until && now >= pulse_until) pulse_until = 0;
    bool want = serial_ok(now) != (pulse_until != 0);
    if (want != led) hal::led_set(led = want);
    if (dcd_off_at && now >= dcd_off_at) {
      hal::cdc_set_dcd(false);
      dcd_off_at = 0;
    }
    if (now >= next_housekeeping) {
      next_housekeeping = now + 1000000;
      // Keep the 64-bit tick unwrap fed. With the spin source, reuse its last
      // reading instead of touching the shared capture channel.
      if (!s_spin) {
        reporter.note_tick(hal::tick_now());
      } else if (s_spin->last_tick() != 0) {
        reporter.note_tick(s_spin->last_tick());
      }

      // Host connected: mounted, and with SOFGNSS_REQUIRE_DTR the port opened.
      // A changed clear count means it was closed and opened again meanwhile.
      bool ready = hal::usb_ready();
      bool listening = host_listening();
      uint32_t clears = hal::cdc_tx_clears();
      if (listening && (!was_listening || clears != tx_clears)) {
        info_countdown = 0;  // send INFO right away
        send_ver();
      }
      was_listening = listening;
      tx_clears = clears;
      if (ready && CONFIG_SOFGNSS_INFO_INTERVAL_S > 0) {
        if (info_countdown == 0) {
          send_info(reporter);
          info_countdown = CONFIG_SOFGNSS_INFO_INTERVAL_S;
        }
        info_countdown--;
      }
#if CONFIG_SOFGNSS_HOST_WDT
      uint32_t now_s = static_cast<uint32_t>(now / 1000000);
      // A suspended bus is supposed to be quiet.
      if (hal::usb_stats().suspended) s_host_activity_s = now_s;
      if (now_s - s_host_activity_s > kHostSilenceS) {
        ESP_LOGW(TAG, "no host activity for %lus, restarting", (unsigned long)kHostSilenceS);
        vTaskDelay(pdMS_TO_TICKS(50));
        hal::restart();
      }
#endif
      if (s_spin) {
        const SofSpinStats& st = s_spin->stats();
        ESP_LOGD(TAG, "pps %lu sof %lu late %lu timeouts %lu replans %lu period %lu dropped %lu",
                 (unsigned long)reporter.pps_count(), (unsigned long)st.tags,
                 (unsigned long)st.late, (unsigned long)st.timeouts, (unsigned long)st.replans,
                 (unsigned long)st.period_ticks, (unsigned long)s_outbox.dropped());
      }
    }
  }
}

}  // namespace

extern "C" void app_main() {
  hal::init();

  // Up to one SOF per ms (divider 1), so 256 ms of timing task stall.
  s_timing_q = xQueueCreate(256, sizeof(TimingEvent));
  s_outbox.init();
  static uint8_t parser_buf[CONFIG_SOFGNSS_PARSER_BYTES];
  s_parser = new FrameParser(parser_buf, sizeof parser_buf, on_frame, nullptr);
  s_ota = new OtaReceiver(emit_sentence, nullptr);

  hal::cdc_set_callbacks(on_line_coding, on_cdc_rx, nullptr);
  hal::usb_start();
  s_tick_hz = hal::tick_hz();
  hal::pps_capture_start(on_pps_isr, nullptr);

  xTaskCreatePinnedToCore(timing_task, "timing", 4096, nullptr, 6, nullptr, 0);
  xTaskCreatePinnedToCore(gps_rx_task, "gps_rx", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(usb_tx_task, "usb_tx", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(host_to_gps_task, "h2g", 6144, nullptr, 3, &s_h2g_task, 0);
#if CONFIG_SOFGNSS_CAL
  // Top priority on core 0 for a steady PPS-to-burst delay.
  xTaskCreatePinnedToCore(cal_task, "cal", 4096, nullptr, 8, &s_cal_task, 0);
#endif

  if (!hal::sof_capture_start(on_sof_isr, nullptr)) {
    // No hardware SOF capture: poll the frame number on core 1 at top priority.
    SofSpinConfig cfg{hal::tick_hz(), CONFIG_SOFGNSS_SOF_DIVIDER};
    cfg.dither = CONFIG_SOFGNSS_SOF_DITHER;
    cfg.lead_us = CONFIG_SOFGNSS_SOF_LEAD_US;
    s_spin = new SofSpinSource(cfg, on_sof_spin, nullptr);
    xTaskCreatePinnedToCore(sof_spin_task, "sof_spin", 4096, nullptr, configMAX_PRIORITIES - 1,
                            nullptr, 1);
  }

  hal::AppInfo app = hal::app_info();
  ESP_LOGI(TAG, "version %s on %s%s, reset reason %s, tick %lu Hz, SOF divider %d", app.version,
           app.partition, hal::ota_pending_verify() ? " (pending verify)" : "",
           hal::reset_reason(), (unsigned long)hal::tick_hz(), CONFIG_SOFGNSS_SOF_DIVIDER);
}
