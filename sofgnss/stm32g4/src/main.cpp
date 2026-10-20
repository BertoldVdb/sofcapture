// STM32G474 main loop. Same behaviour as main/app_main.cpp on the ESP32-S3,
// but in one loop: interrupts only capture timestamps and move bytes.
#include <cstdlib>
#include <cstring>

#include "port.h"
#include "sofgnss/cal_signal.h"
#include "sofgnss/frame_parser.h"
#include "sofgnss/hal.h"
#include "sofgnss/msg_queue.h"
#include "sofgnss/ota_receiver.h"
#include "sofgnss/pps_filter.h"
#include "sofgnss/pvdb.h"
#include "sofgnss/timing_reporter.h"
#include "sofgnss/unwrap.h"
#include "tusb.h"

// Any GPS frame plus its 2-byte header must fit in the outbox.
static_assert(CONFIG_SOFGNSS_PARSER_BYTES + 2 <= CONFIG_SOFGNSS_OUTBOX_BYTES,
              "SOFGNSS_PARSER_BYTES must be below SOFGNSS_OUTBOX_BYTES");

using namespace sofgnss;

namespace {

// ---- Timing events: TIM2 interrupt -> main loop ----------------------------

enum class EvKind : uint8_t { Pps, Sof, Cal };

struct TimingEvent {
  EvKind kind;
  CalMode cal_mode;
  bool cal_synced;  // burst was started by a PPS
  uint32_t tick;
  uint32_t aux;     // PPS: rx_idle_ms, CAL: burst number
  uint64_t frame;   // SOF: frame, CAL: burst length in ticks
};

// Interrupts push (TIM2 and the burst-end DMA), the main loop pops.
class EventRing {
 public:
  bool push(const TimingEvent& ev) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t head = head_;
    bool room = head - tail_ < kSize;
    if (room) {
      buf_[head % kSize] = ev;
      __DMB();
      head_ = head + 1;
    } else {
      dropped_ = dropped_ + 1;
    }
    __set_PRIMASK(primask);
    return room;
  }
  bool pop(TimingEvent* ev) {
    uint32_t tail = tail_;
    if (tail == head_) return false;
    __DMB();
    *ev = buf_[tail % kSize];
    tail_ = tail + 1;
    return true;
  }
  uint32_t dropped() const { return dropped_; }

 private:
  // Up to one SOF per ms (divider 1), so 256 ms of main loop stall.
  static constexpr uint32_t kSize = 256;
  TimingEvent buf_[kSize];
  volatile uint32_t head_ = 0;
  volatile uint32_t tail_ = 0;
  volatile uint32_t dropped_ = 0;
};

EventRing s_events;
PpsFilter s_pps_filter;
FrameUnwrapper s_frames;

// ---- Calibration bursts ----------------------------------------------------
// One burst per PPS, started from its interrupt, or one per second if PPS has
// been gone for 1.5 s.

volatile CalMode s_cal_armed = CalMode::Off;  // Off while the pattern changes
// The *_now values describe the burst in flight. The CAL capture reads them.
volatile CalMode s_cal_mode_now = CalMode::Off;
volatile bool s_cal_synced_now;
volatile uint32_t s_cal_n_now;
volatile uint32_t s_cal_n;
volatile uint32_t s_last_pps_ms;
volatile uint32_t s_last_burst_ms;

// TIM2 interrupt, or interrupts off.
void cal_start(bool synced, uint32_t now_ms) {
  CalMode mode = s_cal_armed;
  if (mode == CalMode::Off || port::cal_busy()) return;
  s_cal_mode_now = mode;
  s_cal_synced_now = synced;
  s_cal_n_now = s_cal_n++;
  if (port::cal_start()) s_last_burst_ms = now_ms;
}

bool s_cal_ok;
CalMode s_cal_want = CalMode::Off;    // asked for by the host
CalMode s_cal_loaded = CalMode::Off;  // in the burst buffer
uint32_t s_cal_burst_seen;            // s_cal_n of the last burst checked
// Start latency and drain time. One chip is 2 ticks.
constexpr uint32_t kCalSlackTicks = 64;

// At the end of a burst, with its start tick.
bool on_cal_isr(uint32_t tick, uint32_t length, void*) {
  TimingEvent ev{EvKind::Cal, s_cal_mode_now, s_cal_synced_now, tick, s_cal_n_now, length};
  s_events.push(ev);
  return false;
}

// USB work that is already running would stall a burst's SPI stream (see
// cal_start). A PPS during it leaves the burst pending until it's done.
volatile bool s_main_usb;
volatile bool s_cal_pending;

bool on_pps_isr(uint32_t tick, void*) {
  uint64_t now = hal::uptime_us();
  if (!s_pps_filter.accept(tick, now, port::kCoreHz)) return false;
  s_events.push({EvKind::Pps, CalMode::Off, false, tick, port::gps_line_idle_ms(now), 0});
  s_last_pps_ms = static_cast<uint32_t>(now / 1000);
  if (s_main_usb || port::usb_isr_active()) {
    s_cal_pending = true;
  } else {
    cal_start(true, s_last_pps_ms);
  }
  return false;
}

// Main loop only. No interrupt is running here, so no USB handler either.
void cal_start_pending() {
  __disable_irq();
  if (s_cal_pending) {
    s_cal_pending = false;
    cal_start(true, static_cast<uint32_t>(hal::uptime_us() / 1000));
  }
  __enable_irq();
}

// Runs USB work unless a burst is running.
template <typename F>
void usb_work(F work) {
  s_main_usb = true;  // before the check, so a PPS from here on waits
  if (!port::cal_busy()) work();
  s_main_usb = false;
  cal_start_pending();
}

// Only the frames the reporter tags are queued. Set from the main loop.
volatile uint32_t s_sof_divider = CONFIG_SOFGNSS_SOF_DIVIDER;

bool on_sof_isr(uint32_t tick, uint16_t frame, void*) {
  uint64_t f = s_frames.update(frame);
  if (SofSchedule{s_sof_divider, CONFIG_SOFGNSS_SOF_DITHER}.is_target(f)) {
    s_events.push({EvKind::Sof, CalMode::Off, false, tick, 0, f});
  }
  return false;
}

// ---- Outbox and parsers ----------------------------------------------------

uint8_t s_outbox_storage[CONFIG_SOFGNSS_OUTBOX_BYTES];
MsgQueue s_outbox{s_outbox_storage, sizeof s_outbox_storage};

void emit_sentence(const char* s, size_t n, void*) {
  s_outbox.push(reinterpret_cast<const uint8_t*>(s), n);
}

uint64_t s_last_good_frame_us;  // 0 if none yet

void on_frame(FrameType, const uint8_t* data, size_t len, void*) {
  s_last_good_frame_us = hal::uptime_us();
  s_outbox.push(data, len);
}

uint8_t s_parser_buf[CONFIG_SOFGNSS_PARSER_BYTES];
FrameParser s_parser{s_parser_buf, sizeof s_parser_buf, on_frame, nullptr};
OtaReceiver s_ota{emit_sentence, nullptr};
TimingReporter s_reporter{CONFIG_SOFGNSS_SOF_DIVIDER, CONFIG_SOFGNSS_SOF_DITHER, emit_sentence,
                          nullptr};

uint32_t s_gps_baud = CONFIG_SOFGNSS_GPS_DEFAULT_BAUD;  // for the frame timeout
uint32_t s_host_activity_s;  // uptime in seconds

bool host_listening() {
  if (!hal::usb_ready()) return false;
#if CONFIG_SOFGNSS_REQUIRE_DTR
  if (!hal::cdc_connected()) return false;
#endif
  return true;
}

// ---- GPS -> host -----------------------------------------------------------

uint64_t s_last_rx_us;
uint64_t s_frame_start_us;  // 0: no frame in progress

void gps_rx_poll(uint64_t now) {
  uint8_t buf[256];
  size_t n;
  while ((n = hal::gps_uart_read(buf, sizeof buf)) > 0) {
    s_parser.feed(buf, n);
    s_last_rx_us = now;
  }
  if (s_parser.in_frame() &&
      now - s_last_rx_us > uint64_t{CONFIG_SOFGNSS_FRAME_TIMEOUT_MS} * 1000) {
    s_parser.abort_partial();
  }
  if (!s_parser.in_frame()) {
    s_frame_start_us = 0;
  } else if (s_frame_start_us == 0) {
    s_frame_start_us = now;
  } else if (now - s_frame_start_us >
             FrameParser::max_frame_us(s_parser.declared_len(), s_gps_baud,
                                       uint64_t{CONFIG_SOFGNSS_FRAME_SLACK_MS} * 1000)) {
    s_parser.abort_partial();
    s_frame_start_us = 0;
  }
}

// Each message goes into the CDC FIFO whole (in pieces of at most the FIFO
// size) so it never gets cut in half.
uint8_t s_msg[CONFIG_SOFGNSS_PARSER_BYTES];
size_t s_msg_len, s_msg_off;
uint32_t s_msg_clears;

void usb_tx_poll() {
  if (!host_listening()) {
    // Nobody listening: drop old data so the next connection starts fresh.
    s_outbox.clear();
    s_msg_len = s_msg_off = 0;
    return;
  }
  // The port dropped the start of this message (closed and opened again).
  if (hal::cdc_tx_clears() != s_msg_clears) {
    s_msg_clears = hal::cdc_tx_clears();
    s_msg_len = s_msg_off = 0;
  }
  for (;;) {
    if (s_msg_off == s_msg_len) {
      size_t n = s_outbox.pop(s_msg, sizeof s_msg);
      if (n == 0) return;
      s_msg_len = n < sizeof s_msg ? n : sizeof s_msg;
      s_msg_off = 0;
    }
    size_t want = s_msg_len - s_msg_off;
    if (want > hal::cdc_write_capacity()) want = hal::cdc_write_capacity();
    if (hal::cdc_write_available() < want) return;
    s_msg_off += hal::cdc_write(s_msg + s_msg_off, want);
    hal::cdc_write_flush();
  }
}

// ---- host -> GPS -----------------------------------------------------------

void send_info(const char* tag = nullptr) {
  pvdb::Info info{};
  info.tag = tag;
  info.tick_hz = hal::tick_hz();
  info.divider = s_reporter.divider();
  info.pps = s_reporter.pps_count();
  info.sof_tags = s_reporter.sof_count();
  info.sof_missed = port::sof_dropped() + s_events.dropped();
  info.dropped = s_outbox.dropped();
  info.parse_errors = s_parser.stats().errors();
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

void send_error(const char* text) {
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_err(s, sizeof s, text);
  emit_sentence(s, n, nullptr);
}

void on_line_coding(const hal::LineCoding& lc, void*) {
  hal::gps_uart_configure(lc);
  if (lc.baud >= 300 && lc.baud <= 5000000) s_gps_baud = lc.baud;  // same range as gps_uart_configure
}

void on_command(const char* fields, size_t len) {
  if (len > 4 && memcmp(fields, "DIV,", 4) == 0) {
    char* end;
    unsigned long d = strtoul(fields + 4, &end, 10);
    if (end == fields + len && d >= 1 && d <= 1000000) {
      s_reporter.set_divider(static_cast<uint32_t>(d));
      s_sof_divider = s_reporter.divider();
      send_info();
    } else {
      send_error("BADDIV");
    }
  } else if (len == 4 && memcmp(fields, "INFO", 4) == 0) {
    send_info();
  } else if (len > 5 && memcmp(fields, "INFO,", 5) == 0) {
    char tag[pvdb::kMaxTag + 1];
    if (!pvdb::valid_tag(fields + 5, len - 5)) {
      send_error("BADTAG");
      return;
    }
    memcpy(tag, fields + 5, len - 5);
    tag[len - 5] = '\0';
    send_info(tag);
  } else if (len == 3 && memcmp(fields, "VER", 3) == 0) {
    send_ver();
  } else if (len == 4 && memcmp(fields, "PING", 4) == 0) {
    // Nothing to do: any host data resets the host watchdog.
  } else if (len > 4 && memcmp(fields, "CAL,", 4) == 0) {
    const char* arg = fields + 4;
    size_t alen = len - 4;
    if (alen == 5 && memcmp(arg, "NOISE", 5) == 0) {
      s_cal_want = CalMode::Noise;
    } else if (alen == 4 && memcmp(arg, "SQ20", 4) == 0) {
      s_cal_want = CalMode::Square20;
    } else if (alen == 3 && memcmp(arg, "SQ2", 3) == 0) {
      s_cal_want = CalMode::Square2;
    } else if (alen == 3 && memcmp(arg, "OFF", 3) == 0) {
      s_cal_want = CalMode::Off;
    } else if (alen == 4 && memcmp(arg, "SQ40", 4) == 0) {
      send_error("NOCAL");  // needs twice the chip rate
    } else {
      send_error("BADCAL");
    }
    if (!s_cal_ok && s_cal_want != CalMode::Off) {
      s_cal_want = CalMode::Off;
      send_error("NOCAL");
    }
  } else if (len == 6 && memcmp(fields, "SIMPPS", 6) == 0) {
    hal::pps_inject();  // test hook: fake a PPS edge
  } else if (len > 4 && memcmp(fields, "OTA,", 4) == 0) {
    s_ota.start(fields + 4, len - 4);  // answers ERR,OTABEGIN: flash over SWD
  } else {
    send_error("UNKNOWN");
  }
}

// Only whole, valid frames go to the GPS. Our own $PVDB commands stay here.
// host_poll() makes sure the frame fits.
void on_host_frame(FrameType type, const uint8_t* data, size_t len, void*) {
  const char* fields;
  size_t fields_len;
  if (type == FrameType::Nmea && pvdb::command_fields(data, len, &fields, &fields_len)) {
    on_command(fields, fields_len);
    return;
  }
  hal::gps_uart_write(data, len);
}

// Largest frame the host can send to the GPS.
constexpr size_t kHostFrameBytes = 2048;
constexpr size_t kHostChunk = 64;
uint8_t s_host_frame_buf[kHostFrameBytes];
FrameParser s_host_frames{s_host_frame_buf, sizeof s_host_frame_buf, on_host_frame, nullptr};
uint64_t s_last_host_byte_us;

void host_poll(uint64_t now) {
  if (s_host_frames.in_frame() &&
      now - s_last_host_byte_us > uint64_t{CONFIG_SOFGNSS_FRAME_TIMEOUT_MS} * 1000) {
    s_host_frames.abort_partial();
  }
  if (s_ota.receiving() && !host_listening()) s_ota.cancel("OTALINK");
  s_ota.poll(now);
  // A chunk completes at most the partial frame plus itself: read only when
  // that fits the GPS TX ring, and let USB hold the rest meanwhile.
  while (hal::cdc_read_available() &&
         hal::gps_uart_write_available() >= kHostFrameBytes + kHostChunk) {
    uint8_t buf[kHostChunk];
    size_t n = hal::cdc_read(buf, sizeof buf);
    if (n == 0) break;
    s_last_host_byte_us = now;
    s_host_activity_s = static_cast<uint32_t>(now / 1000000);
    size_t taken = s_ota.receiving() ? s_ota.feed(buf, n, now) : 0;
    if (taken < n) s_host_frames.feed(buf + taken, n - taken);
  }
}

// ---- Timing and housekeeping -----------------------------------------------

// Status LED: on while valid GPS frames arrive, flipped for 100 ms at each PPS.
constexpr uint64_t kLedPulseUs = 100000;
constexpr uint64_t kSerialOkUs = 2000000;
constexpr uint32_t kHostSilenceS = 60;

uint64_t s_pulse_until;  // end of the LED's PPS pulse
uint64_t s_dcd_off_at;
bool s_dcd;  // wanted level, sent once no burst runs
bool s_led;

void timing_poll() {
  TimingEvent ev;
  while (s_events.pop(&ev)) {
    if (ev.kind == EvKind::Sof) {
      s_reporter.on_sof(ev.tick, ev.frame);
      continue;
    }
    if (ev.kind == EvKind::Cal) {
      s_reporter.on_cal(ev.tick, cal_mode_name(ev.cal_mode), ev.cal_synced, ev.aux,
                        hal::cal_chip_rate(), static_cast<uint32_t>(ev.frame));
      continue;
    }
    uint64_t now = hal::uptime_us();
    // rx_idle_ms promises the GPS bytes from before the edge are in the
    // outbox ahead of this sentence, so take them in first. A partial frame
    // may hold some of them.
    gps_rx_poll(now);
    uint32_t idle = s_parser.partial_len() ? 0 : ev.aux;
    s_reporter.on_pps(ev.tick, idle);
    s_pulse_until = now + kLedPulseUs;
#if CONFIG_SOFGNSS_PPS_DCD
    // Also pulse DCD for LinuxPPS. Only ms accurate.
    s_dcd = true;
    s_dcd_off_at = now + CONFIG_SOFGNSS_DCD_PULSE_MS * 1000ull;
#endif
  }
}

// Loads the pattern the host asked for and starts bursts while PPS is missing.
void cal_poll(uint64_t now) {
  if (s_cal_want != s_cal_loaded) {
    s_cal_armed = CalMode::Off;
    if (port::cal_busy()) return;
    cal_fill(s_cal_want, port::cal_buffer());  // about 25 ms for NOISE
    s_cal_loaded = s_cal_want;
    s_cal_burst_seen = s_cal_n;
  }
  s_cal_armed = s_cal_loaded;
  cal_start_pending();  // a PPS that came during a USB interrupt
  uint32_t now_ms = static_cast<uint32_t>(now / 1000);
  if (s_cal_loaded != CalMode::Off && now_ms - s_last_pps_ms > 1500 &&
      now_ms - s_last_burst_ms >= 1000) {
    __disable_irq();
    cal_start(false, now_ms);
    __enable_irq();
  }
  // A gap in the SPI stream would stretch the burst.
  if (s_cal_burst_seen != s_cal_n && !port::cal_busy()) {
    s_cal_burst_seen = s_cal_n;
    uint32_t want = 2 * static_cast<uint32_t>(kCalBits);
    uint32_t got = port::cal_last_ticks();
    if (got && got - want > kCalSlackTicks) {
      port::log("CAL burst took %lu ticks, %lu expected", static_cast<unsigned long>(got),
                static_cast<unsigned long>(want));
    }
  }
}

uint64_t s_next_housekeeping;
uint32_t s_info_countdown;
bool s_was_listening;
uint32_t s_tx_clears;

void housekeeping(uint64_t now) {
  if (s_pulse_until && now >= s_pulse_until) s_pulse_until = 0;
  bool serial_ok = s_last_good_frame_us && now - s_last_good_frame_us < kSerialOkUs;
  bool want = serial_ok != (s_pulse_until != 0);
  if (want != s_led) hal::led_set(s_led = want);
  if (s_dcd_off_at && now >= s_dcd_off_at) {
    s_dcd = false;
    s_dcd_off_at = 0;
  }
  if (now < s_next_housekeeping) return;
  s_next_housekeeping = now + 1000000;
  s_reporter.note_tick(hal::tick_now());

  // Host connected: mounted, and with SOFGNSS_REQUIRE_DTR the port opened.
  // A changed clear count means it was closed and opened again meanwhile.
  bool ready = hal::usb_ready();
  bool listening = host_listening();
  uint32_t clears = hal::cdc_tx_clears();
  if (listening && (!s_was_listening || clears != s_tx_clears)) {
    s_info_countdown = 0;  // send INFO right away
    send_ver();
  }
  s_was_listening = listening;
  s_tx_clears = clears;
  if (ready && CONFIG_SOFGNSS_INFO_INTERVAL_S > 0) {
    if (s_info_countdown == 0) {
      send_info();
      s_info_countdown = CONFIG_SOFGNSS_INFO_INTERVAL_S;
    }
    s_info_countdown--;
  }
#if CONFIG_SOFGNSS_HOST_WDT
  uint32_t now_s = static_cast<uint32_t>(now / 1000000);
  // A suspended bus is supposed to be quiet.
  if (hal::usb_stats().suspended) s_host_activity_s = now_s;
  if (now_s - s_host_activity_s > kHostSilenceS) {
    port::log("no host activity for %lus, restarting", static_cast<unsigned long>(kHostSilenceS));
    hal::restart();
  }
#endif
}

}  // namespace

int main() {
  hal::init();
  hal::AppInfo app = hal::app_info();
  port::log("sofgnss %s on %s, reset reason %s, tick %lu Hz", app.version, app.idf,
            hal::reset_reason(), static_cast<unsigned long>(hal::tick_hz()));
  port::log("HSE measured %lu Hz as crystal, %lu Hz as external clock",
            static_cast<unsigned long>(port::hse_measured(false)),
            static_cast<unsigned long>(port::hse_measured(true)));
  if (port::hse_hz()) {
    port::log("clock from HSE %lu MHz %s", static_cast<unsigned long>(port::hse_hz() / 1000000),
              port::hse_bypass() ? "external clock" : "crystal");
  } else {
    port::log("no HSE at 8, 12 or 24 MHz: clock from HSI16 (about 1 %% off), USB from HSI48, "
              "SOF captures unreliable");
  }

  s_cal_ok = hal::cal_init(on_cal_isr, nullptr);
  hal::cdc_set_callbacks(on_line_coding, nullptr, nullptr);
  hal::usb_start();
  hal::pps_capture_start(on_pps_isr, nullptr);
  hal::sof_capture_start(on_sof_isr, nullptr);

  for (;;) {
    hal::watchdog_feed();
    // USB work waits out a burst (3.6 ms).
    usb_work([] { tud_task(); });
    uint64_t now = hal::uptime_us();
    gps_rx_poll(now);
    usb_work([now] { host_poll(now); });
    timing_poll();
    cal_poll(now);
    housekeeping(now);
    usb_work([] {
      hal::cdc_set_dcd(s_dcd);
      usb_tx_poll();
    });
    // Woken by any interrupt, at least by SysTick every millisecond.
    __WFI();
  }
}
