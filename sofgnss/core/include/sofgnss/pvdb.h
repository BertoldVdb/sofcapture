// $PVDB,SOFPPS sentences and host commands.
// Device -> host
//   $PVDB,SOFPPS,PPS,<tick>,<rx_idle_ms>*hh
//   $PVDB,SOFPPS,SOF,<tick>,<frame>*hh
//   $PVDB,SOFPPS,INFO,<tick_hz>,<divider>,<pps>,<sof_tags>,<sof_missed>,<dropped>,<parse_errors>,<tag>,<pps_rejected>*hh
//   $PVDB,SOFPPS,ERR,<text>*hh
//   $PVDB,SOFPPS,USB,<mounted>,<suspended>,<dtr>,<line_coding_events>,<line_state_events>,<baud>*hh
//   $PVDB,SOFPPS,VER,<version>,<idf>,<partition>,<pending_verify>,<elf_sha256>,<reset_reason>*hh
//   $PVDB,SOFPPS,OTA,<READY|ACK,<bytes>|OK>*hh
//   $PVDB,SOFPPS,CAL,<tick>,<NOISE|SQ2|SQ20|SQ40>,<pps_synced>,<n>,<chip_rate>,<length>*hh
//     burst start tick, and its measured length in ticks (0: not measured)
// Host -> device (checksum required; commands are not forwarded to the GPS)
//   $PVDB,SOFPPS,DIV,<n>*hh      tag one frame in every n (see sof_schedule.h)
//   $PVDB,SOFPPS,INFO[,<tag>]*hh request INFO; the tag is echoed back in it
//   $PVDB,SOFPPS,PING*hh         no reply; keeps the host-silence watchdog quiet
//   $PVDB,SOFPPS,VER*hh          request a VER sentence (also confirms a new image)
//   $PVDB,SOFPPS,SIMPPS*hh       test hook: act as if a PPS edge arrived now
//   $PVDB,SOFPPS,CAL,<NOISE|SQ2|SQ20|SQ40|OFF>*hh   calibration bursts, one per PPS
//   $PVDB,SOFPPS,OTA,<size>,<sha256hex>*hh   then, after READY, the raw image
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgnss::pvdb {

// Includes "\r\n". VER is the longest.
constexpr size_t kMaxSentence = 192;

uint8_t checksum(const char* body, size_t len);  // XOR between '$' and '*'

// Writes "$<body>*hh\r\n". Returns the length, or 0 if it does not fit.
size_t format(char* out, size_t cap, const char* body);
// rx_idle_ms: how long the GPS line was quiet before the edge (see README).
size_t format_pps(char* out, size_t cap, uint64_t tick, uint32_t rx_idle_ms);
size_t format_sof(char* out, size_t cap, uint64_t tick, uint64_t frame);
size_t format_err(char* out, size_t cap, const char* text);

constexpr size_t kMaxTag = 16;

struct Info {
  uint32_t tick_hz;
  uint32_t divider;
  uint32_t pps;
  uint32_t sof_tags;
  uint32_t sof_missed;
  uint32_t dropped;
  uint32_t parse_errors;
  const char* tag;  // nullptr or "" if not requested
  uint32_t pps_rejected;  // last, so older parsers still work
};

// 1..kMaxTag chars of [A-Za-z0-9._-].
bool valid_tag(const char* tag, size_t len);
size_t format_info(char* out, size_t cap, const Info& info);

struct UsbInfo {
  bool mounted;
  bool suspended;
  bool dtr;
  uint32_t line_coding_events;
  uint32_t line_state_events;
  uint32_t baud;
};
size_t format_usb(char* out, size_t cap, const UsbInfo& u);
size_t format_cal(char* out, size_t cap, uint64_t tick, const char* mode, bool pps_synced,
                  uint32_t n, uint32_t chip_rate, uint32_t length);
size_t format_ver(char* out, size_t cap, const char* version, const char* idf,
                  const char* partition, bool pending_verify, const char* elf_sha256,
                  const char* reset_reason);
size_t format_ota(char* out, size_t cap, const char* status);
size_t format_ota(char* out, size_t cap, const char* status, uint32_t value);

// Checks "$...*hh"; trailing CR/LF is allowed.
bool verify(const char* sentence, size_t len);

// For a $PVDB,SOFPPS command, sets *fields to the text between "SOFPPS," and
// '*'. Expects a checked sentence ending in "\r\n".
bool command_fields(const uint8_t* frame, size_t len, const char** fields, size_t* fields_len);

}  // namespace sofgnss::pvdb
