// Simulated HAL for host tests: a fake tick clock and a synthetic SOF stream.
#pragma once

#include <cstdint>

namespace sim {

struct State {
  uint32_t tick_hz = 80000000;
  uint64_t now = 1000000;       // fake time, ticks
  double period = 80000.0;      // ticks per USB frame
  uint64_t origin = 100;        // tick of SOF index 0
  uint64_t frame_base = 2000;   // frame number of SOF index 0
  bool usb_ready = true;
  bool sof_running = true;
  uint64_t sleep_jitter = 0;    // added to every sleep_us
  uint64_t detect_latency = 8;  // ticks from SOF to register change

  // OTA stubs
  bool ota_begin_ok = true;
  bool ota_end_ok = true;
  bool ota_active = false;
  uint64_t ota_bytes = 0;
  int ota_aborts = 0;
  int ota_confirms = 0;
  bool ota_pending = false;
  int restarts = 0;

  // Instrumentation
  uint64_t spin_ticks = 0;  // time spent spinning
  uint64_t sleep_calls = 0;
  uint64_t last_detected_index = 0;
};

State& state();
void reset();

uint64_t sof_time(uint64_t index);        // tick of SOF `index`
uint64_t frame_index_at(uint64_t tick);   // latest SOF at or before tick

}  // namespace sim
