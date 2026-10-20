// GPS on USART3: RX by circular DMA, TX from a ring buffer by interrupt.
// Also the debug console on LPUART1.
#include <cstdarg>
#include <cstdio>

#include "port.h"
#include "sofgnss/hal.h"

namespace {

using sofgnss::port::kCoreHz;

// 178 ms at 921600 baud. The main loop drains it every millisecond, but may
// block for a while (CAL pattern fill, baud change, console log lines).
constexpr uint32_t kRxRing = 16384;
// Room for one whole host frame (2048) and a USB chunk on top of what's queued.
constexpr uint32_t kTxRing = 3072;
constexpr uint32_t kDmaReqUsart3Rx = 28;
constexpr uint32_t kConsoleBaud = 115200;

uint8_t s_rx[kRxRing];
uint32_t s_rx_rd;
uint8_t s_tx[kTxRing];
volatile uint32_t s_tx_head;  // main loop
volatile uint32_t s_tx_tail;  // interrupt
// Where the DMA was, and when, the last time the line went idle.
volatile uint32_t s_idle_pos;
volatile uint64_t s_idle_us;

uint32_t rx_wr() {
  uint32_t left = DMA1_Channel1->CNDTR;
  return left ? kRxRing - left : 0;
}

uint32_t rx_used() { return (rx_wr() + kRxRing - s_rx_rd) % kRxRing; }
uint32_t tx_used() { return (s_tx_head + kTxRing - s_tx_tail) % kTxRing; }

// The prescaler lets the core clock reach 300 baud.
void set_baud(uint32_t baud) {
  static const uint16_t presc[] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
  uint32_t i = 0;
  while (i + 1 < sizeof presc / sizeof presc[0] && kCoreHz / presc[i] / baud > 0xFFFF) ++i;
  uint32_t clk = kCoreHz / presc[i];
  USART3->PRESC = i;
  USART3->BRR = (clk + baud / 2) / baud;
}

}  // namespace

extern "C" void USART3_IRQHandler() {
  uint32_t isr = USART3->ISR;
  if (isr & USART_ISR_IDLE) {
    USART3->ICR = USART_ICR_IDLECF;
    // Time first: a PPS interrupt in between sees a stale position and says busy.
    s_idle_us = sofgnss::hal::uptime_us();
    s_idle_pos = rx_wr();
  }
  if ((USART3->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
    uint32_t tail = s_tx_tail;
    if (tail == s_tx_head) {
      USART3->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    } else {
      USART3->TDR = s_tx[tail];
      s_tx_tail = (tail + 1) % kTxRing;
    }
  }
}

namespace sofgnss::port {

void uart_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  (void)RCC->APB1ENR2;

  pin_af(kGpsTx);
  pin_af(kGpsRx, Pull::Up);
  DMAMUX1_Channel0->CCR = kDmaReqUsart3Rx;  // DMAMUX channel 0 = DMA1 channel 1
  DMA1_Channel1->CPAR = reinterpret_cast<uint32_t>(&USART3->RDR);
  DMA1_Channel1->CMAR = reinterpret_cast<uint32_t>(s_rx);
  DMA1_Channel1->CNDTR = kRxRing;
  DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;

  set_baud(CONFIG_SOFGNSS_GPS_DEFAULT_BAUD);
  // A lost byte breaks a checksum and the frame gets dropped, so never stall on it.
  USART3->CR3 = USART_CR3_DMAR | USART_CR3_OVRDIS;
  USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;
  NVIC_SetPriority(USART3_IRQn, kPrioUart);
  NVIC_EnableIRQ(USART3_IRQn);

  pin_af(kConsoleTx);
  LPUART1->BRR = static_cast<uint32_t>((uint64_t{kCoreHz} * 256 + kConsoleBaud / 2) / kConsoleBaud);
  LPUART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

uint32_t gps_line_idle_ms(uint64_t now_us) {
  if (USART3->ISR & USART_ISR_BUSY) return 0;
  if (rx_wr() != s_idle_pos) return 0;
  uint64_t idle = s_idle_us;
  return now_us > idle ? static_cast<uint32_t>((now_us - idle) / 1000) : 0;
}

void log(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf - 2, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > static_cast<int>(sizeof buf) - 3) n = sizeof buf - 3;
  buf[n++] = '\r';
  buf[n++] = '\n';
  for (int i = 0; i < n; ++i) {
    while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF)) {
    }
    LPUART1->TDR = static_cast<uint8_t>(buf[i]);
  }
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

// The USART handles 7 to 9 bits per word including parity, so 5 and 6 data bits
// and mark/space parity are not supported.
void gps_uart_configure(const LineCoding& lc) {
  if (lc.baud < 300 || lc.baud > 5000000) {
    port::log("ignoring baud %lu", static_cast<unsigned long>(lc.baud));
    return;
  }
  bool parity = lc.parity == 1 || lc.parity == 2;
  uint32_t data = lc.data_bits == 7 ? 7 : 8;
  uint32_t word = data + (parity ? 1 : 0);
  uint32_t cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
  if (word == 7) cr1 |= USART_CR1_M1;
  if (word == 9) cr1 |= USART_CR1_M0;
  if (parity) cr1 |= USART_CR1_PCE | (lc.parity == 1 ? USART_CR1_PS : 0);
  uint32_t stop = lc.stop_bits == 1 ? 3u : lc.stop_bits == 2 ? 2u : 0u;

  // Let queued bytes go out at the old rate.
  uint64_t deadline = uptime_us() + 100000;
  while ((tx_used() || !(USART3->ISR & USART_ISR_TC)) && uptime_us() < deadline) {
  }
  NVIC_DisableIRQ(USART3_IRQn);
  USART3->CR1 = 0;
  set_baud(lc.baud);
  USART3->CR2 = stop << USART_CR2_STOP_Pos;
  USART3->CR1 = cr1 | (tx_used() ? USART_CR1_TXEIE_TXFNFIE : 0) | USART_CR1_UE;
  NVIC_EnableIRQ(USART3_IRQn);
  port::log("line coding %lu baud, %u data, parity %u, stop %u", static_cast<unsigned long>(lc.baud),
            lc.data_bits, lc.parity, lc.stop_bits);
}

size_t gps_uart_wait(uint32_t timeout_ms) {
  uint64_t deadline = uptime_us() + uint64_t{timeout_ms} * 1000;
  size_t n;
  while ((n = rx_used()) == 0 && uptime_us() < deadline) watchdog_feed();
  return n;
}

size_t gps_uart_read(uint8_t* buf, size_t cap) {
  uint32_t wr = rx_wr();
  size_t n = 0;
  while (s_rx_rd != wr && n < cap) {
    uint32_t end = wr > s_rx_rd ? wr : kRxRing;
    size_t chunk = end - s_rx_rd;
    if (chunk > cap - n) chunk = cap - n;
    for (size_t i = 0; i < chunk; ++i) buf[n + i] = s_rx[s_rx_rd + i];
    n += chunk;
    s_rx_rd = (s_rx_rd + chunk) % kRxRing;
  }
  return n;
}

size_t gps_uart_rx_pending() {
  size_t n = (USART3->ISR & USART_ISR_BUSY) ? 1 : 0;
  return n + rx_used();
}

size_t gps_uart_write_available() { return kTxRing - 1 - tx_used(); }

size_t gps_uart_write(const uint8_t* data, size_t len) {
  size_t room = gps_uart_write_available();
  if (len > room) len = room;
  uint32_t head = s_tx_head;
  for (size_t i = 0; i < len; ++i) {
    s_tx[head] = data[i];
    head = (head + 1) % kTxRing;
  }
  s_tx_head = head;
  if (len) USART3->CR1 |= USART_CR1_TXEIE_TXFNFIE;
  return len;
}

}  // namespace sofgnss::hal
