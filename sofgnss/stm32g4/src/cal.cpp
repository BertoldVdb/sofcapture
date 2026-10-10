// Calibration bursts. SPI1 sends the pattern from CCM SRAM by DMA at PCLK/2
// (72 Mbit/s) on PA7. Its hardware NSS (PA4) falls as the SPI is enabled with
// the FIFO already filled, i.e. as the burst starts. A wire from PA4 to PA9
// (TIM2 CH3) captures that edge on the timestamp counter.
#include <cstring>

#include "port.h"
#include "sofgnss/cal_signal.h"
#include "sofgnss/hal.h"

namespace {

constexpr uint32_t kDmaReqSpi1Tx = 11;

// DMA reaches CCM SRAM through its alias at the end of SRAM2.
uint8_t s_buf[sofgnss::kCalBytes] __attribute__((section(".ccmram"), aligned(4)));
volatile bool s_busy;
bool s_ok;
volatile uint32_t s_start_tick;
volatile bool s_captured;  // s_start_tick is the captured marker edge
volatile uint32_t s_last_ticks;
sofgnss::hal::CalCallback s_cb;
void* s_cb_ctx;

}  // namespace

// The last bytes are still in the FIFO when the DMA finishes. Wait for the
// SPI to drain, then raise NSS. The end tick shows any gap in the stream,
// but only if the SPI was still busy here: a late interrupt can't tell.
extern "C" void DMA1_Channel2_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CGIF2;
  bool waited = false;
  __disable_irq();
  while (SPI1->SR & (SPI_SR_FTLVL | SPI_SR_BSY)) waited = true;
  uint32_t end = TIM2->CNT;
  __enable_irq();
  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  s_last_ticks = waited ? end - s_start_tick : 0;
  if (s_captured && s_cb) s_cb(s_start_tick, s_last_ticks, s_cb_ctx);
  s_busy = false;
  NVIC_EnableIRQ(USB_LP_IRQn);
  NVIC_EnableIRQ(USB_HP_IRQn);
}

namespace sofgnss::port {

uint8_t* cal_buffer() { return s_buf; }
bool cal_busy() { return s_busy; }
void cal_started(uint32_t tick) {
  s_start_tick = tick;
  s_captured = true;
}
uint32_t cal_last_ticks() { return s_last_ticks; }

// USB register and packet memory accesses are slow (48 MHz side) and share
// the peripheral bus with the SPI's DMA. During a burst they stalled the
// stream by up to 30 chips, so the USB interrupt waits. The host is NAKed.
bool cal_start() {
  if (!s_ok || s_busy) return false;
  s_busy = true;
  s_captured = false;
  NVIC_DisableIRQ(USB_LP_IRQn);
  NVIC_DisableIRQ(USB_HP_IRQn);
  DMA1_Channel2->CNDTR = kCalBytes / 2;
  DMA1_Channel2->CCR |= DMA_CCR_EN;
  SPI1->CR2 |= SPI_CR2_TXDMAEN;  // fills the FIFO
  s_start_tick = TIM2->CNT;      // replaced by the captured NSS edge if wired
  SPI1->CR1 |= SPI_CR1_SPE;      // NSS falls, the clock starts
  return true;
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

bool cal_init(CalCallback cb, void* ctx) {
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  (void)RCC->AHB1ENR;
  port::pin_af(port::kCalOut);
  port::pin_af(port::kCalMarker);
  port::pin_af(port::kCalCapture, port::Pull::Up);

  // Transmit-only master, 8-bit frames packed two per 16-bit DMA write, low byte first.
  SPI1->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_MSTR;  // BR = 0: PCLK/2
  SPI1->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_SSOE;

  DMAMUX1_Channel1->CCR = kDmaReqSpi1Tx;  // DMAMUX channel 1 = DMA1 channel 2
  DMA1_Channel2->CPAR = reinterpret_cast<uint32_t>(&SPI1->DR);
  DMA1_Channel2->CMAR = reinterpret_cast<uint32_t>(s_buf);
  DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
                       DMA_CCR_PL | DMA_CCR_TCIE;
  NVIC_SetPriority(DMA1_Channel2_IRQn, port::kPrioUart);
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  s_cb = cb;
  s_cb_ctx = ctx;
  port::cal_capture_start();
  s_ok = true;
  return true;
}

uint32_t cal_chip_rate() { return port::kCoreHz / 2; }

void cal_set_pattern(const uint8_t* data) {
  if (!s_busy) memcpy(s_buf, data, kCalBytes);
}

bool cal_burst() {
  __disable_irq();
  bool ok = port::cal_start();
  __enable_irq();
  while (ok && s_busy) {
  }
  return ok;
}

// SPI can't do twice the chip rate.
bool cal_burst_square40(uint32_t) { return false; }

}  // namespace sofgnss::hal
