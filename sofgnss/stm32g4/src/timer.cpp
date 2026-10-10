// TIM2 is the 32-bit timestamp counter at the core clock (wraps every 30 s).
// CH1 captures USB SOF from internal trigger 11 (only TIM2 has it), CH2 the
// PPS pin. One interrupt serves both and counts the wraps.
#include "port.h"
#include "sofgnss/hal.h"

namespace {

constexpr uint32_t kSmcrTsItr11 = TIM_SMCR_TS_0 | TIM_SMCR_TS_1 | TIM_SMCR_TS_2 | TIM_SMCR_TS_3;

volatile uint32_t s_wraps;
volatile uint32_t s_sof_dropped;
sofgnss::hal::PpsCallback s_pps_cb;
void* s_pps_ctx;
sofgnss::hal::SofCallback s_sof_cb;
void* s_sof_ctx;

}  // namespace

extern "C" void TIM2_IRQHandler() {
  uint32_t sr = TIM2->SR;
  if (sr & TIM_SR_UIF) {
    TIM2->SR = ~TIM_SR_UIF;
    s_wraps = s_wraps + 1;
  }
  if (sr & TIM_SR_CC1IF) {
    uint32_t tick = TIM2->CCR1;  // clears CC1IF
    uint16_t frame = USB->FNR & USB_FNR_FN;
    if (sr & TIM_SR_CC1OF) {
      TIM2->SR = ~TIM_SR_CC1OF;
      s_sof_dropped = s_sof_dropped + 1;
    }
    // Another SOF since the capture: the frame number may belong to it.
    if (TIM2->SR & TIM_SR_CC1IF) {
      s_sof_dropped = s_sof_dropped + 1;
    } else if (s_sof_cb) {
      s_sof_cb(tick, frame, s_sof_ctx);
    }
  }
  if (sr & TIM_SR_CC2IF) {
    uint32_t tick = TIM2->CCR2;
    TIM2->SR = ~TIM_SR_CC2OF;
    if (s_pps_cb) s_pps_cb(tick, s_pps_ctx);
  }
  if (sr & TIM_SR_CC3IF) {
    uint32_t tick = TIM2->CCR3;
    TIM2->SR = ~TIM_SR_CC3OF;
    sofgnss::port::cal_started(tick);
  }
}

namespace sofgnss::port {

void timer_init() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  (void)RCC->APB1ENR1;
  pin_af(kPps, Pull::None);
  TIM2->CR1 = 0;
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->SMCR = kSmcrTsItr11;  // trigger input only, slave mode off
  TIM2->CCMR1 = TIM_CCMR1_CC1S | TIM_CCMR1_CC2S_0;  // IC1 on TRC, IC2 on TI2
  TIM2->CCER = 0;  // both rising
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0;
  TIM2->DIER = TIM_DIER_UIE;
  NVIC_SetPriority(TIM2_IRQn, kPrioTim2);
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->CR1 = TIM_CR1_CEN;
}

uint64_t ticks64() {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t hi = s_wraps;
  uint32_t lo = TIM2->CNT;
  // Wrapped, but the interrupt hasn't counted it yet.
  if ((TIM2->SR & TIM_SR_UIF) && lo < 0x80000000u) hi++;
  __set_PRIMASK(primask);
  return (uint64_t{hi} << 32) | lo;
}

uint32_t sof_dropped() { return s_sof_dropped; }

// Falling edge of the burst marker on CH3.
void cal_capture_start() {
  TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_CC3S) | TIM_CCMR2_CC3S_0;
  TIM2->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E;
  TIM2->DIER |= TIM_DIER_CC3IE;
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

uint32_t tick_hz() { return port::kCoreHz; }
uint32_t tick_now() { return TIM2->CNT; }

void pps_capture_start(PpsCallback cb, void* ctx) {
  s_pps_ctx = ctx;
  s_pps_cb = cb;
  TIM2->CCER |= TIM_CCER_CC2E;
  TIM2->DIER |= TIM_DIER_CC2IE;
}

// A software capture event: the same path as a real edge.
void pps_inject() { TIM2->EGR = TIM_EGR_CC2G; }

bool sof_capture_start(SofCallback cb, void* ctx) {
  s_sof_ctx = ctx;
  s_sof_cb = cb;
  TIM2->CCER |= TIM_CCER_CC1E;
  TIM2->DIER |= TIM_DIER_CC1IE;
  return true;
}

}  // namespace sofgnss::hal
