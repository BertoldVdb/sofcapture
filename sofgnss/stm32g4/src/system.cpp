// Clocks, watchdog, reset handling, app descriptor and the HAL parts this port
// doesn't have (OTA, SOF spin).
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "port.h"
#include "sofgnss/hal.h"

namespace {

// esp_app_desc_t's layout, so host tools read both ports' images the same way.
struct AppDesc {
  uint32_t magic;
  uint32_t secure_version;
  uint32_t reserv1[2];
  char version[32];
  char project_name[32];
  char time[16];
  char date[16];
  char idf_ver[32];  // the platform here
  uint8_t app_elf_sha256[32];  // filled in after the build (app_desc.py)
  uint32_t reserv2[20];
};
static_assert(sizeof(AppDesc) == 256, "AppDesc layout");

constexpr uint32_t kPanicMagic = 0x50414e43;
uint32_t s_panic __attribute__((section(".noinit")));
const char* s_reset_reason = "UNKNOWN";

void read_reset_reason() {
  uint32_t csr = RCC->CSR;
  bool panic = s_panic == kPanicMagic;
  s_panic = 0;
  RCC->CSR |= RCC_CSR_RMVF;
  // A software or watchdog reset also pulls NRST, so check those first.
  if (csr & RCC_CSR_IWDGRSTF) s_reset_reason = "WDT";
  else if (csr & RCC_CSR_WWDGRSTF) s_reset_reason = "WDT";
  else if (csr & RCC_CSR_SFTRSTF) s_reset_reason = panic ? "PANIC" : "SW";
  else if (csr & RCC_CSR_BORRSTF) s_reset_reason = "POWERON";
  else if (csr & RCC_CSR_OBLRSTF) s_reset_reason = "OBL";
  else if (csr & RCC_CSR_LPWRRSTF) s_reset_reason = "LPWR";
  else if (csr & RCC_CSR_PINRSTF) s_reset_reason = "EXT";
}

// About 4 s on the 32 kHz LSI. Stopped while a debugger halts the core.
void watchdog_start() {
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
  IWDG->KR = 0xCCCC;
  IWDG->KR = 0x5555;
  IWDG->PR = 4;  // /64
  IWDG->RLR = 2000;
  while (IWDG->SR) {
  }
  IWDG->KR = 0xAAAA;
}

void hse_stop() {
  RCC->CR &= ~RCC_CR_HSEON;
  while (RCC->CR & RCC_CR_HSERDY) {
  }
  RCC->CR &= ~RCC_CR_HSEBYP;
}

// Waits about 10 ms at 16 MHz. bypass: an external clock instead of a crystal.
bool hse_start(bool bypass) {
  RCC->CR |= bypass ? RCC_CR_HSEBYP : 0;
  RCC->CR |= RCC_CR_HSEON;
  for (uint32_t i = 0; i < 40000; ++i) {
    if (RCC->CR & RCC_CR_HSERDY) return true;
  }
  hse_stop();
  return false;
}

// Counts 4096 HSE cycles (HSE/32 into TIM17, every 8th edge captured) on
// HSI16. Returns the measured frequency, or 0. Run on HSI16.
uint32_t hse_measure() {
  constexpr int kCaptures = 16;
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
  (void)RCC->APB2ENR;
  TIM17->OR = TIM_OR_HSE32EN;
  TIM17->TISEL = 3u << TIM_TISEL_TI1SEL_Pos;
  TIM17->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC;
  TIM17->CCER = TIM_CCER_CC1E;
  TIM17->CR1 = TIM_CR1_CEN;
  TIM17->SR = 0;
  uint16_t first = 0, last = 0;
  bool ok = true;
  // Reading CCR1 clears the flag. Interrupts aren't running yet, so no capture is missed.
  for (int n = 0; n <= kCaptures && ok; ++n) {
    uint32_t i = 0;
    while (!(TIM17->SR & TIM_SR_CC1IF) && ++i < 10000) {
    }
    ok = i < 10000;
    last = static_cast<uint16_t>(TIM17->CCR1);
    if (n == 0) first = last;
  }
  TIM17->CR1 = 0;
  RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN;
  uint16_t ticks = last - first;
  if (!ok || ticks == 0) return 0;
  return static_cast<uint32_t>(16000000ull * kCaptures * 8 * 32 / ticks);
}

// The supported frequency within 5 %, or 0.
uint32_t hse_nominal(uint32_t hz) {
  for (uint32_t f : {8000000u, 12000000u, 24000000u}) {
    if (hz > f - f / 20 && hz < f + f / 20) return f;
  }
  return 0;
}

uint32_t s_hse_hz;  // 0: running from HSI16
bool s_hse_bypass;
uint32_t s_hse_measured[2];  // crystal, external clock

[[noreturn]] void panic() {
  s_panic = kPanicMagic;
  NVIC_SystemReset();
}

}  // namespace

extern "C" {

__attribute__((section(".app_desc"), used)) const AppDesc g_app_desc = {
    0xABCD5432, 0, {}, SOFGNSS_VERSION, "sofgnss", __TIME__, __DATE__, "stm32g4", {}, {}};

// Runs before .data and .bss are set up.
void SystemInit() {
  SCB->CPACR |= (3u << 20) | (3u << 22);  // FPU
  SCB->VTOR = FLASH_BASE;
}

void SysTick_Handler() {}  // only wakes the main loop

void NMI_Handler() { panic(); }
void HardFault_Handler() { panic(); }
void MemManage_Handler() { panic(); }
void BusFault_Handler() { panic(); }
void UsageFault_Handler() { panic(); }

}  // extern "C"

namespace sofgnss::port {

uint32_t hse_hz() { return s_hse_hz; }
uint32_t hse_measured(bool bypass) { return s_hse_measured[bypass]; }
bool hse_bypass() { return s_hse_bypass; }

// 144 MHz from the PLL: 4 MHz in, x72 = 288 MHz VCO, /2 for the core and /6
// = 48 MHz for USB. USB must run from the timer's clock: from HSI48 the SOF
// trigger reached TIM2 for only 80 to 86 % of the frames.
// HSE may be an 8, 12 or 24 MHz external clock or crystal. Bypass goes first:
// a crystal gives no clock there, while an external clock would also pass in
// crystal mode, only with more jitter. A mode only counts if the frequency
// checks out.
// Without HSE the PLL runs from HSI16, too inaccurate for USB, so USB falls
// back to HSI48 trimmed by the CRS.
void clock_init() {
  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | (4u << FLASH_ACR_LATENCY_Pos) |
               FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
  while ((FLASH->ACR & FLASH_ACR_LATENCY) != (4u << FLASH_ACR_LATENCY_Pos)) {
  }

  for (bool bypass : {true, false}) {
    if (!hse_start(bypass)) continue;
    s_hse_measured[bypass] = hse_measure();
    s_hse_hz = hse_nominal(s_hse_measured[bypass]);
    if (s_hse_hz) {
      s_hse_bypass = bypass;
      break;
    }
    hse_stop();
  }
  uint32_t src = s_hse_hz ? 3u : 2u;
  uint32_t pll_in = s_hse_hz ? s_hse_hz : 16000000;
  RCC->PLLCFGR = (src << RCC_PLLCFGR_PLLSRC_Pos) | ((pll_in / 4000000 - 1) << RCC_PLLCFGR_PLLM_Pos) |
                 (72u << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN |
                 (2u << RCC_PLLCFGR_PLLQ_Pos) | RCC_PLLCFGR_PLLQEN;
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) {
  }
  // Through AHB /2 for at least 1 us to limit the current step (RM0440).
  // Nothing timed runs yet. The loop takes well over 100 cycles.
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_3;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | (3u << RCC_CFGR_SW_Pos);
  while ((RCC->CFGR & RCC_CFGR_SWS) != (3u << RCC_CFGR_SWS_Pos)) {
  }
  for (volatile uint32_t i = 0; i < 100; ++i) {
  }
  RCC->CFGR &= ~RCC_CFGR_HPRE;

  if (s_hse_hz) {
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_CLK48SEL) | (2u << RCC_CCIPR_CLK48SEL_Pos);  // PLLQ
    return;
  }
  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {
  }
  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
  (void)RCC->APB1ENR1;
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;  // sync source defaults to USB SOF
}

void pin_af(const Pin& p, Pull pull) {
  GPIO_TypeDef* g = p.port;
  const uint32_t s = p.pin * 2u;
  g->AFR[p.pin / 8] = (g->AFR[p.pin / 8] & ~(0xFu << (p.pin % 8 * 4))) | (uint32_t{p.af} << (p.pin % 8 * 4));
  g->OSPEEDR |= 3u << s;
  g->PUPDR = (g->PUPDR & ~(3u << s)) | (static_cast<uint32_t>(pull) << s);
  g->MODER = (g->MODER & ~(3u << s)) | (2u << s);
}

void pin_output(const Pin& p) {
  const uint32_t s = p.pin * 2u;
  p.port->MODER = (p.port->MODER & ~(3u << s)) | (1u << s);
}

void delay_ms(uint32_t ms) {
  uint64_t end = hal::uptime_us() + uint64_t{ms} * 1000;
  while (hal::uptime_us() < end) hal::watchdog_feed();
}

}  // namespace sofgnss::port

namespace sofgnss::hal {

void init() {
  read_reset_reason();
  port::clock_init();
  SysTick_Config(port::kCoreHz / 1000);
  NVIC_SetPriority(SysTick_IRQn, port::kPrioSysTick);
  DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;  // keep the debugger working across WFI
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
  (void)RCC->AHB2ENR;
  port::pin_output(port::kLed);
  led_set(false);
  watchdog_start();
  port::timer_init();
  port::uart_init();
}

uint64_t uptime_us() { return port::ticks64() / (port::kCoreHz / 1000000); }

void sleep_us(uint32_t us) {
  uint32_t start = tick_now();
  uint32_t n = us * (port::kCoreHz / 1000000);
  while (tick_now() - start < n) {
  }
}

const char* reset_reason() { return s_reset_reason; }

// The main loop is the only task: it feeds the independent watchdog.
void watchdog_register_task() {}
void watchdog_feed() { IWDG->KR = 0xAAAA; }

void restart() {
  port::usb_detach(port::kUsbDetachMs);
  NVIC_SystemReset();
}

void led_set(bool on) { port::pin_write(port::kLed, on); }

AppInfo app_info() {
  AppInfo info{};
  strncpy(info.version, g_app_desc.version, sizeof info.version - 1);
  strncpy(info.idf, g_app_desc.idf_ver, sizeof info.idf - 1);
  strncpy(info.partition, "flash", sizeof info.partition - 1);
  // Patched after linking, so the compiler must not fold it to zeros.
  const volatile uint8_t* sha = g_app_desc.app_elf_sha256;
  for (int i = 0; i < 32; ++i) snprintf(info.elf_sha256 + 2 * i, 3, "%02x", sha[i]);
  return info;
}

// No firmware update yet: flash over SWD.
bool ota_begin(uint32_t) { return false; }
bool ota_write(const uint8_t*, size_t) { return false; }
bool ota_end(const uint8_t*) { return false; }
void ota_abort() {}
bool ota_pending_verify() { return false; }
void ota_confirm() {}

// SOF is captured in hardware, so the spin source is never used.
bool usb_sof_wait_change(uint32_t, uint32_t*, uint16_t*) { return false; }

}  // namespace sofgnss::hal
