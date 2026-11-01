// Resident bootloader. It never starts the app by itself: the host reads VER,
// then sends START (the app is checked again first) or OTA with a signed image.
// Same USB identity and $PVDB protocol as the app.
#include <cstring>

#include "boot_key.h"
#include "image.h"
#include "sha256.h"
#include "sofgnss/hal.h"
#include "sofgnss/ota_receiver.h"
#include "sofgnss/pvdb.h"
#include "stm32g4xx.h"
#include "tusb.h"
#include "uECC.h"

using namespace sofgnss;

namespace {

constexpr uint32_t kCoreHz = 16000000;  // HSI16
constexpr uint32_t kHostSilenceMs = 60000;
constexpr uint32_t kUsbDetachMs = 300;
constexpr uint32_t kLedPin = 6;  // PC6
constexpr uint32_t kSrErrors = FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                               FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR |
                               FLASH_SR_RDERR | FLASH_SR_OPTVERR;

volatile uint32_t s_ms;

void feed() { IWDG->KR = 0xAAAA; }

void delay_ms(uint32_t ms) {
  uint32_t start = s_ms;
  while (s_ms - start < ms) feed();
}

// ---- Flash -------------------------------------------------------------------

bool dual_bank() { return FLASH->OPTR & FLASH_OPTR_DBANK; }
uint32_t page_size() { return dual_bank() ? 2048 : 4096; }

// End of the flash contiguous from the start. With two banks on parts below
// 512 KB, bank 2 does not follow bank 1.
uint32_t flash_end() {
  uint32_t size = *reinterpret_cast<const uint16_t*>(FLASHSIZE_BASE) * 1024u;
  if (dual_bank() && size < 512 * 1024) size /= 2;
  return FLASH_BASE + size;
}

bool flash_wait() {
  while (FLASH->SR & FLASH_SR_BSY) {
  }
  bool ok = !(FLASH->SR & kSrErrors);
  FLASH->SR = kSrErrors | FLASH_SR_EOP;
  return ok;
}

bool flash_unlock() {
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
  }
  FLASH->SR = kSrErrors | FLASH_SR_EOP;
  return !(FLASH->CR & FLASH_CR_LOCK);
}

void flash_lock() { FLASH->CR |= FLASH_CR_LOCK; }

bool erase_page(uint32_t addr) {
  uint32_t off = addr - FLASH_BASE;
  uint32_t cr = FLASH_CR_PER;
  if (dual_bank() && off >= 0x40000) {
    cr |= FLASH_CR_BKER;
    off -= 0x40000;
  }
  cr |= off / page_size() << FLASH_CR_PNB_Pos;
  FLASH->CR = (FLASH->CR & ~(FLASH_CR_PER | FLASH_CR_PNB | FLASH_CR_BKER | FLASH_CR_PG)) | cr;
  FLASH->CR |= FLASH_CR_STRT;
  bool ok = flash_wait();
  FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_BKER);
  feed();
  return ok;
}

bool program(uint32_t addr, const uint8_t* src) {
  uint32_t w[2];
  memcpy(w, src, 8);
  FLASH->CR |= FLASH_CR_PG;
  reinterpret_cast<volatile uint32_t*>(addr)[0] = w[0];
  __ISB();
  reinterpret_cast<volatile uint32_t*>(addr)[1] = w[1];
  bool ok = flash_wait();
  FLASH->CR &= ~FLASH_CR_PG;
  return ok;
}

// ---- Image check -------------------------------------------------------------

enum class State { Empty, Invalid, Valid };

const boot::Header& installed() { return *reinterpret_cast<const boot::Header*>(boot::kInfoAddr); }

// h may be in RAM, the app is read from flash.
State check(const boot::Header& h) {
  if (h.magic == 0xFFFFFFFF) return State::Empty;
  if (h.magic != boot::kMagic || h.app_addr != boot::kAppAddr || h.app_len < 256 ||
      h.app_len > flash_end() - boot::kAppAddr || h.desc_off > h.app_len - 256) {
    return State::Invalid;
  }
  const uint32_t* vec = reinterpret_cast<const uint32_t*>(boot::kAppAddr);
  if ((vec[0] & 0xFFF00000) != SRAM1_BASE || !(vec[1] & 1) || vec[1] < boot::kAppAddr ||
      vec[1] >= boot::kAppAddr + h.app_len) {
    return State::Invalid;
  }
  sha256_ctx c;
  sha256_init(&c);
  for (uint32_t off = 0; off < h.app_len; off += 4096) {
    uint32_t n = h.app_len - off < 4096 ? h.app_len - off : 4096;
    sha256_update(&c, reinterpret_cast<const uint8_t*>(boot::kAppAddr + off), n);
    feed();
  }
  uint8_t digest[32];
  sha256_final(&c, digest);
  if (memcmp(digest, h.app_sha256, 32) != 0) return State::Invalid;
  sha256_init(&c);
  sha256_update(&c, &h, offsetof(boot::Header, sig));
  sha256_final(&c, digest);
  bool ok = uECC_verify(kPublicKey, digest, sizeof digest, h.sig, uECC_secp256r1());
  feed();
  return ok ? State::Valid : State::Invalid;
}

State s_state;

// ---- Host link ---------------------------------------------------------------

void emit(const char* s, size_t n, void*) {
  if (!tud_cdc_connected()) return;
  tud_cdc_write(s, n);
  tud_cdc_write_flush();
}

void send_ver() {
  char version[33] = "";
  char sha[65] = "";
  if (s_state != State::Empty) {
    // esp_app_desc_t: version at 16, ELF SHA-256 at 144.
    const uint8_t* d = reinterpret_cast<const uint8_t*>(boot::kAppAddr + installed().desc_off);
    memcpy(version, d + 16, 32);
    pvdb::hex(sha, d + 144, 32);
  }
  const char* state = s_state == State::Valid ? "valid" : s_state == State::Invalid ? "invalid" : "empty";
  uint32_t csr = RCC->CSR;  // left for the app to read and clear
  const char* reason = csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF) ? "WDT"
                       : csr & RCC_CSR_SFTRSTF                      ? "SW"
                       : csr & RCC_CSR_BORRSTF                      ? "POWERON"
                       : csr & RCC_CSR_PINRSTF                      ? "EXT"
                                                                    : "UNKNOWN";
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_ver(s, sizeof s, version, "stm32g4-boot", state, false, sha, reason);
  emit(s, n, nullptr);
}

void send_error(const char* text) {
  char s[pvdb::kMaxSentence];
  emit(s, pvdb::format_err(s, sizeof s, text), nullptr);
}

// The app starts as if from reset, apart from the watchdog.
[[noreturn]] void start_app() {
  tud_disconnect();
  delay_ms(10);
  __disable_irq();
  SysTick->CTRL = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_USBRST | RCC_APB1RSTR1_CRSRST;
  RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_USBRST | RCC_APB1RSTR1_CRSRST);
  RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN);
  RCC->AHB2RSTR |= RCC_AHB2RSTR_GPIOCRST;
  RCC->AHB2RSTR &= ~RCC_AHB2RSTR_GPIOCRST;
  RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOCEN;
  RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
  const uint32_t* vec = reinterpret_cast<const uint32_t*>(boot::kAppAddr);
  SCB->VTOR = boot::kAppAddr;
  __set_MSP(vec[0]);
  __enable_irq();
  reinterpret_cast<void (*)()>(vec[1])();
  for (;;) {
  }
}

OtaReceiver s_ota{emit, nullptr};

void on_command(const char* f, size_t len) {
  if (len == 3 && memcmp(f, "VER", 3) == 0) {
    send_ver();
  } else if ((len == 4 && memcmp(f, "PING", 4) == 0) || (len == 4 && memcmp(f, "BOOT", 4) == 0)) {
    // PING only feeds the host watchdog, and we're already here for BOOT.
  } else if (len > 5 && memcmp(f, "INFO,", 5) == 0 && pvdb::valid_tag(f + 5, len - 5)) {
    // sofd's handshake: it waits for its tag before anything else.
    char tag[pvdb::kMaxTag + 1];
    memcpy(tag, f + 5, len - 5);
    tag[len - 5] = '\0';
    pvdb::Info info{};
    info.tick_hz = kCoreHz;
    info.tag = tag;
    char s[pvdb::kMaxSentence];
    emit(s, pvdb::format_info(s, sizeof s, info), nullptr);
  } else if (len == 5 && memcmp(f, "START", 5) == 0) {
    s_state = check(installed());
    if (s_state == State::Valid) start_app();
    send_error(s_state == State::Empty ? "NOAPP" : "BADAPP");
  } else if (len > 4 && memcmp(f, "OTA,", 4) == 0) {
    if (s_ota.finished()) s_ota = OtaReceiver{emit, nullptr};
    s_ota.start(f + 4, len - 4);
  } else {
    send_error("UNKNOWN");
  }
}

char s_line[128];
size_t s_line_len;
bool s_line_long;

void line_byte(uint8_t c) {
  if (s_line_len < sizeof s_line) {
    s_line[s_line_len++] = static_cast<char>(c);
  } else {
    s_line_long = true;
  }
  if (c != '\n') return;
  const char* fields;
  size_t n;
  if (!s_line_long && s_line_len >= 2 && s_line[s_line_len - 2] == '\r' && pvdb::verify(s_line, s_line_len) &&
      pvdb::command_fields(reinterpret_cast<const uint8_t*>(s_line), s_line_len, &fields, &n)) {
    on_command(fields, n);
  }
  s_line_len = 0;
  s_line_long = false;
}

// ---- OTA into flash ----------------------------------------------------------
// The header is kept in RAM and written last, once the whole image checks out.

struct {
  sha256_ctx sha;  // of the whole upload
  boot::Header header;
  uint32_t got;
  uint32_t addr;  // next double word to program
  uint8_t dword[8];
  uint32_t dword_len;
} s_up;

}  // namespace

extern "C" {

extern const uint32_t g_pfnVectors[];

// Before .data and .bss: the watchdog runs from the first instruction on.
// About 4 s on the 32 kHz LSI, stopped while a debugger halts the core.
void SystemInit() {
  // Entered by a jump, as from the ROM's DFU after :leave: start over from reset.
  if ((RCC->APB1ENR1 & RCC_APB1ENR1_USBEN) || (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_0) {
    NVIC_SystemReset();
  }
  SCB->VTOR = reinterpret_cast<uint32_t>(g_pfnVectors);
  SCB->CPACR |= (3u << 20) | (3u << 22);  // FPU
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
  IWDG->KR = 0xCCCC;
  IWDG->KR = 0x5555;
  IWDG->PR = 4;
  IWDG->RLR = 2000;
  while (IWDG->SR) {
  }
  IWDG->KR = 0xAAAA;
}

void SysTick_Handler() { s_ms = s_ms + 1; }
void USB_HP_IRQHandler() { tud_int_handler(0); }
void USB_LP_IRQHandler() { tud_int_handler(0); }
void USBWakeUp_IRQHandler() { tud_int_handler(0); }
uint32_t tusb_time_millis_api() { return s_ms; }

}  // extern "C"

namespace sofgnss::hal {

uint64_t uptime_us() { return uint64_t{s_ms} * 1000; }

bool ota_begin(uint32_t size) {
  if (size <= sizeof(boot::Header) || size - sizeof(boot::Header) > flash_end() - boot::kAppAddr) return false;
  s_state = State::Empty;
  if (!flash_unlock()) return false;
  uint32_t end = boot::kAppAddr + size - sizeof(boot::Header);
  for (uint32_t a = boot::kInfoAddr; a < end; a += page_size()) {
    if (!erase_page(a)) return false;
  }
  sha256_init(&s_up.sha);
  s_up.got = 0;
  s_up.addr = boot::kAppAddr;
  s_up.dword_len = 0;
  return true;
}

bool ota_write(const uint8_t* data, size_t len) {
  sha256_update(&s_up.sha, data, len);
  for (size_t i = 0; i < len; ++i, ++s_up.got) {
    if (s_up.got < sizeof(boot::Header)) {
      reinterpret_cast<uint8_t*>(&s_up.header)[s_up.got] = data[i];
      continue;
    }
    s_up.dword[s_up.dword_len++] = data[i];
    if (s_up.dword_len == 8) {
      if (!program(s_up.addr, s_up.dword)) return false;
      s_up.addr += 8;
      s_up.dword_len = 0;
    }
  }
  return true;
}

bool ota_end(const uint8_t expected_sha256[32]) {
  bool ok = true;
  if (s_up.dword_len) {
    memset(s_up.dword + s_up.dword_len, 0xFF, 8 - s_up.dword_len);
    ok = program(s_up.addr, s_up.dword);
  }
  uint8_t digest[32];
  sha256_final(&s_up.sha, digest);
  ok = ok && memcmp(digest, expected_sha256, 32) == 0;
  ok = ok && s_up.header.app_len == s_up.got - sizeof(boot::Header) && check(s_up.header) == State::Valid;
  const uint8_t* h = reinterpret_cast<const uint8_t*>(&s_up.header);
  for (uint32_t off = 0; ok && off < sizeof(boot::Header); off += 8) ok = program(boot::kInfoAddr + off, h + off);
  flash_lock();
  s_state = check(installed());
  return ok && s_state == State::Valid;
}

void ota_abort() {
  flash_lock();
  s_state = check(installed());
}

}  // namespace sofgnss::hal

int main() {
  // The caches could hold flash contents from before an erase. Not needed at 16 MHz.
  FLASH->ACR &= ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
  FLASH->ACR |= FLASH_ACR_ICRST | FLASH_ACR_DCRST;
  FLASH->ACR &= ~(FLASH_ACR_ICRST | FLASH_ACR_DCRST);
  SysTick_Config(kCoreHz / 1000);

  // USB from HSI48, trimmed to the host's SOF by the CRS.
  RCC->CRRCR |= RCC_CRRCR_HSI48ON;
  while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {
  }
  RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN | RCC_APB1ENR1_USBEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  (void)RCC->AHB2ENR;
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
  GPIOC->MODER = (GPIOC->MODER & ~(3u << 2 * kLedPin)) | (1u << 2 * kLedPin);

  s_state = check(installed());
  // After any reset the host must see us leave the bus.
  delay_ms(kUsbDetachMs);
  tud_cdc_configure_t cdc_cfg = TUD_CDC_CONFIGURE_DEFAULT();
  cdc_cfg.tx_overwritabe_if_not_connected = 0;
  tud_cdc_configure(&cdc_cfg);
  tusb_rhport_init_t rh = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
  tusb_init(0, &rh);

  uint32_t last_host = s_ms;
  bool was_connected = false;
  for (;;) {
    feed();
    tud_task();
    uint32_t now = s_ms;
    // Blinks fast while waiting for the host.
    GPIOC->BSRR = (now & 128 ? 1u : 0x10000u) << kLedPin;

    bool connected = tud_cdc_connected();
    if (connected && !was_connected) send_ver();
    was_connected = connected;
    if (s_ota.receiving() && !connected) s_ota.cancel("OTALINK");
    s_ota.poll(hal::uptime_us());

    uint8_t buf[64];
    size_t n = tud_cdc_available() ? tud_cdc_read(buf, sizeof buf) : 0;
    if (n) last_host = now;
    size_t used = s_ota.receiving() ? s_ota.feed(buf, n, hal::uptime_us()) : 0;
    for (size_t i = used; i < n; ++i) line_byte(buf[i]);

    if (now - last_host > kHostSilenceMs) NVIC_SystemReset();
    __WFI();
  }
}
