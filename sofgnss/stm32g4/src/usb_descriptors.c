// USB descriptors: one CDC-ACM device, serial number from the chip UID.
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "stm32g4xx.h"
#include "tusb.h"

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

enum { STRID_LANGID = 0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL, STRID_CDC };

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = CONFIG_SOFGNSS_USB_VID,
    .idProduct = CONFIG_SOFGNSS_USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&desc_device; }

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

static const char* string_desc[] = {
    [STRID_LANGID] = (const char[]){0x09, 0x04},  // English (US)
    [STRID_MANUFACTURER] = "BertoldVdb",
    [STRID_PRODUCT] = "GNSS/SOF/PPS Bridge",
    [STRID_SERIAL] = NULL,  // from the UID
    [STRID_CDC] = "GNSS serial",
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  static uint16_t desc[32];
  static char serial[25];
  size_t chr_count;

  if (index == STRID_LANGID) {
    memcpy(&desc[1], string_desc[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc) / sizeof(string_desc[0])) return NULL;
    const char* str = string_desc[index];
    if (index == STRID_SERIAL) {
      if (serial[0] == '\0') {
        const volatile uint32_t* uid = (const volatile uint32_t*)UID_BASE;
        snprintf(serial, sizeof serial, "%08lX%08lX%08lX", (unsigned long)uid[0],
                 (unsigned long)uid[1], (unsigned long)uid[2]);
      }
      str = serial;
    }
    chr_count = strlen(str);
    if (chr_count > 31) chr_count = 31;
    for (size_t i = 0; i < chr_count; i++) desc[1 + i] = str[i];
  }
  desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return desc;
}
