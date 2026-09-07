/*
 * USB descriptors for the Bandolibre UF2 bootloader.
 * Full-speed mass storage device, one interface, one bulk endpoint pair.
 */

#include "tusb.h"
#include "stm32g4xx_hal.h"

/* Same VID and the same auto-PID convention as the application
 * (main-g474/usb/usb_descriptors.c): 0x4000 | the class bitmap, with MSC on
 * bit 1. The bootloader must not share the application's product id — hosts
 * cache a driver per VID/PID, and the two devices expose different classes.
 *
 *   application (MIDI)   0x4008
 *   bootloader  (MSC)    0x4002
 */
#define USB_VID  0xCafe
#define USB_PID  0x4002

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    /* Class is declared per interface, not per device: a plain MSC device
     * needs no IAD (unlike the application, which uses one for MIDI). */
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_MSC = 0,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

/* Endpoint addresses (STM32 fsdev: each number is a bidirectional pair). */
#define EPNUM_MSC_OUT   0x01
#define EPNUM_MSC_IN    0x81

static uint8_t const desc_configuration[] = {
    /* config number, interface count, string index, total length, attribute, power in mA */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    /* interface number, string index, EP Out & EP In address, EP size */
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 4, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

/* Indices match iManufacturer/iProduct/iSerialNumber above, plus the MSC
 * interface string. ASCII only — unlike the application's descriptors there is
 * no accented text here, so no UTF-8 decoding is needed. */
static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: supported language is English (0x0409)
    "L'Atelier du Bandoneon Libre", // 1: Manufacturer
    "Bandolibre DFU",               // 2: Product
    NULL,                           // 3: Serial, derived from the chip UID
    "Bandolibre Firmware Update",   // 4: MSC interface
};

/* 96-bit chip UID as 24 hex digits, the same serial the application reports.
 * Windows requires an MSC serial of at least 12 characters. */
static const char *serial_string(void) {
  static char serial[25];
  static const char hex[] = "0123456789ABCDEF";
  uint32_t const uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };

  for (size_t w = 0; w < 3; w++) {
    for (size_t n = 0; n < 8; n++) {
      serial[w * 8 + n] = hex[(uid[w] >> (28 - 4 * n)) & 0xF];
    }
  }
  serial[24] = '\0';
  return serial;
}

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;

  size_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;

    const char *str = (index == 3) ? serial_string() : string_desc_arr[index];

    chr_count = strlen(str);
    size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
    if (chr_count > max_count) chr_count = max_count;

    for (size_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = (uint16_t) str[i];
    }
  }

  /* first byte is length (including header), second byte is descriptor type */
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
