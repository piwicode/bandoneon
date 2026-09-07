/*
 * USB descriptors for the Bandolibre main board.
 * Composite full-speed device: MIDI + CDC (console mirror).
 */

#include "tusb.h"
#include "stm32g4xx_hal.h"

/* USB ids sublicensed from STMicroelectronics: ST's vendor id with a product
 * id assigned to this project, valid only for a product built on ST silicon
 * (STM32G474CBT6) and only for Bandolibre. The sublicense does not cover
 * USB-IF certification, so the device must not carry the USB logo.
 *
 * The bootloader (boot-g474) shares this VID/PID and is told apart by
 * bcdDevice: hosts cache a driver per VID/PID/revision, and the two devices
 * expose different classes (MIDI here, MSC there).
 *
 *   application  bcdDevice 0x0100
 *   bootloader   bcdDevice 0x0200
 */
#define USB_VID  0x0483  /* STMicroelectronics */
#define USB_PID  0xA5B4  /* Bandolibre */

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) device class so the CDC
    // interface pair is grouped correctly on the host.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
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
  ITF_NUM_MIDI = 0,
  ITF_NUM_MIDI_STREAMING,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

// Endpoint addresses (STM32 fsdev: each number is a bidirectional pair)
#define EPNUM_MIDI_OUT   0x01
#define EPNUM_MIDI_IN    0x81

static uint8_t const desc_fs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // MIDI: Interface number, string index, EP Out & EP In address, EP size
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 1, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index; // for multiple configurations
  return desc_fs_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

// Strings are UTF-8 encoded and converted to UTF-16 on request.
static char const *string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 },  // 0: supported language is English (0x0409)
  "L'Atelier du Bandon\xc3\xa9on Libre",  // 1: Manufacturer
  "Bandolibre",                   // 2: Product
  NULL,                           // 3: Serial, derived from the chip UID
};

static uint16_t _desc_str[32 + 1];

// Decodes one UTF-8 sequence (1 to 3 bytes, i.e. up to U+FFFF) into a UTF-16
// code unit. Returns the number of bytes consumed, 0 at end of string.
static size_t utf8_next(const char *s, uint16_t *out) {
  uint8_t const c = (uint8_t) s[0];
  if (c == 0) return 0;
  if (c < 0x80) {
    *out = c;
    return 1;
  }
  if ((c & 0xE0) == 0xC0) {
    *out = (uint16_t) (((c & 0x1F) << 6) | ((uint8_t) s[1] & 0x3F));
    return 2;
  }
  if ((c & 0xF0) == 0xE0) {
    *out = (uint16_t) (((c & 0x0F) << 12) | (((uint8_t) s[1] & 0x3F) << 6) | ((uint8_t) s[2] & 0x3F));
    return 3;
  }
  *out = '?';  // code points above U+FFFF are not representable here
  return 4;
}

static size_t serial_string(uint16_t *dst, size_t max_count) {
  uint32_t const uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
  size_t count = 0;
  for (size_t w = 0; w < 3 && count < max_count; w++) {
    for (int shift = 28; shift >= 0 && count < max_count; shift -= 4) {
      dst[count++] = "0123456789ABCDEF"[(uid[w] >> shift) & 0xF];
    }
  }
  return count;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t const max_count = TU_ARRAY_SIZE(_desc_str) - 1;  // -1 for the header
  size_t chr_count;

  switch (index) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = serial_string(_desc_str + 1, max_count);
      break;

    default:
      if (index >= TU_ARRAY_SIZE(string_desc_arr)) return NULL;

      const char *str = string_desc_arr[index];
      chr_count = 0;
      while (chr_count < max_count) {
        uint16_t unit;
        size_t consumed = utf8_next(str, &unit);
        if (consumed == 0) break;
        _desc_str[1 + chr_count++] = unit;
        str += consumed;
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
