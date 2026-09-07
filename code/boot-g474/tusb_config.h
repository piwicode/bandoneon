/*
 * TinyUSB configuration for the Bandolibre UF2 bootloader (STM32G474, USB FS
 * device). Mass storage only — the application's MIDI stack is not built here.
 */

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// CFG_TUSB_MCU is defined to OPT_MCU_STM32G4 by the build system (CMakeLists.txt)
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#define CFG_TUSB_OS               OPT_OS_NONE

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED           1

// STM32G474 USB peripheral is Full Speed only
#define CFG_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE    64

//------------- CLASS -------------//
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               1
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

/* One disk sector per transfer. The bulk endpoints stay at the full-speed
 * maximum of 64 bytes; this is the class driver's RAM buffer, and matching it
 * to the 512-byte sector means tud_msc_write10_cb() is handed a whole UF2
 * block at once instead of eight fragments to reassemble. */
#define CFG_TUD_MSC_EP_BUFSIZE    512

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
