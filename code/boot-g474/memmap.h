#ifndef BOOT_MEMMAP_H
#define BOOT_MEMMAP_H

/* Flash and RAM layout shared by the bootloader and the main board application.
 *
 * This header is the single source of truth for the split. Four other places
 * repeat these numbers and must agree with it:
 *
 *   - boot-g474/STM32G474XX_FLASH.ld        (bootloader region)
 *   - main-g474/STM32G474XX_FLASH_APP.ld    (Release application region)
 *   - main-g474/CMakeLists.txt              (VECT_TAB_OFFSET)
 *   - tool/uf2.py                           (default --base)
 *
 * code/tests/test_dfu_config.py parses all of them and fails if they drift.
 *
 * The STM32G474CBT6 has 128 KB of flash in 2 KB pages:
 *
 *   0x08000000  bootloader          32 KB   this project
 *   0x08008000  application         94 KB   main-g474, Release build
 *   0x0801F800  properties           2 KB   reserved, see below
 *   0x08020000  end
 *
 * The Debug build of main-g474 is a different image entirely: it links at
 * 0x08000000 over the bootloader and uses the whole 126 KB, because at -O0 it
 * is ~116 KB and cannot fit above any bootloader large enough to hold TinyUSB
 * and the MSC class. `just flash` installs that one; `just flash_release`
 * reinstalls the bootloader plus the relocated Release image.
 *
 * The last page is held back from both maps for
 * property_save_to_flash()/property_load_from_flash(), which are stubbed today
 * (see main-g474/properties/properties.c). Reserving it now costs one page and
 * avoids re-cutting the map — and re-flashing every board over SWD — later.
 */

#define FLASH_ORIGIN      0x08000000UL
#define FLASH_TOTAL_SIZE  (128UL * 1024UL)

/* Bootloader: the bottom of flash, where the reset vector always lands. */
#define BOOT_BASE         FLASH_ORIGIN
#define BOOT_SIZE         (32UL * 1024UL)

/* Application, as linked for the Release/bootloader configuration. */
#define APP_BASE          (BOOT_BASE + BOOT_SIZE)
#define APP_SIZE          (94UL * 1024UL)

/* Reserved trailing page, not writable through DFU. */
#define PROPS_BASE        (APP_BASE + APP_SIZE)
#define PROPS_SIZE        (2UL * 1024UL)

/* Erase granularity is not a constant here: it depends on the DBANK option
 * bit, so flash_write.c reads it at run time. The HAL defines its own
 * FLASH_PAGE_SIZE for the dual-bank default; do not shadow it. */

/* Soft entry into DFU mode.
 *
 * The console `dfu` command stores BOOT_FLAG_MAGIC here and resets; the
 * bootloader reads the word, clears it, and stays in DFU mode if it matched.
 * A system reset does not clear RAM, so the value survives the reboot.
 *
 * This is the top 32 bytes of SRAM, carved out of the RAM region in both
 * linker scripts so neither the stack (_estack sits just below it) nor .bss
 * can land on it. A backup register would need the RTC/TAMP backup domain,
 * which the .ioc does not enable.
 *
 * After a power-on reset the word holds whatever the SRAM powered up with, so
 * the magic is a full 32 bits: the odds of a false DFU entry are 1 in 2^32,
 * and the bootloader clears the word before doing anything else either way.
 */
#define BOOT_FLAG_ADDR    0x2001FFE0UL
#define BOOT_FLAG_MAGIC   0xB00710ADUL

/* UF2 family id for ST STM32G4xx, from microsoft/uf2 utils/uf2families.json.
 * Blocks carrying any other family id are ignored, so a UF2 built for a
 * different chip cannot be written to this one. */
#define UF2_FAMILY_ID     0x4C71240AUL

#endif /* BOOT_MEMMAP_H */
