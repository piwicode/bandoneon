#ifndef BOOT_FLASH_WRITE_H
#define BOOT_FLASH_WRITE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Erase and program the application region of internal flash.
 *
 * Only the region described by APP_BASE / APP_SIZE in memmap.h is ever
 * touched; the bootloader below it and the reserved properties page above it
 * are out of reach by construction.
 */

/* Size of one erasable page, read from the DBANK option bit at run time:
 * 2 KB when the flash is in its (factory default) dual-bank configuration,
 * 4 KB in single-bank. */
uint32_t flash_page_size(void);

/* Write `len` bytes to APP_BASE, erasing every page it covers first.
 *
 * The page holding the vector table is programmed LAST, so an interruption
 * (power loss, a reset) leaves the application's first two words erased and
 * the validity check in main.c rejects the image — the board comes back up in
 * DFU mode rather than jumping into a half-written application.
 *
 * `len` is rounded up to a multiple of 8; the padding is 0xFF.
 * Returns false if any erase or program step reported an error.
 */
bool flash_write_app(const uint8_t *image, size_t len);

#endif /* BOOT_FLASH_WRITE_H */
