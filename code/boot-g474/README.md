# UF2 bootloader (main board)

Lets the main board be reflashed by copying a file onto a USB drive, with no
ST-Link, no toolchain and no cable other than the one already carrying MIDI.

Hold **FN2** (the rightmost function button) while powering the instrument on,
and a drive called `BANDOLIBRE` appears. Drop `main-g474.uf2` on it; when the
copy finishes the drive disappears and the instrument restarts on the new
firmware. End-user instructions are in
[documentation/firmware_update.md](../../documentation/firmware_update.md).

## Flash layout

The STM32G474CBT6 has 128 KB of flash, shared:

| Address | Size | Contents |
|---|---|---|
| `0x08000000` | 32 KB | this bootloader (currently ~14 KB used) |
| `0x08008000` | 94 KB | application, Release build (currently ~69 KB used) |
| `0x0801F800` | 2 KB | reserved for the properties store, not yet used |

[`memmap.h`](memmap.h) is the single source of truth for these numbers; four
other files repeat them and [`code/tests/test_dfu_config.py`](../tests/test_dfu_config.py)
fails if any of them drifts.

**The Debug build of the application does not fit in this layout.** At `-O0` it
is ~116 KB, more than the 94 KB above the bootloader and more than what is left
above any bootloader large enough to hold TinyUSB and the MSC class. So there
are two application memory maps:

| | Linker script | Links at | Installed by |
|---|---|---|---|
| Debug | `STM32G474XX_FLASH.ld` | `0x08000000`, 126 KB | `just flash` — **replaces the bootloader** |
| Release | `STM32G474XX_FLASH_APP.ld` | `0x08008000`, 94 KB | `just flash_release`, or a `.uf2` over USB |

`just flash` therefore leaves a board with no DFU support; `just flash_release`
in `../main-g474` puts the bootloader and a Release image back.

## Boot sequence

1. Reset lands here, always.
2. Clocks come up exactly as in the application's `SystemClock_Config()`:
   96 MHz core from the HSI16 PLL, USB on HSI48 trimmed by the CRS against USB
   start-of-frame. The board is crystal-less, so the CRS is not optional.
3. `HAL_MspInit()` clears the UCPD dead-battery pull-down — see the warning
   below.
4. DFU mode is entered if **any** of:
   - PB4 (`SW_FN2`) reads low, i.e. the button is held;
   - the word at `BOOT_FLAG_ADDR` holds `BOOT_FLAG_MAGIC`, written by the
     application's `dfu` console command just before a system reset;
   - the application's vector table fails a sanity check, so a board with
     missing or half-written firmware recovers on its own.
5. Otherwise it de-initialises everything, points `SCB->VTOR` at `APP_BASE`,
   loads the application's stack pointer and jumps to its reset handler.

## Why UF2 rather than copying a `.bin`

A UF2 file is a flat sequence of 512-byte blocks, each carrying the address it
belongs at, a family id identifying the chip, and its position in the file.
[`msc_disk.c`](msc_disk.c) inspects every sector the host writes to the volume,
keeps the ones that are UF2 blocks for an STM32G4 inside the application
region, and throws away everything else — directory entries, FAT updates, and
the `.Spotlight-V100` and `System Volume Information` debris that macOS and
Windows leave on any removable drive.

That means no filesystem code at all: no directory to parse, no cluster chain
to follow, and no assumption about the order in which a host flushes its
writes. Recognising a plain `.bin` would need all three, and each is a place
for a host-specific quirk to corrupt an image. It is also why every
drag-and-drop bootloader (Adafruit, RP2040, micro:bit) uses UF2.

Blocks are staged in RAM and only written to flash once all of them have
arrived, so an interrupted or truncated copy never reaches the chip. The
bootloader has the RAM to spare: the application uses 8 KB of the 128 KB.
`flash_write_app()` then programs the page holding the vector table **last**,
so a power cut during the ~1 s write leaves an image that fails the validity
check and the board comes back up in DFU mode rather than jumping into
something half-written.

## ⚠ PB4 needs the UCPD dead-battery pull-down cleared

`SW_FN2` sits on PB4, which is also **UCPD1_CC2**. Out of reset that pin can
carry a 5.1 kΩ pull-down from the UCPD peripheral, gated by the level on PA10
(STM32G474CB datasheet, pin table note 6). PA10 is this board's `USART1_RX`,
which the attached ST-Link VCP holds high while idle — so the pull-down is
active, it beats the ~40 kΩ internal pull-up, and the button reads as pressed
forever. `HAL_PWREx_DisableUCPDDeadBattery()` in `HAL_MspInit()` removes it,
and `test_dfu_config.py` checks the call is still there.

PB4 is also `NJTRST` and comes out of reset in its JTAG alternate function
(note 5), so it has to be claimed as a plain input. The board is debugged over
SWD, which does not use that pin.

FN0 was **not** used for this: it sits on PB8/BOOT0 and drives BOOT0 *high*
when pressed, which would collide with ST's own boot-mode selection if the
`nBOOT_SEL` option byte were ever cleared.

## USB identity

| | Application | Bootloader |
|---|---|---|
| VID:PID | `0483:a5b4` | `0483:a5b4` |
| bcdDevice | `0x0100` | `0x0200` |
| Class | MIDI (IAD) | Mass storage |
| Product | Bandolibre | Bandolibre DFU |
| Serial | 96-bit chip UID, 24 hex digits | same |

The VID/PID is ST's vendor id with a product id sublicensed to this project
(see [`../main-g474/usb/README.md`](../main-g474/usb/README.md)). One product id
covers the whole product, so application and bootloader share it and differ by
`bcdDevice` instead: hosts cache a driver per VID/PID/revision, and the two
expose different classes.

## Build

Not a CubeMX project — there is no `.ioc` and no generated `Core/`. It
configures its handful of peripherals directly, so there is no second generated
project to keep in sync with the hardware. It does reuse the main board's
vendored HAL and startup file rather than carrying a second copy, the same way
both boards reuse `../common`.

```sh
just build          # Debug (also -Os; the bootloader must fit either way)
just build_release  # what ships
just size           # how much of the 32 KB region is used
just flash          # install at 0x08000000, leaving the application intact
```

`just flash` goes through `../tool/flash.py` with the board type `main-g474`,
so the chip-UID registry still refuses to write main board firmware to a wing.

## Testing

```sh
lsusb -d 0483:a5b4              # bootloader enumerated (bcdDevice 0200)
cat /media/*/BANDOLIBRE/INFO_UF2.TXT
```

The bootloader prints to the same console as the application (USART1 on PA9,
921600 baud, TX only), which is the fastest way to see why an update did not
take:

```
boot-g474: soft_request=0 button=1 app_valid=1
boot-g474: entering DFU mode
uf2: all 270 blocks received (image_len=69116)
flash: writing 69116 bytes, page=2048 bytes, 34 pages, DBANK=1, FLASHSIZE=128 KB
```

Every page is read back after erasing and the commit is abandoned if one did
not clear, so a failure names the address rather than silently writing a
corrupt image.

Things worth exercising after a change, all of which must leave a working
board: unplug mid-copy; copy an unrelated file; copy a `.uf2` built for
`0x08000000`; delete a file from the volume.
