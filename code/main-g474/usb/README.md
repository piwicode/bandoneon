# USB stack (TinyUSB)

The main board enumerates as a full-speed MIDI USB device built on
[TinyUSB](../../third_party/tinyusb) (git submodule, `code/third_party/tinyusb`):

| | |
|---|---|
| VID / PID | `0x0483` / `0xA5B4` (ST's vendor id, product id sublicensed from ST) |
| Manufacturer | L'Atelier du bandonéon libre |
| Product | Bandolibre |
| Serial | 96-bit chip UID, 24 hex digits |
| Interfaces | MIDI only |

`0x0483` is STMicroelectronics' USB-IF vendor id; `0xA5B4` is a product id
sublicensed from ST for this project, valid only for a product built on ST
silicon (STM32G474CBT6) and only for Bandolibre. The sublicense does not carry
USB-IF certification, so the instrument must not display the USB logo. Earlier
revisions used TinyUSB's development ids `0xCafe` / `0x4008` (`0x4000 |`
interface bitmap, computed by a `PID_MAP` macro); that macro is gone, the
product id is now fixed.

The board also carries a [UF2 bootloader](../../boot-g474/README.md), which
enumerates while firmware is being updated — mass storage, product string
"Bandolibre DFU". It shares this VID/PID (one product id covers the product)
and is told apart by `bcdDevice`: `0x0100` here, `0x0200` in the bootloader.
The revisions must differ because hosts cache a driver per VID/PID/revision and
the two devices expose different classes.

## Files

- `tusb_config.h` — TinyUSB feature configuration (device-only, MIDI).
- `usb_descriptors.c` — device/configuration/string descriptors. Strings are
  UTF-8 and converted to UTF-16 on the fly, so accented names work.
- `usb_app.c/.h` — glue: `usb_app_init()` starts the stack (called from
  `main()` after `MX_USB_PCD_Init()`), `usb_app_task()` runs `tud_task()` from
  the main loop, and the three USB IRQ handlers (which override the weak
  symbols in `startup_stm32g474xx.s`).

The TinyUSB sources, include paths, and the `CFG_TUSB_MCU=OPT_MCU_STM32G4`
define are listed in the user sections of [CMakeLists.txt](../CMakeLists.txt).

## How it coexists with CubeMX

CubeMX still generates `MX_USB_PCD_Init()` (HAL PCD), which enables the USB
peripheral clock via `HAL_PCD_MspInit()`. TinyUSB's `dcd_init()` then resets
and takes over the peripheral; `HAL_PCD_Start()` is never called, so the HAL
never enables the DP pull-up or the USB interrupts itself. Don't remove the
USB peripheral from the `.ioc`.

**USB clock**: the board is crystal-less, so the `.ioc` selects HSI48 as USB
kernel clock, trimmed by the CRS against USB SOF (the PLL alternative is fed
by HSI16, ±1%, outside the USB FS ±0.25% budget). All of this lives in the
generated `SystemClock_Config()`. These `.ioc` invariants — HSI48 + CRS, USB
device FS enabled, USB interrupts *not* enabled in the NVIC tab (TinyUSB owns
the handlers) — are checked by `code/tests/test_usb_config.py` (`just test`
from `code/tests`).


## Testing

```sh
lsusb -d 0483:a5b4       # device present (bcdDevice 0200 = bootloader, see boot-g474)
amidi -l                 # MIDI port listed as "Bandolibre"
aseqdump -p Bandolibre &   # then run `midi [note]` in the console to test
```

The `midi [note]` console command (via USART1/ST-Link) sends a note-on/note-off
pair (default 69 = A4) for end-to-end checking over USB MIDI.
