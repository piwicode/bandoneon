
## Connecting the debugger probe

Plug the ST-Link into a USB port and verify the device is detected:

```
$ lsusb | grep STLINK
Bus 001 Device 070: ID 0483:3754 STMicroelectronics STLINK-V3
```

Install [stlink-tools](https://github.com/stlink-org/stlink) from source, in a
directory next to your `bandolibre` checkout:

```
sudo apt remove stlink-tools
sudo apt install build-essential cmake libusb-1.0-0-dev
git clone --depth 1 --branch testing https://github.com/stlink-org/stlink
cd stlink
git apply ../bandolibre/documentation/0001-fix-st-trace-fix-SWO-trace-on-STLINK-V3-HS-bulk-endp.patch
make release && sudo make install && sudo ldconfig
```

Once the ST-Link is plugged into the board:

```
$ st-info --probe
Found 1 stlink programmers
  version:    V3J15
  serial:     002F00413235510637333439
  flash:      131072 (pagesize: 2048)
  sram:       131072
  chipid:     0x469
  dev-type:   STM32G47x_G48x
```

## Install build prerequisites

The ARM toolchain and the build backend:

```
sudo apt install gcc-arm-none-eabi ninja-build
```

I use [`just`](https://just.systems/) to run commands from a configuration file
such as `code/main-g474/justfile`:

```
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to DEST
```

or

```
cargo install just
```

## Build the firmware

Generate the build system once, then build:

```
cd code/main-g474
just init_build_release build_release
```

## Flash the firmware

Flashing wing firmware onto a main board — or the reverse — drives pins against
the connected hardware and can damage the boards, so a chip must be registered
before it can be flashed. `code/tool/boards.csv` maps each STM32 unique device
ID to its board type, and is checked before every flash.

Register a board once, before its first flash:

```
cd code/main-g474
just registry_add
```

Then flash with:

```
cd code/main-g474
just flash_release
```

`just flash_release` programs every connected ST-Link whose chip is registered
as that board type, so both wings can be flashed in a single command.

The same recipe apply in `code/wing-g474`.

## Main board: two images, two memory maps

The main board carries a UF2 bootloader at `0x08000000` so it can be updated by
copying a file onto a USB drive — see
[firmware_update.md](firmware_update.md) and
[code/boot-g474/README.md](../code/boot-g474/README.md). That splits its 128 KB
of flash, and the Debug build does not fit in the application's share of it:
at `-O0` it is ~116 KB against the 94 KB available above the bootloader. So the
two configurations link differently, and the recipes differ accordingly:

| Recipe | Builds | Links at | Bootloader after |
|---|---|---|---|
| `just flash` | Debug, `-O0 -g3` | `0x08000000` | **erased** — this image takes its place |
| `just flash_release` | bootloader + Release, `-Os -flto` | `0x08008000` | installed |
| `just dfu` | Release, packaged as `.uf2` | `0x08008000` | (no flashing; copy over USB) |

Day-to-day development is unchanged: `just flash` still gives a full `-O0 -g3`
image with all its debug information, it just has no DFU drive. Run
`just flash_release` when you want the bootloader back, and once per board
before handing it to anyone who will update it over USB.

`just dfu` is the one to use for a release: it builds the Release
configuration and writes `build/Release/main-g474.uf2`.

None of this affects `code/wing-g474`, which has a single memory map.


## Read UART debug console

Characters written with `printf` (via `_write` retargeted to `HAL_UART_Transmit`) are sent over USART to the STLink VCP bridge, which forwards them to a host `/dev/ttyACMx` device.

Use `tio` to read it — it auto-reconnects across resets, unlike `screen` or `cat`:

```bash
tio /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_*
```

The glob resolves to the correct `/dev/ttyACMx` regardless of what other USB serial devices are present. Or via the justfile recipe:

```bash
just console
```

Press `ctrl-t q` to quit.

