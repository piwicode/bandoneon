# Updating the firmware

The main board can be updated over the same USB cable it uses for MIDI. No
programmer, no cable other than the one you already have, and nothing to
install.

## Doing the update

1. Unplug the instrument's USB cable.
2. Hold down **FN2**, the rightmost of the three function buttons.
3. Plug the USB cable back in, keeping FN2 held until a drive appears.
4. A drive called **BANDOLIBRE** shows up, the way a USB stick would. Its LED
   blinks slowly while it waits.
5. Copy `main-g474.uf2` onto that drive.
6. The LED blinks faster while the file transfers, then stays lit for about a
   second while the firmware is written. The drive disappears and the
   instrument restarts on the new firmware.

That is the whole process. The drive is not real storage — it exists only to
receive the firmware file, so do not expect files copied there to stay.

## If something goes wrong

**The copy was interrupted, or the instrument lost power partway through.**
Nothing is broken. The new firmware is only written after the whole file has
arrived, so an interrupted copy is simply discarded. Start again from step 1.

**The instrument does not start, and shows the BANDOLIBRE drive on its own
without FN2 held.** This is the recovery path working as intended: the
bootloader checks the firmware before starting it, and falls back to the drive
if it is missing or damaged. Copy the `.uf2` again.

**Nothing happens when you copy the file.** The file has to be a `.uf2` built
for this board. A `.bin`, a `.hex`, or a `.uf2` for another project is ignored —
that is deliberate, so an accidental drag cannot damage the instrument.

**The drive does not appear at all.** Make sure FN2 is held *before* the cable
goes in and stays held for a second or two after. If the instrument is running
normally, you can also reach the drive from the debug console (below).

## Rebooting into the drive without unplugging

If you have the console open (see
[programming_boards.md](programming_boards.md)), the `dfu` command does the
same thing:

```
> dfu
Rebooting into DFU mode; the BANDOLIBRE drive will appear.
```

## For developers: producing the .uf2

```sh
cd code/main-g474
just dfu            # -> build/Release/main-g474.uf2
```

This builds the Release configuration — the one linked to sit above the
bootloader — and packages it. `just build` alone produces a Debug image, which
is a different memory map and **cannot** be distributed this way; `just dfu`
always builds Release for you.

A board that has never had the bootloader installed needs one ST-Link session
first:

```sh
cd code/main-g474
just flash_release  # bootloader + application, over SWD
```

After that it can be updated over USB for good. Note that plain `just flash`
installs the Debug image *over* the bootloader, so a development board loses
its DFU drive until the next `just flash_release`. The reason for the split is
that the Debug build is ~116 KB — too large to fit above a bootloader in the
128 KB this chip has.

The full picture, including the flash layout and why the format is UF2, is in
[code/boot-g474/README.md](../code/boot-g474/README.md).
