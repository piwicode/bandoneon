"""Verify the USB identity the two firmware images declare.

The vendor id is STMicroelectronics' and the product id is sublicensed from ST
for this project: valid only for a product built on ST silicon, and only for
Bandolibre. Neither may drift back to TinyUSB's development ids (0xCafe /
0x4000 | class bitmap), which are fine on a bench and not fine on an instrument
that leaves one.

One product id covers the whole product, so the application (MIDI) and the
bootloader (mass storage) share VID:PID and are told apart by bcdDevice. Hosts
cache a driver per VID/PID/revision, so if the two revisions ever collide, a
host that has seen one device can bind the wrong driver to the other. Nothing
in the build notices; it shows up as a board that enumerates but does not work,
on one machine, after a firmware update.
"""

import re
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

APP_DESCRIPTORS = REPO_ROOT / "code" / "main-g474" / "usb" / "usb_descriptors.c"
BOOT_DESCRIPTORS = REPO_ROOT / "code" / "boot-g474" / "usb_descriptors.c"

USB_VID = 0x0483  # STMicroelectronics
USB_PID = 0xA5B4  # sublicensed to Bandolibre


def defined_value(path: Path, name: str) -> int:
    """Read a `#define <name> <hex>` from a descriptor file."""
    match = re.search(rf"^#define\s+{name}\s+(0[xX][0-9a-fA-F]+)", path.read_text(), re.M)
    if match is None:
        raise AssertionError(f"no #define {name} in {path}")
    return int(match.group(1), 16)


def field_value(path: Path, name: str) -> int:
    """Read a `.<name> = <hex>,` initializer from the device descriptor."""
    match = re.search(rf"^\s*\.{name}\s*=\s*(0[xX][0-9a-fA-F]+),", path.read_text(), re.M)
    if match is None:
        raise AssertionError(f"no .{name} initializer in {path}")
    return int(match.group(1), 16)


class TestLicensedIds(unittest.TestCase):
    def test_application_uses_licensed_ids(self):
        self.assertEqual(defined_value(APP_DESCRIPTORS, "USB_VID"), USB_VID)
        self.assertEqual(defined_value(APP_DESCRIPTORS, "USB_PID"), USB_PID)

    def test_bootloader_uses_licensed_ids(self):
        self.assertEqual(defined_value(BOOT_DESCRIPTORS, "USB_VID"), USB_VID)
        self.assertEqual(defined_value(BOOT_DESCRIPTORS, "USB_PID"), USB_PID)

    def test_descriptors_use_the_defines(self):
        """A literal in the descriptor would pass the checks above and still ship
        the wrong id."""
        for path in (APP_DESCRIPTORS, BOOT_DESCRIPTORS):
            with self.subTest(path=path.name):
                text = path.read_text()
                for field, macro in (("idVendor", "USB_VID"), ("idProduct", "USB_PID")):
                    self.assertIsNotNone(
                        re.search(rf"^\s*\.{field}\s*=\s*{macro},", text, re.M),
                        f".{field} in {path.name} does not use {macro}",
                    )


class TestDeviceRevision(unittest.TestCase):
    def test_revisions_differ(self):
        app = field_value(APP_DESCRIPTORS, "bcdDevice")
        boot = field_value(BOOT_DESCRIPTORS, "bcdDevice")
        self.assertNotEqual(
            app, boot,
            "application and bootloader share VID:PID, so bcdDevice is all that "
            "keeps a host from caching one device's driver for the other",
        )


if __name__ == "__main__":
    unittest.main()
