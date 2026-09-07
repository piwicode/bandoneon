"""Verify the flash split that the UF2 bootloader and the main board
application share (code/boot-g474/), and the .ioc settings the bootloader
depends on.

The memory map is repeated in five places that cannot include each other — two
linker scripts, a C header, a CMake define and a Python default. Nothing in the
build fails if they drift apart: the firmware simply lands at the wrong address
and the board stops booting, which is only discoverable with an ST-Link. These
tests are the thing that catches it.

Layout under test (see code/boot-g474/memmap.h):

    0x08000000  bootloader          32 KB
    0x08008000  application         94 KB   Release build
    0x0801F800  properties           2 KB   reserved
    0x08020000  end

The Debug build of main-g474 is deliberately outside that split: it links at
0x08000000 over the bootloader, because at -O0 it is ~116 KB and cannot fit
above one.
"""

import re
import unittest
from pathlib import Path

from ioc_parser import parse_ioc

REPO_ROOT = Path(__file__).resolve().parents[2]

BOOT_DIR = REPO_ROOT / "code" / "boot-g474"
MAIN_DIR = REPO_ROOT / "code" / "main-g474"

FLASH_ORIGIN = 0x08000000
FLASH_TOTAL = 128 * 1024


def parse_memory_regions(path: Path) -> dict[str, tuple[int, int]]:
    """Map region name -> (origin, length) from a linker script MEMORY block.

    Lengths are written as "94K" or "128K - 32"; both forms are evaluated.
    """
    text = path.read_text()
    block = re.search(r"^MEMORY\s*^\{(.*?)^\}", text, re.MULTILINE | re.DOTALL)
    assert block, f"no MEMORY block in {path}"

    regions: dict[str, tuple[int, int]] = {}
    entry_re = re.compile(
        r"^\s*(\w+)\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*([^,]+),\s*LENGTH\s*=\s*(.+?)\s*$",
        re.MULTILINE,
    )
    for name, origin, length in entry_re.findall(block.group(1)):
        regions[name] = (_evaluate(origin), _evaluate(length))
    return regions


def _evaluate(expr: str) -> int:
    """Evaluate a linker-script size expression: hex, decimal, K/M suffixes,
    and the + and - between them."""
    normalised = re.sub(r"\b(0x[0-9a-fA-F]+|\d+)([KM])\b",
                        lambda m: str(int(m.group(1), 0) *
                                      (1024 if m.group(2) == "K" else 1024 * 1024)),
                        expr.strip())
    if not re.fullmatch(r"[0-9a-fA-FxX+\-*\s]+", normalised):
        raise ValueError(f"unexpected linker expression: {expr!r}")
    return eval(normalised, {"__builtins__": {}}, {})  # noqa: S307 - fixed grammar above


def memmap_defines() -> dict[str, int]:
    """The #define'd integers from code/boot-g474/memmap.h, evaluated."""
    text = (BOOT_DIR / "memmap.h").read_text()
    raw = dict(re.findall(r"^#define\s+(\w+)\s+(.+?)\s*$", text, re.MULTILINE))

    values: dict[str, int] = {}
    for name, expr in raw.items():
        cleaned = re.sub(r"\b(0[xX][0-9a-fA-F]+|\d+)U?L?L?\b", r"\1", expr)
        # Substitute names defined earlier, innermost first.
        for other, value in values.items():
            cleaned = re.sub(rf"\b{other}\b", str(value), cleaned)
        if re.fullmatch(r"[0-9a-fA-FxX()+\-*\s]+", cleaned):
            try:
                values[name] = eval(cleaned, {"__builtins__": {}}, {})  # noqa: S307
            except (SyntaxError, NameError, TypeError):
                pass
    return values


class TestMemmapHeader(unittest.TestCase):
    """memmap.h is the single source of truth; check it is self-consistent
    before comparing anything against it."""

    def test_regions_tile_the_flash(self):
        m = memmap_defines()
        self.assertEqual(m["BOOT_BASE"], FLASH_ORIGIN)
        self.assertEqual(m["APP_BASE"], m["BOOT_BASE"] + m["BOOT_SIZE"],
                         "application must start immediately after the bootloader")
        self.assertEqual(m["PROPS_BASE"], m["APP_BASE"] + m["APP_SIZE"])
        self.assertEqual(m["PROPS_BASE"] + m["PROPS_SIZE"], FLASH_ORIGIN + FLASH_TOTAL,
                         "the three regions must exactly fill the 128 KB of flash")

    def test_app_base_is_vector_table_aligned(self):
        """SCB->VTOR ignores the low 7 bits, and the table must be aligned to a
        power of two at least its own size (~0x1D8 here), so 0x200."""
        m = memmap_defines()
        self.assertEqual(m["APP_BASE"] % 0x200, 0)

    def test_boot_flag_is_top_of_ram(self):
        m = memmap_defines()
        self.assertEqual(m["BOOT_FLAG_ADDR"], 0x2001FFE0,
                         "the flag word must sit in the 32 bytes carved out of "
                         "the RAM region by both linker scripts")

    def test_uf2_family_is_stm32g4(self):
        """From microsoft/uf2 utils/uf2families.json. A wrong id means the
        bootloader silently discards every block of a correct image."""
        self.assertEqual(memmap_defines()["UF2_FAMILY_ID"], 0x4C71240A)


class TestLinkerScripts(unittest.TestCase):
    """The three linker scripts must implement what memmap.h describes."""

    def test_bootloader_region(self):
        m = memmap_defines()
        flash = parse_memory_regions(BOOT_DIR / "STM32G474XX_FLASH.ld")["FLASH"]
        self.assertEqual(flash, (m["BOOT_BASE"], m["BOOT_SIZE"]),
                         "boot-g474 linker script disagrees with BOOT_BASE/BOOT_SIZE")

    def test_release_application_region(self):
        m = memmap_defines()
        flash = parse_memory_regions(MAIN_DIR / "STM32G474XX_FLASH_APP.ld")["FLASH"]
        self.assertEqual(flash, (m["APP_BASE"], m["APP_SIZE"]),
                         "STM32G474XX_FLASH_APP.ld disagrees with APP_BASE/APP_SIZE")

    def test_bootloader_and_application_do_not_overlap(self):
        boot = parse_memory_regions(BOOT_DIR / "STM32G474XX_FLASH.ld")["FLASH"]
        app = parse_memory_regions(MAIN_DIR / "STM32G474XX_FLASH_APP.ld")["FLASH"]
        self.assertLessEqual(boot[0] + boot[1], app[0],
                             "the bootloader region runs into the application region")
        self.assertLessEqual(app[0] + app[1], FLASH_ORIGIN + FLASH_TOTAL)

    def test_debug_map_spans_flash_below_the_properties_page(self):
        """The Debug image intentionally replaces the bootloader, but must still
        leave the reserved trailing page alone."""
        m = memmap_defines()
        flash = parse_memory_regions(MAIN_DIR / "STM32G474XX_FLASH.ld")["FLASH"]
        self.assertEqual(flash[0], FLASH_ORIGIN)
        self.assertEqual(flash[0] + flash[1], m["PROPS_BASE"])

    def test_boot_flag_region_matches_in_every_script(self):
        """All three images must agree on where the hand-off word lives, and
        none of them may let the stack or .bss reach it."""
        m = memmap_defines()
        for script in ("boot-g474/STM32G474XX_FLASH.ld",
                       "main-g474/STM32G474XX_FLASH.ld",
                       "main-g474/STM32G474XX_FLASH_APP.ld"):
            with self.subTest(script=script):
                regions = parse_memory_regions(REPO_ROOT / "code" / script)
                self.assertIn("BOOTFLAG", regions,
                              "no BOOTFLAG region; the stack would overwrite the flag")
                origin, length = regions["BOOTFLAG"]
                self.assertEqual(origin, m["BOOT_FLAG_ADDR"])
                self.assertGreaterEqual(length, 4)

                ram_origin, ram_length = regions["RAM"]
                self.assertEqual(ram_origin + ram_length, origin,
                                 "_estack must stop where the flag word starts")


class TestBuildConfiguration(unittest.TestCase):
    """The relocation only works if the build agrees with the linker scripts."""

    def test_vect_tab_offset_matches_app_base(self):
        m = memmap_defines()
        text = (MAIN_DIR / "CMakeLists.txt").read_text()

        match = re.search(r"VECT_TAB_OFFSET=(0[xX][0-9a-fA-F]+)U?", text)
        self.assertIsNotNone(match, "VECT_TAB_OFFSET is not defined in main-g474/CMakeLists.txt")
        self.assertEqual(int(match.group(1), 16), m["APP_BASE"] - FLASH_ORIGIN,
                         "VECT_TAB_OFFSET does not match APP_BASE; the relocated "
                         "image would vector into the bootloader's table")

        self.assertIn("USER_VECT_TAB_ADDRESS", text,
                      "system_stm32g4xx.c only writes SCB->VTOR when "
                      "USER_VECT_TAB_ADDRESS is defined")

    def test_vect_tab_defines_reach_the_system_file(self):
        """system_stm32g4xx.c is compiled into the STM32_Drivers object library,
        not into the executable, so a target_compile_definitions() on the
        executable would never reach it. The defines have to be directory scope
        and precede add_subdirectory()."""
        text = (MAIN_DIR / "CMakeLists.txt").read_text()

        add_defs = text.find("add_compile_definitions")
        add_subdir = text.find("add_subdirectory(cmake/stm32cubemx)")
        self.assertNotEqual(add_defs, -1,
                            "VECT_TAB_* must be set with add_compile_definitions()")
        self.assertLess(add_defs, add_subdir,
                        "add_compile_definitions() must come before "
                        "add_subdirectory(cmake/stm32cubemx) to reach system_stm32g4xx.c")

        drivers = (MAIN_DIR / "cmake" / "stm32cubemx" / "CMakeLists.txt").read_text()
        self.assertIn("system_stm32g4xx.c", drivers)

    def test_release_links_against_the_relocated_script(self):
        toolchain = (MAIN_DIR / "cmake" / "gcc-arm-none-eabi.cmake").read_text()

        # The values embed escaped quotes around ${CMAKE_SOURCE_DIR}, so match
        # to end of line rather than to the closing quote.
        release = re.search(r"^set\(CMAKE_EXE_LINKER_FLAGS_RELEASE\s+(.*)$",
                            toolchain, re.MULTILINE)
        debug = re.search(r"^set\(CMAKE_EXE_LINKER_FLAGS_DEBUG\s+(.*)$",
                          toolchain, re.MULTILINE)
        self.assertIsNotNone(release, "Release config sets no linker script")
        self.assertIsNotNone(debug, "Debug config sets no linker script")
        self.assertIn("STM32G474XX_FLASH_APP.ld", release.group(1),
                      "Release must link with the relocated map")
        self.assertIn("STM32G474XX_FLASH.ld", debug.group(1))
        self.assertNotIn("STM32G474XX_FLASH_APP.ld", debug.group(1),
                         "Debug must link at 0x08000000, not above the bootloader")

        # A -T left in the common flags would be passed to both configurations
        # and silently win or conflict.
        common = re.findall(r"^set\(CMAKE_EXE_LINKER_FLAGS\s+.*$", toolchain, re.MULTILINE)
        self.assertFalse([line for line in common if "-T " in line],
                         "the linker script must not be in the common linker flags")

    def test_uf2_tool_defaults_to_the_application_base(self):
        m = memmap_defines()
        text = (REPO_ROOT / "code" / "tool" / "uf2.py").read_text()

        base = re.search(r"^DEFAULT_BASE\s*=\s*(0[xX][0-9a-fA-F]+)", text, re.MULTILINE)
        self.assertIsNotNone(base, "uf2.py has no DEFAULT_BASE")
        self.assertEqual(int(base.group(1), 16), m["APP_BASE"],
                         "uf2.py would package the image for the wrong address")

        family = re.search(r"^STM32G4_FAMILY_ID\s*=\s*(0[xX][0-9a-fA-F]+)", text, re.MULTILINE)
        self.assertIsNotNone(family)
        self.assertEqual(int(family.group(1), 16), m["UF2_FAMILY_ID"])

    def test_flash_release_recipe_writes_at_the_application_base(self):
        m = memmap_defines()
        justfile = (MAIN_DIR / "justfile").read_text()

        self.assertIn("dfu:", justfile, "no `just dfu` recipe")
        self.assertIn("flash_release:", justfile, "no `just flash_release` recipe")

        match = re.search(r"--address\s+(0[xX][0-9a-fA-F]+)", justfile)
        self.assertIsNotNone(match, "flash_release must pass --address to flash.py")
        self.assertEqual(int(match.group(1), 16), m["APP_BASE"])


class TestDfuButton(unittest.TestCase):
    """The bootloader reads SW_FN2 to decide whether to enter DFU mode. It
    configures the pin itself, so what matters here is that the .ioc keeps the
    net on the pin the bootloader hardcodes (boot-g474/main.c, DFU_BUTTON_PIN).
    """

    def test_sw_fn2_is_pb4(self):
        settings = parse_ioc(MAIN_DIR / "main-g474.ioc")
        self.assertEqual(settings.get("PB4.GPIO_Label"), "SW_FN2",
                         "SW_FN2 moved off PB4; boot-g474/main.c reads PB4")

        source = (BOOT_DIR / "main.c").read_text()
        self.assertIn("GPIO_PIN_4", source)
        self.assertIn("GPIOB", source)

    def test_dead_battery_pulldown_is_disabled(self):
        """PB4 is UCPD1_CC2 and carries a 5.1 kOhm dead-battery pull-down gated
        by PA10, which is this board's USART1_RX and is held high by the idle
        ST-Link VCP. That pull-down beats the internal pull-up, so without
        clearing it the button reads as permanently pressed and the board never
        leaves DFU mode. See the STM32G474CB datasheet pin table, note 6."""
        source = (BOOT_DIR / "main.c").read_text()
        self.assertIn("HAL_PWREx_DisableUCPDDeadBattery", source,
                      "the bootloader must clear the UCPD dead-battery pull-down "
                      "before reading SW_FN2 on PB4")


if __name__ == "__main__":
    unittest.main()
