# Recipes shared by the board justfiles (main-g474, wing-g474).
#
# This file is imported, never run directly: `just` resolves recipes against the
# directory of the justfile it was invoked from, so `../tool/...` and
# `$(basename $(pwd))` below refer to the importing board.

# Listing of the available recipes. Each board justfile makes this its first
# recipe under the name `default`: `just` with no arguments runs the first recipe
# of the justfile it was invoked from, never one reached through `import`.
_list:
    @just --list

# Generate build system into build/Debug. Re-run after adding/removing source files or changing CMakeLists.txt.
init_build:
    cmake --preset Debug

build:
    cmake --build --preset Debug

# Generate build system into build/Release. Re-run after adding/removing source files or changing CMakeLists.txt.
init_build_release:
    cmake --preset Release

build_release:
    cmake --build --preset Release

# Record this board's chip UID in the registry. Run once per board before the first flash.
registry_add:
    ../tool/registry.py add $(basename $(pwd))

# Flash the Debug build over SWD, at 0x08000000. Each board documents it as its own `flash`.
_flash:
    name=$(basename $(pwd)) && \
    arm-none-eabi-objcopy -O binary build/Debug/$name.elf build/Debug/$name.bin && \
    ../tool/flash.py "$name" build/Debug/$name.bin

# Reset the board via STLink
reset:
    st-flash reset
