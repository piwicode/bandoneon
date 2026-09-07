cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME main-g474)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C, and CXX languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# The Release image lives above the UF2 bootloader at 0x08008000, so its vector
# table has to move with it. These are the knobs the generated
# Core/Src/system_stm32g4xx.c already has: with USER_VECT_TAB_ADDRESS defined,
# SystemInit() sets SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET. Using them keeps
# the relocation out of the generated file, so nothing has to be edited outside
# its USER CODE tags.
#
# This has to be directory scope, and has to come before add_subdirectory():
# system_stm32g4xx.c is listed in STM32_Drivers_Src, so it is compiled into the
# STM32_Drivers object library rather than into this executable, and a
# target_compile_definitions() on ${CMAKE_PROJECT_NAME} would never reach it.
#
# Debug defines neither and keeps VTOR at 0x08000000, matching
# STM32G474XX_FLASH.ld. The offset must equal BOOT_SIZE in ../boot-g474/memmap.h.
# The bootloader also sets VTOR itself before jumping, so the application is
# safe either way; this makes the image correct on its own terms.
add_compile_definitions(
    $<$<CONFIG:Release>:USER_VECT_TAB_ADDRESS>
    $<$<CONFIG:Release>:VECT_TAB_OFFSET=0x8000U>
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    app/bellow.c
    app/bellow_classify.c
    app/bellow_phys.c
    app/buttons.c
    app/keyboard.cpp
    app/app.cc
    app/keyboard_layout.c
    app/midi.c
    app/pedals.c
    app/report.c
    Core/Src/swo.c
    properties/properties.c
    properties/properties_console.c
    ../third_party/microrl/src/microrl.c
    ../common/console/console.c
    ../common/hall_report/hall_report.c
    usb/usb_descriptors.c
    usb/usb_app.c
    # TinyUSB device stack with MIDI on the STM32 fsdev port
    ../third_party/tinyusb/src/tusb.c
    ../third_party/tinyusb/src/common/tusb_fifo.c
    ../third_party/tinyusb/src/device/usbd.c
    ../third_party/tinyusb/src/device/usbd_control.c
    ../third_party/tinyusb/src/class/midi/midi_device.c
    ../third_party/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
    ../third_party/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
)

# Force include our config.h for every TU that sees microrl.h, so they all agree
# on sizeof(microrl_t). microrl.h quote-includes the third-party config.h sitting
# next to it; without this, console.c (which allocates microrl_t console_rl) would
# size the struct from the upstream defaults while microrl.c sizes it from ours,
# and microrl writing past console_rl's smaller allocation corrupts .bss.
set_source_files_properties(
    ../third_party/microrl/src/microrl.c
    ../common/console/console.c
    PROPERTIES
    COMPILE_FLAGS "-include${CMAKE_CURRENT_SOURCE_DIR}/../common/console/config.h"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    .
    app
    properties
    ../third_party/microrl/src
    ../common/console
    ../common/spi_link
    ../common/hall_report
    usb
    ../third_party/tinyusb/src
    # memmap.h: the flash split and the DFU hand-off flag, shared with the
    # bootloader so the two cannot disagree about them.
    ../boot-g474
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    CFG_TUSB_MCU=OPT_MCU_STM32G4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
