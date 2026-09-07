cmake_minimum_required(VERSION 3.22)

#
# Bandolibre UF2 bootloader.
#
# Unlike main-g474 and wing-g474 this is not a CubeMX project: there is no .ioc
# and no generated Core/. It configures its handful of peripherals directly, so
# there is no second generated project to keep in sync with the hardware.
#
# It does reuse the main board's vendored HAL and startup file rather than
# carrying a second copy, the same way the boards reuse ../common. Only the HAL
# modules the bootloader actually needs are compiled — pulling in the whole
# STM32_Drivers object library would cost several KB of the 32 KB budget.
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME boot-g474)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../main-g474)
set(HAL_DIR ${MAIN_DIR}/Drivers/STM32G4xx_HAL_Driver)

add_executable(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    main.c
    msc_disk.c
    flash_write.c
    usb_descriptors.c
    debug.c

    # Startup file and HAL, shared with the main board rather than duplicated
    ${MAIN_DIR}/startup_stm32g474xx.s
    ${MAIN_DIR}/Core/Src/system_stm32g4xx.c
    ${MAIN_DIR}/Core/Src/sysmem.c     # newlib _sbrk() stub, needed to link printf
    ${MAIN_DIR}/Core/Src/syscalls.c   # newlib _isatty/_read/_lseek/_close stubs, ditto
    ${HAL_DIR}/Src/stm32g4xx_hal.c
    ${HAL_DIR}/Src/stm32g4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32g4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_flash_ramfunc.c
    ${HAL_DIR}/Src/stm32g4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32g4xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_uart.c   # debug.c's USART1 TX console
    ${HAL_DIR}/Src/stm32g4xx_hal_uart_ex.c
    ${HAL_DIR}/Src/stm32g4xx_hal_dma.c    # HAL_UART_Init() references DMA symbols even unused

    # TinyUSB device stack with MSC on the STM32 fsdev port
    ../third_party/tinyusb/src/tusb.c
    ../third_party/tinyusb/src/common/tusb_fifo.c
    ../third_party/tinyusb/src/device/usbd.c
    ../third_party/tinyusb/src/device/usbd_control.c
    ../third_party/tinyusb/src/class/msc/msc_device.c
    ../third_party/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
    ../third_party/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    .
    ${MAIN_DIR}/Core/Inc
    ${HAL_DIR}/Inc
    ${MAIN_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${MAIN_DIR}/Drivers/CMSIS/Include
    ../third_party/tinyusb/src
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32G474xx
    CFG_TUSB_MCU=OPT_MCU_STM32G4
)

# newlib's retarget hooks (_write, _sbrk, _isatty, _read, _lseek, _close,
# _fstat — defined in main-g474/Core/Src/{syscalls,sysmem}.c and debug.c) are
# only ever called from *inside* the prebuilt, non-LTO libg_nano.a — nothing
# in this project's own LTO-visible code calls them by name (they're reached
# indirectly through printf()). LTO's whole-program analysis can't see that
# call edge, concludes they look unused, and --gc-sections discards them
# before the archive scan that actually needs them ever runs — a Release-only
# link failure (`undefined reference to _write` etc.), not reproducible in
# Debug, which doesn't use LTO. Forcing them as required roots here fixes it
# without touching the shared main-g474 files that also use --gc-sections+LTO
# but never hit this, because their printf() calls in Core/Src/main.c are
# themselves LTO-visible and keep the whole chain alive by inference.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--undefined=_write
    -Wl,--undefined=_sbrk
    -Wl,--undefined=_isatty
    -Wl,--undefined=_read
    -Wl,--undefined=_lseek
    -Wl,--undefined=_close
    -Wl,--undefined=_fstat
)
