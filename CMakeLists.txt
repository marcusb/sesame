# Zephyr Standalone Application CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)

# ZEPHYR_EXTRA_MODULES is required to trigger zephyr_module.py, which generates
# kconfig_module_dirs.cmake. The mw320_sdk module provides SDK driver sources.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/mw320_sdk)

# Custom board and SOC root directories (freestanding application)
# Zephyr looks for $BOARD_ROOT/boards/<arch>/<board> and $SOC_ROOT/soc/<vendor>/<soc>
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Board root directory")
set(SOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "SOC root directory")

# Board selection (use marvell_mw302 for flash/XIP, marvell_mw302_ram for RAM load)
set(BOARD marvell_mw302/88mw320/cpu0 CACHE STRING "Zephyr board")

# Toolchain defaults
set(ZEPHYR_TOOLCHAIN_VARIANT "gnuarmemb" CACHE STRING "Zephyr toolchain variant")
set(GNUARMEMB_TOOLCHAIN_PATH "/usr" CACHE PATH "GNU ARM Embedded toolchain path")

# Load Zephyr build system
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sesame)

# Application Sources
target_sources(app PRIVATE
    src/main.c
    # We will progressively add files as we port them
    # src/network_manager.c (Needs rewrite)
    # src/ota.c (Needs rewrite)
)

zephyr_include_directories(include)

# MW320 SDK sources are provided by the mw320_sdk Zephyr module
# via ZEPHYR_EXTRA_MODULES

# Custom drivers (added directly in application mode)
zephyr_library_sources_ifdef(CONFIG_UART_MW320 drivers/serial/uart_mw320.c)
zephyr_library_sources_ifdef(CONFIG_GPIO_MW320 drivers/gpio/gpio_mw320.c)
zephyr_library_sources_ifdef(CONFIG_ENTROPY_MW320 drivers/entropy/entropy_mw320.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_MW320 drivers/wifi/mw320/wifi_mw320.c)

# Convert ELF to firmware binary for flashing (XIP only, RAM build uses AXF directly)
if(NOT CONFIG_BOARD_MARVELL_MW302_RAM)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/sesame.bin
    COMMAND ${CMAKE_SOURCE_DIR}/build-native/axf2firmware
            ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf
            ${CMAKE_BINARY_DIR}/sesame.bin
    DEPENDS zephyr_final ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf
    COMMENT "Converting zephyr.elf -> sesame.bin"
    VERBATIM
)
add_custom_target(sesame_bin ALL DEPENDS ${CMAKE_BINARY_DIR}/sesame.bin)

# --- Multi-variant Build Support ---
# Build the RAM variant automatically within the main build folder
include(ExternalProject)
ExternalProject_Add(ram_variant
    SOURCE_DIR      "${CMAKE_SOURCE_DIR}"
    BINARY_DIR      "${CMAKE_BINARY_DIR}/ram-build"
    CONFIGURE_COMMAND
        "${CMAKE_COMMAND}" -G Ninja
            -DBOARD=marvell_mw302_ram/88mw320/cpu0
            -DZEPHYR_TOOLCHAIN_VARIANT=${ZEPHYR_TOOLCHAIN_VARIANT}
            -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}/ram-build"
    BUILD_COMMAND
        "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}/ram-build"
    INSTALL_COMMAND ""
    BUILD_ALWAYS    TRUE
)

ExternalProject_Add_Step(ram_variant copy_binaries
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/ram-build/zephyr/zephyr.elf" "${CMAKE_BINARY_DIR}/sesame_ram.axf"
    DEPENDEES build
)
endif()
