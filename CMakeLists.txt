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
set(BOARD genie_idcm/88mw320/cpu0 CACHE STRING "Zephyr board")


# Load Zephyr build system
set(MW320_SDK_DIR ${CMAKE_SOURCE_DIR}/mw320_sdk CACHE PATH "MW320 SDK directory" FORCE)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sesame)

# --- Linker settings ---
zephyr_linker_sources(DATA_SECTIONS custom_bss.ld)
zephyr_linker_sources(RODATA custom_rodata.ld)
target_link_libraries(app PRIVATE -lm)

# Application Sources
target_sources(app PRIVATE
    src/config_manager.c
    src/leds.c
    src/main.c
    src/httpd.c
    src/network_manager.c
)

zephyr_include_directories(include)
zephyr_include_directories(${MW320_SDK_DIR}/components/flash/mflash/mw320)

# MW320 SDK sources are provided by the mw320_sdk Zephyr module
# via ZEPHYR_EXTRA_MODULES

include(FetchContent)
FetchContent_Declare(nanopb
    GIT_REPOSITORY https://github.com/nanopb/nanopb.git
    GIT_TAG        0.4.9.1
)
FetchContent_MakeAvailable(nanopb)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED)

# Generate nanopb sources
nanopb_generate_cpp(TARGET proto_app RELPATH proto proto/app_config.proto proto/api.proto)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(nanopb PRIVATE zephyr_interface)
target_link_libraries(app PRIVATE proto_app nanopb)

# Custom drivers (added directly in application mode)
zephyr_library_sources_ifdef(CONFIG_UART_MW320 drivers/serial/uart_mw320.c)
zephyr_library_sources_ifdef(CONFIG_GPIO_MW320 drivers/gpio/gpio_mw320.c)
zephyr_library_sources_ifdef(CONFIG_ENTROPY_MW320 drivers/entropy/entropy_mw320.c)
zephyr_library_sources_ifdef(CONFIG_FLASH drivers/flash/flash_mw320.c)
if(CONFIG_WIFI_MW320)
    add_subdirectory(drivers/wifi/mw320)
endif()
zephyr_library_sources_ifdef(CONFIG_PINCTRL_MW320 drivers/pinctrl/pinctrl_mw320.c)
zephyr_library_sources_ifdef(CONFIG_WATCHDOG_MW320 drivers/watchdog/wdt_mw320.c)

# Convert ELF to firmware binary for flashing (XIP only, RAM build uses AXF directly)
if(NOT CONFIG_BOARD_GENIE_IDCM_RAM)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/sesame.bin
    COMMAND make -C ${CMAKE_SOURCE_DIR}/tools/axf2firmware
    COMMAND ${CMAKE_SOURCE_DIR}/tools/axf2firmware/axf2firmware
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
            -DBOARD=genie_idcm_ram/88mw320/cpu0
            -DZEPHYR_TOOLCHAIN_VARIANT=${ZEPHYR_TOOLCHAIN_VARIANT}
            -S "${CMAKE_SOURCE_DIR}" -B "${CMAKE_BINARY_DIR}/ram-build"
    BUILD_COMMAND
        "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}/ram-build"
    INSTALL_COMMAND ""
    BUILD_ALWAYS    TRUE
)

endif()
