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

# --- Dynamically set APP_VERSION based on Git commit and VERSION file ---
execute_process(
    COMMAND git rev-parse --short=7 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)


file(READ ${CMAKE_CURRENT_SOURCE_DIR}/VERSION VER_CONTENT)

string(REGEX MATCH "VERSION_MAJOR = ([0-9]*)" _ "${VER_CONTENT}")
set(APP_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "VERSION_MINOR = ([0-9]*)" _ "${VER_CONTENT}")
set(APP_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "PATCHLEVEL = ([0-9]*)" _ "${VER_CONTENT}")
set(APP_PATCHLEVEL ${CMAKE_MATCH_1})

set(APP_TAG "v${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_PATCHLEVEL}")

execute_process(
    COMMAND git rev-list --count ${APP_TAG}..HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_COUNT
    RESULT_VARIABLE GIT_REV_LIST_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT GIT_REV_LIST_RESULT EQUAL 0)
    message(FATAL_ERROR "Tag ${APP_TAG} not found or git error. Please create the tag for the current version in the VERSION file.")
endif()

if(GIT_COMMIT_COUNT GREATER 255)
    message(FATAL_ERROR "Tweak version exceeds 255 (Current: ${GIT_COMMIT_COUNT})! Bump semver in VERSION and create a new tag.")
endif()

math(EXPR MATTER_SOFTWARE_VERSION_INT "(${APP_VERSION_MAJOR} << 24) + (${APP_VERSION_MINOR} << 16) + (${APP_PATCHLEVEL} << 8) + ${GIT_COMMIT_COUNT}")
set(APP_FIRMWARE_VERSION "${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_PATCHLEVEL}-${GIT_COMMIT_HASH}+${GIT_COMMIT_COUNT}")

# Tell MCUboot to sign the image with this version
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/app_version.conf
"CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_PATCHLEVEL}+${GIT_COMMIT_COUNT}\"
")
    list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_BINARY_DIR}/app_version.conf)
# Load Zephyr build system
set(MW320_SDK_DIR ${CMAKE_SOURCE_DIR}/mw320_sdk CACHE PATH "MW320 SDK directory" FORCE)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip/config/zephyr/chip-module)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sesame)
configure_file(src/matter/AppVersion.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/matter/AppVersion.h @ONLY)

if(CONFIG_CHIP)
    get_filename_component(CHIP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip REALPATH)
    include(${CHIP_ROOT}/src/app/chip_data_model.cmake)
    chip_configure_data_model(app
        ZAP_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/matter/window-app.zap
    )
endif()

# --- Linker settings ---
zephyr_linker_sources(SECTIONS custom_bss.ld)
zephyr_linker_sources(RODATA custom_rodata.ld)
target_link_libraries(app PRIVATE -lm)

# Application Sources
target_sources(app PRIVATE
    src/config_manager.c
    src/entropy_mw320.c
    src/leds.c
    src/main.c
    src/httpd.c
    src/network.c
    src/pic_uart.c
    src/mqtt.c
    src/sesame_syslog.c
    src/ota.c
    src/ota_client.c
    src/mw_watchdog.c
)

if(CONFIG_CHIP)
    target_sources(app PRIVATE src/matter/MatterTask.cpp src/matter/MatterOTA.cpp src/matter/MatterInfo.cpp third_party/connectedhomeip/examples/providers/DeviceInfoProviderImpl.cpp)
endif()

zephyr_include_directories(include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip/examples/providers)
zephyr_include_directories(${MW320_SDK_DIR}/components/flash/mflash/mw320)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mcuboot/boot/bootutil/include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mcuboot/boot/zephyr/include)

# MW320 SDK sources are provided by the mw320_sdk Zephyr module
# via ZEPHYR_EXTRA_MODULES

set(nanopb_SOURCE_DIR ${CMAKE_SOURCE_DIR}/third_party/connectedhomeip/third_party/nanopb/repo)
add_subdirectory(${nanopb_SOURCE_DIR} ${CMAKE_BINARY_DIR}/nanopb)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED)

# Generate nanopb sources
nanopb_generate_cpp(TARGET proto_app RELPATH proto proto/app_config.proto proto/api.proto)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(nanopb PRIVATE zephyr_interface)
target_link_libraries(app PRIVATE proto_app nanopb)

if(CONFIG_WIFI_MW320)
    add_subdirectory(drivers/wifi/mw320)
endif()

if(NOT CONFIG_BOARD_GENIE_IDCM_RAM)
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/sesame.bin
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/zephyr/zephyr.signed.bin
            ${CMAKE_BINARY_DIR}/sesame.bin
    DEPENDS zephyr_final ${CMAKE_BINARY_DIR}/zephyr/zephyr.signed.bin
    COMMENT "Copying zephyr.signed.bin -> sesame.bin"
    VERBATIM
)
add_custom_target(sesame_bin ALL DEPENDS ${CMAKE_BINARY_DIR}/sesame.bin)

endif()
