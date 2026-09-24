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

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/app_version_file
"VERSION_MAJOR = ${APP_VERSION_MAJOR}
VERSION_MINOR = ${APP_VERSION_MINOR}
PATCHLEVEL = ${APP_PATCHLEVEL}
VERSION_TWEAK = ${GIT_COMMIT_COUNT}
EXTRAVERSION =
")
set(VERSION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/deps/zephyr/VERSION ${CMAKE_CURRENT_BINARY_DIR}/app_version_file)
set(VERSION_TYPE KERNEL APP)

# Tell MCUboot to sign the image with this version
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/app_version.conf
"CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${APP_PATCHLEVEL}+${GIT_COMMIT_COUNT}\"
")
    list(APPEND EXTRA_CONF_FILE ${CMAKE_CURRENT_BINARY_DIR}/app_version.conf)
# Load Zephyr build system
set(MW320_SDK_DIR ${CMAKE_SOURCE_DIR}/mw320_sdk CACHE PATH "MW320 SDK directory" FORCE)

# Auto-detect or use prebuilt Matter libraries when building sesame_test
get_filename_component(CHIP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip REALPATH)
get_filename_component(CURRENT_BIN_DIR_NAME ${CMAKE_BINARY_DIR} NAME)
set(REUSE_PREBUILT_CHIP FALSE)
if("${CURRENT_BIN_DIR_NAME}" STREQUAL "sesame_test" OR DEFINED CHIP_PREBUILT_DIR)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sysbuild/sesame_test.conf")
        list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sysbuild/sesame_test.conf")
    endif()
    if(NOT DEFINED DTC_OVERLAY_FILE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sysbuild/sesame_test.overlay")
        set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sysbuild/sesame_test.overlay")
    endif()

    if(NOT DEFINED CHIP_PREBUILT_DIR)
        set(CHIP_PREBUILT_DIR "${CMAKE_BINARY_DIR}/../sesame/modules/connectedhomeip")
    endif()
    get_filename_component(CHIP_PREBUILT_DIR "${CHIP_PREBUILT_DIR}" ABSOLUTE)
    if(EXISTS "${CHIP_PREBUILT_DIR}/lib/libCHIP.a")
        set(REUSE_PREBUILT_CHIP TRUE)
        message(STATUS "sesame_test: Reusing prebuilt Matter libraries from ${CHIP_PREBUILT_DIR}")
    endif()
endif()

if(REUSE_PREBUILT_CHIP)
    list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/chip_prebuilt)
else()
    # Inject IPv6 multicast implementation for mDNS in CHIP
    set(CHIP_CFLAGS "${CHIP_CFLAGS} -DIPV6_MULTICAST_IMPLEMENTED=1 -Dipv6mr_interface=ipv6mr_ifindex")
    list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip/config/zephyr/chip-module)
endif()

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
    src/leds.c
    src/main.c
    src/httpd.c
    src/network.c
    src/pic_uart.c
    src/mqtt.c
    src/sesame_syslog.c
    src/time_util.c
)

if(CONFIG_BOOTLOADER_MCUBOOT)
    target_sources(app PRIVATE
        src/ota.c
        src/ota_client.c
    )
endif()

if(CONFIG_FLASH_SIMULATOR)
    target_sources(app PRIVATE tests/system/firmware/test_config_injector.c)
endif()

if(CONFIG_CHIP)
    target_sources(app PRIVATE src/matter/MatterTask.cpp src/matter/MatterOTA.cpp src/matter/MatterInfo.cpp third_party/connectedhomeip/examples/providers/DeviceInfoProviderImpl.cpp)
endif()

zephyr_include_directories(include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip/examples/providers)
zephyr_include_directories(${MW320_SDK_DIR}/components/flash/mflash/mw320)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bootloader/mcuboot/boot/bootutil/include)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bootloader/mcuboot/boot/zephyr/include)

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
    if(CONFIG_BUILD_OUTPUT_BIN)
        get_filename_component(CURRENT_BIN_DIR_NAME ${CMAKE_BINARY_DIR} NAME)
        if("${CURRENT_BIN_DIR_NAME}" STREQUAL "sesame_test")
            set(SESAME_BIN_NAME "sesame_test.bin")
            set(SESAME_TARGET_NAME "sesame_test_bin")
        else()
            set(SESAME_BIN_NAME "sesame.bin")
            set(SESAME_TARGET_NAME "sesame_bin")
        endif()

        add_custom_command(
            OUTPUT ${CMAKE_BINARY_DIR}/${SESAME_BIN_NAME}
            COMMAND ${CMAKE_COMMAND} -E copy
                    ${CMAKE_BINARY_DIR}/zephyr/zephyr.signed.confirmed.bin
                    ${CMAKE_BINARY_DIR}/${SESAME_BIN_NAME}
            DEPENDS zephyr_final ${CMAKE_BINARY_DIR}/zephyr/zephyr.signed.confirmed.bin
            COMMENT "Copying zephyr.signed.confirmed.bin -> ${SESAME_BIN_NAME}"
            VERBATIM
        )
        add_custom_target(${SESAME_TARGET_NAME} ALL DEPENDS ${CMAKE_BINARY_DIR}/${SESAME_BIN_NAME})
    endif()
endif()

if(CONFIG_SOC_88MW320)
    target_sources(app PRIVATE src/mw_watchdog.c)
endif()

# Custom test targets
add_custom_target(test_unit
    COMMAND ctest --output-on-failure
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running unit tests"
)

add_custom_target(test_integration
    COMMAND bash ${CMAKE_SOURCE_DIR}/run_tests.sh integration
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS app
    COMMENT "Running integration tests"
)

add_custom_target(test_system
    COMMAND echo "System tests not implemented yet"
    COMMENT "Running system tests"
)

add_custom_target(test_all
    DEPENDS test_unit test_integration test_system
    COMMENT "Running all tests"
)

# Automatically build sesame_test target alongside the main hardware target
if(NOT CONFIG_BOARD_NATIVE_SIM)
    get_filename_component(BIN_DIR_NAME ${CMAKE_BINARY_DIR} NAME)

    if(NOT "${BIN_DIR_NAME}" STREQUAL "sesame_test")
        if("${BIN_DIR_NAME}" STREQUAL "sesame")
            set(SESAME_TEST_OUTDIR ${CMAKE_BINARY_DIR}/../sesame_test)
        else()
            set(SESAME_TEST_OUTDIR ${CMAKE_BINARY_DIR}/sesame_test)
        endif()
        get_filename_component(SESAME_TEST_OUTDIR ${SESAME_TEST_OUTDIR} ABSOLUTE)

        include(ExternalProject)
        ExternalProject_Add(
            sesame_test_build
            SOURCE_DIR ${CMAKE_SOURCE_DIR}
            BINARY_DIR ${SESAME_TEST_OUTDIR}
            CONFIGURE_COMMAND west build -b genie_idcm/88mw320/cpu0 -d ${SESAME_TEST_OUTDIR} ${CMAKE_SOURCE_DIR}
            BUILD_COMMAND ""
            INSTALL_COMMAND ""
            BUILD_ALWAYS TRUE
        )
        add_dependencies(test_system sesame_test_build)
    endif()

    # When building the on-device system test firmware, also produce the OTA
    # update images (B and C) and the Matter/BDX container for B so the system
    # tests don't have to shell out to imgtool/ota_image_tool themselves.
    # B and C are plain-signed (no --confirm/--pad): they are written to the
    # secondary slot via OTA and booted in test mode, which the tests assert on.
    if("${BIN_DIR_NAME}" STREQUAL "sesame_test")
        set(OTA_IMGTOOL ${CMAKE_CURRENT_SOURCE_DIR}/bootloader/mcuboot/scripts/imgtool.py)
        set(OTA_SIGN_KEY ${CMAKE_CURRENT_SOURCE_DIR}/bootloader/mcuboot/root-rsa-2048.pem)
        set(OTA_IMAGE_TOOL ${CMAKE_CURRENT_SOURCE_DIR}/third_party/connectedhomeip/src/app/ota_image_tool.py)
        math(EXPR OTA_B_PATCH "${APP_PATCHLEVEL} + 1")
        math(EXPR OTA_C_PATCH "${APP_PATCHLEVEL} + 2")
        set(OTA_B_VERSION "${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${OTA_B_PATCH}")
        set(OTA_C_VERSION "${APP_VERSION_MAJOR}.${APP_VERSION_MINOR}.${OTA_C_PATCH}")

        add_custom_command(
            OUTPUT ${SESAME_OUT_DIR}/image_b.signed.bin
            COMMAND ${PYTHON_EXECUTABLE} ${OTA_IMGTOOL} sign
                    --version ${OTA_B_VERSION}+1
                    --header-size ${CONFIG_ROM_START_OFFSET}
                    --slot-size ${CONFIG_FLASH_LOAD_SIZE}
                    --align 4
                    --key ${OTA_SIGN_KEY}
                    ${SESAME_RAW_BIN}
                    ${SESAME_OUT_DIR}/image_b.signed.bin
            DEPENDS zephyr_final ${SESAME_RAW_BIN}
            COMMENT "Signing OTA test image B (v${OTA_B_VERSION})"
            VERBATIM
        )
        add_custom_command(
            OUTPUT ${SESAME_OUT_DIR}/image_c.signed.bin
            COMMAND ${PYTHON_EXECUTABLE} ${OTA_IMGTOOL} sign
                    --version ${OTA_C_VERSION}+1
                    --header-size ${CONFIG_ROM_START_OFFSET}
                    --slot-size ${CONFIG_FLASH_LOAD_SIZE}
                    --align 4
                    --key ${OTA_SIGN_KEY}
                    ${SESAME_RAW_BIN}
                    ${SESAME_OUT_DIR}/image_c.signed.bin
            DEPENDS zephyr_final ${SESAME_RAW_BIN}
            COMMENT "Signing OTA test image C (v${OTA_C_VERSION})"
            VERBATIM
        )
        # The BDX vendor version (-vn) is compared against the running firmware's
        # CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION (MATTER_SOFTWARE_VERSION_INT,
        # e.g. 131144 for 0.2.0+72). It must be strictly greater or the requestor
        # silently ignores the update, so derive it from the same formula with the
        # patch component bumped by one (0.2.1) rather than a hardcoded small int.
        math(EXPR OTA_B_VN "(${APP_VERSION_MAJOR} << 24) + (${APP_VERSION_MINOR} << 16) + ((${APP_PATCHLEVEL} + 1) << 8) + 1")
        add_custom_command(
            OUTPUT ${SESAME_OUT_DIR}/image_b.ota
            COMMAND ${CMAKE_COMMAND} -E env
                    PYTHONPATH=${MATTER_PY_SRC}
                    ${PYTHON_EXECUTABLE} ${OTA_IMAGE_TOOL} create
                    -v 0xFFF1
                    -p 0x8001
                    -vn ${OTA_B_VN}
                    -vs ${OTA_B_VERSION}
                    -da sha256
                    ${SESAME_OUT_DIR}/image_b.signed.bin
                    ${SESAME_OUT_DIR}/image_b.ota
            DEPENDS ${SESAME_OUT_DIR}/image_b.signed.bin
            COMMENT "Creating Matter/BDX OTA container for image B (vn ${OTA_B_VN})"
            VERBATIM
        )
        add_custom_target(sesame_test_ota_images ALL
            DEPENDS ${SESAME_OUT_DIR}/image_b.signed.bin
                    ${SESAME_OUT_DIR}/image_c.signed.bin
                    ${SESAME_OUT_DIR}/image_b.ota
        )
    endif()
endif()
