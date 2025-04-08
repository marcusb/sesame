cmake_minimum_required(VERSION 3.23)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/afr_utils.cmake")

# Needed for CMake's test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(
  sesame
  VERSION 0.1
  LANGUAGES C ASM)

option(USE_BACKTRACE "Include facility for dumping stack traces")

set(mw320_dir "${CMAKE_CURRENT_LIST_DIR}/marvell/WMSDK/mw320")
set(afr_ports_dir "${CMAKE_CURRENT_LIST_DIR}/marvell/boards/mw300_rd/ports")

set(exe_target sesame)
set(CMAKE_BUILD_TYPE Debug)

add_executable(${exe_target})

target_sources(${exe_target}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dhcp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/freertos_agent_message.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/idle_task.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/httpd.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/leds.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mbedtls_error_dummy.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mqtt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/network_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ota.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pic_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/random.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/syslog.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/time_util.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/util.c"
    "${afr_ports_dir}/wifi/iot_wifi.c")
target_include_directories(${exe_target}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "libraries/abstractions/wifi/include")

add_library(globals INTERFACE)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE
    include/config)
target_compile_definitions(freertos_config
    INTERFACE
    projCOVERAGE_TEST=0)
target_link_libraries(freertos_config
    INTERFACE globals)

add_library(logging STATIC)
target_sources(logging
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/logging.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/string_util.c")
target_include_directories(logging
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_options(logging
    PUBLIC
    -DLOGGING_ENABLE_METADATA_WITH_C99_AND_GNU_EXTENSION=0)
target_link_libraries(logging
    PRIVATE
    freertos_kernel
    globals)

# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------

# Compiler flags
set(SDK_VERSION_INTERNAL "\"v1.2.r6.p1\"")
set(SDK_VERSION ${SDK_VERSION_INTERNAL})

set(global-cflags-y
    "-mcpu=cortex-m4"
    "-mthumb"
    "-Os"
    "-fdata-sections"
    "-ffunction-sections"
    "-ffreestanding"
    "-MMD"
    "-Wall"
    "-fno-strict-aliasing"
    "-fno-common"
    "-Wextra"
    "-Wno-unused-parameter"
    "-Wno-empty-body"
    "-Wno-missing-field-initializers"
    "-Wno-sign-compare"
    "-Wno-type-limits"
    "-std=c99"
    "-DSDK_VERSION=${SDK_VERSION}")

# ARM Cortex M4 flags
set(tc-lflags-tc-cortex-m4-y
    "-mcpu=cortex-m4"
    "-mthumb"
    "-Os"
    "-fdata-sections"
    "-ffunction-sections"
    "-ffreestanding"
    "-MMD"
    "-Wall"
    "-fno-strict-aliasing"
    "-fno-common"
    "-mfloat-abi=softfp"
    "-mfpu=fpv4-sp-d16"
    "-D__FPU_PRESENT"
    "-D_DEFAULT_SOURCE"
    "-D_GNU_SOURCE")

target_compile_options(globals
    INTERFACE ${global-cflags-y} ${tc-lflags-tc-cortex-m4-y})

# Linker flags
# gcc specific extra linker flags
set(tc-lflags-y
    "-nostartfiles"
    "-specs=nano.specs"
    "-specs=nosys.specs"
    "-Wl,--wrap,malloc"
    "-Wl,--wrap,free"
    "-Wl,--wrap,calloc"
    "-Wl,--wrap,realloc"
    "-Wl,--wrap,printf"
    "-Wl,--wrap,fprintf")

# Xlinker flags
set(CMAKE_C_LINKER_WRAPPER_FLAG "-Xlinker" " ")
target_link_options(${exe_target}
    PRIVATE
    "LINKER:--undefined"
    "LINKER:uxTopUsedPriority"
    "LINKER:--cref"
    "LINKER:--gc-sections"
    "LINKER:--defsym=_rom_data=64"
    "LINKER:-M"
    "LINKER:-Map"
    "LINKER:${exe_target}.map")

target_link_options(freertos_config
     INTERFACE ${tc-lflags-y} ${tc-lflags-tc-cortex-m4-y})

# -------------------------------------------------------------------------------------------------
# Backtrace
# -------------------------------------------------------------------------------------------------

if (USE_BACKTRACE)
    add_library(backtrace STATIC)
    target_sources(backtrace
        PRIVATE
        ./libraries/backtrace/backtrace/backtrace.c)
    target_include_directories(backtrace
        PUBLIC
        ./libraries/backtrace/include)
    target_link_libraries(backtrace
        PRIVATE globals)

    target_sources(${exe_target}
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/fault.c")
    target_link_libraries(${exe_target} PRIVATE backtrace)

    list(APPEND global-cflags-y
        "-funwind-tables"
        "-mpoke-function-name"
        "-fno-omit-frame-pointer")
endif()

# -------------------------------------------------------------------------------------------------
# nanopb
# -------------------------------------------------------------------------------------------------

include(FetchContent)
FetchContent_Declare(nanopb
    GIT_REPOSITORY https://github.com/nanopb/nanopb.git
    GIT_TAG        0.4.9.1
)
FetchContent_MakeAvailable(nanopb)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED)

target_link_libraries(nanopb
    PUBLIC globals)

nanopb_generate_cpp(TARGET proto RELPATH proto
	proto/app_config.proto)

# -------------------------------------------------------------------------------------------------
# FreeRTOS
# -------------------------------------------------------------------------------------------------

set(FREERTOS_HEAP "5" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)

set(FREERTOS_PLUS_TCP_BUFFER_ALLOCATION "2" CACHE STRING "" FORCE)
set(FREERTOS_PLUS_TCP_NETWORK_IF "A_CUSTOM_NETWORK_IF" CACHE STRING "" FORCE)

FetchContent_Declare(freertos_plus_tcp
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Plus-TCP.git
    GIT_TAG        V4.3.1
)
FetchContent_MakeAvailable(freertos_plus_tcp freertos_kernel)

target_link_libraries(freertos_kernel
    PRIVATE
    wmsdk
    PUBLIC
    freertos_config)

add_library(freertos_plus_tcp_network_if STATIC)
target_sources(freertos_plus_tcp_network_if
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mw300_rd_net.c"
)
target_include_directories(freertos_plus_tcp_network_if
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${mw320_dir}/sdk/src/incl/platform/net/freertos-plus-tcp"
)
target_link_libraries(freertos_plus_tcp_network_if
    PUBLIC
    freertos_plus_tcp_port
    freertos_plus_tcp_network_if_common
    PRIVATE
    freertos_kernel
    freertos_plus_tcp
    wmsdk)

add_library(freertos_plus_transport STATIC)
target_sources(freertos_plus_transport
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/libraries/network_transport/transport_plaintext.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/libraries/network_transport/tcp_sockets_wrapper/ports/freertos_plus_tcp/tcp_sockets_wrapper.c")
target_include_directories(freertos_plus_transport
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/libraries/network_transport"
    "${CMAKE_CURRENT_SOURCE_DIR}/libraries/network_transport/tcp_sockets_wrapper/include")
target_link_libraries(freertos_plus_transport
    PUBLIC
    coreHTTP
    freertos_plus_tcp
    logging)

FetchContent_Declare(backoff_algorithm
    GIT_REPOSITORY https://github.com/FreeRTOS/backoffAlgorithm.git
    GIT_TAG        v1.4.1
)
FetchContent_MakeAvailable(backoff_algorithm)
include("${backoff_algorithm_SOURCE_DIR}/backoffAlgorithmFilePaths.cmake")
add_library(backoff_algorithm STATIC)
target_sources(backoff_algorithm
    PRIVATE
    "${BACKOFF_ALGORITHM_SOURCES}")
target_include_directories(backoff_algorithm
    PUBLIC
    "${BACKOFF_ALGORITHM_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(backoff_algorithm
    PRIVATE globals)

# do not build mbedtls programs and tests
set(ENABLE_PROGRAMS OFF CACHE BOOL INTERNAL)
set(ENABLE_TESTING OFF CACHE BOOL INTERNAL)

FetchContent_Declare(mbedtls
    GIT_REPOSITORY https://github.com/Mbed-TLS/mbedtls.git
    GIT_TAG        v3.6.3
)
FetchContent_MakeAvailable(mbedtls)
target_link_libraries(mbedtls
    PRIVATE
    freertos_kernel)
target_link_libraries(mbedx509
    PRIVATE
    freertos_kernel)
target_link_libraries(mbedcrypto
    PRIVATE
    freertos_kernel)
target_compile_definitions(mbedcrypto
    PUBLIC
    MBEDTLS_CONFIG_FILE="mbedtls_app_config.h")

FetchContent_Declare(core_pkcs11
    GIT_REPOSITORY https://github.com/FreeRTOS/corePKCS11.git
    GIT_TAG        v3.6.2
)
FetchContent_MakeAvailable(core_pkcs11)
include("${core_pkcs11_SOURCE_DIR}/pkcsFilePaths.cmake")
add_library(core_pkcs11 STATIC)
target_sources(core_pkcs11
    PRIVATE
    "${PKCS_SOURCES}"
    "libraries/crypto/src/iot_crypto.c"
    "${afr_ports_dir}/pkcs11/core_pkcs11_pal.c")
target_include_directories(core_pkcs11
    PUBLIC
    "${PKCS_INCLUDE_PUBLIC_DIRS}"
    "${PKCS_PAL_INCLUDE_PUBLIC_DIRS}"
    "libraries/crypto/include")
target_link_libraries(core_pkcs11
    PRIVATE
    wmsdk
    PUBLIC
    freertos_kernel
    mbedtls
    logging)

FetchContent_Declare(coreHTTP
    GIT_REPOSITORY https://github.com/FreeRTOS/coreHTTP.git
    GIT_TAG        v3.1.1
)
FetchContent_MakeAvailable(coreHTTP)
include("${corehttp_SOURCE_DIR}/httpFilePaths.cmake")
add_library(coreHTTP STATIC)
target_sources(coreHTTP
    PRIVATE "${HTTP_SOURCES}")
target_include_directories(coreHTTP
    PUBLIC "${HTTP_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreHTTP
    PUBLIC
    freertos_plus_tcp
    freertos_kernel
    logging)


FetchContent_Declare(coreMQTT
    GIT_REPOSITORY https://github.com/FreeRTOS/coreMQTT.git
    GIT_TAG        v2.3.1
)
FetchContent_MakeAvailable(coreMQTT)
include("${coremqtt_SOURCE_DIR}/mqttFilePaths.cmake")
add_library(coreMQTT STATIC)
target_sources(coreMQTT
    PRIVATE
    "${MQTT_SOURCES}"
    "${MQTT_SERIALIZER_SOURCES}")
target_include_directories(coreMQTT
    PUBLIC
    "${MQTT_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreMQTT
    PUBLIC
    freertos_plus_tcp
    freertos_kernel
    logging)

FetchContent_Declare(coreMQTT-Agent
    GIT_REPOSITORY https://github.com/FreeRTOS/coreMQTT-Agent.git
    GIT_TAG        v1.3.1
)
FetchContent_MakeAvailable(coreMQTT-Agent)
include("${coremqtt-agent_SOURCE_DIR}/mqttAgentFilePaths.cmake")
add_library(coreMQTT_agent STATIC)
target_sources(coreMQTT_agent
    PRIVATE "${MQTT_AGENT_SOURCES}")
target_include_directories(coreMQTT_agent
    PUBLIC "${MQTT_AGENT_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreMQTT_agent
    PUBLIC
    coreMQTT
    logging)

FetchContent_Declare(coreJSON
    GIT_REPOSITORY https://github.com/FreeRTOS/coreJSON.git
    GIT_TAG        v3.3.0
)
FetchContent_MakeAvailable(coreJSON)
include("${corejson_SOURCE_DIR}/jsonFilePaths.cmake")
add_library(coreJSON STATIC)
target_sources(coreJSON
    PRIVATE "${JSON_SOURCES}")
target_include_directories(coreJSON
    PUBLIC "${JSON_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreJSON
    PRIVATE globals)

# ==================== Marvell ====================

add_library(wmsdk STATIC)
target_sources(wmsdk
    PRIVATE
    "board/board.c"
    "${mw320_dir}/sdk/external/FreeRTOS/Source/FreeRTOS-openocd.c"
    "${mw320_dir}/sdk/src/core/cli/cli.c"
    "${mw320_dir}/sdk/src/core/cli/cli_mem_simple.c"
    "${mw320_dir}/sdk/src/core/cli/cli_utils.c"
    "${mw320_dir}/sdk/src/core/mdev/mdev.c"
    "${mw320_dir}/sdk/src/core/pwrmgr/pwrmgr.c"
    "${mw320_dir}/sdk/src/core/pwrmgr/pwrmgr_cli.c"
    "${mw320_dir}/sdk/src/core/util/boot_flags.c"
    "${mw320_dir}/sdk/src/core/util/crc/crc32.c"
    "${mw320_dir}/sdk/src/core/util/crc/soft_crc16.c"
    "${mw320_dir}/sdk/src/core/util/crc/soft_crc32.c"
    "${mw320_dir}/sdk/src/core/util/critical_error.c"
    "${mw320_dir}/sdk/src/core/util/dumphex.c"
    "${mw320_dir}/sdk/src/core/util/flash.c"
    "${mw320_dir}/sdk/src/core/util/hexbin.c"
    "${mw320_dir}/sdk/src/core/util/keystore.c"
    "${mw320_dir}/sdk/src/core/util/panic.c"
    "${mw320_dir}/sdk/src/core/util/wm_utils.c"
    "${mw320_dir}/sdk/src/core/util/wm_wlan.c"
    "${mw320_dir}/sdk/src/core/wmstdio/console.c"
    "${mw320_dir}/sdk/src/core/wmstdio/wmassert.c"
    "${mw320_dir}/sdk/src/core/wmstdio/wmstdio.c"
    "${mw320_dir}/sdk/src/core/wmtime/hwrtc.c"
    "${mw320_dir}/sdk/src/core/wmtime/swrtc.c"
    "${mw320_dir}/sdk/src/core/wmtime/wmtime.c"
    "${mw320_dir}/sdk/src/core/wmtime/wmtime_cli.c"
    "${mw320_dir}/sdk/src/drivers/common/cyccnt.c"
    "${mw320_dir}/sdk/src/drivers/common/hardfault/hardfault.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_acomp.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_aes.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_crc.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_dac.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_gpio.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_gpt.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_usb_host.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_usb_host_port.c"
    "${mw320_dir}/sdk/src/drivers/common/mdev_wdt.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_acomp.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_adc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_aes.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_bg.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_clock.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_crc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_dac.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_dma.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_driver.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_flash.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_flashc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_gpio.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_gpt.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_i2c.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_mpu.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_pinmux.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_pmu.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_qspi.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_rtc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_sdio.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_spi_flash.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_ssp.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_uart.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_usb.c"
    "${mw320_dir}/sdk/src/drivers/mw300/lowlevel/mw300_wdt.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_adc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_dma.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_i2c.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_iflash.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_pinmux.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_pm.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_rfctrl.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_rtc.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_sdio.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_ssp.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_startup.c"
    "${mw320_dir}/sdk/src/drivers/mw300/mdev_uart.c"
    "${mw320_dir}/sdk/src/middleware/xz/decompress.c"
    "${mw320_dir}/sdk/src/middleware/xz/xz_crc32.c"
    "${mw320_dir}/sdk/src/middleware/xz/xz_dec_lzma2.c"
    "${mw320_dir}/sdk/src/middleware/xz/xz_dec_stream.c"
    "${mw320_dir}/sdk/src/platform/os/freertos/os.c"
    "${mw320_dir}/sdk/src/platform/os/freertos/os_cli.c"
)
target_compile_options(wmsdk
    PUBLIC
    -include "${mw320_dir}/sdk/src/incl/autoconf.h"
)
target_include_directories(wmsdk
    PUBLIC
    "${mw320_dir}/sdk/src/incl"
    "${mw320_dir}/sdk/src/incl/sdk"
    "${mw320_dir}/sdk/src/incl/libc/arm_gcc"
    "${mw320_dir}/sdk/src/incl/sdk/drivers"
    "${mw320_dir}/sdk/src/incl/sdk/drivers/wlan"
    "${mw320_dir}/sdk/src/incl/sdk/drivers/mw300"
    "${mw320_dir}/sdk/src/incl/sdk/drivers/mw300/regs"
    "${mw320_dir}/sdk/src/incl/platform/os/freertos"
    "${mw320_dir}/sdk/src/boards"
    "${mw320_dir}/sdk/src/drivers/common/hardfault")
target_link_libraries(wmsdk
    PUBLIC
    freertos_kernel
    mbedcrypto)

target_link_libraries(${exe_target}
    PUBLIC
    "${mw320_dir}/bin/libs/libwifidriver.a"
    "${mw320_dir}/bin/libs/libwlcmgr.a"
    "${mw320_dir}/bin/libs/librfget.a"
    coreHTTP
    coreMQTT_agent
    coreJSON
    freertos_plus_transport
    freertos_plus_tcp
    freertos_plus_tcp_network_if
    mbedtls
    core_pkcs11
    wmsdk
    "${mw320_dir}/bin/libs/libpsm.a"
    "${mw320_dir}/bin/libs/libpart.a"
    logging
    backoff_algorithm
    proto

    PRIVATE
    -T"${mw320_dir}/build/toolchains/arm_gcc/mw300.ld")

set(CMAKE_EXECUTABLE_SUFFIX ".axf")

# Convert afx file to bin file
if("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(axf2fw "${mw320_dir}/sdk/tools/bin/Linux/axf2firmware")
    execute_process(COMMAND chmod +x "${axf2fw}")  # TODO, workaround for FreeRTOS console permission issue.
else()
    message(FATAL_ERROR "Only Linux host is supported for marvell.")
endif()

add_custom_command(
    TARGET ${exe_target} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E remove "${exe_target}.bin"
    COMMAND "${axf2fw}" "${exe_target}.axf" "${exe_target}.bin"
)

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${exe_target}.bin" "${exe_target}.map")
