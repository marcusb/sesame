cmake_minimum_required(VERSION 3.23)

project(
  sesame
  VERSION 0.1
  LANGUAGES C ASM)

set(mw320_sdk_dir "${CMAKE_CURRENT_LIST_DIR}/mw320_sdk")

if(CMAKE_CROSSCOMPILING)
# Needed for CMake's test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_BUILD_TYPE Debug)

option(USE_BACKTRACE "Include facility for dumping stack traces" ON)

add_library(app_shared STATIC)
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
target_compile_definitions(logging
    PUBLIC
    LOGGING_ENABLE_METADATA_WITH_C99_AND_GNU_EXTENSION=0)
target_link_libraries(logging
    PRIVATE
    freertos_kernel
    globals
    mw320_utility_debug_console
)

# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------

target_compile_options(globals
    INTERFACE
    "-mcpu=cortex-m4"
    "-mthumb"
    "-mfloat-abi=softfp"
    "-mfpu=fpv4-sp-d16"
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
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Release>:-Os>)
target_compile_definitions(globals
    INTERFACE
    __FPU_PRESENT
    _DEFAULT_SOURCE
    _GNU_SOURCE)

# -------------------------------------------------------------------------------------------------
# Backtrace
# -------------------------------------------------------------------------------------------------

if (USE_BACKTRACE)
    add_library(backtrace STATIC)
    target_sources(backtrace
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/libraries/backtrace/backtrace/backtrace.c")
    target_include_directories(backtrace
        PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/libraries/backtrace/include")
    target_link_libraries(backtrace
        PRIVATE globals)
    target_link_libraries(app_shared PRIVATE backtrace)
    target_compile_options(globals
        INTERFACE
        "-funwind-tables"
        "-mpoke-function-name"
        "-fno-omit-frame-pointer")
    target_compile_definitions(globals
        INTERFACE
        USE_BACKTRACE)
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
	proto/api.proto
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

target_sources(freertos_kernel
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/idle_task.c")
target_link_libraries(freertos_kernel
    PUBLIC
    freertos_config)

target_sources(freertos_plus_tcp
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/random.c")

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

add_library(freertos_plus_tcp_network_if INTERFACE)
target_link_libraries(freertos_plus_tcp_network_if
    INTERFACE
    mw320_wlcmgr)

# -------------------------------------------------------------------------------------------------
# Other libraries
# -------------------------------------------------------------------------------------------------

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
target_compile_definitions(mbedcrypto
    PUBLIC
    "MBEDTLS_CONFIG_FILE=\"mbedtls_app_config.h\""
)

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

include("mw320_sdk.cmake")

add_library(mw320_sdk INTERFACE)
target_link_libraries(mw320_sdk
    INTERFACE
    mw320_wlcmgr
    mw320_drivers_aes
    mw320_drivers_gpio
    mw320_drivers_rtc
    mw320_drivers_wdt
    mw320_device_startup
    mw320_utility_assert
    mw320_utility_debug_console
    mw320_boot2_psm
)

set(exe_target sesame)
set(exe_target_ram "${exe_target}_ram")
set(CMAKE_EXECUTABLE_SUFFIX ".axf")

add_library(board STATIC)
target_sources(board PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/board/board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/board/clock_config.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/board/sdmmc_config.c"
)
target_compile_definitions(board
    PUBLIC
    SDIO_ENABLED)
target_include_directories(board
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/board"
)
target_link_libraries(board
    PUBLIC
    globals
    mw320_sdmmc_sdio
    mw320_sdmmc_host_sdioc
    mw320_drivers_power
    PRIVATE
    mw320_utility_debug_console
)

# -------------------------------------------------------------------------------------------------
# Application
# -------------------------------------------------------------------------------------------------

target_sources(app_shared
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dhcp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/freertos_agent_message.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardfault.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hardfault_irq.S"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/httpd.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/leds.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mqtt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/network_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ota.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ota_client.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pic_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pin_mux.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/syslog.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/time_util.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/util.c")
target_include_directories(app_shared
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(app_shared
    PUBLIC
    coreHTTP
    coreMQTT_agent
    freertos_plus_transport
    freertos_plus_tcp
    freertos_plus_tcp_network_if
    mbedcrypto
    mw320_sdk
    board
    logging
    backoff_algorithm
    proto)
target_link_options(app_shared
    PUBLIC
    "-mcpu=cortex-m4"
    "-mthumb"
    "-mfloat-abi=softfp"
    "-mfpu=fpv4-sp-d16"
    "-specs=nano.specs"
    "-specs=nosys.specs"
    "-Wl,--wrap,malloc"
    "-Wl,--wrap,free"
    "-Wl,--wrap,calloc"
    "-Wl,--wrap,realloc"
    "-Wl,--wrap,printf"
    "-Wl,--wrap,fprintf"
    "-Wl,--undefined,uxTopUsedPriority"
    "-Wl,--cref"
    "-Wl,--gc-sections"
    "-Wl,--nmagic")

add_executable("${exe_target}")
target_sources("${exe_target}"
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/newlib_wrappers.c")
target_link_libraries("${exe_target}" PRIVATE app_shared)
target_link_options("${exe_target}"
    PRIVATE
    "-Wl,-Map,${exe_target}.map"
    "-Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/board/88MW320_xx_xxxx_flash.ld")

add_executable("${exe_target_ram}")
target_sources("${exe_target_ram}"
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/newlib_wrappers.c")
target_link_libraries("${exe_target_ram}" PRIVATE app_shared)
target_link_options("${exe_target_ram}"
    PRIVATE
    "-Wl,-Map,${exe_target_ram}.map"
    "-Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld")

add_custom_command(
    TARGET ${exe_target} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E remove "${exe_target}.bin"
    COMMAND axf2firmware "${exe_target}.axf" "${exe_target}.bin"
)

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${exe_target}.bin" "${exe_target}.map" "${exe_target_ram}.bin" "${exe_target_ram}.map")

find_package(axf2firmware)

else()

# native build
add_executable(axf2firmware)
target_sources(axf2firmware
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/axf2firmware/axf2firmware.c")
target_include_directories(axf2firmware
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/axf2firmware")
target_compile_options(axf2firmware
    PRIVATE
    "-Wall")
target_compile_definitions(axf2firmware
    PRIVATE
    AXF_BUILT_BY_ARM_GCC)
target_link_libraries(axf2firmware
    PRIVATE)
export(TARGETS axf2firmware FILE
    "${CMAKE_BINARY_DIR}/axf2firmware-config.cmake")
endif()
