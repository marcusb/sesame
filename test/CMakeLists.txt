# ---------------------------------------------------------------------------
# QEMU / Emulation Options
# ---------------------------------------------------------------------------

set(ARM_TEST_LINK_OPTIONS
    "-mcpu=cortex-m4"
    "-mthumb"
    "-mfloat-abi=softfp"
    "-mfpu=fpv4-sp-d16"
    "-specs=picolibc.specs"
    "--oslib=semihost"
    "-Wl,--wrap,malloc"
    "-Wl,--wrap,free"
    "-Wl,--wrap,calloc"
    "-Wl,--wrap,realloc"
    "-Wl,--undefined,uxTopUsedPriority"
    "-Wl,--gc-sections"
    "-Wl,--nmagic"
    "-Wl,--defsym=_keystore_size=0")

# ---------------------------------------------------------------------------
# Unity test framework
# ---------------------------------------------------------------------------
add_library(unity STATIC unity/unity.c)
target_include_directories(unity
    PUBLIC
    unity
    "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
target_link_libraries(unity PRIVATE globals)

# ---------------------------------------------------------------------------
# Test Infrastructure
# ---------------------------------------------------------------------------
add_library(test_infra STATIC
    board_support.c
    unity_port.c
    test_runner.c)
target_include_directories(test_infra PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/unity")
target_link_libraries(test_infra PUBLIC freertos_kernel mw320_sdk logging mw320_mbedtls_port mbedcrypto)

# ---------------------------------------------------------------------------
# Hardware stubs
# ---------------------------------------------------------------------------
add_library(test_stubs STATIC
    stubs/stub_psm.c)
target_include_directories(test_stubs
    PUBLIC
    stubs
    "${CMAKE_SOURCE_DIR}/src"
    "${mw320_sdk_dir}/components/boot2_utils")
target_link_libraries(test_stubs PRIVATE globals)

# ---------------------------------------------------------------------------
# Specialized Matter Shim Tests
# ---------------------------------------------------------------------------

# Object library ensures all shared test code is available to linkers without dropping symbols
add_library(matter_test_support OBJECT
    matter/matter_test_utils.c
    stubs/stub_hostname.c
    "${CMAKE_SOURCE_DIR}/src/libc_wrappers.c"
    "${CMAKE_SOURCE_DIR}/src/pin_mux.c"
    "${CMAKE_SOURCE_DIR}/src/util.c"
    "${CMAKE_SOURCE_DIR}/src/idle_task.c"
    "${CMAKE_SOURCE_DIR}/src/time_util.c")

if(USE_QEMU)
    target_sources(matter_test_support PRIVATE 
        qemu_vectors.c 
        stubs/stub_fsl_uart.c
        "${CMAKE_SOURCE_DIR}/src/hardfault.c"
        "${CMAKE_SOURCE_DIR}/src/hardfault_irq.S")
endif()

target_include_directories(matter_test_support PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/matter"
    "${CMAKE_SOURCE_DIR}/third_party/berry/src")

target_link_libraries(matter_test_support PUBLIC
    globals unity logging board mw320_utility_assert
    mw320_mbedtls_port mw320_drivers_aes mw320_drivers_crc
    freertos_kernel test_infra test_stubs)

if(USE_BACKTRACE)
    target_link_libraries(matter_test_support PUBLIC backtrace)
endif()

if(USE_QEMU)
    target_link_libraries(matter_test_support PUBLIC mw320_device_system)
else()
    target_link_libraries(matter_test_support PUBLIC mw320_device_startup)
endif()

target_link_libraries(matter_test_support PUBLIC
    "LINKER:--start-group" sesame_matter berry_matter berry "LINKER:--end-group")

# Common ARM link options for tests
add_compile_definitions(SDK_DEBUGCONSOLE=0)

set(ARM_TEST_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld")
if(USE_QEMU)
    set(ARM_TEST_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/board/qemu.ld")
endif()

macro(add_matter_test_binary name source)
    add_executable(${name} "matter/${source}" $<TARGET_OBJECTS:matter_test_support>)
    target_link_libraries(${name} PRIVATE globals matter_test_support)

    target_link_options(${name} PRIVATE
        ${ARM_TEST_LINK_OPTIONS}
        "-T${ARM_TEST_LINKER_SCRIPT}"
        "-Wl,-Map,${name}.map")
    if(USE_QEMU)
        target_link_options(${name} PRIVATE
            "-Wl,--wrap,UART_TransferReceiveNonBlocking"
            "-Wl,--wrap,UART_TransferAbortReceive")
    endif()
endmacro()

# matter_crypto_tests still overflows the on-target ram.ld linker region
# (88MW320_xx_xxxx_ram.ld) once the full crypto shim is linked in,
# so only build these for QEMU.
if(USE_QEMU)
    add_matter_test_binary(matter_crypto_tests test_matter_crypto.c)
    add_matter_test_binary(matter_ec_raw_tests test_ec_raw.c)
    add_matter_test_binary(matter_mdns_tests test_matter_mdns.c)
    add_matter_test_binary(matter_sesame_tests test_matter_sesame.c)
    add_matter_test_binary(matter_tasmota_tests test_matter_tasmota.c)
endif()

# ---------------------------------------------------------------------------
# sesame_tests – main unit test runner
# ---------------------------------------------------------------------------
set(SESAME_TEST_SOURCES
    test_main.c
    test_config_manager.c
    test_string_util.c
    test_util.c
    test_pic_uart.c
    test_logging.c
    "${CMAKE_SOURCE_DIR}/src/libc_wrappers.c"
    "${CMAKE_SOURCE_DIR}/src/pin_mux.c"
    "${CMAKE_SOURCE_DIR}/src/util.c"
    "${CMAKE_SOURCE_DIR}/src/config_manager.c"
    "${CMAKE_SOURCE_DIR}/src/idle_task.c")

if(USE_QEMU)
    list(APPEND SESAME_TEST_SOURCES 
        qemu_vectors.c 
        stubs/stub_fsl_uart.c
        "${CMAKE_SOURCE_DIR}/src/hardfault.c"
        "${CMAKE_SOURCE_DIR}/src/hardfault_irq.S")
endif()

add_executable(sesame_tests ${SESAME_TEST_SOURCES})

target_include_directories(sesame_tests
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/board"
    "${mw320_sdk_dir}/components/boot2_utils"
    "${mw320_sdk_dir}/middleware/wifi/incl"
    "${mw320_sdk_dir}/middleware/wifi/incl/port/os")

target_link_libraries(sesame_tests
    PRIVATE
    globals
    unity
    test_infra
    test_stubs
    logging
    proto
    freertos_kernel
    board
    mw320_utility_assert)

if(USE_BACKTRACE)
    target_link_libraries(sesame_tests PRIVATE backtrace)
endif()

target_link_options(sesame_tests
    PRIVATE
    ${ARM_TEST_LINK_OPTIONS}
    "-Wl,-Map,sesame_tests.map"
    "-T${ARM_TEST_LINKER_SCRIPT}"
)
if(USE_QEMU)
    target_link_options(sesame_tests
        PRIVATE
        "-Wl,--wrap,UART_TransferReceiveNonBlocking"
        "-Wl,--wrap,UART_TransferAbortReceive")
endif()
