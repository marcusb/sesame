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
target_link_libraries(test_infra PUBLIC freertos_kernel logging mbedcrypto)
if(NOT USE_QEMU)
    target_link_libraries(test_infra PUBLIC mw320_sdk mw320_mbedtls_port)
endif()

# ---------------------------------------------------------------------------
# Hardware stubs
# ---------------------------------------------------------------------------
add_library(test_stubs STATIC
    stubs/stub_psm.c)
target_include_directories(test_stubs
    PUBLIC
    stubs
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
    "${mw320_sdk_dir}/components/boot2_utils")
target_link_libraries(test_stubs PRIVATE globals mw320_sdk)

# Common ARM link options for tests
add_compile_definitions(SDK_DEBUGCONSOLE=0)

set(ARM_TEST_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld")
if(USE_QEMU)
    set(ARM_TEST_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/board/qemu.ld")
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
    stubs/stub_fsl_uart.c
    "${CMAKE_SOURCE_DIR}/src/hardfault.c"
    "${CMAKE_SOURCE_DIR}/src/hardfault_irq.S"
    "${CMAKE_SOURCE_DIR}/src/qemu/qemu_stubs.c"
    "${CMAKE_SOURCE_DIR}/src/qemu/qemu_vectors.c")
endif()

set(test_target "sesame_tests${target_suffix}")
add_executable(${test_target} ${SESAME_TEST_SOURCES})

target_include_directories(${test_target}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/board"
    "${mw320_sdk_dir}/components/boot2_utils"
    "${mw320_sdk_dir}/middleware/wifi/incl"
    "${mw320_sdk_dir}/middleware/wifi/incl/port/os"
    "${freertos_plus_tcp_SOURCE_DIR}/source/include"
    "${freertos_plus_tcp_SOURCE_DIR}/source/portable/Compiler/GCC")

target_link_libraries(${test_target}
    PRIVATE
    globals
    unity
    test_infra
    test_stubs
    logging
    proto
    freertos_kernel
    mw320_utility_assert)

if(NOT USE_QEMU)
    target_link_libraries(${test_target} PRIVATE board)
endif()

if(USE_QEMU)
    target_link_libraries(${test_target} PRIVATE
        "LINKER:--start-group"
        mbedcrypto
        qemu_mbedtls_port
        test_stubs
        "LINKER:--whole-archive"
        sesame_matter
        "LINKER:--no-whole-archive"
        freertos_plus_tcp
        "LINKER:--end-group")
endif()

if(USE_BACKTRACE)
    target_link_libraries(${test_target} PRIVATE backtrace)
endif()

target_link_options(${test_target}
    PRIVATE
    ${ARM_TEST_LINK_OPTIONS}
    "-Wl,-Map,${test_target}.map"
    "-T${ARM_TEST_LINKER_SCRIPT}")

if(USE_QEMU)
    target_link_options(${test_target}
        PRIVATE
        "-Wl,--wrap,UART_TransferReceiveNonBlocking"
        "-Wl,--wrap,UART_TransferAbortReceive")
endif()

if(USE_QEMU)
    add_subdirectory(integration)
endif()
