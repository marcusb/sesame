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

set(MATTER_TEST_LINK_OPTIONS
    "-Wl,-T,${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld"
    "-Wl,--defsym=_keystore_size=0"
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
    "-Wl,--gc-sections"
    "-Wl,--nmagic")

macro(add_matter_test_binary name source)
    add_executable(${name} 
        "matter/${source}"
        matter/matter_test_utils.c
        matter/board_support.c
        "${CMAKE_SOURCE_DIR}/src/newlib_wrappers.c"
        "${CMAKE_SOURCE_DIR}/src/pin_mux.c"
        "${CMAKE_SOURCE_DIR}/src/util.c"
        "${CMAKE_SOURCE_DIR}/src/idle_task.c")
    
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/matter"
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/board"
        "${CMAKE_SOURCE_DIR}/third_party/berry/src")
    
    target_link_libraries(${name} PRIVATE
        globals
        unity logging board mw320_device_startup mw320_utility_assert
        mw320_mbedtls_port mw320_drivers_aes
        freertos_kernel
        "LINKER:--start-group" sesame_matter berry_matter berry "LINKER:--end-group"
        m)
    
    target_link_options(${name} PRIVATE ${MATTER_TEST_LINK_OPTIONS} "-Wl,-Map,${name}.map")
endmacro()

set(MATTER_FLASH_TEST_LINK_OPTIONS
    "-Wl,-T,${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_flash.ld"
    "-Wl,--defsym=_keystore_size=0"
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
    "-Wl,--gc-sections"
    "-Wl,--nmagic")

macro(add_matter_flash_test_binary name source)
    add_executable(${name} 
        "matter/${source}"
        matter/matter_test_utils.c
        matter/board_support_flash.c
        "${CMAKE_SOURCE_DIR}/src/newlib_wrappers.c"
        "${CMAKE_SOURCE_DIR}/src/pin_mux.c"
        "${CMAKE_SOURCE_DIR}/src/util.c"
        "${CMAKE_SOURCE_DIR}/src/idle_task.c")
    
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/matter"
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/board"
        "${CMAKE_SOURCE_DIR}/third_party/berry/src")
    
    target_link_libraries(${name} PRIVATE
        globals
        unity logging board mw320_device_startup mw320_utility_assert
        mw320_mbedtls_port mw320_drivers_aes mw320_drivers_crc
        freertos_kernel
        "LINKER:--start-group" sesame_matter berry_matter berry "LINKER:--end-group"
        m)
    
    target_link_options(${name} PRIVATE ${MATTER_FLASH_TEST_LINK_OPTIONS} "-Wl,-Map,${name}.map")
endmacro()

add_matter_test_binary(matter_crypto_tests test_matter_crypto.c)
add_matter_test_binary(matter_ec_raw_tests test_ec_raw.c)
add_matter_flash_test_binary(matter_crypto_flash_tests test_matter_crypto.c)
add_matter_test_binary(matter_sesame_tests test_matter_sesame.c)
add_matter_test_binary(matter_tasmota_tests test_matter_tasmota.c)

# ---------------------------------------------------------------------------
# sesame_tests – main unit test runner
# ---------------------------------------------------------------------------
add_executable(sesame_tests)

target_sources(sesame_tests
    PRIVATE
    test_runner.c
    test_config_manager.c
    test_string_util.c
    test_util.c
    test_pic_uart.c
    test_logging.c
    stubs/stub_os.c
    "${CMAKE_SOURCE_DIR}/src/newlib_wrappers.c"
    "${CMAKE_SOURCE_DIR}/src/pin_mux.c"
    "${CMAKE_SOURCE_DIR}/src/util.c"
    "${CMAKE_SOURCE_DIR}/src/config_manager.c"
    "${CMAKE_SOURCE_DIR}/src/idle_task.c")

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
    unity
    test_stubs
    logging
    proto
    freertos_kernel
    board
    mw320_device_startup
    mw320_utility_assert)

if(USE_BACKTRACE)
    target_link_libraries(sesame_tests PRIVATE backtrace)
endif()

target_link_options(sesame_tests
    PRIVATE
    "-Wl,-Map,sesame_tests.map"
    "-Wl,-T,${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld"
    "-Wl,--defsym=_keystore_size=0"
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
    "-Wl,--gc-sections"
    "-Wl,--nmagic")
