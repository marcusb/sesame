# ---------------------------------------------------------------------------
# Unity test framework
# ---------------------------------------------------------------------------
add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC unity "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
target_link_libraries(unity PRIVATE globals)

# ---------------------------------------------------------------------------
# Hardware stubs
# ---------------------------------------------------------------------------
add_library(test_stubs STATIC
    stubs/stub_psm.c)
target_include_directories(test_stubs
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/stubs"
    "${mw320_sdk_dir}/components/boot2_utils"
    "${mw320_sdk_dir}/middleware/wifi/incl")
target_link_libraries(test_stubs PRIVATE globals)

# ---------------------------------------------------------------------------
# sesame_tests – on-device test binary
# ---------------------------------------------------------------------------
add_executable(sesame_tests)

target_sources(sesame_tests
    PRIVATE
    test_runner.c
    test_string_util.c
    test_util.c
    test_logging.c
    test_config_manager.c
    test_pic_uart.c
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
    "${mw320_sdk_dir}/middleware/wifi/incl")

target_link_libraries(sesame_tests
    PRIVATE
    unity
    logging
    proto
    freertos_kernel
    board
    mw320_device_startup
    mw320_utility_assert
    test_stubs)

if(USE_BACKTRACE)
    target_link_libraries(sesame_tests PRIVATE backtrace)
endif()

target_link_options(sesame_tests
    PRIVATE
    "-Wl,-Map,sesame_tests.map"
    "-Wl,-T,${CMAKE_SOURCE_DIR}/board/88MW320_xx_xxxx_ram.ld"
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
