add_executable(host_test_matter
    matter_test_main.c
    psm_stub.c
    "${CMAKE_SOURCE_DIR}/src/matter/matter_task.c"
    "${CMAKE_SOURCE_DIR}/src/matter/matter_berry_port.c"
    "${CMAKE_SOURCE_DIR}/src/matter/matter_tasmota_shim.c"
    "${CMAKE_SOURCE_DIR}/src/matter/matter_crypto_shim.c"
)

target_include_directories(host_test_matter
    PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/src/matter"
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/include/config"
    "${mbedtls_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/mw320_sdk/middleware/wifi/incl"
    "${CMAKE_SOURCE_DIR}/mw320_sdk/components/boot2_utils"
    "${CMAKE_SOURCE_DIR}/test/host/harness/include"
)

target_compile_definitions(host_test_matter
    PRIVATE
    TASMOTA
    BERRY_MODULE_CONTINUE
    LOGGING_ENABLE_METADATA_WITH_C99_AND_GNU_EXTENSION=0
)

target_link_libraries(host_test_matter
    PRIVATE
    host_harness
    host_logging
    berry
    berry_matter
    freertos_kernel
    freertos_plus_tcp
    mbedcrypto
    mbedtls
    proto
    m
    "LINKER:--start-group"
    berry
    berry_matter
    "LINKER:--end-group"
)

add_test(NAME host_test_matter
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}/.." pytest -xvs "${CMAKE_CURRENT_SOURCE_DIR}/test_matter.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties(host_test_matter PROPERTIES
    ENVIRONMENT "HOST_TEST_BIN=$<TARGET_FILE:host_test_matter>;LD_PRELOAD=$<TARGET_FILE:slirp_hostfwd>"
    ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_prepend:${PROTO_PY_DIR}")
