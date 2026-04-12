add_executable(host_test_ota
    main_ota.c
    ota_stub.c
    "${CMAKE_SOURCE_DIR}/src/ota_client.c"
    "${CMAKE_SOURCE_DIR}/test/host/harness/host_sockets.c")

target_link_libraries(host_test_ota
    PRIVATE
    freertos_plus_tcp_network_if
    host_harness
    freertos_plus_transport
    coreHTTP
    coreMQTT
    backoff_algorithm
    host_logging
    proto)

add_dependencies(host_test_ota slirp_hostfwd)

add_test(NAME host_test_ota
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}/.." pytest -xvs "${CMAKE_CURRENT_SOURCE_DIR}/test_ota.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties(host_test_ota PROPERTIES
    ENVIRONMENT "HOST_TEST_BIN=$<TARGET_FILE:host_test_ota>;LD_PRELOAD=$<TARGET_FILE:slirp_hostfwd>"
    ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_prepend:${PROTO_PY_DIR}")
