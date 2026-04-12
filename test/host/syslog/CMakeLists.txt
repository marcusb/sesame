add_executable(host_test_syslog
    main_syslog.c
    "${CMAKE_SOURCE_DIR}/src/syslog.c")

target_link_libraries(host_test_syslog
    PRIVATE
    freertos_plus_tcp_network_if
    host_harness
    host_logging
    proto)

add_dependencies(host_test_syslog slirp_hostfwd)

add_test(NAME host_test_syslog
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}/.." pytest -xvs "${CMAKE_CURRENT_SOURCE_DIR}/test_syslog.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties(host_test_syslog PROPERTIES
    ENVIRONMENT "HOST_TEST_BIN=$<TARGET_FILE:host_test_syslog>;LD_PRELOAD=$<TARGET_FILE:slirp_hostfwd>"
    ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_prepend:${PROTO_PY_DIR}")
