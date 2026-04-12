add_executable(host_test_httpd
    main_httpd.c
    "${CMAKE_SOURCE_DIR}/src/httpd.c")

target_link_libraries(host_test_httpd
    PRIVATE
    freertos_plus_tcp_network_if
    host_harness
    host_logging
    proto)

add_dependencies(host_test_httpd slirp_hostfwd)

add_test(NAME host_test_httpd
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}/.." pytest -xvs "${CMAKE_CURRENT_SOURCE_DIR}/test_httpd.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties(host_test_httpd PROPERTIES
    ENVIRONMENT "HOST_TEST_BIN=$<TARGET_FILE:host_test_httpd>;LD_PRELOAD=$<TARGET_FILE:slirp_hostfwd>"
    ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_prepend:${PROTO_PY_DIR}")
