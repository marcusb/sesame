add_executable(host_test_mqtt
    main_mqtt.c
    "${CMAKE_SOURCE_DIR}/src/mqtt.c"
    "${CMAKE_SOURCE_DIR}/src/freertos_agent_message.c")

target_link_libraries(host_test_mqtt
    PRIVATE
    freertos_plus_tcp_network_if
    host_harness
    freertos_plus_transport
    coreMQTT
    coreMQTT_agent
    backoff_algorithm
    host_logging
    proto)

add_dependencies(host_test_mqtt slirp_hostfwd)

add_test(NAME host_test_mqtt
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}/.." pytest -xvs "${CMAKE_CURRENT_SOURCE_DIR}/test_mqtt.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
set_tests_properties(host_test_mqtt PROPERTIES
    ENVIRONMENT "HOST_TEST_BIN=$<TARGET_FILE:host_test_mqtt>;LD_PRELOAD=$<TARGET_FILE:slirp_hostfwd>"
    ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_prepend:${PROTO_PY_DIR}")
