add_integration_test(mqtt
    main_mqtt.c
    "${CMAKE_SOURCE_DIR}/src/mqtt.c"
    "${CMAKE_SOURCE_DIR}/src/freertos_agent_message.c")
