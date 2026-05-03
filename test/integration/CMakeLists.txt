# -------------------------------------------------------------------------------------------------
# Integration tests (QEMU only)
#
# These tests run as full FreeRTOS application tasks in QEMU, communicating with
# the python test harness via semihosting stdin/stdout.
# -------------------------------------------------------------------------------------------------

if(NOT USE_QEMU)
    return()
endif()

find_package(Python REQUIRED COMPONENTS Interpreter)

# Generate Python protobuf bindings for use by pytest.
set(PROTO_PY_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_py")
file(MAKE_DIRECTORY "${PROTO_PY_DIR}")
add_custom_command(
    OUTPUT
        "${PROTO_PY_DIR}/api_pb2.py"
        "${PROTO_PY_DIR}/app_config_pb2.py"
    COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}"
        "--python_out=${PROTO_PY_DIR}"
        "-I${CMAKE_SOURCE_DIR}/proto"
        "${CMAKE_SOURCE_DIR}/proto/api.proto"
        "${CMAKE_SOURCE_DIR}/proto/app_config.proto"
    DEPENDS
        "${CMAKE_SOURCE_DIR}/proto/api.proto"
        "${CMAKE_SOURCE_DIR}/proto/app_config.proto"
)
add_custom_target(it_proto_py ALL
    DEPENDS "${PROTO_PY_DIR}/api_pb2.py" "${PROTO_PY_DIR}/app_config_pb2.py")

# -------------------------------------------------------------------------------------------------
# Integration Test infrastructure
# -------------------------------------------------------------------------------------------------

add_subdirectory(harness)

# Macro to add an integration test executable
macro(add_integration_test name)
    set(target "${name}_it${target_suffix}")
    add_executable(${target} ${ARGN}
        "${CMAKE_SOURCE_DIR}/src/libc_wrappers.c"
        "${CMAKE_SOURCE_DIR}/src/hardfault.c"
        "${CMAKE_SOURCE_DIR}/src/hardfault_irq.S"
        "${CMAKE_SOURCE_DIR}/src/qemu/qemu_vectors.c"
    )

    target_link_libraries(${target}
        PRIVATE
        app_deps
        integration_harness
    )

    target_link_options(${target}
        PRIVATE
        ${ARM_TEST_LINK_OPTIONS}
        "-T${ARM_TEST_LINKER_SCRIPT}"
        "-Wl,-Map,${target}.map"
        "--oslib=semihost"
    )

    add_dependencies(integration_tests ${target})
endmacro()

# -------------------------------------------------------------------------------------------------
# Test targets
# -------------------------------------------------------------------------------------------------

add_custom_target(integration_tests)
add_dependencies(integration_tests it_proto_py)

add_subdirectory(httpd)
add_subdirectory(mqtt)
add_subdirectory(ota)
add_subdirectory(syslog)

add_test(NAME integration_tests
    COMMAND uv run --project "${CMAKE_CURRENT_SOURCE_DIR}" pytest -xvs
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
