# -------------------------------------------------------------------------------------------------
# Host-based module tests
#
# Runs app modules on Linux by linking against the FreeRTOS POSIX port and FreeRTOS-Plus-TCP
# with the libslirp driver.
# -------------------------------------------------------------------------------------------------

include(FetchContent)

find_package(Python REQUIRED COMPONENTS Interpreter)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SLIRP REQUIRED IMPORTED_TARGET slirp)

# Re-export as "slirp" for the freertos_plus_tcp libslirp NIC CMake.
add_library(slirp INTERFACE)
target_link_libraries(slirp INTERFACE PkgConfig::SLIRP)

add_library(globals INTERFACE)
target_compile_definitions(globals INTERFACE _DEFAULT_SOURCE _GNU_SOURCE)

# -------------------------------------------------------------------------------------------------
# nanopb
# -------------------------------------------------------------------------------------------------

FetchContent_Declare(nanopb
    GIT_REPOSITORY https://github.com/nanopb/nanopb.git
    GIT_TAG        0.4.9.1
)
FetchContent_MakeAvailable(nanopb)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED)

# Patch the nanopb protoc plugin shebang to use the explicit python path
# instead of #!/usr/bin/env python3 (which may resolve to a venv without protobuf).
set(_nanopb_plugin "${CMAKE_CURRENT_BINARY_DIR}/nanopb/generator/protoc-gen-nanopb")
file(READ "${nanopb_SOURCE_DIR}/generator/protoc-gen-nanopb" _plugin_src)
string(REGEX REPLACE "^#![^\n]*\n" "#!${Python_EXECUTABLE}\n" _plugin_src "${_plugin_src}")
file(WRITE "${_nanopb_plugin}" "${_plugin_src}")
file(CHMOD "${_nanopb_plugin}"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
set(NANOPB_GENERATOR_PLUGIN "${_nanopb_plugin}" CACHE FILEPATH "" FORCE)

nanopb_generate_cpp(TARGET proto RELPATH "${CMAKE_SOURCE_DIR}/proto"
    "${CMAKE_SOURCE_DIR}/proto/api.proto"
    "${CMAKE_SOURCE_DIR}/proto/app_config.proto")

# Generate Python protobuf bindings for use by pytest.
# nanopb already located protoc as PROTOBUF_PROTOC_EXECUTABLE.
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
add_custom_target(proto_py
    DEPENDS "${PROTO_PY_DIR}/api_pb2.py" "${PROTO_PY_DIR}/app_config_pb2.py")

# -------------------------------------------------------------------------------------------------
# FreeRTOS and FreeRTOS-Plus-TCP
# -------------------------------------------------------------------------------------------------

set(FREERTOS_HEAP "3" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)
set(FREERTOS_PLUS_TCP_BUFFER_ALLOCATION "2" CACHE STRING "" FORCE)
set(FREERTOS_PLUS_TCP_NETWORK_IF "LIBSLIRP" CACHE STRING "" FORCE)

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/harness/include")

FetchContent_Declare(freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
)
FetchContent_Declare(freertos_plus_tcp
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Plus-TCP.git
    GIT_TAG        V4.3.1
)
FetchContent_MakeAvailable(freertos_kernel freertos_plus_tcp)

# TCP checksum fix: https://github.com/FreeRTOS/FreeRTOS-Plus-TCP/pull/1332
execute_process(
    COMMAND patch -p1 -N
        -i "${CMAKE_SOURCE_DIR}/patches/freertos_plus_tcp_ipv4_checksum.patch"
    WORKING_DIRECTORY "${freertos_plus_tcp_SOURCE_DIR}"
    RESULT_VARIABLE _patch_result
    OUTPUT_QUIET ERROR_QUIET)
if(_patch_result GREATER 1)
    message(FATAL_ERROR "Failed to apply FreeRTOS-Plus-TCP checksum patch (exit ${_patch_result})")
endif()

# -------------------------------------------------------------------------------------------------
# host_logging
# -------------------------------------------------------------------------------------------------

add_library(host_logging STATIC
    "${CMAKE_SOURCE_DIR}/src/logging.c"
    "${CMAKE_SOURCE_DIR}/src/string_util.c")
target_include_directories(host_logging
    PUBLIC
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/include/config"
    "${CMAKE_CURRENT_SOURCE_DIR}/harness/include")
target_compile_definitions(host_logging
    PUBLIC LOGGING_ENABLE_METADATA_WITH_C99_AND_GNU_EXTENSION=0)
target_link_libraries(host_logging
    PUBLIC freertos_kernel)

# -------------------------------------------------------------------------------------------------
# libraries
# -------------------------------------------------------------------------------------------------

FetchContent_Declare(coreMQTT
    GIT_REPOSITORY https://github.com/FreeRTOS/coreMQTT.git
    GIT_TAG        v2.3.1
)
FetchContent_MakeAvailable(coreMQTT)
include("${coremqtt_SOURCE_DIR}/mqttFilePaths.cmake")
add_library(coreMQTT STATIC)
target_sources(coreMQTT
    PRIVATE "${MQTT_SOURCES}" "${MQTT_SERIALIZER_SOURCES}")
target_include_directories(coreMQTT
    PUBLIC "${MQTT_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreMQTT
    PUBLIC freertos_kernel freertos_plus_tcp host_logging)

FetchContent_Declare(coreMQTT-Agent
    GIT_REPOSITORY https://github.com/FreeRTOS/coreMQTT-Agent.git
    GIT_TAG        v1.3.1
)
FetchContent_MakeAvailable(coreMQTT-Agent)
include("${coremqtt-agent_SOURCE_DIR}/mqttAgentFilePaths.cmake")
add_library(coreMQTT_agent STATIC)
target_sources(coreMQTT_agent
    PRIVATE "${MQTT_AGENT_SOURCES}")
target_include_directories(coreMQTT_agent
    PUBLIC "${MQTT_AGENT_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreMQTT_agent
    PUBLIC coreMQTT host_logging)

FetchContent_Declare(coreHTTP
    GIT_REPOSITORY https://github.com/FreeRTOS/coreHTTP.git
    GIT_TAG        v3.1.1
)
FetchContent_MakeAvailable(coreHTTP)
include("${corehttp_SOURCE_DIR}/httpFilePaths.cmake")
add_library(coreHTTP STATIC)
target_sources(coreHTTP
    PRIVATE "${HTTP_SOURCES}")
target_include_directories(coreHTTP
    PUBLIC "${HTTP_INCLUDE_PUBLIC_DIRS}")
target_link_libraries(coreHTTP
    PUBLIC
    freertos_plus_tcp
    freertos_kernel
    host_logging)

FetchContent_Declare(backoff_algorithm
    GIT_REPOSITORY https://github.com/FreeRTOS/backoffAlgorithm.git
    GIT_TAG        v1.4.1
)
FetchContent_MakeAvailable(backoff_algorithm)
include("${backoff_algorithm_SOURCE_DIR}/backoffAlgorithmFilePaths.cmake")
add_library(backoff_algorithm STATIC)
target_sources(backoff_algorithm
    PRIVATE "${BACKOFF_ALGORITHM_SOURCES}")
target_include_directories(backoff_algorithm
    PUBLIC "${BACKOFF_ALGORITHM_INCLUDE_PUBLIC_DIRS}")

add_library(freertos_plus_transport STATIC
    "${CMAKE_SOURCE_DIR}/libraries/network_transport/transport_plaintext.c"
    "${CMAKE_SOURCE_DIR}/libraries/network_transport/tcp_sockets_wrapper/ports/freertos_plus_tcp/tcp_sockets_wrapper.c")
target_include_directories(freertos_plus_transport
    PUBLIC
    "${CMAKE_SOURCE_DIR}/libraries/network_transport"
    "${CMAKE_SOURCE_DIR}/libraries/network_transport/tcp_sockets_wrapper/include")
target_link_libraries(freertos_plus_transport
    PUBLIC freertos_plus_tcp host_logging coreMQTT)

# -------------------------------------------------------------------------------------------------
# Harness + SUT sources
# -------------------------------------------------------------------------------------------------

FetchContent_Declare(mbedtls
    GIT_REPOSITORY https://github.com/Mbed-TLS/mbedtls.git
    GIT_TAG        v3.6.3
)
# do not build mbedtls programs and tests
set(ENABLE_PROGRAMS OFF CACHE BOOL INTERNAL)
set(ENABLE_TESTING OFF CACHE BOOL INTERNAL)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error=unterminated-string-initialization -Wno-error=logical-op -Wno-error=format-overflow")

FetchContent_MakeAvailable(mbedtls)

add_subdirectory(harness)
add_subdirectory(httpd)
add_subdirectory(mqtt)
add_subdirectory(ota)
add_subdirectory(syslog)
add_subdirectory(matter)

add_custom_target(host_tests
    DEPENDS host_test_httpd host_test_mqtt host_test_ota host_test_syslog host_test_matter proto_py)
