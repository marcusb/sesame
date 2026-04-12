add_library(host_harness STATIC
    harness.c
    host_sockets.c
    log_stdout.c
    stubs/time_util.c
    stubs/util_stub.c
)

target_include_directories(host_harness
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/include"
    ${SLIRP_INCLUDE_DIRECTORIES})

target_compile_definitions(host_harness
    PUBLIC
    _GNU_SOURCE
    _DEFAULT_SOURCE)

target_link_libraries(host_harness
    PUBLIC
    freertos_kernel
    freertos_plus_tcp
    host_logging
    proto
    PRIVATE
    PkgConfig::SLIRP)

# LD_PRELOAD shim that intercepts slirp_new() to add port forwarding.
add_library(slirp_hostfwd SHARED slirp_hostfwd.c)
target_link_libraries(slirp_hostfwd PRIVATE PkgConfig::SLIRP ${CMAKE_DL_LIBS})
set_target_properties(slirp_hostfwd PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
