add_library(integration_harness STATIC
    harness.c
    stubs/time_util.c
    stubs/util_stub.c
)

target_include_directories(integration_harness
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/include")

target_link_libraries(integration_harness
    PUBLIC
    app_deps)
