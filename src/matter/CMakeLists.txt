cmake_minimum_required(VERSION 3.23)

project(sesame_matter LANGUAGES C CXX)

add_library(sesame_matter STATIC
    matter_app.cpp
    psm_safe.c
    sesame_matter_stubs.c
)

target_include_directories(sesame_matter
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(sesame_matter
    PUBLIC
    chip
    freertos_plus_tcp
    globals
    logging
    mbedcrypto
    proto
    backoff_algorithm
)

if(NOT USE_QEMU)
    target_link_libraries(sesame_matter PUBLIC mw320_sdk)
endif()
