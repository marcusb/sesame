cmake_minimum_required(VERSION 3.23)

project(berry_matter LANGUAGES C)

set(BERRY_MATTER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(BERRY_MATTER_SOURCES
    "${BERRY_MATTER_SRC_DIR}/be_matter_counter.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_misc.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_module.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_verhoeff.c"
)

add_library(berry_matter STATIC ${BERRY_MATTER_SOURCES})

target_include_directories(berry_matter
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/solidify"
    "${CMAKE_CURRENT_SOURCE_DIR}/generate"
    "${CMAKE_SOURCE_DIR}/mw320_sdk/middleware/wifi/incl"
    "${CMAKE_SOURCE_DIR}/mw320_sdk/components/boot2_utils"
)

target_link_libraries(berry_matter
    PUBLIC
    berry
)

target_link_libraries(berry_matter PUBLIC freertos_kernel freertos_plus_tcp)
target_link_libraries(berry_matter PUBLIC globals)
