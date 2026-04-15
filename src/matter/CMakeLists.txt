cmake_minimum_required(VERSION 3.23)

project(sesame_matter LANGUAGES C)

add_library(sesame_matter STATIC
    matter_task.c
    matter_tasmota_shim.c
    matter_crypto_shim.c
    matter_berry_port.c
)

target_include_directories(sesame_matter
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(sesame_matter
    PUBLIC
    berry
    berry_matter
    freertos_plus_tcp
    globals
    logging
    mbedcrypto
    mw320_sdk
    proto
)
