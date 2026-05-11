cmake_minimum_required(VERSION 3.23)

project(sesame_matter LANGUAGES C)

add_library(sesame_matter STATIC
    matter_task.c
    matter_tasmota_shim.c
    matter_crypto_shim.c
    matter_berry_port.c
    matter_mdns.c
    mdns_responder.c
    mdns_mcast_join.c
    psm_safe.c
)

# crypto_spake2p_matter.be is now solidified at build time by the
# berry_matter_solidify_extra target — its bytecode is linked into the
# binary and registered onto the runtime-built `crypto` module from
# matter_crypto_shim.c. Make sure the solidified header is produced before
# this library compiles.
add_dependencies(sesame_matter berry_matter_solidify_extra)

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
    proto
    backoff_algorithm
)

if(NOT USE_QEMU)
    target_link_libraries(sesame_matter PUBLIC mw320_sdk)
endif()
