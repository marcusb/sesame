cmake_minimum_required(VERSION 3.23)

project(sesame_matter LANGUAGES C)

add_library(sesame_matter STATIC
    matter_task.c
    matter_tasmota_shim.c
    matter_crypto_shim.c
    matter_berry_port.c
    matter_mdns.c
)

# Embed Berry sources we run at startup as C string constants. The upstream
# solidify pipeline (which would compile these to bytecode) can't run on the
# host yet (see tools/solidify_berry_matter.sh), so we ship the raw .be source
# and compile it in-process with BE_USE_SCRIPT_COMPILER.
set(BERRY_EMBED_DIR "${CMAKE_CURRENT_BINARY_DIR}/embedded_be")
file(MAKE_DIRECTORY "${BERRY_EMBED_DIR}")
set(BERRY_EMBED_HDR "${BERRY_EMBED_DIR}/embedded_be.h")
set(BERRY_EMBED_INPUTS
    "${CMAKE_SOURCE_DIR}/third_party/berry_matter/extra/crypto_spake2p_matter.be"
)
add_custom_command(
    OUTPUT "${BERRY_EMBED_HDR}"
    COMMAND ${CMAKE_COMMAND}
        -DOUT=${BERRY_EMBED_HDR}
        -DINPUTS=${BERRY_EMBED_INPUTS}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/embed_be.cmake"
    DEPENDS ${BERRY_EMBED_INPUTS} "${CMAKE_CURRENT_SOURCE_DIR}/embed_be.cmake"
    COMMENT "Embedding Berry sources -> embedded_be.h"
    VERBATIM
)
add_custom_target(sesame_matter_embed_be DEPENDS "${BERRY_EMBED_HDR}")
add_dependencies(sesame_matter sesame_matter_embed_be)
target_include_directories(sesame_matter PRIVATE "${BERRY_EMBED_DIR}")

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
