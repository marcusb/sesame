cmake_minimum_required(VERSION 3.23)

project(berry_matter LANGUAGES C)

set(BERRY_MATTER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(BERRY_MATTER_EMBED_DIR "${BERRY_MATTER_SRC_DIR}/embedded")

# ------------------------------------------------------------------------
# Solidify .be sources -> generated solidified_*.h headers.
#
# berry_host is built in the native tree and exported via berry_host-config.cmake;
# the top-level CMakeLists.txt runs find_package(berry_host) before this
# subdirectory is added, so the imported target is available here.
#
# Output goes to ${CMAKE_CURRENT_BINARY_DIR}/solidify/; that path replaces
# the (no-longer-checked-in) src/solidify/ on the berry_matter include path.
# ------------------------------------------------------------------------
set(BERRY_MATTER_SOLIDIFY_OUT "${CMAKE_CURRENT_BINARY_DIR}/solidify")
file(MAKE_DIRECTORY "${BERRY_MATTER_SOLIDIFY_OUT}")

file(GLOB BERRY_MATTER_BE_SOURCES CONFIGURE_DEPENDS
    "${BERRY_MATTER_EMBED_DIR}/*.be")

# Compute the list of expected output headers from the .be inputs so we can
# express the dependency precisely: every solidified_*.h is regenerated when
# any .be source changes (the driver script is a single batch invocation).
set(BERRY_MATTER_SOLIDIFY_HEADERS "")
foreach(be ${BERRY_MATTER_BE_SOURCES})
    get_filename_component(stem "${be}" NAME_WE)
    if(stem MATCHES "_test$")
        continue()
    endif()
    list(APPEND BERRY_MATTER_SOLIDIFY_HEADERS
        "${BERRY_MATTER_SOLIDIFY_OUT}/solidified_${stem}.h")
endforeach()

set(BERRY_MATTER_SOLIDIFY_DRIVER
    "${CMAKE_SOURCE_DIR}/third_party/berry/sesame_solidify/solidify_dir.be")

add_custom_command(
    OUTPUT ${BERRY_MATTER_SOLIDIFY_HEADERS}
    COMMAND $<TARGET_FILE:berry_host> -g
        "${BERRY_MATTER_SOLIDIFY_DRIVER}"
        "${BERRY_MATTER_EMBED_DIR}"
        "${BERRY_MATTER_SOLIDIFY_OUT}"
    DEPENDS
        berry_host
        ${BERRY_MATTER_SOLIDIFY_DRIVER}
        ${BERRY_MATTER_BE_SOURCES}
    COMMENT "Solidifying berry_matter .be sources"
    VERBATIM
)

add_custom_target(berry_matter_solidify
    DEPENDS ${BERRY_MATTER_SOLIDIFY_HEADERS})

# Solidify the .be sources under extra/ (currently just crypto_spake2p_matter.be)
# into a parallel output dir. These aren't part of the matter Berry module
# bundle — they're consumed by C code in src/matter/matter_crypto_shim.c that
# attaches the resulting classes to runtime-constructed native modules.
set(BERRY_MATTER_EXTRA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extra")
set(BERRY_MATTER_EXTRA_OUT "${CMAKE_CURRENT_BINARY_DIR}/solidify_extra")
file(MAKE_DIRECTORY "${BERRY_MATTER_EXTRA_OUT}")

file(GLOB BERRY_MATTER_EXTRA_BE CONFIGURE_DEPENDS
    "${BERRY_MATTER_EXTRA_DIR}/*.be")

set(BERRY_MATTER_EXTRA_HEADERS "")
foreach(be ${BERRY_MATTER_EXTRA_BE})
    get_filename_component(stem "${be}" NAME_WE)
    list(APPEND BERRY_MATTER_EXTRA_HEADERS
        "${BERRY_MATTER_EXTRA_OUT}/solidified_${stem}.h")
endforeach()

add_custom_command(
    OUTPUT ${BERRY_MATTER_EXTRA_HEADERS}
    COMMAND $<TARGET_FILE:berry_host> -g
        "${BERRY_MATTER_SOLIDIFY_DRIVER}"
        "${BERRY_MATTER_EXTRA_DIR}"
        "${BERRY_MATTER_EXTRA_OUT}"
    DEPENDS
        berry_host
        ${BERRY_MATTER_SOLIDIFY_DRIVER}
        ${BERRY_MATTER_EXTRA_BE}
    COMMENT "Solidifying berry_matter extra/ .be sources"
    VERBATIM
)

add_custom_target(berry_matter_solidify_extra
    DEPENDS ${BERRY_MATTER_EXTRA_HEADERS})

set(BERRY_MATTER_SOURCES
    "${BERRY_MATTER_SRC_DIR}/be_matter_counter.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_misc.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_module.c"
    "${BERRY_MATTER_SRC_DIR}/be_matter_verhoeff.c"
)

add_library(berry_matter STATIC ${BERRY_MATTER_SOURCES})
add_dependencies(berry_matter berry_matter_solidify)

target_include_directories(berry_matter
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${BERRY_MATTER_SOLIDIFY_OUT}/.."   # so #include "solidify/solidified_*.h" resolves
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
