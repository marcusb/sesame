cmake_minimum_required(VERSION 3.23)

project(berry LANGUAGES C)

set(BERRY_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(BERRY_DEF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/default")

set(BERRY_MAPPING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../berry_mapping/src")
set(BERRY_INT64_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/../berry_int64/src")
set(BERRY_CUSTOM_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../berry_custom/src")
set(BERRY_MATTER_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../berry_matter/src")
set(RE15_DIR          "${CMAKE_CURRENT_SOURCE_DIR}/../re1.5")

set(SESAME_MATTER_SHIM_DIR "${CMAKE_SOURCE_DIR}/src/matter")

set(BERRY_GEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/generate")
file(MAKE_DIRECTORY "${BERRY_GEN_OUT}")

# ----- coc prebuild -----
set(COC_TOOL "${CMAKE_CURRENT_SOURCE_DIR}/tools/coc/coc")
set(COC_CONF "${BERRY_DEF_DIR}/berry_conf.h")

set(COC_INPUT_DIRS
    "${BERRY_SRC_DIR}"
    "${BERRY_MAPPING_DIR}"
    "${BERRY_INT64_DIR}"
    "${BERRY_CUSTOM_DIR}"
    "${BERRY_MATTER_DIR}"
    "${BERRY_MATTER_DIR}/solidify"
    "${SESAME_MATTER_SHIM_DIR}"
)

file(GLOB COC_DEP_FILES
    "${BERRY_SRC_DIR}/*.c" "${BERRY_SRC_DIR}/*.h"
    "${BERRY_MAPPING_DIR}/*.c" "${BERRY_MAPPING_DIR}/*.h"
    "${BERRY_INT64_DIR}/*.c" "${BERRY_INT64_DIR}/*.h"
    "${BERRY_CUSTOM_DIR}/*.c" "${BERRY_CUSTOM_DIR}/*.h"
    "${BERRY_MATTER_DIR}/*.c" "${BERRY_MATTER_DIR}/*.h"
    "${BERRY_MATTER_DIR}/solidify/*.h"
    "${SESAME_MATTER_SHIM_DIR}/*.c" "${SESAME_MATTER_SHIM_DIR}/*.h"
)

set(COC_STAMP "${BERRY_GEN_OUT}/.coc.stamp")

add_custom_command(
    OUTPUT "${COC_STAMP}"
    COMMAND python3 ${COC_TOOL} -o ${BERRY_GEN_OUT} -c ${COC_CONF} -- ${COC_INPUT_DIRS}
    COMMAND ${CMAKE_COMMAND} -E touch ${COC_STAMP}
    DEPENDS ${COC_DEP_FILES} ${COC_TOOL} ${COC_CONF}
    COMMENT "Running coc prebuild (Berry const tables)"
    VERBATIM
)

add_custom_target(berry_coc DEPENDS "${COC_STAMP}")

# ----- berry library -----
file(GLOB BERRY_SOURCES "${BERRY_SRC_DIR}/*.c")
file(GLOB BERRY_MAPPING_SOURCES "${BERRY_MAPPING_DIR}/*.c")
file(GLOB BERRY_INT64_SOURCES   "${BERRY_INT64_DIR}/*.c")
file(GLOB BERRY_CUSTOM_SOURCES  "${BERRY_CUSTOM_DIR}/*.c")

list(REMOVE_ITEM BERRY_MAPPING_SOURCES "${BERRY_MAPPING_DIR}/be_mapping_security_tests.c")
list(REMOVE_ITEM BERRY_SOURCES "${BERRY_SRC_DIR}/be_oslib.c")

set(RE15_SOURCES
    "${RE15_DIR}/compilecode.c"
    "${RE15_DIR}/pike.c"
    "${RE15_DIR}/charclass.c"
)

add_library(berry STATIC
    ${BERRY_SOURCES}
    ${BERRY_MAPPING_SOURCES}
    ${BERRY_INT64_SOURCES}
    ${BERRY_CUSTOM_SOURCES}
    ${RE15_SOURCES}
)

add_dependencies(berry berry_coc)

target_include_directories(berry
    PUBLIC
    "${BERRY_SRC_DIR}"
    "${BERRY_GEN_OUT}"
    "${BERRY_DEF_DIR}"
    "${BERRY_MAPPING_DIR}"
    "${BERRY_INT64_DIR}"
    "${BERRY_CUSTOM_DIR}"
    "${RE15_DIR}"
)

target_compile_definitions(berry
    PUBLIC
    TASMOTA
    BERRY_MODULE_CONTINUE
)

if(CMAKE_CROSSCOMPILING)
    target_link_libraries(berry PUBLIC globals)
endif()
