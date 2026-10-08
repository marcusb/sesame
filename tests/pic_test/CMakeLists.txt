cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "board root")
set(SOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "soc root")
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "dts root")
set(MW320_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mw320_sdk CACHE PATH "MW320 SDK directory")
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/mw320_sdk)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pic_test LANGUAGES C)

set(PIC_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../build/pic)
set(PIC_HEX_C ${PIC_BUILD_DIR}/pic_firmware_hex.c)

set(PIC_HEX_H ${PIC_BUILD_DIR}/pic_firmware_hex.h)

set(PIC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pic/sensor.asm)
set(PIC_HEX ${PIC_BUILD_DIR}/sensor.hex)

find_program(GPASM_BIN NAMES gpasm HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/gputils/usr/bin)

if(GPASM_BIN AND EXISTS ${PIC_SRC})
    add_custom_command(
        OUTPUT ${PIC_HEX} ${PIC_HEX_C} ${PIC_HEX_H}
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/pic/build_pic.sh ${PIC_BUILD_DIR}
        DEPENDS ${PIC_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/../../VERSION ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/pic/build_pic.sh
        COMMENT "Assembling PIC firmware ${PIC_HEX} for pic_test"
    )
    add_custom_target(pic_firmware_for_test DEPENDS ${PIC_HEX} ${PIC_HEX_C} ${PIC_HEX_H})
    add_dependencies(app pic_firmware_for_test)
else()
    if(NOT EXISTS ${PIC_HEX_C})
        file(MAKE_DIRECTORY ${PIC_BUILD_DIR})
        file(WRITE ${PIC_HEX_C}
            "/* Placeholder stub */\n"
            "#include <stddef.h>\n"
            "#include <stdint.h>\n"
            "const uint8_t pic_firmware_hex[1] = { 0 };\n"
            "const size_t pic_firmware_hex_len = 0;\n"
        )
    endif()

    if(NOT EXISTS ${PIC_HEX_H})
        file(MAKE_DIRECTORY ${PIC_BUILD_DIR})
        file(WRITE ${PIC_HEX_H}
            "/* Placeholder stub */\n"
            "#pragma once\n"
            "#include <stddef.h>\n"
            "#include <stdint.h>\n"
            "#define PIC_EXPECTED_VER_MAJOR 0\n"
            "#define PIC_EXPECTED_VER_MINOR 0\n"
            "#define PIC_EXPECTED_VER_PATCH 0\n"
            "#define PIC_EXPECTED_VER_COMMIT 0\n"
            "extern const uint8_t pic_firmware_hex[];\n"
            "extern const size_t pic_firmware_hex_len;\n"
        )
    endif()
endif()

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${PIC_BUILD_DIR}
)

target_sources(app PRIVATE
    src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pic_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pic_updater.c
    ${PIC_HEX_C}
)

