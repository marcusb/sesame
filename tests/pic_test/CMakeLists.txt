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

