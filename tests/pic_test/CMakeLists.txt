cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "board root")
set(SOC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "soc root")
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "dts root")
set(MW320_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mw320_sdk CACHE PATH "MW320 SDK directory")
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/mw320_sdk)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pic_test LANGUAGES C)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

target_sources(app PRIVATE
    src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pic_uart.c
)
