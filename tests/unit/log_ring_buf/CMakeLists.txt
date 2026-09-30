cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_log_ring_buf)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

target_sources(app PRIVATE
    ${ROOT_DIR}/src/log_ring_buf.c
    test_log_ring_buf.c
)

target_include_directories(app PRIVATE
    ${ROOT_DIR}/include
)
