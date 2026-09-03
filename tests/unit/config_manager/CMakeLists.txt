cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_config_manager)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# Add nanopb target manually for tests
set(nanopb_SOURCE_DIR ${ROOT_DIR}/third_party/connectedhomeip/third_party/nanopb/repo)
add_subdirectory(${nanopb_SOURCE_DIR} ${CMAKE_BINARY_DIR}/nanopb)
list(APPEND CMAKE_MODULE_PATH "${nanopb_SOURCE_DIR}/extra")
find_package(Nanopb REQUIRED)

# Generate nanopb sources
nanopb_generate_cpp(TARGET proto_app RELPATH proto ${ROOT_DIR}/proto/app_config.proto ${ROOT_DIR}/proto/api.proto)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(nanopb PRIVATE zephyr_interface)
target_link_libraries(app PRIVATE proto_app nanopb)

target_sources(app PRIVATE
    ${ROOT_DIR}/src/config_manager.c
    test_config_manager.c
)

target_include_directories(app PRIVATE
    ${ROOT_DIR}/include
)
