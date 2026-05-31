# Zephyr Standalone Application CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)


list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/mw320_sdk)

# Load Zephyr build system
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sesame)

# Application Sources
target_sources(app PRIVATE 
    src/main.c
    # We will progressively add files as we port them
    # src/network_manager.c (Needs rewrite)
    # src/ota.c (Needs rewrite)
)

# Bring in the custom sdk module and drivers
zephyr_include_directories(modules/mw320_sdk/zephyr)
zephyr_include_directories(include)
# Drivers are built as a Zephyr module via zephyr/module.yml
