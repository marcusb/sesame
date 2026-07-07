# MW320 SDK targets for Zephyr (no FreeRTOS, no CMSIS v2)
# Only includes bare metal drivers, no OSA/FreeRTOS code

macro(add_driver drv)
    add_library("mw320_drivers_${drv}" STATIC)
    target_sources("mw320_drivers_${drv}"
        PRIVATE
        "${mw320_sdk_dir}/devices/88MW320/drivers/fsl_${drv}.c")
    target_include_directories("mw320_drivers_${drv}"
        PUBLIC
        "${mw320_sdk_dir}/devices/88MW320/drivers")
    target_link_libraries("mw320_drivers_${drv}"
        PUBLIC
        mw320_drivers_common)
endmacro()

# SDK compile definitions (applied to all targets)
set(__sdk_defs
    CPU_88MW320_A0_NAPC
    PRINTF_FLOAT_ENABLE=0
    SCANF_FLOAT_ENABLE=0
    PRINTF_ADVANCED_ENABLE=0
    SCANF_ADVANCED_ENABLE=0
    SERIAL_PORT_TYPE_UART=1)

add_library(mw320_drivers_common STATIC)
target_sources(mw320_drivers_common
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/drivers/fsl_common.c")
target_include_directories(mw320_drivers_common
    PUBLIC
    "${mw320_sdk_dir}/devices/88MW320/drivers"
    "${mw320_sdk_dir}/devices/88MW320"
    "${mw320_sdk_dir}/CMSIS/Include")
target_compile_definitions(mw320_drivers_common PUBLIC ${__sdk_defs})
target_compile_options(mw320_drivers_common PRIVATE
    -imacros "${CMAKE_CURRENT_LIST_DIR}/include/wifi_config.h")
target_link_libraries(mw320_drivers_common PUBLIC mw320_drivers_clock)

add_driver(clock)
add_driver(gpio)
add_driver(pinmux)
add_driver(power)
add_driver(uart)

add_library(mw320_device_system STATIC)
target_sources(mw320_device_system
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/system_88MW320.c")
target_include_directories(mw320_device_system
    PUBLIC
    "${mw320_sdk_dir}/devices/88MW320")
target_compile_definitions(mw320_device_system PUBLIC ${__sdk_defs})

# Apply Zephyr compiler flags to all SDK targets
function(sdk_apply_zephyr_flags target)
    target_compile_options(${target} PRIVATE
        $<TARGET_PROPERTY:app,COMPILE_OPTIONS>)
    target_include_directories(${target} PRIVATE
        $<TARGET_PROPERTY:app,INCLUDE_DIRECTORIES>)
    target_compile_definitions(${target} PRIVATE
        $<TARGET_PROPERTY:app,COMPILE_DEFINITIONS>)
endfunction()

sdk_apply_zephyr_flags(mw320_drivers_common)
sdk_apply_zephyr_flags(mw320_drivers_clock)
sdk_apply_zephyr_flags(mw320_drivers_gpio)
sdk_apply_zephyr_flags(mw320_drivers_pinmux)
sdk_apply_zephyr_flags(mw320_drivers_power)
sdk_apply_zephyr_flags(mw320_drivers_uart)
sdk_apply_zephyr_flags(mw320_device_system)
