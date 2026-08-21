zephyr_library()
add_subdirectory_ifdef(CONFIG_FLASH_MW320 ../drivers/flash drivers/flash)
add_subdirectory_ifdef(CONFIG_UART_MW320 ../drivers/serial drivers/serial)
add_subdirectory_ifdef(CONFIG_GPIO_MW320 ../drivers/gpio drivers/gpio)
add_subdirectory_ifdef(CONFIG_PINCTRL_MW320 ../drivers/pinctrl drivers/pinctrl)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../tools/axf2firmware
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/axf2firmware/axf2firmware
            ${PROJECT_BINARY_DIR}/zephyr/zephyr.elf
            ${PROJECT_BINARY_DIR}/zephyr/mcuboot.bin
)
