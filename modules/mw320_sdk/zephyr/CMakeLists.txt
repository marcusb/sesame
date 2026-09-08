zephyr_library()

if(NOT DEFINED MW320_SDK_DIR)
    get_filename_component(MW320_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../mw320_sdk ABSOLUTE)
endif()

if(CONFIG_SOC_88MW320)
    zephyr_library_sources(
        ${MW320_SDK_DIR}/devices/88MW320/system_88MW320.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_common.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_gpio.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_uart.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_aes.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_clock.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_pinmux.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_power.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_wdt.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_flashc.c
        ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_qspi.c
        fsl_debug_console.c
        ${MW320_SDK_DIR}/devices/88MW320/utilities/str/fsl_str.c
        ${MW320_SDK_DIR}/components/serial_manager/fsl_component_serial_manager.c
        ${MW320_SDK_DIR}/components/serial_manager/fsl_component_serial_port_uart.c
        ${MW320_SDK_DIR}/components/uart/fsl_adapter_mwuart.c
        ${MW320_SDK_DIR}/components/flash/mflash/mw320/mflash_drv.c
    )
endif()

zephyr_compile_definitions(CONFIG_FLASH_PARTITION_COUNT=10)
zephyr_include_directories(
    ${MW320_SDK_DIR}/components/serial_manager
    ${MW320_SDK_DIR}/components/uart
    ${MW320_SDK_DIR}/components/flash/mflash
    ${MW320_SDK_DIR}/components/flash/mflash/mw320
    ${MW320_SDK_DIR}/devices/88MW320
    ${MW320_SDK_DIR}/devices/88MW320/drivers
    ${MW320_SDK_DIR}/devices/88MW320/utilities/debug_console
    ${MW320_SDK_DIR}/devices/88MW320/utilities/str
    ${CMAKE_CURRENT_SOURCE_DIR}/dummy_includes
    ../../../include
)

zephyr_library_sources_ifdef(CONFIG_MBEDTLS_AES_C ${MW320_SDK_DIR}/middleware/mbedtls/port/mw/aes_alt.c)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_CCM_C ${MW320_SDK_DIR}/middleware/mbedtls/port/mw/ccm_alt.c)

# Only relocate code for flash build; RAM build runs entirely from SRAM
if(NOT CONFIG_BOARD_MARVELL_MW302_RAM)
zephyr_code_relocate(FILES 
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_clock.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_power.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_flashc.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_qspi.c
    ${MW320_SDK_DIR}/components/flash/mflash/mw320/mflash_drv.c
    LOCATION RAM)
endif()
if(CONFIG_SOC_88MW320)
    zephyr_library_sources_ifdef(CONFIG_UART_MW320 ../../../drivers/serial/uart_mw320.c)
    zephyr_library_sources_ifdef(CONFIG_GPIO_MW320 ../../../drivers/gpio/gpio_mw320.c)
    zephyr_library_sources_ifdef(CONFIG_ENTROPY_MW320 ../../../drivers/entropy/entropy_mw320.c)
    zephyr_library_sources_ifdef(CONFIG_FLASH ../../../drivers/flash/flash_mw320.c)
    zephyr_library_sources_ifdef(CONFIG_PINCTRL_MW320 ../../../drivers/pinctrl/pinctrl_mw320.c)
    zephyr_library_sources_ifdef(CONFIG_WATCHDOG_MW320 ../../../drivers/watchdog/wdt_mw320.c)
endif()
set_property(TARGET drivers__serial PROPERTY ALLOW_EMPTY TRUE)
set_property(TARGET drivers__entropy PROPERTY ALLOW_EMPTY TRUE)
set_property(TARGET drivers__retained_mem PROPERTY ALLOW_EMPTY TRUE)
