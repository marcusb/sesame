zephyr_library()
set(MW320_SDK_DIR ${CMAKE_SOURCE_DIR}/mw320_sdk)

zephyr_library_sources(
    ${MW320_SDK_DIR}/devices/88MW320/system_88MW320.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_common.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_gpio.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_uart.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_aes.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_clock.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_pinmux.c
)
zephyr_include_directories(
    ${MW320_SDK_DIR}/devices/88MW320
    ${MW320_SDK_DIR}/devices/88MW320/drivers
)

zephyr_library_sources_ifdef(CONFIG_MBEDTLS_AES_C ${MW320_SDK_DIR}/middleware/mbedtls/port/mw/aes_alt.c)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_CCM_C ${MW320_SDK_DIR}/middleware/mbedtls/port/mw/ccm_alt.c)

# Only relocate code for flash build; RAM build runs entirely from SRAM
if(NOT CONFIG_BOARD_MARVELL_MW302_RAM)
zephyr_code_relocate(FILES 
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_clock.c
    LOCATION RAM)
endif()
