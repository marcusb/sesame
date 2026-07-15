if(CONFIG_WIFI_MW320)

set(MW320_WIFI_DIR ${MW320_SDK_DIR}/middleware/wifi)

file(GLOB MW320_WIFIDRIVER_SRCS
    ${MW320_WIFI_DIR}/wifidriver/*.c
)

target_sources(app PRIVATE
    wifi_mw320.c
    os_zephyr.c
    ${MW320_WIFIDRIVER_SRCS}
    ${MW320_SDK_DIR}/boards/rdmw320_r0/sdmmc_config.c
    ${MW320_SDK_DIR}/middleware/sdmmc/sdio/fsl_sdio.c
    ${MW320_SDK_DIR}/middleware/sdmmc/host/sdioc/non_blocking/fsl_sdmmc_host.c
    ${MW320_SDK_DIR}/middleware/sdmmc/common/fsl_sdmmc_common.c
    ${MW320_SDK_DIR}/middleware/sdmmc/osa/fsl_sdmmc_osa.c
    ${MW320_SDK_DIR}/devices/88MW320/drivers/fsl_sdioc.c
    ${MW320_WIFI_DIR}/wlcmgr/wlan.c
    ${MW320_WIFI_DIR}/wlcmgr/wlan_txpwrlimit_cfg.c
)

# Use Zephyr's port of the OS abstractions instead of the FreeRTOS ones in the SDK
target_include_directories(app PRIVATE
    os_zephyr
    ${MW320_WIFI_DIR}/incl
    ${MW320_WIFI_DIR}/incl/port/os
    ${MW320_WIFI_DIR}/incl/wlcmgr
    ${MW320_WIFI_DIR}/incl/wifidriver
    ${MW320_WIFI_DIR}/wifidriver
    ${MW320_WIFI_DIR}/wifidriver/incl
    ${MW320_SDK_DIR}/boards/rdmw320_r0
    ${MW320_SDK_DIR}/middleware/sdmmc/sdio
    ${MW320_SDK_DIR}/middleware/sdmmc/host
    ${MW320_SDK_DIR}/middleware/sdmmc/host/sdioc
    ${MW320_SDK_DIR}/middleware/sdmmc/common
    ${MW320_SDK_DIR}/middleware/sdmmc/osa
    ${MW320_SDK_DIR}/components/osa
    ${MW320_SDK_DIR}/components/lists
)

target_compile_options(app PRIVATE
    -DSD8801
    -DSDIO_ENABLED
    -include ${CMAKE_CURRENT_SOURCE_DIR}/os_zephyr/wm_os.h
)

endif()
