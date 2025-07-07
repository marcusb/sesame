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

add_library(mw320_cmsis_cm4 INTERFACE)
target_include_directories(mw320_cmsis_cm4
    INTERFACE
    "${mw320_sdk_dir}/CMSIS/Include"
    "${mw320_sdk_dir}/devices/88MW320")
target_compile_definitions(mw320_cmsis_cm4
    INTERFACE
    CPU_88MW320_A0_NAPC
    PRINTF_FLOAT_ENABLE=0
    SCANF_FLOAT_ENABLE=0
    PRINTF_ADVANCED_ENABLE=0
    SCANF_ADVANCED_ENABLE=0
    SERIAL_PORT_TYPE_UART=1)
target_compile_options(mw320_cmsis_cm4
    INTERFACE
    -imacros "${CMAKE_CURRENT_LIST_DIR}/include/wifi_config.h")
target_link_libraries(mw320_cmsis_cm4
    INTERFACE
    globals)

add_library(mw320_device_system STATIC)
target_sources(mw320_device_system
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/system_88MW320.c")
target_include_directories(mw320_device_system
    PUBLIC
    "${mw320_sdk_dir}/devices/88MW320")
target_link_libraries(mw320_device_system
    PUBLIC
    mw320_cmsis_cm4)

add_library(mw320_device_startup STATIC)
target_sources(mw320_device_startup
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/gcc/startup_88MW320.S")
target_link_libraries(mw320_device_startup
    PUBLIC
    mw320_device_system
)

add_library(mw320_lists STATIC)
target_sources(mw320_lists
    PRIVATE
    "${mw320_sdk_dir}/components/lists/fsl_component_generic_list.c")
target_include_directories(mw320_lists
    PUBLIC
    "${mw320_sdk_dir}/components/lists")
target_link_libraries(mw320_lists
    PUBLIC
    mw320_drivers_common)

add_library(mw320_drivers_common STATIC)
target_sources(mw320_drivers_common
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/drivers/fsl_common.c")
target_include_directories(mw320_drivers_common
    PUBLIC
    "${mw320_sdk_dir}/devices/88MW320/drivers"
)
target_link_libraries(mw320_drivers_common
    PUBLIC
    mw320_drivers_clock
    mw320_cmsis_cm4
)

add_driver(aes)
add_driver(clock)
add_driver(crc)
add_driver(flashc)
add_driver(gpio)
add_driver(pinmux)
add_driver(power)
add_driver(qspi)
add_driver(rtc)
add_driver(sdioc)
add_driver(uart)
add_driver(wdt)

add_library(mw320_osa STATIC)
target_sources(mw320_osa
    PRIVATE
    "${mw320_sdk_dir}/components/osa/fsl_os_abstraction_free_rtos.c"
)
target_include_directories(mw320_osa
    PUBLIC
    "${mw320_sdk_dir}/components/osa"
)
target_link_libraries(mw320_osa
    PUBLIC
    mw320_drivers_common
    mw320_drivers_pinmux
    mw320_lists
    PRIVATE
    freertos_kernel
)

add_library(mw320_component_mflash STATIC)
target_sources(mw320_component_mflash
    PRIVATE
    "${mw320_sdk_dir}/components/flash/mflash/mw320/mflash_drv.c"
)
target_include_directories(mw320_component_mflash
    PUBLIC
    "${mw320_sdk_dir}/components/flash/mflash"
    "${mw320_sdk_dir}/components/flash/mflash/mw320"
)
target_link_libraries(mw320_component_mflash
    PUBLIC
    mw320_drivers_qspi
    mw320_drivers_flashc
    board
)

add_library(mw320_boot2_partition STATIC)
target_sources(mw320_boot2_partition
    PRIVATE
    "${mw320_sdk_dir}/components/boot2_utils/boot_flags.c"
    "${mw320_sdk_dir}/components/boot2_utils/crc32.c"
    "${mw320_sdk_dir}/components/boot2_utils/partition.c"
)
target_include_directories(mw320_boot2_partition
    PUBLIC
    "${mw320_sdk_dir}/components/boot2_utils"
    "${mw320_sdk_dir}/middleware/wifi/incl"
)
target_link_libraries(mw320_boot2_partition
    PUBLIC
    mw320_drivers_crc
    mw320_drivers_common
    mw320_component_mflash
    PRIVATE
    mw320_utility_debug_console
)

add_library(mw320_boot2_psm STATIC)
target_sources(mw320_boot2_psm
    PRIVATE
    "${mw320_sdk_dir}/components/boot2_utils/keystore.c"
    "${mw320_sdk_dir}/components/boot2_utils/psm-v2.c"
    "${mw320_sdk_dir}/components/boot2_utils/psm-v2-secure.c"
    "${mw320_sdk_dir}/components/boot2_utils/wmtlv.c"
)
target_include_directories(mw320_boot2_psm
    PUBLIC
    "${mw320_sdk_dir}/components/boot2_utils"
    PRIVATE
    "${mw320_sdk_dir}/middleware/wifi/incl/port/os"
)
target_link_libraries(mw320_boot2_psm
    PUBLIC
    mw320_boot2_partition
    mw320_utility_debug_console
    freertos_kernel
)

add_library(mw320_mw_uart_adapter STATIC)
target_sources(mw320_mw_uart_adapter
    PRIVATE
    "${mw320_sdk_dir}/components/uart/fsl_adapter_mwuart.c"
)
target_include_directories(mw320_mw_uart_adapter
    PUBLIC
    "${mw320_sdk_dir}/components/uart"
)
target_link_libraries(mw320_mw_uart_adapter
    PUBLIC
    mw320_drivers_common
    mw320_drivers_uart
)

add_library(mw320_serial_manager_uart STATIC)
target_sources(mw320_serial_manager_uart
    PRIVATE
    "${mw320_sdk_dir}/components/serial_manager/fsl_component_serial_port_uart.c"
)
target_include_directories(mw320_serial_manager_uart
    PUBLIC
    "${mw320_sdk_dir}/components/serial_manager"
)
target_link_libraries(mw320_serial_manager_uart
    PUBLIC
    mw320_drivers_uart
    mw320_mw_uart_adapter
)

add_library(mw320_serial_manager STATIC)
target_sources(mw320_serial_manager
    PRIVATE
    "${mw320_sdk_dir}/components/serial_manager/fsl_component_serial_manager.c"
)
target_include_directories(mw320_serial_manager
    PUBLIC
    "${mw320_sdk_dir}/components/serial_manager"
)
target_link_libraries(mw320_serial_manager
    PUBLIC
    mw320_drivers_common
    mw320_lists
    mw320_serial_manager_uart
)

add_library(mw320_utility_debug_console STATIC)
target_sources(mw320_utility_debug_console
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/utilities/debug_console/fsl_debug_console.c"
    "${mw320_sdk_dir}/devices/88MW320/utilities/str/fsl_str.c"
)
target_include_directories(mw320_utility_debug_console
    PUBLIC
    "${mw320_sdk_dir}/devices/88MW320/utilities/debug_console"
    "${mw320_sdk_dir}/devices/88MW320/utilities/str"
)
target_link_libraries(mw320_utility_debug_console
    PUBLIC
    mw320_drivers_common
    mw320_serial_manager
)

add_library(mw320_utility_assert STATIC)
target_sources(mw320_utility_assert
    PRIVATE
    "${mw320_sdk_dir}/devices/88MW320/utilities/fsl_assert.c")
target_link_libraries(mw320_utility_assert
    PUBLIC
    mw320_utility_debug_console)

add_library(mw320_sdmmc_common STATIC)
target_sources(mw320_sdmmc_common
    PRIVATE
    "${mw320_sdk_dir}/middleware/sdmmc/common/fsl_sdmmc_common.c"
)
target_include_directories(mw320_sdmmc_common
    PUBLIC
    "${mw320_sdk_dir}/middleware/sdmmc/common"
)
target_link_libraries(mw320_sdmmc_common
    PUBLIC
    mw320_drivers_common
    mw320_sdmmc_host_sdioc
)

add_library(mw320_sdmmc_host_sdioc INTERFACE)
target_include_directories(mw320_sdmmc_host_sdioc
    INTERFACE
    "${mw320_sdk_dir}/middleware/sdmmc/host/sdioc"
)
target_link_libraries(mw320_sdmmc_host_sdioc
    INTERFACE
    mw320_sdmmc_host_sdioc_freertos
)

add_library(mw320_sdmmc_osa_freertos STATIC)
target_sources(mw320_sdmmc_osa_freertos
    PRIVATE
    "${mw320_sdk_dir}/middleware/sdmmc/osa/fsl_sdmmc_osa.c"
)
target_include_directories(mw320_sdmmc_osa_freertos
    PUBLIC
    "${mw320_sdk_dir}/middleware/sdmmc/osa"
)
target_link_libraries(mw320_sdmmc_osa_freertos
    PUBLIC
    mw320_osa
)

add_library(mw320_sdmmc_host_sdioc_freertos STATIC)
target_sources(mw320_sdmmc_host_sdioc_freertos
    PRIVATE
    "${mw320_sdk_dir}/middleware/sdmmc/host/sdioc/non_blocking/fsl_sdmmc_host.c"
)
target_include_directories(mw320_sdmmc_host_sdioc_freertos
    PUBLIC
    "${mw320_sdk_dir}/middleware/sdmmc/host/sdioc"
)
target_link_libraries(mw320_sdmmc_host_sdioc_freertos
    PUBLIC
    mw320_sdmmc_common
    mw320_sdmmc_osa_freertos
    mw320_drivers_sdioc
)

add_library(mw320_sdmmc_sdio STATIC)
target_sources(mw320_sdmmc_sdio
    PRIVATE
    "${mw320_sdk_dir}/middleware/sdmmc/sdio/fsl_sdio.c"
)
target_include_directories(mw320_sdmmc_sdio
    PUBLIC
    "${mw320_sdk_dir}/middleware/sdmmc/sdio"
)
target_link_libraries(mw320_sdmmc_sdio
    PUBLIC
    mw320_sdmmc_common
    mw320_sdmmc_host_sdioc
)

add_library(mw320_wifidriver STATIC)
target_sources(mw320_wifidriver
    PRIVATE
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11ac.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_scan.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_api.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sta_cmdresp.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sdio.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_txrx.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_cmdevt.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11n_rxreorder.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_join.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi-sdio.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sta_rx.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_cfp.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11n_aggr.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_uap_cmdevent.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11d.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_init.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11h.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_wmm.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sta_cmd.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi-uap.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi-debug.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_misc.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_shim.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_11n.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi_pwrmgr.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_uap_ioctl.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/wifi-mem.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_glue.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sta_event.c"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/mlan_sta_ioctl.c"
)
target_include_directories(mw320_wifidriver
    PUBLIC
    "${mw320_sdk_dir}/middleware/wifi/incl"
    "${mw320_sdk_dir}/middleware/wifi/incl/port/os"
    "${mw320_sdk_dir}/middleware/wifi/incl/wifidriver"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver"
    "${mw320_sdk_dir}/middleware/wifi/wifidriver/incl"
    PRIVATE
    "${mw320_sdk_dir}/middleware/wifi/incl/wlcmgr"
)
target_link_libraries(mw320_wifidriver
    PUBLIC
    mw320_sdmmc_sdio
    PRIVATE
    board
    mw320_utility_debug_console
    freertos_kernel
)

add_library(mw320_wlcmgr STATIC)
target_sources(mw320_wlcmgr
    PRIVATE
    "${mw320_sdk_dir}/middleware/wifi/wlcmgr/wlan.c"
    "${mw320_sdk_dir}/middleware/wifi/wlcmgr/wlan_txpwrlimit_cfg.c"
    "${mw320_sdk_dir}/middleware/wifi/port/os/os.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mw300_rd_net.c"
)
target_include_directories(mw320_wlcmgr
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${mw320_sdk_dir}/middleware/wifi/incl"
    "${mw320_sdk_dir}/middleware/wifi/incl/port/os"
    "${mw320_sdk_dir}/middleware/wifi/incl/wlcmgr"
)
target_link_libraries(mw320_wlcmgr
    PUBLIC
    freertos_plus_tcp_port
    freertos_plus_tcp_network_if_common
    freertos_plus_tcp
    freertos_kernel
    mw320_wifidriver
    PRIVATE
    mw320_utility_debug_console
    mw320_board_mbedtls
)

add_library(mw320_board_mbedtls STATIC)
target_sources(mw320_board_mbedtls
    PRIVATE
    "${mw320_sdk_dir}/boards/rdmw320_r0/mbedtls_common/board_hash.c"
    "${mw320_sdk_dir}/boards/rdmw320_r0/mbedtls_common/sha224-256.c"
)
target_include_directories(mw320_board_mbedtls
    PUBLIC
    "${mw320_sdk_dir}/boards/rdmw320_r0/mbedtls_common"
)
target_link_libraries(mw320_board_mbedtls
    PUBLIC
    mw320_drivers_common
)

# only include the single additions header file from the embedded FreeRTOS distribution
configure_file(
    "${mw320_sdk_dir}/rtos/freertos/freertos_kernel/include/freertos_tasks_c_additions.h"
    "${CMAKE_CURRENT_BINARY_DIR}/freertos_tasks_c_additions.h" COPYONLY)
target_include_directories(freertos_kernel PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(freertos_kernel
    PUBLIC
    USE_RTOS
    FSL_RTOS_FREE_RTOS
    SDK_OS_FREE_RTOS
)

target_include_directories(mbedcrypto
    PRIVATE
    "${mw320_sdk_dir}/middleware/mbedtls/include"
    PUBLIC
    "${mw320_sdk_dir}/middleware/mbedtls/port/mw"
)
target_sources(mbedcrypto
    PRIVATE
    "${mw320_sdk_dir}/middleware/mbedtls/port/mw/aes_alt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ccm_alt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ksdk_mbedtls.c"
)
target_link_libraries(mbedcrypto
    PUBLIC
    freertos_config
    mw320_drivers_aes
    mw320_utility_debug_console
    freertos_kernel
)
