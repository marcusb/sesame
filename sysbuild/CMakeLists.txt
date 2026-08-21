# Set board root for sysbuild
set(BOARD_ROOT ${APP_DIR} CACHE PATH "board root" FORCE)
set(SOC_ROOT ${APP_DIR} CACHE PATH "soc root" FORCE)
set(DTS_ROOT ${APP_DIR} CACHE PATH "dts root" FORCE)

set(ZEPHYR_MCUBOOT_MODULE_DIR ${APP_DIR}/mcuboot CACHE PATH "MCUBOOT MODULE DIR" FORCE)
set(ENV{ZEPHYR_MCUBOOT_MODULE_DIR} "${APP_DIR}/mcuboot")

# Forward roots and modules to mcuboot
set(mcuboot_BOARD_ROOT ${APP_DIR} CACHE PATH "mcuboot board root")
set(mcuboot_SOC_ROOT ${APP_DIR} CACHE PATH "mcuboot soc root")
set(mcuboot_DTS_ROOT ${APP_DIR} CACHE PATH "mcuboot dts root")
set(mcuboot_ZEPHYR_EXTRA_MODULES "${APP_DIR}/modules/mw320_sdk;${APP_DIR}/mcuboot" CACHE STRING "mcuboot modules" FORCE)
set(mcuboot_MW320_SDK_DIR ${APP_DIR}/mw320_sdk CACHE PATH "mcuboot mw320 sdk")
set(mcuboot_EXTRA_DTC_OVERLAY_FILE ${APP_DIR}/boards/arm/marvell_mw302/mcuboot_offset.overlay CACHE PATH "mcuboot overlay")

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)
