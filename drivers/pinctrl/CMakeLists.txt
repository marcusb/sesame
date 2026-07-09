zephyr_library()
zephyr_library_sources_ifdef(CONFIG_PINCTRL_MW320 pinctrl_mw320.c)
zephyr_include_directories(${CMAKE_SOURCE_DIR}/mw320_sdk/devices/88MW320)
zephyr_include_directories(${CMAKE_SOURCE_DIR}/mw320_sdk/devices/88MW320/drivers)

