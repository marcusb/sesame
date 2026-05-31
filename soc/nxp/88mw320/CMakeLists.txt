zephyr_sources(soc.c)
zephyr_include_directories(.)
zephyr_compile_definitions(CPU_88MW320_A0_NAPC)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
# Only relocate code for flash build; RAM build runs entirely from SRAM
if(NOT CONFIG_BOARD_MARVELL_MW302_RAM)
zephyr_code_relocate(FILES soc.c LOCATION RAM)
endif()
