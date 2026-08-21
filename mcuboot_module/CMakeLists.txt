zephyr_library()

add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/zephyr/mcuboot.bin
  COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/../tools/axf2firmware
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../tools/axf2firmware/axf2firmware ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf ${CMAKE_BINARY_DIR}/zephyr/mcuboot.bin
  DEPENDS zephyr_final ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf
)
add_custom_target(mcuboot_bin ALL DEPENDS ${CMAKE_BINARY_DIR}/zephyr/mcuboot.bin)
