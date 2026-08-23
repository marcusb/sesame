add_integration_test(matter
    main_matter.c
    "${CMAKE_SOURCE_DIR}/src/qemu/qemu_psm.c"
    "${CMAKE_SOURCE_DIR}/src/qemu/qemu_stubs.c")

set(matter_target "matter_it${target_suffix}")

target_include_directories(${matter_target} PRIVATE
    "${CMAKE_SOURCE_DIR}/src/matter")

# Same circular-dep-tolerant link group the full sesame-qemu.axf uses; matter
# initializers in sesame_matter must be force-linked.
target_link_libraries(${matter_target} PRIVATE
    "LINKER:--start-group"
    mbedcrypto
    qemu_mbedtls_port
    "LINKER:--whole-archive"
    sesame_matter
    "LINKER:--no-whole-archive"
    "LINKER:--end-group")
