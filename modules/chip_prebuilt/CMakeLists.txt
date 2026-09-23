# Prebuilt Matter (CHIP) library interface for Zephyr
if(CONFIG_CHIP)
    if(NOT CHIP_ROOT)
        get_filename_component(CHIP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/connectedhomeip REALPATH)
    endif()

    if(NOT DEFINED CHIP_PREBUILT_DIR)
        set(CHIP_PREBUILT_DIR "${CMAKE_BINARY_DIR}/../sesame/modules/connectedhomeip")
    endif()
    get_filename_component(CHIP_PREBUILT_DIR "${CHIP_PREBUILT_DIR}" ABSOLUTE)

    message(STATUS "chip_prebuilt: Using prebuilt Matter libraries from ${CHIP_PREBUILT_DIR}")

    add_library(chip INTERFACE)
    target_compile_definitions(chip INTERFACE CHIP_HAVE_CONFIG_H)
    target_include_directories(chip INTERFACE
        ${CHIP_ROOT}/src
        ${CHIP_ROOT}/src/include
        ${CHIP_ROOT}/third_party/nlassert/repo/include
        ${CHIP_ROOT}/third_party/nlio/repo/include
        ${CHIP_ROOT}/third_party/nlfaultinjection/include
        ${CHIP_ROOT}/zzz_generated/app-common
        ${CHIP_ROOT}/third_party/mbedtls/repo/include
        ${CHIP_ROOT}/examples/providers
        ${CHIP_PREBUILT_DIR}/gen/include
    )
    target_link_libraries(chip INTERFACE
        -Wl,--start-group
        ${CHIP_PREBUILT_DIR}/lib/libCHIP.a
        ${CHIP_PREBUILT_DIR}/lib/libMatterDeviceInfoProviderExample.a
        ${CHIP_PREBUILT_DIR}/lib/libmbedtls.a
        -Wl,--end-group
    )
    set_property(GLOBAL APPEND PROPERTY ZEPHYR_INTERFACE_LIBS chip)

    if(CONFIG_CHIP_MALLOC_SYS_HEAP_OVERRIDE)
        target_link_options(chip INTERFACE
            -Wl,--wrap=malloc
            -Wl,--wrap=calloc
            -Wl,--wrap=realloc
            -Wl,--wrap=free
            -Wl,--wrap=_malloc_r
            -Wl,--wrap=_calloc_r
            -Wl,--wrap=_realloc_r
            -Wl,--wrap=_free_r
        )
    endif()
endif()
