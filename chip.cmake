# chip.cmake — hand-rolled CMake build for the vendored connectedhomeip
# submodule. The CHIP tree at third_party/connectedhomeip is read-only;
# all build rules and overrides live here or in src/matter/.
#
# Stage 1 (current): compile a tiny stub of CHIP's support layer to validate
# cross-compile, include paths, C++17, and exception/RTTI-free build.
# Subsequent stages expand the file lists toward full Server + DataModel.

enable_language(CXX)

set(CHIP_ROOT "${CMAKE_CURRENT_LIST_DIR}/third_party/connectedhomeip")

# ----------------------------------------------------------------------------
# Public include interface — all CHIP code is reached via these include roots.
# Anything that links against `chip` (or its parts) inherits these.
# ----------------------------------------------------------------------------
add_library(chip_includes INTERFACE)
target_include_directories(chip_includes
    INTERFACE
    # POSIX sockets shim — declarations-only. Must come before system
    # headers so CHIP's <sys/socket.h>/<netinet/in.h>/<net/if.h> resolve
    # to our shim, not newlib's (which doesn't ship them on bare metal
    # anyway, but explicit beats implicit).
    "${CMAKE_CURRENT_LIST_DIR}/include/matter/posix_shim"
    "${CHIP_ROOT}/src"
    "${CHIP_ROOT}/src/include"
    # Some upstream files (e.g. platform/nxp/mw320/Logging.cpp) use
    # repo-root-relative include paths like <src/lib/support/...>.
    "${CHIP_ROOT}"
    "${CHIP_ROOT}/zzz_generated/app-common"
    "${CHIP_ROOT}/third_party/nlassert/repo/include"
    "${CHIP_ROOT}/third_party/nlio/repo/include"
    "${CHIP_ROOT}/third_party/nlfaultinjection/include"
    # Generated build configs (vendored from a prior GN build; we maintain
    # them by hand for our target).
    "${CMAKE_CURRENT_LIST_DIR}/include/matter/chip_buildconfig"
    # Project config: CHIP_PROJECT_CONFIG_INCLUDE = <CHIPProjectConfig.h>
    "${CMAKE_CURRENT_LIST_DIR}/src/matter"
)
target_compile_definitions(chip_includes
    INTERFACE
    CHIP_HAVE_CONFIG_H=1
)

# ----------------------------------------------------------------------------
# Compile options for every CHIP translation unit.
# CHIP is large; build it size-optimized regardless of project build type so
# Debug builds remain linkable on the MW320's 2 MB flash budget.
# ----------------------------------------------------------------------------
add_library(chip_compile_flags INTERFACE)
target_compile_options(chip_compile_flags
    INTERFACE
    "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wno-format-nonliteral>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wno-missing-field-initializers>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wno-shadow>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Os>"
    # See include/matter/posix_shim/chip_prelude.h — kills the max/min
    # macros from FreeRTOSConfig.h before CHIP C++ encounters them.
    "$<$<COMPILE_LANGUAGE:CXX>:-include>"
    "$<$<COMPILE_LANGUAGE:CXX>:chip_prelude.h>"
)

# ----------------------------------------------------------------------------
# Stage 1 stub library: prove CHIP code compiles under our toolchain.
# Expand the source list in subsequent stages — eventually splitting into
# libchip_core, libchip_app, libchip_platform_mw320, libchip_minmdns, etc.
# ----------------------------------------------------------------------------
add_library(chip_support STATIC
    "${CHIP_ROOT}/src/lib/support/CHIPMem.cpp"
    "${CHIP_ROOT}/src/lib/support/CHIPMem-Malloc.cpp"
    "${CHIP_ROOT}/src/lib/support/CHIPPlatformMemory.cpp"
)
target_link_libraries(chip_support
    PUBLIC
    globals
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# Stage 3: MW320 platform layer.
# Compiles the NXP-provided src/platform/nxp/mw320/*.cpp files (minus BLE
# and minus those that include lwip — we'll override those with Sesame
# subclasses against FreeRTOS+TCP).
# ----------------------------------------------------------------------------
if(NOT USE_QEMU)
    # MW320Config.cpp is excluded — it depends on NXP's example-tree
    # helper network_flash_storage.h. Per the migration plan, a Sesame
    # subclass backs storage with psm_safe.c instead.
    add_library(chip_platform_mw320 STATIC
        "${CHIP_ROOT}/src/platform/nxp/mw320/Logging.cpp"
    )
    target_link_libraries(chip_platform_mw320
        PUBLIC
        globals
        chip_includes
        chip_compile_flags
        # mw320_sdk propagates NXP SDK include paths (wifi.h, wlan.h,
        # mflash_drv.h, partition.h, FreeRTOS.h, fsl_debug_console.h, ...).
        mw320_sdk
        freertos_kernel
    )
endif()

# Single aggregate target the rest of Sesame links against. Currently just
# the stub; later stages add more libraries to this interface.
add_library(chip INTERFACE)
target_link_libraries(chip INTERFACE chip_support)
if(NOT USE_QEMU)
    target_link_libraries(chip INTERFACE chip_platform_mw320)
endif()
