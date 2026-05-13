# chip.cmake — hand-rolled CMake build for the vendored connectedhomeip
# submodule. The CHIP tree at third_party/connectedhomeip is read-only;
# all build rules and overrides live here or in src/matter/.
#
# Stages:
#   chip_support        — lib/support CHIPMem + CHIPPlatformMemory
#   chip_system         — System layer (SystemLayerImplSelect + timer/packet/wake)
#   chip_inet           — Inet layer (UDP/TCP over sockets)
#   chip_platform_generic — platform/ generic C++ (DeviceLayer, Diagnostics, etc.)
#   chip_platform_mw320 — src/platform/nxp/mw320/*.cpp (hardware only)
#   chip                — aggregate INTERFACE for the rest of Sesame to link

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
# FreeRTOS include paths (include/config, freertos_kernel/include, portable/)
# are needed by chip_prelude.h which force-includes FreeRTOSConfig.h for every
# CHIP translation unit.
target_link_libraries(chip_includes INTERFACE freertos_kernel)
# mw320_sdk provides NXP SDK headers that CHIP headers pull in transitively
# through CHIP_DEVICE_LAYER_TARGET=nxp/mw320 (e.g. ConfigurationManagerImpl.h
# → wifi.h). Hardware builds only — QEMU has no mw320 SDK.
if(NOT USE_QEMU)
    target_link_libraries(chip_includes INTERFACE mw320_sdk)
endif()

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
# chip_support: lib/support — CHIP utility layer.
# Excludes JniReferences.cpp (Android JNI) and ThreadOperationalDataset.cpp
# (OpenThread, CHIP_ENABLE_OPENTHREAD=0).  CHIPFaultInjection.cpp excluded —
# CHIP_WITH_NLFAULTINJECTION=0.
# ----------------------------------------------------------------------------
add_library(chip_support STATIC
    "${CHIP_ROOT}/src/lib/support/Base64.cpp"
    "${CHIP_ROOT}/src/lib/support/BufferReader.cpp"
    "${CHIP_ROOT}/src/lib/support/BufferWriter.cpp"
    "${CHIP_ROOT}/src/lib/support/BytesCircularBuffer.cpp"
    "${CHIP_ROOT}/src/lib/support/BytesToHex.cpp"
    "${CHIP_ROOT}/src/lib/support/CHIPMem.cpp"
    "${CHIP_ROOT}/src/lib/support/CHIPMem-Malloc.cpp"
    "${CHIP_ROOT}/src/lib/support/CHIPPlatformMemory.cpp"
    "${CHIP_ROOT}/src/lib/support/FibonacciUtils.cpp"
    "${CHIP_ROOT}/src/lib/support/FixedBufferAllocator.cpp"
    "${CHIP_ROOT}/src/lib/support/IniEscaping.cpp"
    "${CHIP_ROOT}/src/lib/support/Pool.cpp"
    "${CHIP_ROOT}/src/lib/support/PrivateHeap.cpp"
    "${CHIP_ROOT}/src/lib/support/ReadOnlyBuffer.cpp"
    "${CHIP_ROOT}/src/lib/support/StringBuilder.cpp"
    "${CHIP_ROOT}/src/lib/support/TimeUtils.cpp"
    "${CHIP_ROOT}/src/lib/support/utf8.cpp"
    "${CHIP_ROOT}/src/lib/support/ZclString.cpp"
    # PersistentStorageAudit.cpp — audit helper, include
    "${CHIP_ROOT}/src/lib/support/PersistentStorageAudit.cpp"
)
target_link_libraries(chip_support
    PUBLIC
    globals
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_core: lib/core — TLV, error codes, OTA image header.
# ----------------------------------------------------------------------------
add_library(chip_core STATIC
    "${CHIP_ROOT}/src/lib/core/CHIPError.cpp"
    "${CHIP_ROOT}/src/lib/core/CHIPKeyIds.cpp"
    "${CHIP_ROOT}/src/lib/core/ErrorStr.cpp"
    "${CHIP_ROOT}/src/lib/core/OTAImageHeader.cpp"
    # StringBuilderAdapters.cpp excluded — requires Pigweed pw_string headers
    "${CHIP_ROOT}/src/lib/core/TLVCircularBuffer.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVDebug.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVReader.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVTags.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVUpdater.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVUtilities.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVVectorWriter.cpp"
    "${CHIP_ROOT}/src/lib/core/TLVWriter.cpp"
)
target_link_libraries(chip_core
    PUBLIC
    chip_support
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_crypto: Crypto PAL — mbedTLS backend (CHIP_CRYPTO_MBEDTLS=1).
# PSA / OpenSSL variants excluded.
# ----------------------------------------------------------------------------
add_library(chip_crypto STATIC
    "${CHIP_ROOT}/src/crypto/CHIPCryptoPAL.cpp"
    "${CHIP_ROOT}/src/crypto/CHIPCryptoPALmbedTLS.cpp"
    "${CHIP_ROOT}/src/crypto/CHIPCryptoPALmbedTLSCert.cpp"
    "${CHIP_ROOT}/src/crypto/PersistentStorageOperationalKeystore.cpp"
    "${CHIP_ROOT}/src/crypto/RandUtils.cpp"
    "${CHIP_ROOT}/src/crypto/RawKeySessionKeystore.cpp"
    # PSA variants excluded — CHIP_CRYPTO_PSA=0
)
target_link_libraries(chip_crypto
    PUBLIC
    chip_core
    chip_includes
    chip_compile_flags
    mbedcrypto
)

# ----------------------------------------------------------------------------
# chip_system: System layer (timer, packet buffer, select-loop wake event).
# Uses SystemLayerImplSelect (sockets path); SystemLayerImplFreeRTOS is for
# the lwIP path and is excluded.
# ----------------------------------------------------------------------------
add_library(chip_system STATIC
    "${CHIP_ROOT}/src/system/SystemClock.cpp"
    "${CHIP_ROOT}/src/system/SystemError.cpp"
    "${CHIP_ROOT}/src/system/SystemLayer.cpp"
    "${CHIP_ROOT}/src/system/SystemLayerImplSelect.cpp"
    "${CHIP_ROOT}/src/system/SystemMutex.cpp"
    "${CHIP_ROOT}/src/system/SystemPacketBuffer.cpp"
    "${CHIP_ROOT}/src/system/SystemStats.cpp"
    "${CHIP_ROOT}/src/system/SystemTimer.cpp"
    "${CHIP_ROOT}/src/system/TLVPacketBufferBackingStore.cpp"
    "${CHIP_ROOT}/src/system/WakeEvent.cpp"
    # SystemFaultInjection.cpp excluded — CHIP_WITH_NLFAULTINJECTION=0
)
target_link_libraries(chip_system
    PUBLIC
    chip_core
    chip_crypto
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_inet: Inet layer (IP address, UDP/TCP endpoints over sockets).
# LwIP / OpenThread endpoint impls excluded — we use USE_SOCKETS=1.
# ----------------------------------------------------------------------------
add_library(chip_inet STATIC
    "${CHIP_ROOT}/src/inet/InetArgParser.cpp"
    "${CHIP_ROOT}/src/inet/InetError.cpp"
    "${CHIP_ROOT}/src/inet/InetInterface.cpp"
    "${CHIP_ROOT}/src/inet/InetInterfaceImplDefault.cpp"
    "${CHIP_ROOT}/src/inet/IPAddress.cpp"
    "${CHIP_ROOT}/src/inet/IPAddress-StringFuncts.cpp"
    "${CHIP_ROOT}/src/inet/IPPacketInfo.cpp"
    "${CHIP_ROOT}/src/inet/IPPrefix.cpp"
    "${CHIP_ROOT}/src/inet/TCPEndPoint.cpp"
    "${CHIP_ROOT}/src/inet/TCPEndPointImplSockets.cpp"
    "${CHIP_ROOT}/src/inet/UDPEndPoint.cpp"
    "${CHIP_ROOT}/src/inet/UDPEndPointImplSockets.cpp"
    # EndPointStateLwIP.cpp / *LwIP.cpp / *OpenThread.cpp excluded
    # InetFaultInjection.cpp excluded — CHIP_WITH_NLFAULTINJECTION=0
)
target_link_libraries(chip_inet
    PUBLIC
    chip_system
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_platform_generic: Generic DeviceLayer C++ sources.
# These are platform-independent; the mw320-specific code lives in
# chip_platform_mw320 below.
# ----------------------------------------------------------------------------
add_library(chip_platform_generic STATIC
    "${CHIP_ROOT}/src/platform/CommissionableDataProvider.cpp"
    "${CHIP_ROOT}/src/platform/DeviceControlServer.cpp"
    "${CHIP_ROOT}/src/platform/DeviceInfoProvider.cpp"
    "${CHIP_ROOT}/src/platform/DeviceInstanceInfoProvider.cpp"
    # DeviceSafeQueue.cpp excluded — uses std::mutex which requires gthreads,
    # unavailable on arm-none-eabi bare-metal. The mw320 platform uses
    # GenericPlatformManagerImpl_FreeRTOS (not POSIX), so this is unreferenced.
    "${CHIP_ROOT}/src/platform/DiagnosticDataProvider.cpp"
    "${CHIP_ROOT}/src/platform/Entropy.cpp"
    "${CHIP_ROOT}/src/platform/GeneralUtils.cpp"
    "${CHIP_ROOT}/src/platform/Globals.cpp"
    "${CHIP_ROOT}/src/platform/LockTracker.cpp"
    "${CHIP_ROOT}/src/platform/PersistedStorage.cpp"
    "${CHIP_ROOT}/src/platform/PlatformEventSupport.cpp"
    "${CHIP_ROOT}/src/platform/RuntimeOptionsProvider.cpp"
    "${CHIP_ROOT}/src/platform/SingletonConfigurationManager.cpp"
    "${CHIP_ROOT}/src/platform/SyscallStubs.cpp"
)
target_link_libraries(chip_platform_generic
    PUBLIC
    chip_system
    chip_inet
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

        # Configuration — excludes MW320Config.cpp (needs network_flash_storage.h;
        # replaced by Sesame PSM-backed subclass in src/matter/platform_sesame/).
        "${CHIP_ROOT}/src/platform/nxp/mw320/ConfigurationManagerImpl.cpp"

        # Device info / factory data / attestation.
        "${CHIP_ROOT}/src/platform/nxp/mw320/DeviceInfoProviderImpl.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/FactoryDataProvider.cpp"

        # Network commissioning drivers.
        # ConnectivityUtils.cpp excluded: needs netdb.h/wm_net.h (lwIP port header).
        "${CHIP_ROOT}/src/platform/nxp/mw320/NetworkCommissioningWiFiDriver.cpp"

        # OTA — mw320_ota.cpp provides the flash-write primitives that
        # OTAImageProcessorImpl.cpp calls.
        "${CHIP_ROOT}/src/platform/nxp/mw320/mw320_ota.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/OTAImageProcessorImpl.cpp"

        # Excluded — Sesame subclasses replace these:
        #   ConnectivityManagerImpl.cpp   (lwIP-dependent; subclass uses network_manager.c)
        #   DiagnosticDataProviderImpl.cpp (lwIP-dependent; subclass uses FreeRTOS APIs)
        #   PlatformManagerImpl.cpp       (lwIP-dependent; init ordering handled in matter_app.cpp)
        #   KeyValueStoreManagerImpl.cpp  (needs network_flash_storage.h; subclass uses psm_safe.c)
        # Excluded — compile-time dead code:
        #   SoftwareUpdateManagerImpl.cpp (broken include paths, deprecated pre-Matter-OTA)
        #   NetworkProvisioningServerImpl.cpp (legacy include paths, superseded by commissioning API)
        #   NetworkCommissioningEthernetDriver.cpp (needs ConnectivityUtils.h/wm_net.h)
        # Excluded — BLE disabled:
        #   BLEManagerImpl.cpp
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

# Aggregate target the rest of Sesame links against.
add_library(chip INTERFACE)
target_link_libraries(chip INTERFACE
    chip_platform_generic
    chip_system
    chip_inet
    chip_crypto
    chip_core
    chip_support
)
if(NOT USE_QEMU)
    target_link_libraries(chip INTERFACE chip_platform_mw320)
endif()
