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
    # Sesame overrides for vendored CHIP headers. Must come before the CHIP
    # source tree so our patched versions shadow the originals.
    "${CMAKE_CURRENT_LIST_DIR}/include/matter/chip_overrides"
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
    # ZAP-generated headers (endpoint_config.h, gen_config.h, access.h)
    # Code does #include <zap-generated/gen_config.h> so we need the parent
    # directory of the zap-generated/ output folder.
    "${CMAKE_BINARY_DIR}"
    # Stub headers for NXP SDK files used by mw320 platform code but not
    # present in our SDK tree (e.g. network_flash_storage.h from wifi_examples)
    "${CMAKE_CURRENT_LIST_DIR}/include/matter/mw320_stubs"
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
    # FreeRTOS platform clock implementation (defines gClockImpl)
    "${CHIP_ROOT}/src/platform/FreeRTOS/SystemTimeSupport.cpp"
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
    # Logging — text-only backend (BinaryLogging.cpp excluded)
    "${CHIP_ROOT}/src/lib/support/logging/TextOnlyLogging.cpp"
    # Verhoeff check digit (used by ManualSetupPayloadGenerator)
    "${CHIP_ROOT}/src/lib/support/verhoeff/Verhoeff.cpp"
    "${CHIP_ROOT}/src/lib/support/verhoeff/Verhoeff10.cpp"
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
    # mbedtls_ccm_update/update_ad/finish stubs — MBEDTLS_CCM_ALT disables all
    # of ccm.c; the HW accelerator provides one-shot encrypt/decrypt only.
    # CHIP never calls the streaming API so these are unreachable, but the linker
    # needs the symbols.  Must be in a library processed before mbedcrypto.a.
    "${CMAKE_CURRENT_LIST_DIR}/src/matter/sesame_mbedtls_ccm_stubs.c"
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

    # BSD-socket shim for CHIP over FreeRTOS+TCP.
    # Lives here (inside the linker group) so the symbols are visible when
    # chip_system/chip_inet demand select(), pipe(), htonl(), getifaddrs(), etc.
    "${CMAKE_CURRENT_LIST_DIR}/src/matter/freertos_socket_shim.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/matter/mdns_mcast_join.c"
)
target_link_libraries(chip_system
    PUBLIC
    chip_core
    chip_crypto
    chip_includes
    chip_compile_flags
    freertos_plus_tcp
)

# ----------------------------------------------------------------------------
# chip_inet: Inet layer (IP address, UDP/TCP endpoints over sockets).
# LwIP / OpenThread endpoint impls excluded — we use USE_SOCKETS=1.
# ----------------------------------------------------------------------------
add_library(chip_inet STATIC
    "${CHIP_ROOT}/src/inet/InetArgParser.cpp"
    "${CHIP_ROOT}/src/inet/InetError.cpp"
    "${CHIP_ROOT}/src/inet/InetInterface.cpp"
    # InetInterfaceImplDefault.cpp excluded — calls POSIX if_nameindex(); replaced
    # by inet_interface_impl.cpp below which uses our single FreeRTOS+TCP interface.
    "${CMAKE_CURRENT_LIST_DIR}/src/matter/inet_interface_impl.cpp"
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

        # Configuration.
        "${CHIP_ROOT}/src/platform/nxp/mw320/MW320Config.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/ConfigurationManagerImpl.cpp"

        # Key-value store — uses network_flash_storage stubs for now; will be
        # replaced with a PSM-backed Sesame subclass in a later step.
        "${CHIP_ROOT}/src/platform/nxp/mw320/KeyValueStoreManagerImpl.cpp"

        # Platform manager — provides PlatformManagerImpl::sInstance.
        "${CHIP_ROOT}/src/platform/nxp/mw320/PlatformManagerImpl.cpp"

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

        # Sesame ConnectivityManagerImpl — replaces the LwIP-dependent upstream version.
        # Provides template instantiations for UDP/TCP EndPointManager and minimal WiFi stubs.
        "${CMAKE_CURRENT_LIST_DIR}/src/matter/ConnectivityManagerImpl_sesame.cpp"

        # Sesame DiagnosticDataProviderImpl — replaces the LwIP-dependent upstream version.
        "${CMAKE_CURRENT_LIST_DIR}/src/matter/DiagnosticDataProviderImpl_sesame.cpp"

        # Excluded — Sesame subclasses replace these:
        #   ConnectivityManagerImpl.cpp   (lwIP-dependent; replaced by ConnectivityManagerImpl_sesame.cpp)
        #   DiagnosticDataProviderImpl.cpp (lwIP-dependent; subclass uses FreeRTOS APIs)
        #   PlatformManagerImpl.cpp       (lwIP-dependent; init ordering handled in matter_app.cpp)
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
    target_compile_definitions(chip_platform_mw320 PRIVATE MW320_LOG_ENABLED=1)
else()
    # QEMU platform: same mw320 source files but without the hardware SDK.
    # OTA files (mw320_ota.cpp, OTAImageProcessorImpl.cpp) are excluded because
    # they need mflash_drv.h / partition.h which have no QEMU equivalents.
    # All other mw320 platform files compile against our mw320_stubs headers.
    add_library(chip_platform_qemu STATIC
        "${CHIP_ROOT}/src/platform/nxp/mw320/Logging.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/MW320Config.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/ConfigurationManagerImpl.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/KeyValueStoreManagerImpl.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/PlatformManagerImpl.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/DeviceInfoProviderImpl.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/FactoryDataProvider.cpp"
        "${CHIP_ROOT}/src/platform/nxp/mw320/NetworkCommissioningWiFiDriver.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/src/matter/ConnectivityManagerImpl_sesame.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/src/matter/DiagnosticDataProviderImpl_sesame.cpp"
        # RAM-backed KVS for QEMU (replaces the stub network_flash_storage.h functions).
        "${CMAKE_CURRENT_LIST_DIR}/src/matter/kvs_ram.c"
    )
    target_link_libraries(chip_platform_qemu
        PUBLIC
        globals
        chip_includes
        chip_compile_flags
        freertos_kernel
    )
    target_compile_definitions(chip_platform_qemu PRIVATE MW320_LOG_ENABLED=1 "PRINTF=printf")
endif()

# ----------------------------------------------------------------------------
# chip_credentials: Fabric table, cert chain, attestation, group data.
# ----------------------------------------------------------------------------
add_library(chip_credentials STATIC
    "${CHIP_ROOT}/src/credentials/CertificationDeclaration.cpp"
    "${CHIP_ROOT}/src/credentials/CHIPCert.cpp"
    "${CHIP_ROOT}/src/credentials/CHIPCertFromX509.cpp"
    "${CHIP_ROOT}/src/credentials/CHIPCertToX509.cpp"
    "${CHIP_ROOT}/src/credentials/DeviceAttestationConstructor.cpp"
    "${CHIP_ROOT}/src/credentials/DeviceAttestationCredsProvider.cpp"
    "${CHIP_ROOT}/src/credentials/FabricTable.cpp"
    "${CHIP_ROOT}/src/credentials/GenerateChipX509Cert.cpp"
    "${CHIP_ROOT}/src/credentials/GroupDataProviderImpl.cpp"
    "${CHIP_ROOT}/src/credentials/LastKnownGoodTime.cpp"
    "${CHIP_ROOT}/src/credentials/PersistentStorageOpCertStore.cpp"
)
target_link_libraries(chip_credentials
    PUBLIC
    chip_crypto
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_messaging: Exchange manager, reliable messaging, session layer.
# ----------------------------------------------------------------------------
add_library(chip_messaging STATIC
    "${CHIP_ROOT}/src/messaging/ApplicationExchangeDispatch.cpp"
    "${CHIP_ROOT}/src/messaging/ErrorCategory.cpp"
    "${CHIP_ROOT}/src/messaging/ExchangeContext.cpp"
    "${CHIP_ROOT}/src/messaging/ExchangeMessageDispatch.cpp"
    "${CHIP_ROOT}/src/messaging/ExchangeMgr.cpp"
    "${CHIP_ROOT}/src/messaging/ReliableMessageContext.cpp"
    "${CHIP_ROOT}/src/messaging/ReliableMessageMgr.cpp"
    "${CHIP_ROOT}/src/messaging/ReliableMessageProtocolConfig.cpp"
)
target_link_libraries(chip_messaging
    PUBLIC
    chip_credentials
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_transport: Secure session and transport manager.
# ----------------------------------------------------------------------------
add_library(chip_transport STATIC
    "${CHIP_ROOT}/src/transport/CryptoContext.cpp"
    "${CHIP_ROOT}/src/transport/GroupPeerMessageCounter.cpp"
    "${CHIP_ROOT}/src/transport/SecureMessageCodec.cpp"
    "${CHIP_ROOT}/src/transport/SecureSession.cpp"
    "${CHIP_ROOT}/src/transport/SecureSessionTable.cpp"
    "${CHIP_ROOT}/src/transport/Session.cpp"
    "${CHIP_ROOT}/src/transport/SessionHolder.cpp"
    "${CHIP_ROOT}/src/transport/SessionManager.cpp"
    # TraceMessage.cpp excluded — CHIP_CONFIG_TRANSPORT_TRACE_ENABLED not set;
    # TransportTraceHandler is only declared when that flag is on.
    "${CHIP_ROOT}/src/transport/TransportMgrBase.cpp"
    # Raw transport implementations (UDP and TCP over sockets)
    "${CHIP_ROOT}/src/transport/raw/MessageHeader.cpp"
    "${CHIP_ROOT}/src/transport/raw/UDP.cpp"
    "${CHIP_ROOT}/src/transport/raw/TCP.cpp"
)
target_link_libraries(chip_transport
    PUBLIC
    chip_messaging
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# chip_secure_channel: CASE/PASE session establishment.
# ----------------------------------------------------------------------------
add_library(chip_secure_channel STATIC
    "${CHIP_ROOT}/src/protocols/secure_channel/CASEDestinationId.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/CASEServer.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/CASESession.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/CheckInCounter.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/CheckinMessage.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/DefaultSessionResumptionStorage.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/MessageCounterManager.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/PairingSession.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/PASESession.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/SessionEstablishmentExchangeDispatch.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/SimpleSessionResumptionStorage.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/StatusReport.cpp"
    "${CHIP_ROOT}/src/protocols/secure_channel/UnsolicitedStatusHandler.cpp"
)
target_link_libraries(chip_secure_channel
    PUBLIC
    chip_transport
    chip_includes
    chip_compile_flags
)

# ----------------------------------------------------------------------------
# ZAP code generation — run zap-cli at configure time to produce
# endpoint_config.h, gen_config.h, access.h, and IMClusterCommandHandler.cpp.
# ----------------------------------------------------------------------------
set(ZAP_CLI "${CMAKE_CURRENT_LIST_DIR}/.zap/zap-v2026.05.12-nightly/zap-cli")
set(ZAP_INPUT "${CMAKE_CURRENT_LIST_DIR}/src/matter/window-covering-app.zap")
set(ZAP_ZCL "${CHIP_ROOT}/src/app/zap-templates/zcl/zcl.json")
set(ZAP_TEMPLATES "${CHIP_ROOT}/src/app/zap-templates/app-templates.json")
set(ZAP_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/zap-generated")

file(MAKE_DIRECTORY "${ZAP_OUTPUT_DIR}")
execute_process(
    COMMAND "${ZAP_CLI}"
        generate
        -z "${ZAP_ZCL}"
        -g "${ZAP_TEMPLATES}"
        -i "${ZAP_INPUT}"
        -o "${ZAP_OUTPUT_DIR}"
        --tempState
        --noZapFileLog
    RESULT_VARIABLE ZAP_RESULT
    OUTPUT_VARIABLE ZAP_OUTPUT
    ERROR_VARIABLE ZAP_ERROR
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
)
if(ZAP_RESULT)
    message(FATAL_ERROR "ZAP generation failed with code ${ZAP_RESULT}")
endif()

# ----------------------------------------------------------------------------
# chip_app: Interaction Model engine, data model, server, cluster utils.
# ----------------------------------------------------------------------------
add_library(chip_app STATIC
    # Core interaction model
    "${CHIP_ROOT}/src/app/AttributeAccessInterfaceRegistry.cpp"
    "${CHIP_ROOT}/src/app/AttributePathExpandIterator.cpp"
    "${CHIP_ROOT}/src/app/AttributeReportBuilder.cpp"
    "${CHIP_ROOT}/src/app/AttributeValueEncoder.cpp"
    "${CHIP_ROOT}/src/app/BufferedReadCallback.cpp"
    "${CHIP_ROOT}/src/app/CASEClient.cpp"
    "${CHIP_ROOT}/src/app/CASESessionManager.cpp"
    "${CHIP_ROOT}/src/app/ChunkedWriteCallback.cpp"
    "${CHIP_ROOT}/src/app/ClusterStateCache.cpp"
    "${CHIP_ROOT}/src/app/CommandHandler.cpp"
    "${CHIP_ROOT}/src/app/CommandHandlerImpl.cpp"
    "${CHIP_ROOT}/src/app/CommandHandlerInterfaceRegistry.cpp"
    "${CHIP_ROOT}/src/app/CommandResponseSender.cpp"
    "${CHIP_ROOT}/src/app/CommandSender.cpp"
    "${CHIP_ROOT}/src/app/DeviceProxy.cpp"
    "${CHIP_ROOT}/src/app/EventManagement.cpp"
    "${CHIP_ROOT}/src/app/FailSafeContext.cpp"
    "${CHIP_ROOT}/src/app/GlobalAttributes.cpp"
    "${CHIP_ROOT}/src/app/InteractionModelDelegatePointers.cpp"
    "${CHIP_ROOT}/src/app/InteractionModelEngine.cpp"
    "${CHIP_ROOT}/src/app/OperationalSessionSetup.cpp"
    "${CHIP_ROOT}/src/app/PendingResponseTrackerImpl.cpp"
    "${CHIP_ROOT}/src/app/ReadClient.cpp"
    "${CHIP_ROOT}/src/app/ReadHandler.cpp"
    "${CHIP_ROOT}/src/app/SafeAttributePersistenceProvider.cpp"
    # SimpleSubscriptionResumptionStorage and SubscriptionResumptionSessionEstablisher
    # excluded — CHIP_CONFIG_PERSIST_SUBSCRIPTIONS=0 in CHIPProjectConfig.h
    "${CHIP_ROOT}/src/app/StatusResponse.cpp"
    "${CHIP_ROOT}/src/app/StorageDelegateWrapper.cpp"
    "${CHIP_ROOT}/src/app/TimedHandler.cpp"
    "${CHIP_ROOT}/src/app/TimedRequest.cpp"
    "${CHIP_ROOT}/src/app/TimerDelegates.cpp"
    "${CHIP_ROOT}/src/app/WriteClient.cpp"
    "${CHIP_ROOT}/src/app/WriteHandler.cpp"

    # Server
    "${CHIP_ROOT}/src/app/server/AclStorage.cpp"
    "${CHIP_ROOT}/src/app/server/CommissioningWindowManager.cpp"
    "${CHIP_ROOT}/src/app/server/DefaultAclStorage.cpp"
    "${CHIP_ROOT}/src/app/server/DefaultTermsAndConditionsProvider.cpp"
    "${CHIP_ROOT}/src/app/server/Dnssd.cpp"
    "${CHIP_ROOT}/src/app/server/EchoHandler.cpp"
    "${CHIP_ROOT}/src/app/server/Server.cpp"
    "${CHIP_ROOT}/src/app/server/TermsAndConditionsManager.cpp"
    # JointFabricDatastore.cpp excluded — CHIP_DEVICE_CONFIG_ENABLE_JOINT_FABRIC=0

    # MessageDef — TLV builders/parsers for interaction model messages
    "${CHIP_ROOT}/src/app/MessageDef/ArrayBuilder.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ArrayParser.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeDataIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeDataIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributePathIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributePathIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeReportIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeReportIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeStatusIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/AttributeStatusIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/Builder.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ClusterPathIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/CommandDataIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/CommandPathIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/CommandStatusIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/DataVersionFilterIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/DataVersionFilterIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventDataIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventFilterIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventFilterIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventPathIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventPathIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventReportIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventReportIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/EventStatusIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/InvokeRequestMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/InvokeRequests.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/InvokeResponseIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/InvokeResponseIBs.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/InvokeResponseMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ListBuilder.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ListParser.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/MessageBuilder.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/MessageDefHelper.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/MessageParser.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/Parser.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ReadRequestMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/ReportDataMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/StatusIB.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/StatusResponseMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/StructBuilder.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/StructParser.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/SubscribeRequestMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/SubscribeResponseMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/TimedRequestMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/WriteRequestMessage.cpp"
    "${CHIP_ROOT}/src/app/MessageDef/WriteResponseMessage.cpp"

    # Reporting engine — required by InteractionModelEngine
    "${CHIP_ROOT}/src/app/reporting/Engine.cpp"
    "${CHIP_ROOT}/src/app/reporting/reporting.cpp"
    "${CHIP_ROOT}/src/app/reporting/ReportSchedulerImpl.cpp"
    # SynchronizedReportSchedulerImpl.cpp excluded (uses separate sync task)

    # Access control
    "${CHIP_ROOT}/src/access/AccessControl.cpp"
    "${CHIP_ROOT}/src/access/AccessRestrictionProvider.cpp"

    # Address resolver — used by CASESessionManager
    "${CHIP_ROOT}/src/lib/address_resolve/AddressResolve.cpp"
    "${CHIP_ROOT}/src/lib/address_resolve/AddressResolve_DefaultImpl.cpp"

    # ASN.1 encoder/decoder (used by cert chain validation in CHIPCert)
    "${CHIP_ROOT}/src/lib/asn1/ASN1Error.cpp"
    "${CHIP_ROOT}/src/lib/asn1/ASN1OID.cpp"
    "${CHIP_ROOT}/src/lib/asn1/ASN1Reader.cpp"
    "${CHIP_ROOT}/src/lib/asn1/ASN1Time.cpp"
    "${CHIP_ROOT}/src/lib/asn1/ASN1Writer.cpp"

    # Credentials examples (dev-only attestation credentials)
    "${CHIP_ROOT}/src/credentials/examples/DeviceAttestationCredsExample.cpp"
    "${CHIP_ROOT}/src/credentials/examples/ExampleDACs.cpp"
    "${CHIP_ROOT}/src/credentials/examples/ExamplePAI.cpp"

    # Setup payload — QR code / onboarding codes
    "${CHIP_ROOT}/src/setup_payload/AdditionalDataPayloadGenerator.cpp"
    "${CHIP_ROOT}/src/setup_payload/AdditionalDataPayloadParser.cpp"
    "${CHIP_ROOT}/src/setup_payload/Base38Decode.cpp"
    "${CHIP_ROOT}/src/setup_payload/Base38Encode.cpp"
    "${CHIP_ROOT}/src/setup_payload/ManualSetupPayloadGenerator.cpp"
    "${CHIP_ROOT}/src/setup_payload/ManualSetupPayloadParser.cpp"
    "${CHIP_ROOT}/src/setup_payload/OnboardingCodesUtil.cpp"
    "${CHIP_ROOT}/src/setup_payload/QRCodeSetupPayloadGenerator.cpp"
    "${CHIP_ROOT}/src/setup_payload/QRCodeSetupPayloadParser.cpp"
    "${CHIP_ROOT}/src/setup_payload/SetupPayload.cpp"
    "${CHIP_ROOT}/src/setup_payload/SetupPayloadHelper.cpp"

    # Codegen data model provider (needed for CommonCaseDeviceServerInitParams)
    "${CHIP_ROOT}/src/data-model-providers/codegen/CodegenDataModelProvider.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/CodegenDataModelProvider_Read.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/CodegenDataModelProvider_Write.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/EmberAttributeDataBuffer.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/EmberMetadata.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/Instance.cpp"
    "${CHIP_ROOT}/src/data-model-providers/codegen/ServerClusterInterfaceRegistry.cpp"

    # Data model provider base (ActionReturnStatus, MetadataLookup, ProviderMetadataTree)
    # StringBuilderAdapters.cpp excluded — requires Pigweed pw_string headers
    "${CHIP_ROOT}/src/app/data-model-provider/ActionReturnStatus.cpp"
    "${CHIP_ROOT}/src/app/data-model-provider/MetadataLookup.cpp"
    "${CHIP_ROOT}/src/app/data-model-provider/ProviderMetadataTree.cpp"

    # Access control example delegate (dev-only; production replaces with real ACL)
    "${CHIP_ROOT}/src/access/examples/ExampleAccessControlDelegate.cpp"

    # Attribute persistence provider (default: writes to KeyValueStore)
    "${CHIP_ROOT}/src/app/util/persistence/AttributePersistenceProvider.cpp"
    "${CHIP_ROOT}/src/app/util/persistence/DefaultAttributePersistenceProvider.cpp"

    # InteractionModel status codes
    "${CHIP_ROOT}/src/protocols/interaction_model/StatusCode.cpp"

    # Protocol name/type lookup (used by ExchangeMgr and SessionManager logging)
    "${CHIP_ROOT}/src/protocols/Protocols.cpp"

    # TLV struct decode/encode helpers
    "${CHIP_ROOT}/src/app/data-model/StructDecodeIterator.cpp"
    "${CHIP_ROOT}/src/app/data-model/WrappedStructEncoder.cpp"

    # Per-application command dispatch (generated by ZAP at configure time)
    "${ZAP_OUTPUT_DIR}/IMClusterCommandHandler.cpp"

    # ServerClusterInterface base implementation (PathsContains, etc.)
    "${CHIP_ROOT}/src/app/server-cluster/ServerClusterInterface.cpp"

    # emberAfClusterInitCallback stub (not in generic-callback-stubs.cpp)
    "${CMAKE_CURRENT_LIST_DIR}/src/matter/sesame_chip_app_callbacks.cpp"

    # Ember data model / util
    "${CHIP_ROOT}/src/app/util/attribute-metadata.cpp"
    "${CHIP_ROOT}/src/app/util/attribute-storage.cpp"
    "${CHIP_ROOT}/src/app/util/attribute-table.cpp"
    "${CHIP_ROOT}/src/app/util/binding-table.cpp"
    "${CHIP_ROOT}/src/app/util/DataModelHandler.cpp"
    "${CHIP_ROOT}/src/app/util/ember-io-storage.cpp"
    "${CHIP_ROOT}/src/app/util/ember-strings.cpp"
    "${CHIP_ROOT}/src/app/util/generic-callback-stubs.cpp"
    "${CHIP_ROOT}/src/app/util/MatterCallbacks.cpp"
    "${CHIP_ROOT}/src/app/util/privilege-storage.cpp"
    "${CHIP_ROOT}/src/app/util/util.cpp"
)
target_link_libraries(chip_app
    PUBLIC
    chip_secure_channel
    chip_includes
    chip_compile_flags
)

# chip_minmdns: MinMdns DNS-SD advertiser and resolver.
# ----------------------------------------------------------------------------
add_library(chip_minmdns STATIC
    # Minimal mDNS core
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/core/QName.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/core/QNameString.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/core/RecordWriter.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/records/IP.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/records/ResourceRecord.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/responders/IP.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/responders/QueryResponder.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/AddressPolicy.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/AddressPolicy_DefaultImpl.cpp"
    # AddressPolicy_LibNlImpl.cpp excluded — uses netlink, not our FreeRTOS+TCP shim
    # Logging.cpp excluded — when CHIP_MINMDNS_HIGH_VERBOSITY=0 the header already
    # provides inline no-ops; including the .cpp causes redefinition errors.
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/Parser.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/RecordData.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/ResponseSender.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/minimal_mdns/Server.cpp"
    # dnssd upper layer
    "${CHIP_ROOT}/src/lib/dnssd/ActiveResolveAttempts.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/Advertiser.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/Advertiser_ImplMinimalMdns.cpp"
    # Advertiser_ImplNone.cpp excluded — conflicts with ImplMinimalMdns
    "${CHIP_ROOT}/src/lib/dnssd/Discovery_ImplPlatform.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/IncrementalResolve.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/IPAddressSorter.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/MinimalMdnsServer.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/Resolver.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/Resolver_ImplMinimalMdns.cpp"
    # Resolver_ImplNone.cpp excluded — conflicts with ImplMinimalMdns
    "${CHIP_ROOT}/src/lib/dnssd/ResolverProxy.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/ServiceNaming.cpp"
    "${CHIP_ROOT}/src/lib/dnssd/TxtFields.cpp"
)
target_link_libraries(chip_minmdns
    PUBLIC
    chip_app
    chip_includes
    chip_compile_flags
)
target_compile_definitions(chip_minmdns
    PUBLIC
    CHIP_DNSSD_DEFAULT_MINIMAL=1
    CHIP_MINMDNS_DEFAULT_POLICY=1
)

# chip_clusters: mandatory server cluster implementations.
# These provide the MatterXxxPluginServerInitCallback symbols declared in
# app/PluginApplicationCallbacks.h and the cluster command handling.
# ----------------------------------------------------------------------------
set(CHIP_CLUSTERS_DIR "${CHIP_ROOT}/src/app/clusters")
add_library(chip_clusters STATIC
    # ZAP-generated cluster struct encode/decode (all clusters, including root-node)
    "${CHIP_ROOT}/zzz_generated/app-common/app-common/zap-generated/cluster-objects.cpp"
    # ZAP-generated attribute accessors (Get/Set for all cluster attributes)
    "${CHIP_ROOT}/zzz_generated/app-common/app-common/zap-generated/attributes/Accessors.cpp"

    # Mandatory root-node clusters
    "${CHIP_CLUSTERS_DIR}/basic-information/basic-information.cpp"
    "${CHIP_CLUSTERS_DIR}/general-commissioning-server/general-commissioning-server.cpp"
    "${CHIP_CLUSTERS_DIR}/network-commissioning/network-commissioning.cpp"
    "${CHIP_CLUSTERS_DIR}/network-commissioning/WifiScanResponse.cpp"
    "${CHIP_CLUSTERS_DIR}/general-diagnostics-server/general-diagnostics-server.cpp"
    "${CHIP_CLUSTERS_DIR}/general-diagnostics-server/GenericFaultTestEventTriggerHandler.cpp"
    "${CHIP_CLUSTERS_DIR}/wifi-network-diagnostics-server/wifi-network-diagnostics-server.cpp"
    "${CHIP_CLUSTERS_DIR}/administrator-commissioning-server/administrator-commissioning-server.cpp"
    "${CHIP_CLUSTERS_DIR}/operational-credentials-server/operational-credentials-server.cpp"
    "${CHIP_CLUSTERS_DIR}/access-control-server/access-control-server.cpp"
    "${CHIP_CLUSTERS_DIR}/group-key-mgmt-server/group-key-mgmt-server.cpp"
    "${CHIP_CLUSTERS_DIR}/descriptor/descriptor.cpp"
    # Application cluster: Window Covering
    "${CHIP_CLUSTERS_DIR}/window-covering-server/window-covering-server.cpp"
    # Standard clusters required on endpoint 1 (Window Covering device type)
    "${CHIP_CLUSTERS_DIR}/groups-server/groups-server.cpp"
    "${CHIP_CLUSTERS_DIR}/identify-server/identify-server.cpp"
    "${CHIP_CLUSTERS_DIR}/scenes-server/scenes-server.cpp"
    "${CHIP_CLUSTERS_DIR}/scenes-server/SceneHandlerImpl.cpp"
    "${CHIP_CLUSTERS_DIR}/scenes-server/ExtensionFieldSetsImpl.cpp"
    "${CHIP_CLUSTERS_DIR}/scenes-server/SceneTableImpl.cpp"
    "${CHIP_CLUSTERS_DIR}/localization-configuration-server/localization-configuration-server.cpp"
    "${CHIP_CLUSTERS_DIR}/time-format-localization-server/time-format-localization-server.cpp"
    # Diagnostics clusters with command callbacks referenced by IMClusterCommandHandler
    "${CHIP_CLUSTERS_DIR}/diagnostic-logs-server/diagnostic-logs-server.cpp"
    "${CHIP_CLUSTERS_DIR}/ethernet-network-diagnostics-server/ethernet-network-diagnostics-server.cpp"
    "${CHIP_CLUSTERS_DIR}/thread-network-diagnostics-server/thread-network-diagnostics-server.cpp"
    "${CHIP_CLUSTERS_DIR}/thread-network-diagnostics-server/thread-network-diagnostics-provider.cpp"
    # OTA Requestor cluster + BDX downloader
    "${CHIP_CLUSTERS_DIR}/ota-requestor/DefaultOTARequestor.cpp"
    "${CHIP_CLUSTERS_DIR}/ota-requestor/DefaultOTARequestorDriver.cpp"
    "${CHIP_CLUSTERS_DIR}/ota-requestor/DefaultOTARequestorStorage.cpp"
    "${CHIP_CLUSTERS_DIR}/ota-requestor/BDXDownloader.cpp"
    "${CHIP_CLUSTERS_DIR}/ota-requestor/ota-requestor-server.cpp"
)
target_link_libraries(chip_clusters
    PUBLIC
    chip_app
    chip_minmdns
    chip_includes
    chip_compile_flags
)

# Aggregate target the rest of Sesame links against.
# The CHIP static libraries have mutual dependencies (e.g. chip_app pulls in
# symbols from chip_minmdns and chip_credentials which in turn need chip_app
# symbols).  Wrap the whole group with --start-group/--end-group so the GNU
# linker iterates until all cross-archive references are satisfied.
#
# chip_platform_mw320 is INSIDE the group so that:
#  - Its instantiation of GenericPlatformManagerImpl creates demands for symbols
#    from chip_core/chip_inet/chip_platform_generic (RegisterCHIPLayerErrorFormatter,
#    RegisterLayerErrorFormatter, InitEntropy, etc.) that are satisfied within
#    the group iteration.
#  - ConnectivityManagerImpl_sesame.cpp (in chip_platform_mw320) creates a demand
#    for vtable for UDPEndPointImplSockets / TCPEndPointImplSockets which are
#    defined in chip_inet — both must be in the same group pass.
add_library(chip INTERFACE)
if(NOT USE_QEMU)
    target_link_libraries(chip INTERFACE
        -Wl,--start-group
        chip_clusters
        chip_minmdns
        chip_app
        chip_secure_channel
        chip_transport
        chip_messaging
        chip_credentials
        chip_platform_generic
        chip_platform_mw320
        chip_system
        chip_inet
        chip_crypto
        chip_core
        chip_support
        mbedcrypto
        -Wl,--end-group
    )
else()
    target_link_libraries(chip INTERFACE
        -Wl,--start-group
        chip_clusters
        chip_minmdns
        chip_app
        chip_secure_channel
        chip_transport
        chip_messaging
        chip_credentials
        chip_platform_generic
        chip_platform_qemu
        chip_system
        chip_inet
        chip_crypto
        chip_core
        chip_support
        mbedcrypto
        -Wl,--end-group
    )
endif()
