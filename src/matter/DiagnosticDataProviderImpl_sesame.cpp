/* Minimal DiagnosticDataProviderImpl for Sesame.
 *
 * The upstream mw320/DiagnosticDataProviderImpl.cpp includes <lwip/tcpip.h>
 * and ConnectivityUtils.h which we don't have.  This file provides the
 * GetDefaultInstance() singleton and GetDiagnosticDataProviderImpl() factory
 * needed by PlatformManagerImpl and DiagnosticDataProvider.cpp, plus basic
 * heap and uptime diagnostics via FreeRTOS APIs.
 */

#include <platform/internal/CHIPDeviceLayerInternal.h>
#include <platform/DiagnosticDataProvider.h>
#include <platform/nxp/mw320/DiagnosticDataProviderImpl.h>

extern "C" {
#include "FreeRTOS.h"
}

using namespace chip;
using namespace chip::DeviceLayer;

namespace chip {
namespace DeviceLayer {

DiagnosticDataProviderImpl & DiagnosticDataProviderImpl::GetDefaultInstance()
{
    static DiagnosticDataProviderImpl sInstance;
    return sInstance;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetCurrentHeapFree(uint64_t & currentHeapFree)
{
    currentHeapFree = static_cast<uint64_t>(xPortGetFreeHeapSize());
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetCurrentHeapUsed(uint64_t & currentHeapUsed)
{
    currentHeapUsed = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetCurrentHeapHighWatermark(uint64_t & currentHeapHighWatermark)
{
    currentHeapHighWatermark = static_cast<uint64_t>(xPortGetMinimumEverFreeHeapSize());
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::ResetWatermarks()
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetRebootCount(uint16_t & rebootCount)
{
    rebootCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetUpTime(uint64_t & upTime)
{
    upTime = static_cast<uint64_t>(xTaskGetTickCount()) / configTICK_RATE_HZ;
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetTotalOperationalHours(uint32_t & totalOperationalHours)
{
    totalOperationalHours = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetBootReason(app::Clusters::GeneralDiagnostics::BootReasonEnum & bootReason)
{
    bootReason = app::Clusters::GeneralDiagnostics::BootReasonEnum::kUnspecified;
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetActiveHardwareFaults(GeneralFaults<kMaxHardwareFaults> & hardwareFaults)
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetActiveRadioFaults(GeneralFaults<kMaxRadioFaults> & radioFaults)
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetActiveNetworkFaults(GeneralFaults<kMaxNetworkFaults> & networkFaults)
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetNetworkInterfaces(NetworkInterface ** netifpp)
{
    *netifpp = nullptr;
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiChannelNumber(uint16_t & channelNumber)
{
    channelNumber = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiRssi(int8_t & rssi)
{
    rssi = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiBeaconRxCount(uint32_t & beaconRxCount)
{
    beaconRxCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiBeaconLostCount(uint32_t & beaconLostCount)
{
    beaconLostCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiPacketMulticastRxCount(uint32_t & packetMulticastRxCount)
{
    packetMulticastRxCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiPacketMulticastTxCount(uint32_t & packetMulticastTxCount)
{
    packetMulticastTxCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiPacketUnicastRxCount(uint32_t & packetUnicastRxCount)
{
    packetUnicastRxCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiPacketUnicastTxCount(uint32_t & packetUnicastTxCount)
{
    packetUnicastTxCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiCurrentMaxRate(uint64_t & currentMaxRate)
{
    currentMaxRate = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiOverrunCount(uint64_t & overrunCount)
{
    overrunCount = 0;
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::ResetWiFiNetworkDiagnosticsCounts()
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiVersion(app::Clusters::WiFiNetworkDiagnostics::WiFiVersionEnum & wiFiVersion)
{
    wiFiVersion = app::Clusters::WiFiNetworkDiagnostics::WiFiVersionEnum::kA;
    return CHIP_NO_ERROR;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiBssId(MutableByteSpan & value)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR DiagnosticDataProviderImpl::GetWiFiSecurityType(app::Clusters::WiFiNetworkDiagnostics::SecurityTypeEnum & securityType)
{
    securityType = app::Clusters::WiFiNetworkDiagnostics::SecurityTypeEnum::kWpa2;
    return CHIP_NO_ERROR;
}

DiagnosticDataProvider & GetDiagnosticDataProviderImpl()
{
    return DiagnosticDataProviderImpl::GetDefaultInstance();
}

} // namespace DeviceLayer
} // namespace chip
