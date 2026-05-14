/* Minimal ConnectivityManagerImpl for Sesame.
 *
 * The upstream mw320/ConnectivityManagerImpl.cpp is LwIP-dependent.  This
 * file provides the template instantiations (defining _UDPEndPointManager()
 * and _TCPEndPointManager()) and minimal WiFi management stubs.  The real
 * WiFi integration is through network_manager.c; a fuller subclass that
 * bridges the two will replace this in a later migration step.
 *
 * Note: _HaveIPv4InternetConnectivity(), _HaveIPv6InternetConnectivity(), and
 * _HaveServiceConnectivity() are already defined as inline in ConnectivityManagerImpl.h
 * and must NOT be redefined here.
 */
#include <platform/internal/CHIPDeviceLayerInternal.h>
#include <platform/internal/GenericConnectivityManagerImpl_UDP.ipp>

#if INET_CONFIG_ENABLE_TCP_ENDPOINT
#include <platform/internal/GenericConnectivityManagerImpl_TCP.ipp>
#endif

#include <platform/nxp/mw320/ConnectivityManagerImpl.h>

using namespace ::chip;
using namespace ::chip::Inet;
using namespace ::chip::System;
using namespace ::chip::DeviceLayer::Internal;

namespace chip {
namespace DeviceLayer {

ConnectivityManagerImpl ConnectivityManagerImpl::sInstance;

uint8_t ConnectivityManagerImpl::sInterestedSSID[Internal::kMaxWiFiSSIDLength];
uint8_t ConnectivityManagerImpl::sInterestedSSIDLen;
uint8_t ConnectivityManagerImpl::sCfgSSID[Internal::kMaxWiFiSSIDLength];
uint8_t ConnectivityManagerImpl::sCfgSSIDLen;

NetworkCommissioning::WiFiDriver::ScanCallback * ConnectivityManagerImpl::mpScanCallback;
NetworkCommissioning::Internal::WirelessDriver::ConnectCallback * ConnectivityManagerImpl::mpConnectCallback;

CHIP_ERROR ConnectivityManagerImpl::_Init()
{
    return CHIP_NO_ERROR;
}

void ConnectivityManagerImpl::_OnPlatformEvent(const ChipDeviceEvent *)
{
}

ConnectivityManager::WiFiStationMode ConnectivityManagerImpl::_GetWiFiStationMode()
{
    return ConnectivityManager::kWiFiStationMode_Enabled;
}

CHIP_ERROR ConnectivityManagerImpl::_SetWiFiStationMode(ConnectivityManager::WiFiStationMode)
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR ConnectivityManagerImpl::_SetWiFiAPMode(WiFiAPMode)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

bool ConnectivityManagerImpl::_IsWiFiStationEnabled()
{
    return true;
}

bool ConnectivityManagerImpl::_IsWiFiStationConnected()
{
    return false;
}

bool ConnectivityManagerImpl::_IsWiFiStationApplicationControlled()
{
    return true;
}

void ConnectivityManagerImpl::StartWiFiManagement()
{
}

CHIP_ERROR ConnectivityManagerImpl::CommitConfig()
{
    return CHIP_NO_ERROR;
}

CHIP_ERROR ConnectivityManagerImpl::ConnectWiFiNetworkAsync(ByteSpan, ByteSpan,
                                                            NetworkCommissioning::Internal::WirelessDriver::ConnectCallback *)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR ConnectivityManagerImpl::GetWiFiBssId(MutableByteSpan & value)
{
    memset(value.data(), 0, value.size());
    return CHIP_NO_ERROR;
}

CHIP_ERROR ConnectivityManagerImpl::GetWiFiSecurityType(app::Clusters::WiFiNetworkDiagnostics::SecurityTypeEnum & out)
{
    out = app::Clusters::WiFiNetworkDiagnostics::SecurityTypeEnum::kWpa2;
    return CHIP_NO_ERROR;
}

CHIP_ERROR ConnectivityManagerImpl::GetWiFiVersion(app::Clusters::WiFiNetworkDiagnostics::WiFiVersionEnum & out)
{
    out = app::Clusters::WiFiNetworkDiagnostics::WiFiVersionEnum::kB;
    return CHIP_NO_ERROR;
}

CHIP_ERROR ConnectivityManagerImpl::GetConfiguredNetwork(NetworkCommissioning::Network &)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

CHIP_ERROR ConnectivityManagerImpl::StartWiFiScan(ByteSpan, NetworkCommissioning::WiFiDriver::ScanCallback *)
{
    return CHIP_ERROR_NOT_IMPLEMENTED;
}

void ConnectivityManagerImpl::UpdateNetworkStatus()
{
}

} // namespace DeviceLayer
} // namespace chip
