/*
 * Minimal PluginApplicationCallbacks.h for Sesame.
 * Declares cluster plugin init callbacks and defines MATTER_PLUGINS_INIT.
 * Cluster implementations provide these symbols; add entries here as cluster
 * server libs are linked in.
 */
#pragma once

void MatterDescriptorPluginServerInitCallback();
void MatterAccessControlPluginServerInitCallback();
void MatterBasicInformationPluginServerInitCallback();
void MatterOtaSoftwareUpdateRequestorPluginServerInitCallback();
void MatterGeneralCommissioningPluginServerInitCallback();
void MatterNetworkCommissioningPluginServerInitCallback();
void MatterGeneralDiagnosticsPluginServerInitCallback();
void MatterWiFiNetworkDiagnosticsPluginServerInitCallback();
void MatterAdministratorCommissioningPluginServerInitCallback();
void MatterOperationalCredentialsPluginServerInitCallback();
void MatterGroupKeyManagementPluginServerInitCallback();
void MatterWindowCoveringPluginServerInitCallback();

#define MATTER_PLUGINS_INIT                                     \
    MatterDescriptorPluginServerInitCallback();                 \
    MatterAccessControlPluginServerInitCallback();              \
    MatterBasicInformationPluginServerInitCallback();           \
    MatterOtaSoftwareUpdateRequestorPluginServerInitCallback(); \
    MatterGeneralCommissioningPluginServerInitCallback();       \
    MatterNetworkCommissioningPluginServerInitCallback();       \
    MatterGeneralDiagnosticsPluginServerInitCallback();         \
    MatterWiFiNetworkDiagnosticsPluginServerInitCallback();     \
    MatterAdministratorCommissioningPluginServerInitCallback(); \
    MatterOperationalCredentialsPluginServerInitCallback();     \
    MatterGroupKeyManagementPluginServerInitCallback();         \
    MatterWindowCoveringPluginServerInitCallback();
