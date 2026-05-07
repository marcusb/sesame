/*
 * Default (weak) implementations of FreeRTOS-Plus-TCP DNS hooks.
 */

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"

/*
 * xApplicationDNSQueryHook_Multi is called by FreeRTOS-Plus-TCP when
 * ipconfigDNSQuery_MULTI is enabled. We return pdFALSE as we don't handle
 * generic DNS queries here (the bespoke mDNS responder handles 5353 via
 * its own socket).
 */
__attribute__((weak)) BaseType_t xApplicationDNSQueryHook_Multi(
    struct xNetworkEndPoint* pxEndPoint, const char* pcName) {
    (void)pxEndPoint;
    (void)pcName;
    return pdFALSE;
}
