/*
 * Weak stubs for FreeRTOS+TCP DNS responder hooks.
 *
 * ipconfigUSE_MDNS=1 (needed by the matter host test) makes FreeRTOS_DNS_Parser
 * reference these hooks in every linkage. Non-matter tests don't provide them,
 * so we ship weak stubs here. matter_mdns.c supplies a strong
 * xApplicationDNSRecordQueryHook_Multi that overrides the stub.
 */

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"

/* Forces this TU to be pulled out of libhost_harness.a even though all the
   hook functions below are weak. Referenced (as extern) from harness.c. */
int matter_dns_hook_stub_anchor;

__attribute__((weak)) DNSRecord_t* xApplicationDNSRecordQueryHook_Multi(
    struct xNetworkEndPoint* pxEndPoint, UBaseType_t* outLen) {
    (void)pxEndPoint;
    *outLen = 0;
    return NULL;
}

__attribute__((weak)) DNSRecord_t* xApplicationDNSRecordQueryHook(
    UBaseType_t* outLen) {
    *outLen = 0;
    return NULL;
}

__attribute__((weak)) void xApplicationDNSRecordsMatchedHook(void) {}
