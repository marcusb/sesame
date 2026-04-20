/*
 * Default (weak) implementations of FreeRTOS-Plus-TCP mDNS responder hooks.
 *
 * ipconfigUSE_MDNS=1 makes FreeRTOS_DNS_Parser reference these hooks in every
 * build. Matter provides a strong xApplicationDNSRecordQueryHook_Multi in
 * matter_mdns.c; when Matter is disabled (RAM build), the weak defaults here
 * respond with an empty record set.
 */

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"

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
