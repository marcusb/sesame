#pragma once

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

#include <stdint.h>

struct xNetworkInterface;
struct xNetworkInterface *pxMW300_FillInterfaceDescriptor(
    uint8_t if_type, struct xNetworkInterface *pxInterface);

void notify_dhcp_configured();

/* *INDENT-OFF* */
#ifdef __cplusplus
} /* extern "C" */
#endif
/* *INDENT-ON* */
