#ifndef QEMU
#include "fsl_uart.h"
#else
#include <stddef.h>
#include <stdint.h>
typedef int32_t status_t;
#define kStatus_Success 0
typedef void UART_Type;
typedef void uart_handle_t;
typedef struct {
    uint8_t* data;
    size_t dataSize;
} uart_transfer_t;
#endif

status_t __wrap_UART_TransferReceiveNonBlocking(UART_Type* base,
                                                uart_handle_t* handle,
                                                uart_transfer_t* xfer,
                                                size_t* receivedBytes) {
    (void)base;
    (void)handle;
    (void)xfer;
    (void)receivedBytes;
    return kStatus_Success;
}

void __wrap_UART_TransferAbortReceive(UART_Type* base, uart_handle_t* handle) {
    (void)base;
    (void)handle;
}
