#include "fsl_uart.h"

status_t UART_TransferReceiveNonBlocking(UART_Type *base, uart_handle_t *handle,
                                         uart_transfer_t *xfer,
                                         size_t *receivedBytes) {
    (void)base; (void)handle; (void)xfer; (void)receivedBytes;
    return kStatus_Success;
}

void UART_TransferAbortReceive(UART_Type *base, uart_handle_t *handle) {
    (void)base; (void)handle;
}
