#pragma once

#include "clock_config.h"
#include "fsl_common.h"

#define BOARD_NAME "genie"

#define BOARD_IS_XIP                                \
    (((uint32_t)init_debug_console >= 0x1F000000U) && \
     ((uint32_t)init_debug_console < 0x20000000U))

/*! @brief The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE kSerialPort_Uart
#define BOARD_DEBUG_UART_BASEADDR (uint32_t)UART0
#define BOARD_DEBUG_UART_INSTANCE 0U
#define BOARD_DEBUG_UART UART0
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetUartClkFreq(0)
#define BOARD_DEBUG_UART_CLK_ATTACH kSYS_CLK_to_FAST_UART0
#define BOARD_DEBUG_UART_CLK_DIV kCLOCK_DivUartFast
#define BOARD_UART_IRQ_HANDLER UART0_IRQHandler
#define BOARD_UART_IRQ UART0_IRQn

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

void init_debug_console(void);

/* Only used for mbedtls entropy, implemented in board_hash.c */
void BOARD_GetHash(uint8_t *buf, uint32_t *len);
