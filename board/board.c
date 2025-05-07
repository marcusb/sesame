#include "board.h"

#include "fsl_clock.h"
#include "fsl_debug_console.h"

void init_debug_console(void) {
    /* attach FAST clock to UART0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE,
                    BOARD_DEBUG_UART_TYPE, BOARD_DEBUG_UART_CLK_FREQ);
}
