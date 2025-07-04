// mw320
#include "fsl_pinmux.h"

// application
#include "pin_mux.h"

void board_init_pins(void) {
    PINMUX_PinMuxSet(BOARD_UART0_TX_PIN,
                     BOARD_UART0_TX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART0_RX_PIN,
                     BOARD_UART0_RX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART1_TX_PIN,
                     BOARD_UART1_TX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART1_RX_PIN,
                     BOARD_UART1_RX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(BOARD_LED_OTA_BLUE_PIN,
                     BOARD_LED_OTA_BLUE_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_LED_OTA_GREEN_PIN,
                     BOARD_LED_OTA_GREEN_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_LED_OTA_RED_PIN,
                     BOARD_LED_OTA_RED_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_LED_WIFI_BLUE_PIN,
                     BOARD_LED_WIFI_BLUE_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(
        BOARD_LED_WIFI_GREEN_PIN,
        BOARD_LED_WIFI_GREEN_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_LED_WIFI_RED_PIN,
                     BOARD_LED_WIFI_RED_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(BOARD_SW_WIFI_PIN,
                     BOARD_SW_WIFI_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_SW_OTA_PIN,
                     BOARD_SW_OTA_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(0, PINMUX_GPIO0_GPIO0 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(46, PINMUX_GPIO46_GPIO46 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(48, PINMUX_GPIO48_GPIO48 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(1, PINMUX_GPIO1_GPIO1 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(49, PINMUX_GPIO49_GPIO49 | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(BOARD_XTAL32K_IN_PIN,
                     BOARD_XTAL32K_IN_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_XTAL32K_OUT_PIN,
                     BOARD_XTAL32K_OUT_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
}
