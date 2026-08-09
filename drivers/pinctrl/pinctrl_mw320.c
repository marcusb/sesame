/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include "fsl_pinmux.h"
#include "fsl_clock.h"

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
    uint32_t modefunc = pin->alt_func;

    if (pin->pull_up) {
        modefunc |= PINMUX_MODE_PULLUP;
    } else if (pin->pull_down) {
        modefunc |= PINMUX_MODE_PULLDOWN;
    } else if (pin->bias_disable) {
        modefunc |= PINMUX_MODE_NOPULL;
    } else {
        modefunc |= PINMUX_MODE_DEFAULT;
    }

    PINMUX_PinMuxSet(pin->pin_num, modefunc);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
                           uintptr_t reg)
{
    ARG_UNUSED(reg);

    /* Ensure GPIO clock is enabled for PINMUX register access */
    CLOCK_EnableClock(kCLOCK_Gpio);

    for (uint8_t i = 0U; i < pin_cnt; i++) {
        pinctrl_configure_pin(pins++);
    }

    return 0;
}
