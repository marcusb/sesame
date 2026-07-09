/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MW320_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MW320_PINCTRL_H_

/**
 * @brief Macro to encode a pinmux configuration for Marvell MW320.
 *
 * @param pin Pin number (0..49)
 * @param func Alternative function number (0..7)
 */
#define MW320_PINMUX(pin, func) (((pin) & 0x7F) | (((func) & 0x7) << 8))

#define MW320_GET_PIN(pinmux) ((pinmux) & 0x7F)
#define MW320_GET_FUNC(pinmux) (((pinmux) >> 8) & 0x7)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_MW320_PINCTRL_H_ */
