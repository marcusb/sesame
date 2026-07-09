/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_NXP_MW320_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_NXP_MW320_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <dt-bindings/pinctrl/mw320-pinctrl.h>

/**
 * @brief Type to hold a pin's pinctrl configuration.
 */
typedef struct pinctrl_soc_pin {
    uint32_t pin_num: 7;
    uint32_t alt_func: 3;
    uint32_t pull_up: 1;
    uint32_t pull_down: 1;
    uint32_t bias_disable: 1;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx) \
    { \
        .pin_num = MW320_GET_PIN(DT_PROP_BY_IDX(node_id, prop, idx)), \
        .alt_func = MW320_GET_FUNC(DT_PROP_BY_IDX(node_id, prop, idx)), \
        .pull_up = DT_PROP(node_id, bias_pull_up), \
        .pull_down = DT_PROP(node_id, bias_pull_down), \
        .bias_disable = DT_PROP(node_id, bias_disable), \
    },

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    {DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux, \
                Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_ARM_NXP_MW320_PINCTRL_SOC_H_ */
