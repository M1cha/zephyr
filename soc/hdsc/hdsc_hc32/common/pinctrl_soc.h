/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_HDSC_HC32_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_HDSC_HC32_COMMON_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#include <zephyr/dt-bindings/pinctrl/hc32-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

#define HC32_BIAS_DISABLE_POS     11
#define HC32_BIAS_PULL_DOWN_POS   12
#define HC32_BIAS_PULL_UP_POS     13
#define HC32_DRIVE_PUSH_PULL_POS  14
#define HC32_DRIVE_OPEN_DRAIN_POS 15
#define HC32_OUTPUT_DISABLE_POS   16
#define HC32_OUTPUT_ENABLE_POS    17
#define HC32_OUTPUT_HIGH_POS      18
#define HC32_OUTPUT_LOW_POS       19
#define HC32_DRIVE_STRENGTH_POS   20
#define HC32_ANALOG_MODE_POS      21

/** @brief Type for HC32 pin. */
typedef uint32_t pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                         \
	(DT_PROP_BY_IDX(node_id, prop, idx) | \
	 (DT_PROP(node_id, bias_disable) << HC32_BIAS_DISABLE_POS) |                               \
	 (DT_PROP(node_id, bias_pull_down) << HC32_BIAS_PULL_DOWN_POS) |                           \
	 (DT_PROP(node_id, bias_pull_up) << HC32_BIAS_PULL_UP_POS) |                               \
	 (DT_PROP(node_id, drive_push_pull) << HC32_DRIVE_PUSH_PULL_POS) |                         \
	 (DT_PROP(node_id, drive_open_drain) << HC32_DRIVE_OPEN_DRAIN_POS) |                       \
	 (DT_PROP(node_id, output_disable) << HC32_OUTPUT_DISABLE_POS) |                           \
	 (DT_PROP(node_id, output_enable) << HC32_OUTPUT_ENABLE_POS) |                             \
	 (DT_PROP(node_id, output_high) << HC32_OUTPUT_HIGH_POS) |                                 \
	 (DT_PROP(node_id, output_low) << HC32_OUTPUT_LOW_POS) |                                   \
	 (DT_ENUM_IDX(node_id, drive_strength) << HC32_DRIVE_STRENGTH_POS) | \
	 (DT_PROP(node_id, analog_mode) << HC32_ANALOG_MODE_POS)),

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop),		       \
				DT_FOREACH_PROP_ELEM, pinmux,		       \
				Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_HDSC_HC32_COMMON_PINCTRL_SOC_H_ */
