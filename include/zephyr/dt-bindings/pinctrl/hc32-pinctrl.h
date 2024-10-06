/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_

#define HC32_AF0 0
#define HC32_AF1 1
#define HC32_AF2 2
#define HC32_AF3 3
#define HC32_AF4 4
#define HC32_AF5 5
#define HC32_AF6 6
#define HC32_AF7 7

#define HC32_PORT_MASK  0xF
#define HC32_PORT_SHIFT 0
#define HC32_PIN_MASK   0xF
#define HC32_PIN_SHIFT  4
#define HC32_AF_MASK    0x7
#define HC32_AF_SHIFT   8

/**
 * @brief Build HC32 pinmux property value.
 *
 * Fields:
 *
 * - 0..3: port
 * - 4..7: pin
 * - 8..10: af
 *
 * @param port Port ('A'..'P')
 * @param pin Pin (0..15)
 * @param af Alternate function (AFx, x=0..7).
 */
#define HC32_PINMUX(port, pin, af)                                                                 \
	(((((port) - 'A') & HC32_PORT_MASK) << HC32_PORT_SHIFT) |                                  \
	 (((pin) & HC32_PIN_MASK) << HC32_PIN_SHIFT) |                                             \
	 (((HC32_##af) & HC32_AF_MASK) << HC32_AF_SHIFT))

#define HC32_PINMUX_PORT(pin) (((pin) >> HC32_PORT_SHIFT) & HC32_PORT_MASK)
#define HC32_PINMUX_PIN(pin)  (((pin) >> HC32_PIN_SHIFT) & HC32_PIN_MASK)
#define HC32_PINMUX_AF(pin)   (((pin) >> HC32_AF_SHIFT) & HC32_AF_MASK)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HC32_PINCTRL_H_ */
