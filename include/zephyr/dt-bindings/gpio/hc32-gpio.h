/*
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_HC32_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_HC32_GPIO_H_

/**
 * @brief HC32-specific GPIO Flags
 * @defgroup gpio_interface_hc32 HC32-specific GPIO Flags
 * @ingroup gpio_interface
 * @{
 */

/** High drive (default) */
#define HC32_GPIO_DRIVE_HIGH	(0U << 8U)
/** Low drive */
#define HC32_GPIO_DRIVE_LOW	(1U << 8U)

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_HC32_GPIO_H_ */
