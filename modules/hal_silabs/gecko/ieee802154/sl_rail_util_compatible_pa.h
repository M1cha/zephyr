/**
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_UTIL_COMPATIBLE_PA_H_
#define ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_UTIL_COMPATIBLE_PA_H_

#define SL_RAIL_TX_PA_MODE_2P4_GHZ 0

static inline sl_rail_status_t sl_rail_util_pa_post_init(sl_rail_handle_t rail_handle,
							 sl_rail_tx_pa_mode_t pa_mode)
{
	__ASSERT_NO_MSG(pa_mode == SL_RAIL_TX_PA_MODE_2P4_GHZ);
	return RAIL_ConfigTxPower(rail_handle, sl_rail_util_pa_get_tx_power_config_2p4ghz());
}

#endif /* ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_UTIL_COMPATIBLE_PA_H_ */

