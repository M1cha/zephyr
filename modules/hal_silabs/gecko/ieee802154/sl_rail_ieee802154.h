/**
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_IEEE802154_H_
#define ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_IEEE802154_H_

#include <rail.h>
#include <rail_ieee802154.h>

#define SL_RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES
#define SL_RAIL_IEEE802154_CCA_MODE_RSSI          RAIL_IEEE802154_CCA_MODE_RSSI
#define SL_RAIL_IEEE802154_E_OPTION_ENH_ACK       RAIL_IEEE802154_E_OPTION_ENH_ACK
#define SL_RAIL_IEEE802154_E_OPTION_GB868         RAIL_IEEE802154_E_OPTION_GB868
#define SL_RAIL_IEEE802154_LONG_ADDRESS           RAIL_IEEE802154_LongAddress
#define SL_RAIL_IEEE802154_SHORT_ADDRESS          RAIL_IEEE802154_ShortAddress

typedef struct sl_rail_ieee802154_address {
	union {
		uint16_t short_address;
		uint8_t long_address[8];
	};
	RAIL_IEEE802154_AddressLength_t address_length;
	RAIL_AddrFilterMask_t filter_mask;
} sl_rail_ieee802154_address_t;
BUILD_ASSERT(sizeof(sl_rail_ieee802154_address_t) == sizeof(RAIL_IEEE802154_Address_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_address_t, short_address, RAIL_IEEE802154_Address_t,
			 shortAddress);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_address_t, long_address, RAIL_IEEE802154_Address_t,
			 longAddress);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_address_t, address_length, RAIL_IEEE802154_Address_t,
			 length);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_address_t, filter_mask, RAIL_IEEE802154_Address_t,
			 filterMask);

typedef struct sl_rail_ieee802154_config {
	const RAIL_IEEE802154_AddrConfig_t *p_addresses;
	sl_rail_auto_ack_config_t ack_config;
	sl_rail_state_timing_t timings;
	uint8_t frames_mask;
	bool promiscuous_mode;
	bool is_pan_coordinator;
	bool default_frame_pending_in_outgoing_acks;
} sl_rail_ieee802154_config_t;

BUILD_ASSERT(sizeof(sl_rail_ieee802154_config_t) == sizeof(RAIL_IEEE802154_Config_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, p_addresses, RAIL_IEEE802154_Config_t,
			 addresses);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, ack_config, RAIL_IEEE802154_Config_t,
			 ackConfig);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, timings, RAIL_IEEE802154_Config_t, timings);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, frames_mask, RAIL_IEEE802154_Config_t,
			 framesMask);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, promiscuous_mode, RAIL_IEEE802154_Config_t,
			 promiscuousMode);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, is_pan_coordinator, RAIL_IEEE802154_Config_t,
			 isPanCoordinator);
ASSERT_FIELDS_COMPATIBLE(sl_rail_ieee802154_config_t, default_frame_pending_in_outgoing_acks,
			 RAIL_IEEE802154_Config_t, defaultFramePendingInOutgoingAcks);

static inline sl_rail_status_t
sl_rail_ieee802154_get_address(sl_rail_handle_t rail_handle,
			       sl_rail_ieee802154_address_t *p_address)
{
	return RAIL_IEEE802154_GetAddress(rail_handle, PCAST(RAIL_IEEE802154_Address_t, p_address));
}

static inline sl_rail_status_t sl_rail_ieee802154_init(sl_rail_handle_t rail_handle,
						       const sl_rail_ieee802154_config_t *p_config)
{
	return RAIL_IEEE802154_Init(rail_handle, CONST_PCAST(RAIL_IEEE802154_Config_t, p_config));
}

static inline sl_rail_status_t sl_rail_ieee802154_toggle_frame_pending(sl_rail_handle_t rail_handle)
{
	return RAIL_IEEE802154_SetFramePending(rail_handle);
}

static inline sl_rail_status_t sl_rail_ieee802154_write_enh_ack(sl_rail_handle_t rail_handle,
								const uint8_t *p_ack_data,
								uint16_t ack_data_bytes)
{
	return RAIL_IEEE802154_WriteEnhAck(rail_handle, p_ack_data, ack_data_bytes);
}

static inline sl_rail_status_t
sl_rail_ieee802154_enable_early_frame_pending(sl_rail_handle_t rail_handle, bool enable)
{
	return RAIL_IEEE802154_EnableEarlyFramePending(rail_handle, enable);
}

static inline sl_rail_status_t
sl_rail_ieee802154_enable_data_frame_pending(sl_rail_handle_t rail_handle, bool enable)
{
	return RAIL_IEEE802154_EnableDataFramePending(rail_handle, enable);
}

static inline sl_rail_status_t
sl_rail_ieee802154_config_cca_mode(sl_rail_handle_t rail_handle,
				   sl_rail_ieee802154_cca_mode_t cca_mode)
{
	return RAIL_IEEE802154_ConfigCcaMode(rail_handle, cca_mode);
}

static inline sl_rail_status_t sl_rail_ieee802154_set_short_address(sl_rail_handle_t rail_handle,
								    uint16_t short_addr,
								    uint8_t index)
{
	return RAIL_IEEE802154_SetShortAddress(rail_handle, short_addr, index);
}

static inline sl_rail_status_t sl_rail_ieee802154_set_long_address(sl_rail_handle_t rail_handle,
								   const uint8_t *p_long_addr,
								   uint8_t index)
{
	return RAIL_IEEE802154_SetLongAddress(rail_handle, p_long_addr, index);
}

static inline sl_rail_status_t sl_rail_ieee802154_set_promiscuous_mode(sl_rail_handle_t rail_handle,
								       bool enable)
{
	return RAIL_IEEE802154_SetPromiscuousMode(rail_handle, enable);
}

static inline sl_rail_status_t sl_rail_calibrate(sl_rail_handle_t rail_handle,
						 sl_rail_cal_values_t *p_cal_values,
						 sl_rail_cal_mask_t cal_force_mask)
{
	return RAIL_Calibrate(rail_handle, p_cal_values, cal_force_mask);
}

static inline sl_rail_status_t sl_rail_ieee802154_set_pan_coordinator(sl_rail_handle_t rail_handle,
								      bool is_pan_coordinator)
{
	return RAIL_IEEE802154_SetPanCoordinator(rail_handle, is_pan_coordinator);
}

static inline sl_rail_status_t sl_rail_ieee802154_config_2p4_ghz_radio(sl_rail_handle_t rail_handle)
{
	return RAIL_IEEE802154_Config2p4GHzRadio(rail_handle);
}

static inline sl_rail_status_t
sl_rail_ieee802154_config_e_options(sl_rail_handle_t rail_handle,
				    sl_rail_ieee802154_e_options_t mask,
				    sl_rail_ieee802154_e_options_t options)
{
	return RAIL_IEEE802154_ConfigEOptions(rail_handle, mask, options);
}

static inline sl_rail_status_t sl_rail_ieee802154_set_pan_id(sl_rail_handle_t rail_handle,
							     uint16_t pan_id, uint8_t index)
{
	return RAIL_IEEE802154_SetPanId(rail_handle, pan_id, index);
}

static inline sl_rail_status_t
sl_rail_ieee802154_set_rx_to_enh_ack_tx(sl_rail_handle_t rail_handle,
					sl_rail_transition_time_t *p_rx_to_enh_ack_tx)
{
	return RAIL_IEEE802154_SetRxToEnhAckTx(rail_handle, p_rx_to_enh_ack_tx);
}

#endif /* ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_IEEE802154_H_ */
