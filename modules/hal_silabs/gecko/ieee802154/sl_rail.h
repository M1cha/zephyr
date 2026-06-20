/**
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_H_
#define ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_H_

#include <rail.h>
#include <rail_ieee802154.h>
#include <pa_conversions_efr32.h>
#include <sli_protocol_crypto.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#define ASSERT_FIELDS_COMPATIBLE(s1, f1, s2, f2)                                                   \
	BUILD_ASSERT(offsetof(s1, f1) == offsetof(s2, f2));                                        \
	BUILD_ASSERT(SIZEOF_FIELD(s1, f1) == SIZEOF_FIELD(s2, f2));
#define PCAST(type, v)       ((type *)(void *)(v))
#define CONST_PCAST(type, v) ((const type *)(const void *)(v))

#define SL_RAIL_EVENT_CAL_NEEDED                      RAIL_EVENT_CAL_NEEDED
#define SL_RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND
#define SL_RAIL_EVENT_RX_ACK_TIMEOUT                  RAIL_EVENT_RX_ACK_TIMEOUT
#define SL_RAIL_EVENT_RX_PACKET_RECEIVED              RAIL_EVENT_RX_PACKET_RECEIVED
#define SL_RAIL_EVENT_RX_SCHEDULED_RX_END             RAIL_EVENT_RX_SCHEDULED_RX_END
#define SL_RAIL_EVENT_RX_SCHEDULED_RX_MISSED          RAIL_EVENT_RX_SCHEDULED_RX_MISSED
#define SL_RAIL_EVENT_RX_SCHEDULED_RX_STARTED         RAIL_EVENT_SCHEDULED_RX_STARTED
#define SL_RAIL_EVENT_SCHEDULER_STATUS                RAIL_EVENT_SCHEDULER_STATUS
#define SL_RAIL_EVENT_TX_SCHEDULED_TX_MISSED          RAIL_EVENT_TX_SCHEDULED_TX_MISSED
#define SL_RAIL_EVENT_TX_SCHEDULED_TX_STARTED         RAIL_EVENT_SCHEDULED_TX_STARTED
#define SL_RAIL_EVENT_TX_ABORTED                      RAIL_EVENT_TX_ABORTED
#define SL_RAIL_EVENT_TX_BLOCKED                      RAIL_EVENT_TX_BLOCKED
#define SL_RAIL_EVENT_TX_CHANNEL_BUSY                 RAIL_EVENT_TX_CHANNEL_BUSY
#define SL_RAIL_EVENT_TX_PACKET_SENT                  RAIL_EVENT_TX_PACKET_SENT
#define SL_RAIL_EVENT_TX_UNDERFLOW                    RAIL_EVENT_TX_UNDERFLOW
#define SL_RAIL_EVENT_TXACK_ABORTED                   RAIL_EVENT_TXACK_ABORTED
#define SL_RAIL_EVENT_TXACK_BLOCKED                   RAIL_EVENT_TXACK_BLOCKED
#define SL_RAIL_EVENT_TXACK_PACKET_SENT               RAIL_EVENT_TXACK_PACKET_SENT
#define SL_RAIL_EVENT_TXACK_UNDERFLOW                 RAIL_EVENT_TXACK_UNDERFLOW

#define SL_RAIL_EVENTS_ALL              RAIL_EVENTS_ALL
#define SL_RAIL_EVENTS_NONE             RAIL_EVENTS_NONE
#define SL_RAIL_EVENTS_TXACK_COMPLETION RAIL_EVENTS_TXACK_COMPLETION
#define SL_RAIL_EVENTS_TX_COMPLETION    RAIL_EVENTS_TX_COMPLETION

#define SL_RAIL_PTI_PROTOCOL_THREAD RAIL_PTI_PROTOCOL_THREAD

#define SL_RAIL_CAL_ALL         RAIL_CAL_ALL
#define SL_RAIL_CAL_ALL_PENDING RAIL_CAL_ALL_PENDING

#define SL_RAIL_IDLE RAIL_IDLE

#define SL_RAIL_RX_PACKET_READY_SUCCESS  RAIL_RX_PACKET_READY_SUCCESS
#define SL_RAIL_RX_PACKET_HANDLE_NEWEST  RAIL_RX_PACKET_HANDLE_NEWEST
#define SL_RAIL_RX_PACKET_HANDLE_INVALID RAIL_RX_PACKET_HANDLE_INVALID

#define SL_RAIL_TIME_DELAY RAIL_TIME_DELAY

#define SL_RAIL_RSSI_INVALID RAIL_RSSI_INVALID

#define SL_RAIL_GET_RSSI_NO_WAIT              RAIL_GET_RSSI_NO_WAIT
#define SL_RAIL_GET_RSSI_WAIT_WITHOUT_TIMEOUT RAIL_GET_RSSI_WAIT_WITHOUT_TIMEOUT

#define SL_RAIL_STREAM_CARRIER_WAVE RAIL_STREAM_CARRIER_WAVE

#define SL_RAIL_RF_STATE_IDLE RAIL_RF_STATE_IDLE
#define SL_RAIL_RF_STATE_RX   RAIL_RF_STATE_RX

#define SL_RAIL_STATUS_NO_ERROR      RAIL_STATUS_NO_ERROR
#define SL_RAIL_STATUS_INVALID_STATE RAIL_STATUS_INVALID_STATE

#define SL_RAIL_SCHEDULER_STATUS_NO_ERROR RAIL_SCHEDULER_STATUS_NO_ERROR

#define SL_RAIL_RX_OPTION_DISABLE_FRAME_DETECTION RAIL_RX_OPTION_DISABLE_FRAME_DETECTION
#define SL_RAIL_RX_OPTION_STORE_CRC               RAIL_RX_OPTION_STORE_CRC
#define SL_RAIL_RX_OPTION_TRACK_ABORTED_FRAMES    RAIL_RX_OPTION_TRACK_ABORTED_FRAMES

#define SL_RAIL_RX_OPTIONS_NONE                   RAIL_RX_OPTIONS_NONE
#define SL_RAIL_TX_OPTIONS_DEFAULT     RAIL_TX_OPTIONS_DEFAULT

#define SL_RAIL_TX_OPTION_WAIT_FOR_ACK RAIL_TX_OPTION_WAIT_FOR_ACK

#define SL_RAIL_PACKET_TIME_INVALID RAIL_PACKET_TIME_INVALID

#define SL_RAIL_CSMA_CONFIG_802_15_4_2003_2P4_GHZ_OQPSK_CSMA                                       \
	RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA

#define SL_RAIL_TIMER_SYNC_DEFAULT RAIL_TIMER_SYNC_DEFAULT

#define SL_RAIL_EFR32_HANDLE RAIL_EFR32_HANDLE

typedef RAIL_CalMask_t sl_rail_cal_mask_t;
typedef RAIL_CalValues_t sl_rail_cal_values_t;
typedef RAIL_Events_t sl_rail_events_t;
typedef RAIL_Handle_t sl_rail_handle_t;
typedef RAIL_IEEE802154_CcaMode_t sl_rail_ieee802154_cca_mode_t;
typedef RAIL_IEEE802154_EOptions_t sl_rail_ieee802154_e_options_t;
typedef RAIL_IdleMode_t sl_rail_idle_mode_t;
typedef RAIL_InitCompleteCallbackPtr_t sl_rail_init_complete_callback_t;
typedef RAIL_MultiTimerCallback_t sl_rail_multi_timer_callback_t;
typedef RAIL_MultiTimer_t sl_rail_multi_timer_t;
typedef RAIL_PtiProtocol_t sl_rail_pti_protocol_t;
typedef RAIL_RadioState_t sl_rail_radio_state_t;
typedef RAIL_RxOptions_t sl_rail_rx_options_t;
typedef RAIL_RxPacketHandle_t sl_rail_rx_packet_handle_t;
typedef RAIL_SchedulerStatus_t sl_rail_scheduler_status_t;
typedef RAIL_Status_t sl_rail_status_t;
typedef RAIL_StreamMode_t sl_rail_stream_mode_t;
typedef RAIL_TimeMode_t sl_rail_time_mode_t;
typedef RAIL_TimerSyncConfig_t sl_rail_timer_sync_config_t;
typedef RAIL_TransitionTime_t sl_rail_transition_time_t;
typedef RAIL_TxOptions_t sl_rail_tx_options_t;
typedef RAIL_TxPowerMode_t sl_rail_tx_pa_mode_t;
typedef RAIL_TxPower_t sl_rail_tx_power_t;
typedef uint32_t sl_rail_fifo_buffer_align_t;
typedef uint32_t sl_rail_time_t;
typedef uint64_t sl_rail_packet_queue_entry_t;

typedef struct sl_rail_rx_packet_info {
	RAIL_RxPacketStatus_t packet_status;
	uint16_t packet_bytes;
	uint16_t first_portion_bytes;
	uint8_t *p_first_portion_data;
	uint8_t *p_last_portion_data;
	RAIL_AddrFilterMask_t filter_mask;
} sl_rail_rx_packet_info_t;
BUILD_ASSERT(sizeof(sl_rail_rx_packet_info_t) == sizeof(RAIL_RxPacketInfo_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, packet_status, RAIL_RxPacketInfo_t,
			 packetStatus);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, packet_bytes, RAIL_RxPacketInfo_t, packetBytes);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, first_portion_bytes, RAIL_RxPacketInfo_t,
			 firstPortionBytes);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, p_first_portion_data, RAIL_RxPacketInfo_t,
			 firstPortionData);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, p_last_portion_data, RAIL_RxPacketInfo_t,
			 lastPortionData);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_info_t, filter_mask, RAIL_RxPacketInfo_t, filterMask);

typedef struct sl_rail_packet_time_stamp {
	RAIL_Time_t packet_time;
	uint16_t total_packet_bytes;
	RAIL_PacketTimePosition_t time_position;
	RAIL_Time_t packet_duration_us;
} sl_rail_packet_time_stamp_t;
BUILD_ASSERT(sizeof(sl_rail_packet_time_stamp_t) == sizeof(RAIL_PacketTimeStamp_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_packet_time_stamp_t, packet_time, RAIL_PacketTimeStamp_t,
			 packetTime);
ASSERT_FIELDS_COMPATIBLE(sl_rail_packet_time_stamp_t, total_packet_bytes, RAIL_PacketTimeStamp_t,
			 totalPacketBytes);
ASSERT_FIELDS_COMPATIBLE(sl_rail_packet_time_stamp_t, time_position, RAIL_PacketTimeStamp_t,
			 timePosition);
ASSERT_FIELDS_COMPATIBLE(sl_rail_packet_time_stamp_t, packet_duration_us, RAIL_PacketTimeStamp_t,
			 packetDurationUs);

typedef struct sl_rail_rx_packet_details {
	sl_rail_packet_time_stamp_t time_received;
	bool crc_passed;
	bool is_ack;
	int8_t rssi_dbm;
	uint8_t lqi;
	uint8_t sync_word_id;
	uint8_t sub_phy_id;
	uint8_t antenna_id;
	uint8_t channel_hopping_channel_index;
	uint16_t channel;
} sl_rail_rx_packet_details_t;
BUILD_ASSERT(sizeof(sl_rail_rx_packet_details_t) == sizeof(RAIL_RxPacketDetails_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, time_received, RAIL_RxPacketDetails_t,
			 timeReceived);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, crc_passed, RAIL_RxPacketDetails_t,
			 crcPassed);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, is_ack, RAIL_RxPacketDetails_t, isAck);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, rssi_dbm, RAIL_RxPacketDetails_t, rssi);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, lqi, RAIL_RxPacketDetails_t, lqi);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, sync_word_id, RAIL_RxPacketDetails_t,
			 syncWordId);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, sub_phy_id, RAIL_RxPacketDetails_t, subPhyId);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, antenna_id, RAIL_RxPacketDetails_t,
			 antennaId);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, channel_hopping_channel_index,
			 RAIL_RxPacketDetails_t, channelHoppingChannelIndex);
ASSERT_FIELDS_COMPATIBLE(sl_rail_rx_packet_details_t, channel, RAIL_RxPacketDetails_t, channel);

typedef struct sl_rail_scheduler_info {
	uint8_t priority;
	RAIL_Time_t slip_time;
	RAIL_Time_t transaction_time;
} sl_rail_scheduler_info_t;
BUILD_ASSERT(sizeof(sl_rail_scheduler_info_t) == sizeof(RAIL_SchedulerInfo_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_scheduler_info_t, priority, RAIL_SchedulerInfo_t, priority);
ASSERT_FIELDS_COMPATIBLE(sl_rail_scheduler_info_t, slip_time, RAIL_SchedulerInfo_t, slipTime);
ASSERT_FIELDS_COMPATIBLE(sl_rail_scheduler_info_t, transaction_time, RAIL_SchedulerInfo_t,
			 transactionTime);

typedef struct sl_rail_auto_ack_config {
	bool enable;
	uint16_t ack_timeout_us;
	RAIL_StateTransitions_t rx_transitions;
	RAIL_StateTransitions_t tx_transitions;
} sl_rail_auto_ack_config_t;
BUILD_ASSERT(sizeof(sl_rail_auto_ack_config_t) == sizeof(RAIL_AutoAckConfig_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_auto_ack_config_t, enable, RAIL_AutoAckConfig_t, enable);
ASSERT_FIELDS_COMPATIBLE(sl_rail_auto_ack_config_t, ack_timeout_us, RAIL_AutoAckConfig_t,
			 ackTimeout);
ASSERT_FIELDS_COMPATIBLE(sl_rail_auto_ack_config_t, rx_transitions, RAIL_AutoAckConfig_t,
			 rxTransitions);
ASSERT_FIELDS_COMPATIBLE(sl_rail_auto_ack_config_t, tx_transitions, RAIL_AutoAckConfig_t,
			 txTransitions);

typedef struct sl_rail_state_timing {
	RAIL_TransitionTime_t idle_to_rx;
	RAIL_TransitionTime_t tx_to_rx;
	RAIL_TransitionTime_t idle_to_tx;
	RAIL_TransitionTime_t rx_to_tx;
	RAIL_TransitionTime_t rxsearch_timeout;
	RAIL_TransitionTime_t tx_to_rxsearch_timeout;
	RAIL_TransitionTime_t tx_to_tx;
} sl_rail_state_timing_t;
BUILD_ASSERT(sizeof(sl_rail_state_timing_t) == sizeof(RAIL_StateTiming_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, idle_to_rx, RAIL_StateTiming_t, idleToRx);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, tx_to_rx, RAIL_StateTiming_t, txToRx);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, idle_to_tx, RAIL_StateTiming_t, idleToTx);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, rx_to_tx, RAIL_StateTiming_t, rxToTx);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, rxsearch_timeout, RAIL_StateTiming_t,
			 rxSearchTimeout);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, tx_to_rxsearch_timeout, RAIL_StateTiming_t,
			 txToRxSearchTimeout);
ASSERT_FIELDS_COMPATIBLE(sl_rail_state_timing_t, tx_to_tx, RAIL_StateTiming_t, txToTx);

typedef struct sl_rail_csma_config {
	uint8_t csma_min_bo_exp;
	uint8_t csma_max_bo_exp;
	uint8_t csma_tries;
	int8_t cca_threshold_dbm;
	uint16_t cca_backoff;
	uint16_t cca_duration;
	RAIL_Time_t csma_timeout;
} sl_rail_csma_config_t;
BUILD_ASSERT(sizeof(sl_rail_csma_config_t) == sizeof(RAIL_CsmaConfig_t));
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, csma_min_bo_exp, RAIL_CsmaConfig_t, csmaMinBoExp);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, csma_max_bo_exp, RAIL_CsmaConfig_t, csmaMaxBoExp);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, csma_tries, RAIL_CsmaConfig_t, csmaTries);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, cca_threshold_dbm, RAIL_CsmaConfig_t, ccaThreshold);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, cca_backoff, RAIL_CsmaConfig_t, ccaBackoff);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, cca_duration, RAIL_CsmaConfig_t, ccaDuration);
ASSERT_FIELDS_COMPATIBLE(sl_rail_csma_config_t, csma_timeout, RAIL_CsmaConfig_t, csmaTimeout);

typedef struct sl_rail_config {
	void (*events_callback)(RAIL_Handle_t railHandle, RAIL_Events_t events);
	void *p_opaque_handle_0;
	void *p_opaque_handle_1;

	uint16_t rx_packet_queue_entries;
	uint16_t rx_fifo_bytes;
	uint16_t tx_fifo_bytes;

	sl_rail_packet_queue_entry_t *p_rx_packet_queue;
	sl_rail_fifo_buffer_align_t *p_rx_fifo_buffer;
	sl_rail_fifo_buffer_align_t *p_tx_fifo_buffer;
} sl_rail_config_t;

static const uint16_t sl_rail_builtin_rx_packet_queue_entries = 0;
static sl_rail_packet_queue_entry_t *const sl_rail_builtin_rx_packet_queue_ptr = NULL;

static const uint16_t sl_rail_builtin_rx_fifo_bytes = 0;
static sl_rail_fifo_buffer_align_t *const sl_rail_builtin_rx_fifo_ptr = NULL;

extern sl_rail_config_t current_rail_config;

static inline sl_rail_radio_state_t sl_rail_get_radio_state(sl_rail_handle_t rail_handle)
{
	return RAIL_GetRadioState(rail_handle);
}

static inline sl_rail_time_t sl_rail_get_time(sl_rail_handle_t rail_handle)
{
	ARG_UNUSED(rail_handle);
	return RAIL_GetTime();
}

static inline sl_rail_status_t sl_rail_init_power_manager(void)
{
	return RAIL_InitPowerManager();
}

static inline sl_rail_status_t sl_rail_config_cal(sl_rail_handle_t rail_handle,
						  sl_rail_cal_mask_t cal_enable_mask)
{
	return RAIL_ConfigCal(rail_handle, cal_enable_mask);
}

static inline sl_rail_status_t sl_rail_set_pti_protocol(sl_rail_handle_t rail_handle,
							sl_rail_pti_protocol_t protocol)
{
	return RAIL_SetPtiProtocol(rail_handle, protocol);
}

static inline sl_rail_status_t sl_rail_config_multi_timer(sl_rail_handle_t rail_handle, bool enable)
{
	ARG_UNUSED(rail_handle);
	return RAIL_ConfigMultiTimer(enable);
}

/* NOTE: unlike the original, p_timer_sync_config is not const. */
static inline sl_rail_status_t
sl_rail_config_sleep(sl_rail_handle_t rail_handle, sl_rail_timer_sync_config_t *p_timer_sync_config)
{
	return RAIL_ConfigSleepAlt(rail_handle, p_timer_sync_config);
}

static inline sl_rail_status_t sl_rail_config_rx_options(sl_rail_handle_t rail_handle,
							 sl_rail_rx_options_t rx_options_mask,
							 sl_rail_rx_options_t rx_options)
{
	return RAIL_ConfigRxOptions(rail_handle, rx_options_mask, rx_options);
}

static inline uint32_t sl_rail_get_symbol_rate(sl_rail_handle_t rail_handle)
{
	return RAIL_GetSymbolRate(rail_handle);
}

static inline uint16_t sl_rail_write_tx_fifo(sl_rail_handle_t rail_handle, const uint8_t *p_data,
					     uint16_t write_bytes, bool reset)
{
	return RAIL_WriteTxFifo(rail_handle, p_data, write_bytes, reset);
}

static inline uint32_t sl_rail_get_bit_rate(sl_rail_handle_t rail_handle)
{
	return RAIL_GetBitRate(rail_handle);
}

static inline sl_rail_status_t sl_rail_cancel_multi_timer(sl_rail_handle_t rail_handle,
							  sl_rail_multi_timer_t *p_tmr)
{
	ARG_UNUSED(rail_handle);
	return RAIL_CancelMultiTimer(p_tmr);
}

static inline sl_rail_status_t
sl_rail_set_multi_timer(sl_rail_handle_t rail_handle, sl_rail_multi_timer_t *p_tmr,
			sl_rail_time_t expiration_time, sl_rail_time_mode_t expiration_mode,
			sl_rail_multi_timer_callback_t expiration_callback, void *cb_arg)
{
	ARG_UNUSED(rail_handle);
	return RAIL_SetMultiTimer(p_tmr, expiration_time, expiration_mode, expiration_callback,
				  cb_arg);
}

static inline sl_rail_status_t sl_rail_set_tx_power_dbm(sl_rail_handle_t rail_handle,
							sl_rail_tx_power_t power_ddbm)
{
	return RAIL_SetTxPowerDbm(rail_handle, power_ddbm);
}

static inline int16_t sl_rail_get_rssi(sl_rail_handle_t rail_handle, sl_rail_time_t wait_timeout_us)
{
	return RAIL_GetRssiAlt(rail_handle, wait_timeout_us);
}

static inline sl_rail_status_t sl_rail_config_events(sl_rail_handle_t rail_handle,
						     sl_rail_events_t mask, sl_rail_events_t events)
{
	return RAIL_ConfigEvents(rail_handle, mask, events);
}

static inline sl_rail_status_t sl_rail_start_tx_stream(sl_rail_handle_t rail_handle,
						       uint16_t channel, sl_rail_stream_mode_t mode,
						       sl_rail_tx_options_t tx_options)
{
	return RAIL_StartTxStreamAlt(rail_handle, channel, mode, tx_options);
}

static inline sl_rail_status_t
sl_rail_start_cca_csma_tx(sl_rail_handle_t rail_handle, uint16_t channel,
			  sl_rail_tx_options_t tx_options,
			  const sl_rail_csma_config_t *p_csma_config,
			  const sl_rail_scheduler_info_t *p_scheduler_info)
{
	return RAIL_StartCcaCsmaTx(rail_handle, channel, tx_options,
				   CONST_PCAST(RAIL_CsmaConfig_t, p_csma_config),
				   CONST_PCAST(RAIL_SchedulerInfo_t, p_scheduler_info));
}

static inline sl_rail_status_t sl_rail_start_rx(sl_rail_handle_t rail_handle, uint16_t channel,
						const sl_rail_scheduler_info_t *p_scheduler_info)
{
	return RAIL_StartRx(rail_handle, channel,
			    CONST_PCAST(RAIL_SchedulerInfo_t, p_scheduler_info));
}

static inline sl_rail_status_t sl_rail_start_tx(sl_rail_handle_t rail_handle, uint16_t channel,
						sl_rail_tx_options_t tx_options,
						const sl_rail_scheduler_info_t *p_scheduler_info)
{
	return RAIL_StartTx(rail_handle, channel, tx_options,
			    CONST_PCAST(RAIL_SchedulerInfo_t, p_scheduler_info));
}

static inline sl_rail_status_t
sl_rail_get_rx_time_sync_word_end(sl_rail_handle_t rail_handle,
				  sl_rail_rx_packet_details_t *p_packet_details)
{
	return RAIL_GetRxTimeSyncWordEndAlt(rail_handle,
					    PCAST(RAIL_RxPacketDetails_t, p_packet_details));
}

static inline sl_rail_status_t
sl_rail_get_rx_packet_details(sl_rail_handle_t rail_handle,
			      sl_rail_rx_packet_handle_t packet_handle,
			      sl_rail_rx_packet_details_t *p_packet_details)
{
	return RAIL_GetRxPacketDetailsAlt(rail_handle, packet_handle,
					  PCAST(RAIL_RxPacketDetails_t, p_packet_details));
}

static inline sl_rail_rx_packet_handle_t
sl_rail_get_rx_packet_info(sl_rail_handle_t rail_handle, sl_rail_rx_packet_handle_t packet_handle,
			   sl_rail_rx_packet_info_t *p_packet_info)
{
	return RAIL_GetRxPacketInfo(rail_handle, packet_handle,
				    PCAST(RAIL_RxPacketInfo_t, p_packet_info));
}

static inline sl_rail_status_t sl_rail_yield_radio(sl_rail_handle_t rail_handle)
{
	RAIL_YieldRadio(rail_handle);
	return RAIL_STATUS_NO_ERROR;
}

static inline sl_rail_status_t sl_rail_idle(sl_rail_handle_t rail_handle, sl_rail_idle_mode_t mode,
					    bool wait)
{
	RAIL_Idle(rail_handle, mode, wait);
	return RAIL_STATUS_NO_ERROR;
}

static inline sl_rail_status_t
sl_rail_get_rx_incoming_packet_info(sl_rail_handle_t rail_handle,
				    sl_rail_rx_packet_info_t *p_packet_info)
{
	RAIL_GetRxIncomingPacketInfo(rail_handle, (void *)p_packet_info);
	return RAIL_STATUS_NO_ERROR;
}

static inline sl_rail_status_t sl_rail_copy_rx_packet(sl_rail_handle_t rail_handle, uint8_t *p_dest,
						      const sl_rail_rx_packet_info_t *p_packet_info)
{
	RAIL_CopyRxPacket(p_dest, (const void *)p_packet_info);
	return RAIL_STATUS_NO_ERROR;
}

static inline sl_rail_status_t
sl_rail_get_scheduler_status(sl_rail_handle_t rail_handle,
			     sl_rail_scheduler_status_t *p_scheduler_status,
			     sl_rail_status_t *p_rail_status)
{
	*p_scheduler_status = RAIL_GetSchedulerStatus(rail_handle);
	*p_rail_status = RAIL_STATUS_NO_ERROR;
	return RAIL_STATUS_NO_ERROR;
}

static inline sl_rail_status_t sl_rail_init(sl_rail_handle_t *p_rail_handle,
					    sl_rail_config_t *p_rail_config,
					    sl_rail_init_complete_callback_t init_complete_callback)
{
	RAIL_Config_t rail_config = {
		.eventsCallback = p_rail_config->events_callback,
	};
	uint16_t tx_fifo_bytes;

	if (p_rail_config->p_rx_fifo_buffer != NULL) {
		return RAIL_STATUS_INVALID_PARAMETER;
	}
	if (p_rail_config->p_rx_packet_queue != NULL) {
		return RAIL_STATUS_INVALID_PARAMETER;
	}

	*p_rail_handle = RAIL_Init(&rail_config, init_complete_callback);
	if ((*p_rail_handle) == NULL) {
		return RAIL_STATUS_INVALID_PARAMETER;
	}
	current_rail_config = *p_rail_config;

	tx_fifo_bytes = RAIL_SetTxFifo(*p_rail_handle, (void *)p_rail_config->p_tx_fifo_buffer, 0,
				       p_rail_config->tx_fifo_bytes);
	if (tx_fifo_bytes != p_rail_config->tx_fifo_bytes) {
		return RAIL_STATUS_INVALID_PARAMETER;
	}

	return RAIL_STATUS_NO_ERROR;
}

static inline const sl_rail_config_t *sl_rail_get_config(sl_rail_handle_t rail_handle)
{
	return &current_rail_config;
}

#endif /* ZEPHYR_MODULES_HAL_SILABS_GECKO_IEEE802154_SL_RAIL_H_ */
