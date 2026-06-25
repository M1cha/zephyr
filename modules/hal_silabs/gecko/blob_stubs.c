#include <rail.h>

#ifdef CONFIG_BT_SILABS_EFR32
#include <sl_bluetooth_controller.h>
#include <sl_btctrl_hci.h>
#include <sl_btctrl_linklayer.h>
#include <sl_bt_ll_config.h>
#endif

#ifdef CONFIG_IEEE802154_SILABS_EFR32
#include <rail_ieee802154.h>
#endif

bool RAIL_CancelMultiTimer(RAIL_MultiTimer_t *tmr)
{
	return true;
}

RAIL_Status_t RAIL_Calibrate(RAIL_Handle_t railHandle, RAIL_CalValues_t *calValues,
			     RAIL_CalMask_t calForce)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_ConfigCal(RAIL_Handle_t railHandle, RAIL_CalMask_t calEnable)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_ConfigEvents(RAIL_Handle_t railHandle, RAIL_Events_t mask, RAIL_Events_t events)
{
	return RAIL_STATUS_NO_ERROR;
}

bool RAIL_ConfigMultiTimer(bool enable)
{
	return true;
}

RAIL_Status_t RAIL_ConfigRxOptions(RAIL_Handle_t railHandle, RAIL_RxOptions_t mask,
				   RAIL_RxOptions_t options)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_ConfigTxPower(RAIL_Handle_t railHandle, const RAIL_TxPowerConfig_t *config)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_ConfigSleep(RAIL_Handle_t railHandle, RAIL_SleepConfig_t sleepConfig)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_ConfigSleepAlt(RAIL_Handle_t railHandle, RAIL_TimerSyncConfig_t *syncConfig)
{
	return RAIL_STATUS_NO_ERROR;
}

void RAIL_EnablePaCal(bool enable)
{
}

RAIL_RadioState_t RAIL_GetRadioState(RAIL_Handle_t railHandle)
{
	return RAIL_RF_STATE_INACTIVE;
}

int16_t RAIL_GetRssiAlt(RAIL_Handle_t railHandle, RAIL_Time_t waitTimeout)
{
	return 0;
}

void RAIL_GetRxIncomingPacketInfo(RAIL_Handle_t railHandle, RAIL_RxPacketInfo_t *pPacketInfo)
{
}

RAIL_Status_t RAIL_GetRxPacketDetailsAlt(RAIL_Handle_t railHandle,
					 RAIL_RxPacketHandle_t packetHandle,
					 RAIL_RxPacketDetails_t *pPacketDetails)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_RxPacketHandle_t RAIL_GetRxPacketInfo(RAIL_Handle_t railHandle,
					   RAIL_RxPacketHandle_t packetHandle,
					   RAIL_RxPacketInfo_t *pPacketInfo)
{
	return NULL;
}

RAIL_SchedulerStatus_t RAIL_GetSchedulerStatus(RAIL_Handle_t railHandle)
{
	return RAIL_SCHEDULER_STATUS_NO_ERROR;
}

uint32_t RAIL_GetSymbolRate(RAIL_Handle_t railHandle)
{
	return 0;
}

RAIL_Status_t RAIL_GetRxTimeSyncWordEndAlt(RAIL_Handle_t railHandle,
					   RAIL_RxPacketDetails_t *pPacketDetails)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Time_t RAIL_GetTime(void)
{
	return 0;
}

void RAIL_Idle(RAIL_Handle_t railHandle, RAIL_IdleMode_t mode, bool wait)
{
}

RAIL_Handle_t RAIL_Init(RAIL_Config_t *railCfg, RAIL_InitCompleteCallbackPtr_t cb)
{
	return NULL;
}

RAIL_Status_t RAIL_InitPowerManager(void)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_SetMultiTimer(RAIL_MultiTimer_t *tmr, RAIL_Time_t expirationTime,
				 RAIL_TimeMode_t expirationMode, RAIL_MultiTimerCallback_t callback,
				 void *cbArg)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_SetPtiProtocol(RAIL_Handle_t railHandle, RAIL_PtiProtocol_t protocol)
{
	return RAIL_STATUS_NO_ERROR;
}

uint16_t RAIL_SetTxFifo(RAIL_Handle_t railHandle, uint8_t *addr, uint16_t initLength, uint16_t size)
{
	return 0;
}

RAIL_Status_t RAIL_SetTxPowerDbm(RAIL_Handle_t railHandle, RAIL_TxPower_t power)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_StartCcaCsmaTx(RAIL_Handle_t railHandle, uint16_t channel,
				  RAIL_TxOptions_t options, const RAIL_CsmaConfig_t *csmaConfig,
				  const RAIL_SchedulerInfo_t *schedulerInfo)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_StartRx(RAIL_Handle_t railHandle, uint16_t channel,
			   const RAIL_SchedulerInfo_t *schedulerInfo)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_StartTx(RAIL_Handle_t railHandle, uint16_t channel, RAIL_TxOptions_t options,
			   const RAIL_SchedulerInfo_t *schedulerInfo)
{
	return RAIL_STATUS_NO_ERROR;
}

void RAIL_VerifyTxPowerCurves(const struct RAIL_TxPowerCurvesConfigAlt *config)
{
}

uint16_t RAIL_WriteTxFifo(RAIL_Handle_t railHandle, const uint8_t *dataPtr, uint16_t writeLength,
			  bool reset)
{
	return 0;
}

void RAIL_YieldRadio(RAIL_Handle_t railHandle)
{
}

void AGC_IRQHandler(void)
{
}

void BUFC_IRQHandler(void)
{
}

void FRC_IRQHandler(void)
{
}

void FRC_PRI_IRQHandler(void)
{
}

void MODEM_IRQHandler(void)
{
}

void PRORTC_IRQHandler(void)
{
}

void PROTIMER_IRQHandler(void)
{
}

void RAC_RSM_IRQHandler(void)
{
}

void RAC_SEQ_IRQHandler(void)
{
}

void RFSENSE_IRQHandler(void)
{
}

void SYNTH_IRQHandler(void)
{
}

#ifdef CONFIG_BT_SILABS_EFR32
void BTLE_LL_Process(uint32_t events)
{
}

int16_t BTLE_LL_SetMaxPower(int16_t power)
{
	return 0;
}

sl_status_t ll_connPowerControlEnable(const sl_bt_ll_power_control_config_t *)
{
	return SL_STATUS_OK;
}

sl_status_t ll_initDefaultPowerLevelRange(int16_t minPower, int16_t maxPower)
{
	return SL_STATUS_OK;
}

sl_status_t sl_btctrl_alloc_periodic_adv(uint8_t num_adv)
{
	return SL_STATUS_OK;
}

sl_status_t sl_btctrl_alloc_periodic_scan(uint8_t num_scan)
{
	return SL_STATUS_OK;
}

void sl_btctrl_configure_completed_packets_reporting(uint8_t packets, uint8_t events)
{
}

void sl_btctrl_configure_le_buffer_size(uint8_t count)
{
}

void sl_btctrl_configure_max_queued_adv_reports(uint8_t num_reports)
{
}

sl_status_t sl_btctrl_config_adv(struct sl_btctrl_adv_config *adv_config)
{
	return SL_STATUS_OK;
}

void sli_btctrl_deinit_mem(void)
{
}

void sl_btctrl_disable_coded_phy(void)
{
}

sl_status_t sl_btctrl_init_afh(uint32_t flags)
{
	return SL_STATUS_OK;
}

void sl_btctrl_init_multiprotocol(void)
{
}

void sl_btctrl_init_past_local_sync_transfer(void)
{
}

void sl_btctrl_init_past_receiver(void)
{
}

void sl_btctrl_init_past_remote_sync_transfer(void)
{
}

int16_t sl_btctrl_hci_receive(uint8_t *data, int16_t len, bool lastFragment)
{
	return 0;
}

bool sl_btctrl_is_initialized(void)
{
	return false;
}

void sl_btctrl_init_adv(void)
{
}

void sl_btctrl_init_adv_ext(void)
{
}

sl_status_t sl_btctrl_allocate_resolving_list_memory(uint8_t resolvingListSize)
{
	return SL_STATUS_OK;
}

void sl_btctrl_hci_parser_init_adv(void)
{
}

void sl_btctrl_hci_parser_init_conn(void)
{
}

void sl_btctrl_hci_parser_init_default(void)
{
}

void sl_btctrl_hci_parser_init_phy(void)
{
}

void sl_btctrl_hci_parser_init_past(void)
{
}

void sl_btctrl_hci_parser_init_privacy(void)
{
}

sl_status_t sl_btctrl_init_basic(uint8_t connections, uint8_t adv_sets, uint8_t whitelist)
{
	return SL_STATUS_OK;
}

void sl_btctrl_init_conn(void)
{
}

void sl_btctrl_init_highpower(void)
{
}

sl_status_t sl_btctrl_init_ll(void)
{
	return SL_STATUS_OK;
}

uint32_t sl_btctrl_init_mem(uint32_t memsize)
{
	return 0;
}

void sl_btctrl_init_phy(void)
{
}

void sl_btctrl_init_privacy(void)
{
}

void sl_btctrl_init_scan(void)
{
}

void sl_btctrl_init_scan_ext(void)
{
}

sl_status_t
sl_btctrl_pawr_advertiser_configure(struct sl_btctrl_pawr_advertiser_config *pawr_adv_config)
{
	return SL_STATUS_OK;
}

sl_status_t
sl_btctrl_pawr_synchronizer_configure(struct sl_btctrl_pawr_synchronizer_config *pawr_sync_config)
{
	return SL_STATUS_OK;
}

void sl_bthci_init_upper(void)
{
}

void sl_bthci_init_vs(void)
{
}

sl_status_t sl_bt_ll_deinit(void)
{
	return SL_STATUS_OK;
}
#endif

#ifdef CONFIG_IEEE802154_SILABS_EFR32
RAIL_Status_t RAIL_IEEE802154_Config2p4GHzRadio(RAIL_Handle_t railHandle)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_ConfigCcaMode(RAIL_Handle_t railHandle,
					    RAIL_IEEE802154_CcaMode_t ccaMode)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_GetAddress(RAIL_Handle_t railHandle,
					 RAIL_IEEE802154_Address_t *pAddress)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_Init(RAIL_Handle_t railHandle, const RAIL_IEEE802154_Config_t *config)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetFramePending(RAIL_Handle_t railHandle)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetLongAddress(RAIL_Handle_t railHandle, const uint8_t *longAddr,
					     uint8_t index)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetPanCoordinator(RAIL_Handle_t railHandle, bool isPanCoordinator)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetPanId(RAIL_Handle_t railHandle, uint16_t panId, uint8_t index)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetPromiscuousMode(RAIL_Handle_t railHandle, bool enable)
{
	return RAIL_STATUS_NO_ERROR;
}

RAIL_Status_t RAIL_IEEE802154_SetShortAddress(RAIL_Handle_t railHandle, uint16_t shortAddr,
					      uint8_t index)
{
	return RAIL_STATUS_NO_ERROR;
}

void sl_openthread_init(void)
{
}
#endif
