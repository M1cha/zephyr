/*
 * Copyright (c) 2025 sevenlab engineering GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BT_CONN) && defined(CONFIG_SILABS_GECKO_POWER_MANAGER)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <sl_power_manager.h>

static bool em_acquired;

static void has_connections_cb(struct bt_conn *conn, void *result_)
{
	bool *result = result_;
	int ret;
	struct bt_conn_info info;

	ret = bt_conn_get_info(conn, &info);
	if (ret != 0) {
		return;
	}

	if (info.state == BT_CONN_STATE_CONNECTED) {
		*result = true;
	}
}

static bool has_connections(void)
{
	bool result = false;

	bt_conn_foreach(BT_CONN_TYPE_LE, has_connections_cb, &result);
	return result;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (!em_acquired && has_connections()) {
		sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
		em_acquired = true;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (em_acquired && !has_connections()) {
		sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
		em_acquired = false;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};
#endif
