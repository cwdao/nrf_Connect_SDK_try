/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/direction.h>

#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/net_buf.h>
#include <string.h>

#define DF_FEAT_ENABLED BIT64(BT_LE_FEAT_BIT_CONN_CTE_RESP)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_LE_SUPPORTED_FEATURES, BT_LE_SUPP_FEAT_24_ENCODE(DF_FEAT_ENABLED)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Latency set to zero, to enforce PDU exchange every connection event */
#define CONN_LATENCY 0U
/* Interval used to run CTE request procedure periodically.
 * Value is a number of connection events.
 */
#define CTE_REQ_INTERVAL (CONN_LATENCY + 10U)
/* Length of CTE in unit of 8 us */
#define CTE_LEN (0x14U)

#if defined(CONFIG_BT_DF_CTE_TX_AOD)
/* Example sequence of antenna switch patterns for antenna matrix designed by
 * Nordic. For more information about antenna switch patterns see README.rst.
 */
static uint8_t ant_patterns[] = {0x2, 0x0, 0x5, 0x6, 0x1, 0x4, 0xC, 0x9, 0xE,
				 0xD, 0x8, 0xA};
#endif /* CONFIG_BT_DF_CTE_TX_AOD */

static void print_conn_tx_power(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_tx_power p = {0};

    /* phy=0 => 走 BT_HCI_OP_READ_TX_POWER_LEVEL，返回 current/max */
    p.phy = 0;

    err = bt_conn_le_get_tx_power_level(conn, &p);
    if (err) {
        printk("Read TX power failed (err %d)\n", err);
        return;
    }

    printk("TX power (conn): current=%d dBm, max=%d dBm\n",
           p.current_level, p.max_level);
}

//设置发射功率
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		printk("Set Tx power err: %d\n", err);
		return;
	}

	rp = (void *)rsp->data;
	printk("Actual Tx Power: %d\n", rp->selected_tx_power);

	net_buf_unref(rsp);
}


static void enable_cte_response(struct bt_conn *conn)
{
	int err;

	const struct bt_df_conn_cte_tx_param cte_tx_params = {
#if defined(CONFIG_BT_DF_CTE_TX_AOD)
		.cte_types = BT_DF_CTE_TYPE_ALL,
		.num_ant_ids = ARRAY_SIZE(ant_patterns),
		.ant_ids = ant_patterns,
#else
		.cte_types = BT_DF_CTE_TYPE_AOA,
#endif /* CONFIG_BT_DF_CTE_TX_AOD */
	};

	printk("Set CTE transmission params...");
	err = bt_df_set_conn_cte_tx_param(conn, &cte_tx_params);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("success.\n");

	printk("Set CTE response enable...");
	err = bt_df_conn_cte_rsp_enable(conn);
	if (err) {
		printk("failed (err %d).\n", err);
		return;
	}
	printk("success.\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
	/* 连接建立后设置本端（peripheral）连接 TX power，并打印控制器选中的值 */
	// set_and_print_conn_tx_power(conn, 8);
	set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, 1, 8);
	print_conn_tx_power(conn);

		enable_cte_response(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

static void le_param_updated_cb(struct bt_conn *conn, uint16_t interval,
	uint16_t latency, uint16_t timeout) {
printk("LE param updated: interval %u *1.25(ms) latency %u timeout %u ("
"ms)\n",
interval, latency, timeout);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated_cb,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, 1, 8);

	bt_ready();

	return 0;
}
