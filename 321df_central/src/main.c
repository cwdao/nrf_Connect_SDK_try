/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/direction.h>
#include <zephyr/bluetooth/hci.h>

// for cte pkt channel map
// #include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/net_buf.h>

/* Latency set to zero, to enforce PDU exchange every connection event */
#define CONN_LATENCY 0U
/* Arbitrary selected timeout value */
#define CONN_TIMEOUT 400U
/* Inteval used to run CTE request procedure periodically.
 * Value is a number of connection events.
 */
// #define CTE_REQ_INTERVAL (CONN_LATENCY + 10U)
#define CTE_REQ_INTERVAL 1U
/* Length of CTE in unit of 8 us */
#define CTE_LEN (0x02U)

#define DF_FEAT_ENABLED BIT64(BT_LE_FEAT_BIT_CONN_CTE_RESP)

#define CONN_INT_MIN 0x08 /* 20*1.25=25ms */
#define CONN_INT_MAX 0x08 /* 20*1.25=25ms */

static struct bt_conn *default_conn;
static const struct bt_le_conn_param conn_params = BT_LE_CONN_PARAM_INIT(
    CONN_INT_MIN, CONN_INT_MAX, CONN_LATENCY, CONN_TIMEOUT);
// cte channel map
static uint8_t my_channels[] = {3};

#if defined(CONFIG_BT_DF_CTE_RX_AOA)
/* Example sequence of antenna switch patterns for antenna matrix designed by
 * Nordic. For more information about antenna switch patterns see README.rst.
 */
static const uint8_t ant_patterns[] = {0x2, 0x0, 0x5, 0x6, 0x1, 0x4,
                                       0xC, 0x9, 0xE, 0xD, 0x8, 0xA};
#endif /* CONFIG_BT_DF_CTE_RX_AOA */

static void start_scan(void);

static const char *cte_type2str(uint8_t type) {
  switch (type) {
  case BT_DF_CTE_TYPE_AOA:
    return "AOA";
  case BT_DF_CTE_TYPE_AOD_1US:
    return "AOD 1 [us]";
  case BT_DF_CTE_TYPE_AOD_2US:
    return "AOD 2 [us]";
  default:
    return "Unknown";
  }
}

static const char *packet_status2str(uint8_t status) {
  switch (status) {
  case BT_DF_CTE_CRC_OK:
    return "CRC OK";
  case BT_DF_CTE_CRC_ERR_CTE_BASED_TIME:
    return "CRC not OK, CTE Info OK";
  case BT_DF_CTE_CRC_ERR_CTE_BASED_OTHER:
    return "CRC not OK, Sampled other way";
  case BT_DF_CTE_INSUFFICIENT_RESOURCES:
    return "No resources";
  default:
    return "Unknown";
  }
}

static bool eir_found(struct bt_data *data, void *user_data) {
  bt_addr_le_t *addr = user_data;
  uint64_t u64 = 0U;
  int err;

  // printk("[AD]: %u data_len %u\n", data->type, data->data_len);

  switch (data->type) {
  case BT_DATA_LE_SUPPORTED_FEATURES:
    if (data->data_len > sizeof(u64)) {
      return true;
    }

    (void)memcpy(&u64, data->data, data->data_len);

    u64 = sys_le64_to_cpu(u64);

    if (!(u64 & DF_FEAT_ENABLED)) {
      return true;
    }

    err = bt_le_scan_stop();
    if (err) {
      printk("Stop LE scan failed (err %d)\n", err);
      return true;
    }

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_params,
                            &default_conn);
    if (err) {
      printk("Create conn failed (err %d)\n", err);
      start_scan();
    }
    return false;
  }

  return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad) {
  char dev[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(addr, dev, sizeof(dev));
  // printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n", dev,
  // type,
  //        ad->len, rssi);

  /* We're only interested in connectable events */
  if (type == BT_GAP_ADV_TYPE_ADV_IND ||
      type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
    bt_data_parse(ad, eir_found, (void *)addr);
  }
}

static void enable_cte_reqest(void) {
  int err;

  const struct bt_df_conn_cte_rx_param cte_rx_params = {
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
      .cte_types = BT_DF_CTE_TYPE_ALL,
      .slot_durations = BT_DF_ANTENNA_SWITCHING_SLOT_1US,
      .num_ant_ids = ARRAY_SIZE(ant_patterns),
      .ant_ids = ant_patterns,
#else
      .cte_types = BT_DF_CTE_TYPE_AOD_1US | BT_DF_CTE_TYPE_AOD_2US,
#endif /* CONFIG_BT_DF_CTE_RX_AOA */
  };

  const struct bt_df_conn_cte_req_params cte_req_params = {
      .interval = CTE_REQ_INTERVAL,
      .cte_length = CTE_LEN,
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
      .cte_type = BT_DF_CTE_TYPE_AOA,
#else
      .cte_type = BT_DF_CTE_TYPE_AOD_2US,
#endif /* CONFIG_BT_DF_CTE_RX_AOA */
  };

  printk("Enable receiving of CTE...\n");
  err = bt_df_conn_cte_rx_enable(default_conn, &cte_rx_params);
  if (err) {
    printk("failed (err %d)\n", err);
    return;
  }
  printk("success. CTE receive enabled.\n");

  printk("Request CTE from peer device...\n");
  err = bt_df_conn_cte_req_enable(default_conn, &cte_req_params);
  if (err) {
    printk("failed (err %d)\n", err);
    return;
  }
  printk("success. CTE request enabled.\n");
}

static void start_scan(void) {
  int err;

  /* Use active scanning and disable duplicate filtering to handle any
   * devices that might update their advertising data at runtime.
   */
  struct bt_le_scan_param scan_param = {
      .type = BT_LE_SCAN_TYPE_ACTIVE,
      .options = BT_LE_SCAN_OPT_NONE,
      .interval = BT_GAP_SCAN_FAST_INTERVAL,
      .window = BT_GAP_SCAN_FAST_WINDOW,
  };

  err = bt_le_scan_start(&scan_param, device_found);
  if (err) {
    printk("Scanning failed to start (err %d)\n", err);
    return;
  }

  printk("Scanning successfully started\n");
}

// 单信道是不合规的，至少需要双信道才合规。但我们只启用一个也能跑，看起来sdc没有限制。这和ble
// cs的情况完全不同
static bool is_valid_data_channel(uint8_t ch) {
  /* Data channels are 0–36, advertising channels are 37–39 */
  return (ch <= 36);
}

int set_custom_channel_map_from_list(const uint8_t *channels,
                                     size_t channel_cnt) {
  struct net_buf *buf;
  struct bt_hci_cp_le_set_host_chan_classif *cp;
  int err;
  size_t i;
  uint8_t valid_cnt = 0;

  if (!channels || channel_cnt == 0) {
    printk("Channel list empty\n");
    return -EINVAL;
  }

  buf = bt_hci_cmd_alloc(K_FOREVER);
  if (!buf) {
    printk("No HCI buffer\n");
    return -ENOMEM;
  }

  cp = net_buf_add(buf, sizeof(*cp));

  /* Clear all channels first */
  memset(cp->ch_map, 0x00, sizeof(cp->ch_map));

  for (i = 0; i < channel_cnt; i++) {
    uint8_t ch = channels[i];

    /* Skip advertising channels and invalid values */
    if (!is_valid_data_channel(ch)) {
      printk("Skip invalid/adv channel %u\n", ch);
      continue;
    }

    cp->ch_map[ch / 8] |= BIT(ch % 8);
    valid_cnt++;
  }

  // if (valid_cnt < 2) {
  //     printk("Too few valid data channels (%u)\n", valid_cnt);
  //     return -EINVAL;
  // }

  err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_HOST_CHAN_CLASSIF, buf, NULL);
  if (err) {
    printk("Set channel map failed (err %d)\n", err);
    return err;
  }

  printk("Custom channel map applied (%u channels)\n", valid_cnt);
  return 0;
}

static void connected(struct bt_conn *conn, uint8_t conn_err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (conn_err) {
    printk("Failed to connect to %s, 0x%02x %s\n", addr, conn_err,
           bt_hci_err_to_str(conn_err));

    bt_conn_unref(default_conn);
    default_conn = NULL;

    start_scan();
    return;
  }

  printk("Connected: %s\n", addr);
  /* Apply custom channel map */
  set_custom_channel_map_from_list(my_channels, ARRAY_SIZE(my_channels));

  // 检查连接间隔参数
  int err;
  struct bt_conn_info info = {0};
  err = bt_conn_get_info(conn, &info);
  if (err) {
    printk("Failed to get connection info %d", err);
    return;
  }

  printk("Conn. interval is %u us", info.le.interval_us);

  if (conn == default_conn) {
    enable_cte_reqest();
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason,
         bt_hci_err_to_str(reason));

  if (default_conn != conn) {
    return;
  }

  bt_conn_unref(default_conn);
  default_conn = NULL;

  start_scan();
}

static void cte_recv_cb(struct bt_conn *conn,
                        struct bt_df_conn_iq_samples_report const *report) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (report->err == BT_DF_IQ_REPORT_ERR_SUCCESS) {
    // printk("CTE[%s]: ch %u, evt %u, samples %u, cte %s, slot %u [us], "
    //        "status %s, RSSI %d.%d dBm\n",
    //        addr, report->chan_idx,   /* Data Channel index */
    //        report->conn_evt_counter, /* Connection event counter */
    //        report->sample_count, cte_type2str(report->cte_type),
    //        report->slot_durations, packet_status2str(report->packet_status),
    //        report->rssi / 10);

    // printk("ch %u, evt %u, smp %u, slt %u [us]\n", report->chan_idx,
    //        report->conn_evt_counter, report->sample_count,
    //        report->slot_durations);

    // printk("Ch %u, evt %u\n", report->chan_idx, report->conn_evt_counter);

    printk("ch=%u,evt=%u,phy=%u,cte=%u,slot=%u,status=%u,rssi=%d.%d,smp_type=%"
           "u,smp_cnt=%u\n",
           // addr,
           report->chan_idx, report->conn_evt_counter, report->rx_phy,
           report->cte_type, report->slot_durations, report->packet_status,
           report->rssi / 10,
           (report->rssi < 0 ? -report->rssi : report->rssi) % 10,
           report->sample_type, report->sample_count);

    /* 默认按 int8_t 版本处理（包含 BT_DF_IQ_SAMPLE_8_BITS） */
    for (uint8_t n = 0; n < report->sample_count; n++) {
      int8_t i = report->sample[n].i;
      int8_t q = report->sample[n].q;
      printk("%d,%d;", i, q);
    }
    printk("\n");
  } else {
    printk("CTE[%s]: request failed, err %u\n", addr, report->err);
  }
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
    .cte_report_cb = cte_recv_cb,
    .le_param_updated = le_param_updated_cb,
};

int main(void) {
  int err;

  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  printk("Bluetooth initialized\n");

  start_scan();
  return 0;
}
