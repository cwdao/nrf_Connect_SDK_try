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

#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/net_buf.h>

#include <string.h>
#include <stdlib.h>

#include "uart_cmd.h"

/* Latency set to zero, to enforce PDU exchange every connection event */
#define CONN_LATENCY 0U
/* Arbitrary selected timeout value */
#define CONN_TIMEOUT 400U

/* Default Inteval: 25ms => 20 * 1.25ms */
#define CONN_INT_DEFAULT_UNITS 0x14 /* 20 */

#define DF_FEAT_ENABLED BIT64(BT_LE_FEAT_BIT_CONN_CTE_RESP)

/* 协议帧版本 */
#define DF_VER 1
static uint32_t df_seq_num = 0;

static struct bt_conn *default_conn;

/* 连接参数改成可变：interval 由命令设置 */
static struct bt_le_conn_param g_conn_params =
    BT_LE_CONN_PARAM_INIT(CONN_INT_DEFAULT_UNITS,
                          CONN_INT_DEFAULT_UNITS,
                          CONN_LATENCY,
                          CONN_TIMEOUT);

/* ---------- CTE 参数：由命令设置---------- */
static uint8_t g_cte_len = 0x02; /* unit 8us */
static uint8_t g_cte_req_interval = 1U; /* connection events */
static uint8_t g_cte_type = BT_DF_CTE_TYPE_AOD_2US;

/* 自定义信道列表（由命令设置） */
static uint8_t g_channels[10] = {3};
static size_t g_channel_cnt = 1;

#if defined(CONFIG_BT_DF_CTE_RX_AOA)
static const uint8_t ant_patterns[] = {0x2, 0x0, 0x5, 0x6, 0x1, 0x4,
                                       0xC, 0x9, 0xE, 0xD, 0x8, 0xA};
#endif /* CONFIG_BT_DF_CTE_RX_AOA */

static void start_scan(void);

static void print_conn_tx_power(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_tx_power p = {0};

    p.phy = 0;
    err = bt_conn_le_get_tx_power_level(conn, &p);
    if (err) {
        printk("Read TX power failed (err %d)\n", err);
        return;
    }

    printk("TX power (conn): current=%d dBm, max=%d dBm\n",
           p.current_level, p.max_level);
}

static bool eir_found(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    uint64_t u64 = 0U;
    int err;

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

        err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                &g_conn_params, &default_conn);
        if (err) {
            printk("Create conn failed (err %d)\n", err);
            start_scan();
        }
        return false;
    }

    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    ARG_UNUSED(rssi);

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
        type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_data_parse(ad, eir_found, (void *)addr);
    }
}

/*  单信道是不合规的，至少需要双信道才合规。但我们只启用一个也能跑，看起来sdc没有限制。这和ble
 cs的情况完全不同 */
static bool is_valid_data_channel(uint8_t ch)
{
    return (ch <= 36);
}

int set_custom_channel_map_from_list(const uint8_t *channels,
                                     size_t channel_cnt)
{
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

    memset(cp->ch_map, 0x00, sizeof(cp->ch_map));

    for (i = 0; i < channel_cnt; i++) {
        uint8_t ch = channels[i];
        if (!is_valid_data_channel(ch)) {
            printk("Skip invalid/adv channel %u\n", ch);
            continue;
        }
        cp->ch_map[ch / 8] |= BIT(ch % 8);
        valid_cnt++;
    }

    err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_HOST_CHAN_CLASSIF, buf, NULL);
    if (err) {
        printk("Set channel map failed (err %d)\n", err);
        return err;
    }

    printk("Custom channel map applied (%u channels)\n", valid_cnt);
    return 0;
}

static void enable_cte_request(void)
{
    int err;

    const struct bt_df_conn_cte_rx_param cte_rx_params = {
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
        .cte_types = BT_DF_CTE_TYPE_ALL,
        .slot_durations = BT_DF_ANTENNA_SWITCHING_SLOT_1US,
        .num_ant_ids = ARRAY_SIZE(ant_patterns),
        .ant_ids = ant_patterns,
#else
        .cte_types = BT_DF_CTE_TYPE_AOD_1US | BT_DF_CTE_TYPE_AOD_2US,
#endif
    };

    const struct bt_df_conn_cte_req_params cte_req_params = {
        .interval = g_cte_req_interval,
        .cte_length = g_cte_len,
        .cte_type = g_cte_type,
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

static void disable_cte_request(void)
{
    if (!default_conn) {
        return;
    }

    /* 不同 NCS/Zephyr 版本 disable API 名称可能不一致：
     * - bt_df_conn_cte_req_disable
     * - bt_df_conn_cte_rx_disable
     * 如果你编译报找不到符号，就注释对应行。
     */
#if defined(CONFIG_BT_DF_CONNECTION_CTE_RX)
    (void)bt_df_conn_cte_rx_disable(default_conn);
#endif
#if defined(CONFIG_BT_DF_CONNECTION_CTE_REQ)
    (void)bt_df_conn_cte_req_disable(default_conn);
#endif
}

static void start_scan(void)
{
    int err;

    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        uart_cmd_send_evt("BLE", "scan_start_failed,err=%d", err);
        return;
    }

    printk("Scanning successfully started\n");
    uart_cmd_send_evt("BLE", "scan_started");
}

/* ---------- IQ report -> $DF 帧 ---------- */

static void sort_u16(uint16_t *a, size_t n)
{
    for (size_t i = 1; i < n; i++) {
        uint16_t key = a[i];
        size_t j = i;
        while (j > 0 && a[j - 1] > key) {
            a[j] = a[j - 1];
            j--;
        }
        a[j] = key;
    }
}

static void cte_recv_cb(struct bt_conn *conn,
                        struct bt_df_conn_iq_samples_report const *report)
{
    ARG_UNUSED(conn);

    if (!report || report->err != BT_DF_IQ_REPORT_ERR_SUCCESS) {
        return;
    }

    if (report->sample_type != BT_DF_IQ_SAMPLE_8_BITS_INT ||
        report->sample == NULL ||
        report->sample_count == 0) {
        return;
    }

    uint16_t p_array[8];
    int reference_sample_count = 8;

    if (report->sample_count < reference_sample_count) {
        return;
    }

    for (uint8_t n = 0; n < reference_sample_count; n++) {
        int32_t i = report->sample[n].i;
        int32_t q = report->sample[n].q;
        p_array[n] = (uint16_t)(i * i + q * q);
    }

    sort_u16(p_array, reference_sample_count);
    uint32_t sum_mid = 0;
    for (uint8_t k = 1; k <= 6; k++) {
        sum_mid += p_array[k];
    }
    uint32_t p_avg = sum_mid / 6;

    uint32_t ts_ms = k_uptime_get_32();

    printk("$DF,%u,%u,%u,%u,%u\r\n",
           DF_VER, report->chan_idx, df_seq_num++, ts_ms, p_avg);
}

/* ---------- BT callbacks ---------- */

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        printk("Failed to connect to %s, 0x%02x %s\n",
               addr, conn_err, bt_hci_err_to_str(conn_err));

        uart_cmd_send_evt("BLE", "connect_failed,addr=%s,err=0x%02x", addr, conn_err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    printk("Connected: %s\n", addr);
    uart_cmd_send_evt("BLE", "connected,addr=%s", addr);

    /* Apply custom channel map */
    (void)set_custom_channel_map_from_list(g_channels, g_channel_cnt);

    /* 检查连接间隔参数 */
    int err;
    struct bt_conn_info info = {0};
    err = bt_conn_get_info(conn, &info);
    if (!err) {
        printk("Conn. interval is %u us\n", info.le.interval_us);
        uart_cmd_send_evt("BLE", "interval_us=%u", info.le.interval_us);
    }

    print_conn_tx_power(conn);

    if (conn == default_conn) {
        enable_cte_request();
        uart_cmd_send_evt("DF", "cte_enabled,len=%u,type=%u,intv=%u",
                          g_cte_len, g_cte_type, g_cte_req_interval);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s, reason 0x%02x %s\n",
           addr, reason, bt_hci_err_to_str(reason));
    uart_cmd_send_evt("BLE", "disconnected,addr=%s,reason=0x%02x", addr, reason);

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    start_scan();
}

static void le_param_updated_cb(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);
    printk("LE param updated: interval %u *1.25(ms) latency %u timeout %u (ms)\n",
           interval, latency, timeout);

    uart_cmd_send_evt("BLE", "param_updated,interval_units=%u,lat=%u,to=%u",
                      interval, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .cte_report_cb = cte_recv_cb,
    .le_param_updated = le_param_updated_cb,
};

/* ---------- 命令解析辅助 ---------- */

static const char *kv_find(int argc, const char *argv[], const char *key)
{
    size_t klen = strlen(key);
    for (int i = 0; i < argc; i++) {
        const char *s = argv[i];
        if (!s) continue;
        if (!strncmp(s, key, klen) && s[klen] == '=') {
            return s + klen + 1;
        }
    }
    return NULL;
}

static int parse_u32(const char *s, uint32_t *out)
{
    if (!s || !out) return -EINVAL;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') return -EINVAL;
    *out = (uint32_t)v;
    return 0;
}

/* "3|10|25" -> [3,10,25] */
static int parse_channels(const char *s, uint8_t *out, size_t out_cap, size_t *out_cnt)
{
    if (!s || !out || !out_cnt) return -EINVAL;
    *out_cnt = 0;

    const char *p = s;
    while (*p) {
        char *end = NULL;
        unsigned long v = strtoul(p, &end, 10);
        if (end == p) return -EINVAL;
        if (v > 255) return -EINVAL;

        if (*out_cnt >= out_cap) return -ENOMEM;
        out[(*out_cnt)++] = (uint8_t)v;

        if (*end == '\0') break;
        if (*end != '|') return -EINVAL;
        p = end + 1;
    }
    return 0;
}

/* interval_ms 必须能转成 1.25ms units：units = interval_ms * 100 / 125 */
static int interval_ms_to_units(uint32_t interval_ms, uint16_t *units)
{
    if (!units) return -EINVAL;
    /* 用整数避免浮点：检查 interval_ms*100 是否能被125整除 */
    uint32_t x = interval_ms * 100U;
    if ((x % 125U) != 0U) return -EINVAL;

    uint32_t u = x / 125U;
    if (u < 6U || u > 3200U) return -ERANGE; /* BLE spec: 7.5ms..4s */
    *units = (uint16_t)u;
    return 0;
}

static void on_cmd(const char *cmd, int argc, const char *argv[])
{
    if (!strcmp(cmd, "PING")) {
        uart_cmd_send_ok("PING", "pong");
        return;
    }

    if (!strcmp(cmd, "BLE_SCAN")) {
        const char *act = kv_find(argc, argv, "action");
        if (!act) {
            uart_cmd_send_err("BLE_SCAN", 1, "MISSING_PARAM:action");
            return;
        }
        if (!strcmp(act, "start")) {
            start_scan();
            uart_cmd_send_ok("BLE_SCAN", "action=start");
            return;
        }
        if (!strcmp(act, "stop")) {
            int err = bt_le_scan_stop();
            if (err) {
                uart_cmd_send_err("BLE_SCAN", 2, "scan_stop_failed,err=%d", err);
            } else {
                uart_cmd_send_ok("BLE_SCAN", "action=stop");
                uart_cmd_send_evt("BLE", "scan_stopped");
            }
            return;
        }

        uart_cmd_send_err("BLE_SCAN", 3, "INVALID_PARAM:action");
        return;
    }

    if (!strcmp(cmd, "BLE_CONN")) {
        const char *act = kv_find(argc, argv, "action");
        if (!act) {
            uart_cmd_send_err("BLE_CONN", 1, "MISSING_PARAM:action");
            return;
        }
        if (!strcmp(act, "disconnect")) {
            if (!default_conn) {
                uart_cmd_send_err("BLE_CONN", 2, "NOT_CONNECTED");
                return;
            }
            int err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            if (err) {
                uart_cmd_send_err("BLE_CONN", 3, "disconnect_failed,err=%d", err);
            } else {
                uart_cmd_send_ok("BLE_CONN", "action=disconnect");
            }
            return;
        }

        uart_cmd_send_err("BLE_CONN", 4, "INVALID_PARAM:action");
        return;
    }

    if (!strcmp(cmd, "DF_START")) {
        /* channels */
        const char *chs = kv_find(argc, argv, "channels");
        if (chs) {
            size_t cnt = 0;
            uint8_t tmp[ARRAY_SIZE(g_channels)];
            int err = parse_channels(chs, tmp, ARRAY_SIZE(tmp), &cnt);
            if (err) {
                uart_cmd_send_err("DF_START", 1, "INVALID_PARAM:channels");
                return;
            }
            memcpy(g_channels, tmp, cnt);
            g_channel_cnt = cnt;
        }

        /* interval_ms */
        const char *ims = kv_find(argc, argv, "interval_ms");
        if (ims) {
            uint32_t interval_ms = 0;
            uint16_t units = 0;
            if (parse_u32(ims, &interval_ms) ||
                interval_ms_to_units(interval_ms, &units)) {
                uart_cmd_send_err("DF_START", 2, "INVALID_PARAM:interval_ms");
                return;
            }
            g_conn_params.interval_min = units;
            g_conn_params.interval_max = units;
        }

        /* cte_len (8us units) */
        const char *cl = kv_find(argc, argv, "cte_len");
        if (cl) {
            uint32_t v = 0;
            if (parse_u32(cl, &v) || v > 0xFF) {
                uart_cmd_send_err("DF_START", 3, "INVALID_PARAM:cte_len");
                return;
            }
            g_cte_len = (uint8_t)v;
        }

        /* cte_type: aod1/aod2/aoa (若编译支持 AOA RX) */
        const char *ct = kv_find(argc, argv, "cte_type");
        if (ct) {
            if (!strcmp(ct, "aod1")) g_cte_type = BT_DF_CTE_TYPE_AOD_1US;
            else if (!strcmp(ct, "aod2")) g_cte_type = BT_DF_CTE_TYPE_AOD_2US;
#if defined(CONFIG_BT_DF_CTE_RX_AOA)
            else if (!strcmp(ct, "aoa")) g_cte_type = BT_DF_CTE_TYPE_AOA;
#endif
            else {
                uart_cmd_send_err("DF_START", 4, "INVALID_PARAM:cte_type");
                return;
            }
        }

        uart_cmd_send_ok("DF_START",
                         "channels_cnt=%u,interval_units=%u,cte_len=%u,cte_type=%u",
                         (unsigned)g_channel_cnt,
                         (unsigned)g_conn_params.interval_min,
                         (unsigned)g_cte_len,
                         (unsigned)g_cte_type);

        /* 连接后会自动 enable_cte_request()（在 connected 回调里） */
        if (!default_conn) {
            start_scan();
            return;
        }

        /* 已连接：立即应用信道图 + 重新 enable */
        (void)set_custom_channel_map_from_list(g_channels, g_channel_cnt);
        enable_cte_request();
        return;
    }

    if (!strcmp(cmd, "DF_STOP")) {
        disable_cte_request();
        uart_cmd_send_ok("DF_STOP", "done");
        return;
    }

    uart_cmd_send_err(cmd, 99, "UNKNOWN_CMD");
}

int main(void)
{
    int err;

    printk("BOOT: console up\r\n");

    err = uart_cmd_init(on_cmd);
    if (err) {
        printk("uart_cmd_init failed (err %d)\n", err);
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        uart_cmd_send_evt("BLE", "bt_enable_failed,err=%d", err);
        return 0;
    }

    printk("Bluetooth initialized\n");
    uart_cmd_send_evt("BLE", "initialized");

    /* 默认自动扫描；你也可以改成等待 $CMD,BLE_SCAN,action=start */
    start_scan();

    return 0;
}
