/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Inline PCT Transfer sample
 *
 *  Adds type 0x03 binary IQ upload (procedure-complete) and Button0/1
 *  start/stop. See doc/IPT_数据特性与上位机适配方案.md.
 */

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/cs_de.h>

#include <dk_buttons_and_leds.h>

#include "cs_uart_binary.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_WRN);

#define CON_STATUS_LED	 DK_LED1
#define CS_STATUS_LED	 DK_LED2
#define CS_CONFIG_ID	 0
#define NUM_MODE_0_STEPS 3
#define REFLECTOR_NAME	 "Nordic CS IPT Reflector"

#define CHANNEL_INDEX_OFFSET   (2)
#define MEDIAN_FILTER_SIZE (9)

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_subevent_results_parsed, 0, 1);
static K_SEM_DEFINE(sem_distance_estimate_updated, 1, 1);

static struct bt_conn *connection;
static struct bt_conn_le_cs_config cs_config;

static struct bt_le_cs_procedure_enable_param procedure_enable_param = {
	.config_id = CS_CONFIG_ID,
	.enable = 0,
};

static struct k_work button0_work;
static struct k_work button1_work;

/* Store local initiator IQs in this union.
 * The size is based on requirements of cs_de_ifft.
 * The scratch memory is used for the ifft operation.
 */
static union {
	struct {
		float i;
		float q;
	} values[CS_DE_NUM_CHANNELS];
	float scratch_mem[2 * CONFIG_BT_CS_DE_NFFT_SIZE];
} iq;

/** int16 IQ + validity for type 0x03 framing (before IFFT overwrites float buffer) */
static struct {
	int16_t i;
	int16_t q;
	bool valid;
} iq_int16[CS_DE_NUM_CHANNELS];

static uint16_t frame_procedure_counter;
static uint8_t ipt_bin_frame_buf[CS_UART_BIN_MAX_FRAME_SIZE];

/* --- Sliding-window median filter for distance tracking ---
 *
 * Keeps the most recent MEDIAN_FILTER_SIZE distance measurements in a
 * circular buffer and reports the median of the valid (finite) samples.
 */
struct distance_estimate_buffer {
	float estimates[MEDIAN_FILTER_SIZE];
	uint8_t num_valid;
	uint8_t index;
};

static struct distance_estimate_buffer distance_estimate_buffer;

static void store_distance_estimate(float distance)
{
	distance_estimate_buffer.estimates[distance_estimate_buffer.index] = distance;
	distance_estimate_buffer.index =
		(distance_estimate_buffer.index + 1) % MEDIAN_FILTER_SIZE;

	if (distance_estimate_buffer.num_valid < MEDIAN_FILTER_SIZE) {
		distance_estimate_buffer.num_valid++;
	}
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count / 2] + values[count / 2 - 1]) / 2;
	} else {
		return values[count / 2];
	}
}

static float get_filtered_distance(void)
{
	static float temp_ifft[MEDIAN_FILTER_SIZE];
	uint8_t num_ifft = 0;

	for (uint8_t i = 0; i < distance_estimate_buffer.num_valid; i++) {
		if (isfinite(distance_estimate_buffer.estimates[i])) {
			temp_ifft[num_ifft] = distance_estimate_buffer.estimates[i];
			num_ifft++;
		}
	}

	return median_inplace(num_ifft, temp_ifft);
}

static void ipt_iq_buffers_clear(void)
{
	memset(iq.scratch_mem, 0, sizeof(iq.scratch_mem));
	memset(iq_int16, 0, sizeof(iq_int16));
}

static void ipt_bin_fill_channel_bitmap(uint8_t *bitmap, uint8_t *channel_count)
{
	memset(bitmap, 0, CS_UART_BIN_CHANNEL_BITMAP_BYTES);

	uint8_t count = 0U;

	for (uint8_t ch = 0; ch < CS_DE_NUM_CHANNELS; ch++) {
		if (!iq_int16[ch].valid) {
			continue;
		}

		bitmap[ch / 8U] |= (uint8_t)(1U << (ch % 8U));
		count++;
	}

	*channel_count = count;
}

static size_t ipt_bin_pack_iq_payload(const uint8_t *bitmap, uint8_t *out, size_t out_max)
{
	size_t offset = 0U;

	for (uint8_t ch = 0; ch < CS_DE_NUM_CHANNELS; ch++) {
		if ((bitmap[ch / 8U] & (uint8_t)(1U << (ch % 8U))) == 0U) {
			continue;
		}

		if (offset + sizeof(int16_t) + sizeof(int16_t) > out_max) {
			break;
		}

		sys_put_le16((uint16_t)iq_int16[ch].i, &out[offset]);
		offset += sizeof(int16_t);
		sys_put_le16((uint16_t)iq_int16[ch].q, &out[offset]);
		offset += sizeof(int16_t);
	}

	return offset;
}

/**
 * Build type 0x03 IPT combined-IQ frame (same header as DIP, 4 B/channel).
 * Must run before cs_de_ifft() overwrites the float IQ buffer.
 */
static int ipt_bin_build_frame(uint8_t *frame, size_t frame_cap, uint16_t *out_len)
{
	if (frame == NULL || out_len == NULL) {
		return -EINVAL;
	}

	uint8_t channel_count = 0U;
	uint8_t bitmap[CS_UART_BIN_CHANNEL_BITMAP_BYTES];

	ipt_bin_fill_channel_bitmap(bitmap, &channel_count);

	if (channel_count == 0U) {
		return -ENOENT;
	}

	const size_t hdr_size = sizeof(struct cs_uart_bin_header);
	const size_t iq_size = (size_t)channel_count * 2U * sizeof(int16_t);
	const size_t frame_len = hdr_size + iq_size + sizeof(uint16_t);

	if (frame_len > frame_cap) {
		LOG_WRN("IPT binary: frame too large (ch=%u)", channel_count);
		return -ENOSPC;
	}

	struct cs_uart_bin_header *hdr = (struct cs_uart_bin_header *)frame;

	hdr->sync1 = CS_UART_BIN_SYNC1;
	hdr->sync2 = CS_UART_BIN_SYNC2;
	hdr->version = CS_UART_BIN_VERSION;
	hdr->type = CS_UART_BIN_TYPE_IPT_COMBINED_IQ;
	hdr->procedure_counter = sys_cpu_to_le16(frame_procedure_counter);
	hdr->timestamp_ms = sys_cpu_to_le64(k_uptime_get());
	hdr->ap = 0U;
	hdr->iq_format = CS_UART_BIN_IQ_FORMAT_INT16;
	hdr->channel_count = channel_count;
	hdr->reserved = 0U;
	memcpy(hdr->channel_bitmap, bitmap, CS_UART_BIN_CHANNEL_BITMAP_BYTES);

	const uint16_t payload_len =
		(uint16_t)(sizeof(hdr->procedure_counter) + sizeof(hdr->timestamp_ms) +
			   sizeof(hdr->ap) + sizeof(hdr->iq_format) + sizeof(hdr->channel_count) +
			   sizeof(hdr->reserved) + CS_UART_BIN_CHANNEL_BITMAP_BYTES + iq_size);

	hdr->payload_len = sys_cpu_to_le16(payload_len);

	const size_t iq_written =
		ipt_bin_pack_iq_payload(bitmap, &frame[hdr_size], frame_cap - hdr_size - sizeof(uint16_t));

	if (iq_written != iq_size) {
		LOG_WRN("IPT binary: IQ pack mismatch (expect %u got %u)", (unsigned)iq_size,
			(unsigned)iq_written);
		return -EIO;
	}

	const uint16_t crc = cs_uart_bin_crc16_ccitt(frame, hdr_size + iq_written);

	sys_put_le16(crc, &frame[hdr_size + iq_written]);
	*out_len = (uint16_t)frame_len;

	return 0;
}

static void ipt_output_procedure_binary(void)
{
	uint16_t len = 0U;
	int err = ipt_bin_build_frame(ipt_bin_frame_buf, sizeof(ipt_bin_frame_buf), &len);

	if (err != 0) {
		return;
	}

	err = cs_uart_bin_enqueue_frame(ipt_bin_frame_buf, len);
	if (err == -ENOMEM) {
		LOG_WRN("IPT binary msgq full, drop pc=%u", frame_procedure_counter);
	}
}

static void distance_estimates_update(void)
{
	/* Enqueue type 0x03 before IFFT clobbers the float buffer. */
	ipt_output_procedure_binary();

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; i++) {
		LOG_DBG("iq[%d]: %.1f + j * %.1f", i, (double)iq.values[i].i,
			(double)iq.values[i].q);
	}

	float distance_ifft = cs_de_ifft(iq.scratch_mem);

	if (isfinite(distance_ifft)) {
		static int64_t prev_ts = -1;

		int64_t now = k_uptime_get();
		int64_t delta = 0;

		if (prev_ts != -1) {
			delta = now - prev_ts;
		}
		prev_ts = now;

		store_distance_estimate(distance_ifft);

		float distance_median = get_filtered_distance();

		LOG_DBG("Distance estimates: median: %.2fm, update: %.2fm, time_delta: %lldms",
			(double)distance_median, (double)distance_ifft, delta);
	}
	k_sem_give(&sem_distance_estimate_updated);
}

static void pcts_parse(uint8_t channel_index,
		       struct bt_hci_le_cs_step_data_tone_info *local_tone_info)
{
	if (channel_index < CHANNEL_INDEX_OFFSET) {
		return;
	}

	const uint8_t fft_ch = channel_index - CHANNEL_INDEX_OFFSET;

	if (fft_ch >= CS_DE_NUM_CHANNELS) {
		return;
	}

	/* Match DIP: only HIGH quality tones contribute to IQ / bitmap. */
	if (local_tone_info[0].quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
		return;
	}

	struct bt_le_cs_iq_sample local_iq =
		bt_le_cs_parse_pct(local_tone_info[0].phase_correction_term);

	iq_int16[fft_ch].i = local_iq.i;
	iq_int16[fft_ch].q = local_iq.q;
	iq_int16[fft_ch].valid = true;

	iq.values[fft_ch].i = (float)local_iq.i;
	iq.values[fft_ch].q = (float)local_iq.q;
}

static void subevent_steps_parse(struct bt_conn_le_cs_subevent_result *result)
{
	for (uint8_t i = 0; i < result->header.num_steps_reported; i++) {
		if (result->step_data_buf->len < 3) {
			LOG_WRN("Local step data appears malformed.");
			return;
		}
		struct bt_le_cs_subevent_step local_step = {0};

		local_step.mode = net_buf_simple_pull_u8(result->step_data_buf);
		local_step.channel = net_buf_simple_pull_u8(result->step_data_buf);
		local_step.data_len = net_buf_simple_pull_u8(result->step_data_buf);

		if (local_step.data_len == 0) {
			LOG_WRN("Encountered zero-length step data.");
			return;
		}
		if (local_step.data_len > result->step_data_buf->len) {
			LOG_WRN("Local step data appears malformed.");
			return;
		}

		local_step.data = result->step_data_buf->data;

		if (local_step.mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
			struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
				(struct bt_hci_le_cs_step_data_mode_2 *)local_step.data;
			pcts_parse(local_step.channel, local_step_data->tone_info);
		}

		net_buf_simple_pull(result->step_data_buf, local_step.data_len);
	}
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	static uint32_t prev_procedure_counter = UINT16_MAX + 1;

	ARG_UNUSED(conn);

	LOG_DBG("Subevent result callback for procedure counter %u procedure done status "
		"%u subevent done status %u, number of steps reported %u",
		result->header.procedure_counter, result->header.procedure_done_status,
		result->header.subevent_done_status, result->header.num_steps_reported);

	const bool cs_aborted = (result->header.procedure_abort_reason !=
				 BT_HCI_LE_CS_PROCEDURE_ABORT_REASON_NO_ABORT) ||
				(result->header.subevent_abort_reason !=
				 BT_HCI_LE_CS_SUBEVENT_ABORT_REASON_NO_ABORT);

	if (cs_aborted) {
		LOG_DBG("CS aborted. Procedure counter: %u", result->header.procedure_counter);
		return;
	}

	if (result->header.procedure_counter != prev_procedure_counter) {
		k_sem_take(&sem_distance_estimate_updated, K_FOREVER);
		ipt_iq_buffers_clear();
	}
	prev_procedure_counter = result->header.procedure_counter;
	frame_procedure_counter = result->header.procedure_counter;

	subevent_steps_parse(result);

	/* All steps from the procedure parsed; distance estimation can run. */
	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		k_sem_give(&sem_subevent_results_parsed);
	}
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
		return;
	}

	LOG_WRN("Security changed: %s level %u", addr, level);
	k_sem_give(&sem_security);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
	} else {
		connection = bt_conn_ref(conn);

		k_sem_give(&sem_connected);

		dk_set_led_on(CON_STATUS_LED);
	}
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int err;

	LOG_WRN("Disconnected (reason 0x%02X)", reason);

	if (connection) {
		bt_conn_unref(connection);
		connection = NULL;
	}

	dk_set_led_off(CON_STATUS_LED);
	dk_set_led_off(CS_STATUS_LED);
	procedure_enable_param.enable = 0;

	k_sem_give(&sem_disconnected);

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Failed to restart scanning (err %i)", err);
	}
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS capability exchange completed.");
		k_sem_give(&sem_remote_capabilities_obtained);
	} else {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		cs_config = *config;
		LOG_WRN("CS config creation complete. ID: %u", config->id);
		k_sem_give(&sem_config_created);
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS security enabled.");
		k_sem_give(&sem_cs_security_enabled);
	} else {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);

	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			LOG_WRN("CS procedures enabled (interval %u, subevent_len %u us)",
				params->procedure_interval, params->subevent_len);
		} else {
			LOG_WRN("CS procedures disabled.");
		}
	} else {
		LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
	}
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	ARG_UNUSED(filter_match);

	(void)bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_DBG("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	int err;

	ARG_UNUSED(device_info);

	LOG_WRN("Connecting failed, restarting scanning");

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Failed to restart scanning (err %i)", err);
		return;
	}
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	ARG_UNUSED(device_info);
	ARG_UNUSED(conn);

	LOG_DBG("Connecting");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(struct bt_scan_init_param *p_param)
{
	int err;

	bt_scan_init(p_param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, REFLECTOR_NAME);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return 0;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

static void cs_config_get(struct bt_le_cs_create_config_params *config_params)
{
	config_params->id = CS_CONFIG_ID;
	config_params->mode = BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE;
	config_params->min_main_mode_steps = 2;
	config_params->max_main_mode_steps = 5;
	config_params->main_mode_repetition = 0;
	config_params->mode_0_steps = NUM_MODE_0_STEPS;
	config_params->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	config_params->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	config_params->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	config_params->channel_map_repetition = 1;
	config_params->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	config_params->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	config_params->ch3c_jump = 2;
	/* Enable inline PCT transfer. */
	config_params->cs_enhancements_1 = 1;
}

static void button0_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (connection == NULL) {
		LOG_WRN("Button 0: no connection");
		return;
	}

	procedure_enable_param.enable = 1;
	err = bt_le_cs_procedure_enable(connection, &procedure_enable_param);
	if (err) {
		LOG_ERR("Failed to enable CS procedures (err %d)", err);
		return;
	}

	dk_set_led_on(CS_STATUS_LED);
	LOG_WRN("Button 0: CS procedures started");
}

static void button1_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (connection == NULL) {
		LOG_WRN("Button 1: no connection");
		return;
	}

	procedure_enable_param.enable = 0;
	err = bt_le_cs_procedure_enable(connection, &procedure_enable_param);
	if (err) {
		LOG_ERR("Failed to disable CS procedures (err %d)", err);
		return;
	}

	dk_set_led_off(CS_STATUS_LED);
	LOG_WRN("Button 1: CS procedures stopped");
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if ((has_changed & DK_BTN1_MSK) && (button_state & DK_BTN1_MSK)) {
		k_work_submit(&button0_work);
	}

	if ((has_changed & DK_BTN2_MSK) && (button_state & DK_BTN2_MSK)) {
		k_work_submit(&button1_work);
	}
}

static void drain_sem(struct k_sem *sem)
{
	while (k_sem_take(sem, K_NO_WAIT) == 0) {
	}
}

/** Wait for @p sem, or return -ENOTCONN if the link drops first. */
static int wait_sem_or_disconnect(struct k_sem *sem)
{
	while (true) {
		if (k_sem_take(sem, K_MSEC(100)) == 0) {
			return 0;
		}
		if (k_sem_take(&sem_disconnected, K_NO_WAIT) == 0) {
			return -ENOTCONN;
		}
	}
}

static int cs_setup_on_connection(const struct bt_le_conn_param *conn_param)
{
	int err;

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if (err) {
		LOG_ERR("Failed to encrypt connection (err %d)", err);
		return err;
	}
	err = wait_sem_or_disconnect(&sem_security);
	if (err) {
		return err;
	}

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		LOG_ERR("Failed to configure default CS settings (err %d)", err);
		return err;
	}

	err = bt_le_cs_read_remote_supported_capabilities(connection);
	if (err) {
		LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
		return err;
	}

	err = wait_sem_or_disconnect(&sem_remote_capabilities_obtained);
	if (err) {
		return err;
	}

	struct bt_le_cs_create_config_params config_params;

	memset(&config_params, 0, sizeof(config_params));
	cs_config_get(&config_params);
	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	err = bt_le_cs_create_config(connection, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Failed to create CS config (err %d)", err);
		return err;
	}
	err = wait_sem_or_disconnect(&sem_config_created);
	if (err) {
		return err;
	}

	err = bt_le_cs_security_enable(connection);
	if (err) {
		LOG_ERR("Failed to start CS Security (err %d)", err);
		return err;
	}

	err = wait_sem_or_disconnect(&sem_cs_security_enabled);
	if (err) {
		return err;
	}

	/* scale factor of conn_interval units to proc_interval units is 1.25/0.625 = 2 */
	const uint16_t acl_interval_in_proc_interval_units = conn_param->interval_max * 2;
	uint16_t desired_procedure_interval = 2;
	uint16_t desired_max_procedure_length =
		acl_interval_in_proc_interval_units * (desired_procedure_interval - 1);

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = desired_max_procedure_length,
		.min_procedure_interval = desired_procedure_interval,
		.max_procedure_interval = desired_procedure_interval,
		.max_procedure_count = 0,
		.min_subevent_len = 11000,
		.max_subevent_len = 11000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_2M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters (err %d)", err);
		return err;
	}

	LOG_WRN("CS ready. Press Button 1 to start, Button 2 to stop.");
	return 0;
}

int main(void)
{
	int err;

	LOG_WRN("Starting Channel Sounding IPT Initiator Sample");

	dk_leds_init();
	dk_buttons_init(button_handler);

	k_work_init(&button0_work, button0_work_handler);
	k_work_init(&button1_work, button1_work_handler);

	cs_uart_bin_tx_thread_start();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	struct bt_le_conn_param conn_param_storage = *BT_LE_CONN_PARAM(6, 6, 0,
								      BT_GAP_MS_TO_CONN_TIMEOUT(4000));
	struct bt_scan_init_param scan_params = {
		.scan_param = NULL,
		.conn_param = &conn_param_storage,
		.connect_if_match = 1};

	err = scan_init(&scan_params);
	if (err) {
		LOG_ERR("Scan init failed (err %d)", err);
		return 0;
	}

	while (true) {
		drain_sem(&sem_connected);
		drain_sem(&sem_disconnected);
		drain_sem(&sem_security);
		drain_sem(&sem_remote_capabilities_obtained);
		drain_sem(&sem_config_created);
		drain_sem(&sem_cs_security_enabled);
		drain_sem(&sem_subevent_results_parsed);

		err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
		if (err && err != -EALREADY) {
			LOG_ERR("Scanning failed to start (err %i)", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_WRN("Scanning for %s", REFLECTOR_NAME);
		k_sem_take(&sem_connected, K_FOREVER);

		err = cs_setup_on_connection(scan_params.conn_param);
		if (err) {
			if (err == -ENOTCONN) {
				/* disconnected_cb already restarted scan */
				continue;
			}
			if (connection) {
				bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				k_sem_take(&sem_disconnected, K_FOREVER);
			}
			continue;
		}

		/* Wait for disconnect; process completed procedures while connected. */
		while (true) {
			if (k_sem_take(&sem_disconnected, K_NO_WAIT) == 0) {
				break;
			}

			if (k_sem_take(&sem_subevent_results_parsed, K_MSEC(50)) == 0) {
				distance_estimates_update();
			}
		}
	}

	return 0;
}
