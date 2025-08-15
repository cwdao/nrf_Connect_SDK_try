/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Ranging Requestor sample
 */

#include <bluetooth/cs_de.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <math.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>

#include "cs_de_data_parse.h"
#include "flash/flash_ops.h"
#include "interface/button_led.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

// 定义每个按键的工作任务,中断现在只提交队列，不再执行具体复杂操作以提升系统实时性
static struct k_work button0_work;
static struct k_work button1_work;
static struct k_work button2_work;
static struct k_work button3_work;
// 定义flash写入工作队列
static struct k_work_delayable flash_write_work;
// 定义单次flash写入工作队列
static struct k_work flash_single_write_work;
#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
// 定义定时刷新缓冲区的工作队列
static struct k_work_delayable flash_timer_work;
#endif
// 指示蓝牙连接状态
// static bool bt_is_connected = false;

enum bt_cs_state_t {
  BT_CS_STATE_IDLE,                   // 蓝牙CS空闲状态
  BT_CS_STATE_SCANNING,               // 蓝牙CS正在扫描
  BT_CS_STATE_ENABLED,                // 蓝牙CS已启用
  BT_CS_STATE_DISABLED,               // 蓝牙CS已禁用
  BT_CS_STATE_DISABLED_AND_UPLOADING, // 蓝牙CS已禁用且正在上传
  BT_CS_STATE_DATA_ERASING            // 蓝牙CS已禁用且正在擦除
};

enum flash_state_t {
  FLASH_STATE_IDLE,         // 蓝牙CS空闲状态
  FLASH_STATE_DATA_WRITING, // 蓝牙CS已禁用且正在写入
  FLASH_STATE_DATA_READING, // 蓝牙CS已禁用且正在读取
  FLASH_STATE_DATA_ERASING  // 蓝牙CS已禁用且正在擦除
};

// typedef struct {
//   uint32_t report_index; // 测距结果的索引
//   uint64_t timestamp_ms; // 记录写入时的毫秒数
//   cs_de_report_t report; // 原始测距结果
// } store_cs_de_report_t;

static enum bt_cs_state_t bt_cs_state = BT_CS_STATE_IDLE; // 初始化为空闲状态
static enum flash_state_t flash_state = FLASH_STATE_IDLE; // 初始化为空闲状态

// 测试测距相关变量
#define TEST_RANGING_ENABLED 0
#if TEST_RANGING_ENABLED
static uint32_t ranging_count = 0; // 测距计数器（总尝试次数）
static uint32_t ranging_success_count = 0;     // 成功获取数据的次数
static uint32_t ranging_error_count = 0;       // 数据获取失败的次数
static uint32_t ranging_overwritten_count = 0; // 数据被覆盖的次数
static const uint32_t TEST_RANGING_COUNT = 25; // 测试测距次数
#endif

#define NEED_LOG 0 // 是否需要信息输出（用于提升速度）


#define CON_STATUS_LED DK_LED1

#define CS_CONFIG_ID 0
#define NUM_MODE_0_STEPS 3
#define PROCEDURE_COUNTER_NONE (-1)
#define DE_SLIDING_WINDOW_SIZE (10)
#define MAX_AP (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

// 增加缓冲区大小，提高数据存储能力，乘不乘2好像都没用……
#define LOCAL_PROCEDURE_MEM                                                    \
  ((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +  \
   (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);

static struct bt_conn *connection;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

static uint8_t buffer_index;
static uint8_t buffer_num_valid;
static cs_de_dist_estimates_t distance_estimate_buffer[MAX_AP]
                                                      [DE_SLIDING_WINDOW_SIZE];

// 此时基本上进入更细致的信道探测功能的配置了，设备已经连接，服务成功发现。
// 设置默认的信道探测参数：
// enable_initiator_role = true：启用发起者角色。
// enable_reflector_role = false：禁用反射器角色。
// 其他参数包括天线选择、最大功率等。传给bt_le_cs_set_default_settings
static const struct bt_le_cs_set_default_settings_param default_settings = {
    .enable_initiator_role = true,
    .enable_reflector_role = false,
    .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
    .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
};

// 创建CS配置参数，步骤类型，信道选择等等。传给bt_le_cs_create_config
static struct bt_le_cs_create_config_params config_params = {
    .id = CS_CONFIG_ID,
    .main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
    .sub_mode_type = BT_CONN_LE_CS_SUB_MODE_UNUSED,
    .min_main_mode_steps = 1,
    .max_main_mode_steps = 1,
    .main_mode_repetition = 0,
    .mode_0_steps = NUM_MODE_0_STEPS,
    .role = BT_CONN_LE_CS_ROLE_INITIATOR,
    .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
    .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
    .channel_map_repetition = 1,
    .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
    .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
    .ch3c_jump = 2,
};

// 测距参数：interval应该是最核心的一个。假设一个procedure是50ms，那么采样间隔就是间隔N个50ms
static const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
    .config_id = CS_CONFIG_ID,
    .max_procedure_len = 500,
    .min_procedure_interval = 1,
    .max_procedure_interval = 6,
    .max_procedure_count = 0,
    .min_subevent_len = 10000,
    .max_subevent_len = 50000, // 这个就是us
    .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
    .phy = BT_LE_CS_PROCEDURE_PHY_2M,
    .tx_power_delta = 0x80,
    .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
    .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
    .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
};

// 启用测距参数，传给bt_le_cs_procedure_enable
static struct bt_le_cs_procedure_enable_param params = {
    .config_id = CS_CONFIG_ID,
    .enable = 1,
};

static void store_distance_estimates(cs_de_report_t *p_report) {
  int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

  __ASSERT_NO_MSG(lock_state == 0);

  for (uint8_t ap = 0; ap < p_report->n_ap; ap++) {
    memcpy(&distance_estimate_buffer[ap][buffer_index],
           &p_report->distance_estimates[ap], sizeof(cs_de_dist_estimates_t));
  }

  buffer_index = (buffer_index + 1) % DE_SLIDING_WINDOW_SIZE;

  if (buffer_num_valid < DE_SLIDING_WINDOW_SIZE) {
    buffer_num_valid++;
  }

  k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static cs_de_dist_estimates_t get_distance(uint8_t ap) {
  cs_de_dist_estimates_t averaged_result = {};
  uint8_t num_ifft = 0;
  uint8_t num_phase_slope = 0;
  uint8_t num_rtt = 0;

  int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

  __ASSERT_NO_MSG(lock_state == 0);

  for (uint8_t i = 0; i < buffer_num_valid; i++) {
    if (isfinite(distance_estimate_buffer[ap][i].ifft)) {
      num_ifft++;
      averaged_result.ifft += distance_estimate_buffer[ap][i].ifft;
    }
    if (isfinite(distance_estimate_buffer[ap][i].phase_slope)) {
      num_phase_slope++;
      averaged_result.phase_slope +=
          distance_estimate_buffer[ap][i].phase_slope;
    }
    if (isfinite(distance_estimate_buffer[ap][i].rtt)) {
      num_rtt++;
      averaged_result.rtt += distance_estimate_buffer[ap][i].rtt;
    }
  }

  k_mutex_unlock(&distance_estimate_buffer_mutex);

  if (num_ifft) {
    averaged_result.ifft /= num_ifft;
  }

  if (num_phase_slope) {
    averaged_result.phase_slope /= num_phase_slope;
  }

  if (num_rtt) {
    averaged_result.rtt /= num_rtt;
  }

  return averaged_result;
}

// 每次取到一个样本，就给一个全局标号
static __IO uint64_t cs_data_index = 0;
static store_cs_de_report_t temp_flash_data;
static void ranging_data_get_complete_cb(struct bt_conn *conn,
                                         uint16_t ranging_counter, int err) {
  ARG_UNUSED(conn);

  uint32_t start_cycles, end_cycles, elapsed_cycles;
  uint64_t elapsed_ns;
  uint64_t report_timestamp_ms = k_uptime_get();
  // 获取开始的时钟周期
  start_cycles = k_cycle_get_32();

  // 无论成功失败，都先递增测试计数器，确保测试能正常结束
#if TEST_RANGING_ENABLED
  ranging_count++;
  LOG_DBG("Ranging test: %d/%d (ranging_counter: %d)", ranging_count,
          TEST_RANGING_COUNT, ranging_counter);
#endif

  if (err) {
    LOG_ERR("Error when getting ranging data with ranging counter %d (err %d)",
            ranging_counter, err);
#if TEST_RANGING_ENABLED
    ranging_error_count++;
    LOG_INF("Ranging error: %d/%d attempts, %d errors, %d success",
            ranging_count, TEST_RANGING_COUNT, ranging_error_count,
            ranging_success_count);

    // 检查是否达到测试次数（即使失败也要检查）
    if (ranging_count >= TEST_RANGING_COUNT) {
      LOG_INF(
          "Test completed! %d attempts (%d success, %d errors), disabling CS",
          TEST_RANGING_COUNT, ranging_success_count, ranging_error_count);
      params.enable = 0;
      bt_le_cs_procedure_enable(connection, &params);
      bt_cs_state = BT_CS_STATE_DISABLED;
      led_off(0);
    }
#endif
    return;
  }

  LOG_DBG("Ranging data get completed for ranging counter %d", ranging_counter);

#if TEST_RANGING_ENABLED
  ranging_success_count++;
#endif

  /* This struct is static to avoid putting it on the stack (it's very large) */
  static cs_de_report_t cs_de_report;
  // 初始化和填充测距报告.从本地设备和远程设备的步骤数据缓冲区中提取原始数据，并将其存储到
  // cs_de_report_t 数据结构中，供后续的测距计算和质量评估使用
  cs_de_populate_report(&latest_local_steps, &latest_peer_steps,
                        BT_CONN_LE_CS_ROLE_INITIATOR, &cs_de_report);

  net_buf_simple_reset(&latest_local_steps);
  net_buf_simple_reset(&latest_peer_steps);
  k_sem_give(&sem_local_steps);
  // 这是正儿八经算距离了
  cs_de_quality_t quality = cs_de_calc(&cs_de_report);

  if (quality == CS_DE_QUALITY_OK) {
    for (uint8_t ap = 0; ap < cs_de_report.n_ap; ap++) {
      if (cs_de_report.tone_quality[ap] == CS_DE_TONE_QUALITY_OK) {
        store_distance_estimates(&cs_de_report);
      }
    }
  }
  if (flash_state == FLASH_STATE_IDLE) {
    // 准备数据结构

    temp_flash_data.report_index = cs_data_index;
    temp_flash_data.timestamp_ms = report_timestamp_ms;
    memcpy(&temp_flash_data.report, &cs_de_report, sizeof(cs_de_report_t));

    cs_data_index++; // 无论写入模式如何，都递增编号确保连续性

#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_SINGLE
    // 单个写入模式 - 使用k_work异步写入flash
    // LOG_DBG("Using single write mode with k_work");
    k_work_submit(&flash_single_write_work);
#else
    // 批量写入模式 - 使用环形缓冲区
    LOG_DBG("Using batch write mode");
    int err = flash_buffer_put(&temp_data, sizeof(store_cs_de_report_t));
    if (err) {
      LOG_ERR("Failed to add data to buffer: %d", err);
      return;
    }

    // 如果缓冲区达到一定数量，触发批量写入
    uint32_t buffer_count = flash_buffer_get_count();
    if (buffer_count >= FLASH_BUFFER_WRITE_TRIGGER_COUNT ||
        flash_buffer_is_full()) { // 27个记录（9个扇区）约1.35秒的数据
      LOG_INF("Buffer count: %d, triggering flush", buffer_count);
      k_work_schedule(&flash_write_work, K_NO_WAIT);
    }
#endif
  } else {
    // Flash忙时跳过但仍递增编号保持连续性
    cs_data_index++;
    LOG_WRN("Flash busy, skipping data with index %llu", cs_data_index - 1);
  }

// 检查是否达到测试次数（成功情况下的检查）
#if TEST_RANGING_ENABLED
  if (ranging_count >= TEST_RANGING_COUNT) {
    LOG_INF("Test completed! %d attempts (%d success, %d errors, %d "
            "overwritten), disabling CS",
            TEST_RANGING_COUNT, ranging_success_count, ranging_error_count,
            ranging_overwritten_count);
    LOG_INF("Success rate: %.1f%% (%d/%d)",
            (double)ranging_success_count / TEST_RANGING_COUNT * 100.0,
            ranging_success_count, TEST_RANGING_COUNT);
    if (ranging_overwritten_count > 0) {
      LOG_INF("Overwrite rate: %.1f%% (%d/%d)",
              (double)ranging_overwritten_count / TEST_RANGING_COUNT * 100.0,
              ranging_overwritten_count, TEST_RANGING_COUNT);
    }

    // 禁用信道探测
    params.enable = 0;
    bt_le_cs_procedure_enable(connection, &params);
    bt_cs_state = BT_CS_STATE_DISABLED;

    // 关闭CS指示灯
    led_off(0);

    // 强制刷新缓冲区
    LOG_INF("Forcing buffer flush...");
    uint32_t final_buffer_count = flash_buffer_get_count();
    LOG_DBG("data count before flush: %d", final_buffer_count);
    k_work_schedule(&flash_write_work, K_NO_WAIT);
    LOG_INF("Buffer flush enabled");
  }
#endif

  // 获取结束的时钟周期
  end_cycles = k_cycle_get_32();

  // 计算消耗的周期数
  elapsed_cycles = end_cycles - start_cycles;

  // 将周期转换为纳秒
  elapsed_ns = k_cyc_to_ns_floor64(elapsed_cycles);

  // printk("Cb exec : %llu ns\n", elapsed_ns);
}

// 每次测距子事件结束之后，得到了测距结果，会进入这个回调
static void subevent_result_cb(struct bt_conn *conn,
                               struct bt_conn_le_cs_subevent_result *result) {
  if (dropped_ranging_counter == result->header.procedure_counter) {
    return;
  }

  if (most_recent_local_ranging_counter !=
      bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
    int sem_state = k_sem_take(&sem_local_steps, K_NO_WAIT);

    if (sem_state < 0) {
      dropped_ranging_counter = result->header.procedure_counter;
      LOG_DBG(
          "Dropped subevent results due to unfinished ranging data request.");
      return;
    }

    most_recent_local_ranging_counter =
        bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
  }

  if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
    /* The steps from this subevent will not be used. */
  } else if (result->step_data_buf) {
    if (result->step_data_buf->len <=
        net_buf_simple_tailroom(&latest_local_steps)) {
      uint16_t len = result->step_data_buf->len;
      uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

      net_buf_simple_add_mem(&latest_local_steps, step_data, len);
    } else {
      LOG_ERR("Not enough memory to store step data. (%d > %d)",
              latest_local_steps.len + result->step_data_buf->len,
              latest_local_steps.size);
      net_buf_simple_reset(&latest_local_steps);
      dropped_ranging_counter = result->header.procedure_counter;
      return;
    }
  }

  dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

  if (result->header.procedure_done_status ==
      BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
    most_recent_local_ranging_counter =
        bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
  } else if (result->header.procedure_done_status ==
             BT_CONN_LE_CS_PROCEDURE_ABORTED) {
    LOG_WRN("Procedure %u aborted", result->header.procedure_counter);
    net_buf_simple_reset(&latest_local_steps);
    k_sem_give(&sem_local_steps);
  }
}

static void ranging_data_ready_cb(struct bt_conn *conn,
                                  uint16_t ranging_counter) {
  LOG_DBG("Ranging data ready %i", ranging_counter);

  if (ranging_counter == most_recent_local_ranging_counter) {
    int err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps,
                                              ranging_counter,
                                              ranging_data_get_complete_cb);
    if (err) {
      LOG_ERR("Get ranging data failed (err %d)", err);
      net_buf_simple_reset(&latest_local_steps);
      net_buf_simple_reset(&latest_peer_steps);
      k_sem_give(&sem_local_steps);
    }
  }
}

static void ranging_data_overwritten_cb(struct bt_conn *conn,
                                        uint16_t ranging_counter) {
#if TEST_RANGING_ENABLED
  ranging_overwritten_count++;
  // LOG_INF("OW! %i (total overwritten: %d)", ranging_counter,
  // ranging_overwritten_count);

  // 如果覆盖频率过高，给出警告
  // if (ranging_overwritten_count > 0 && ranging_count > 0) {
  //   float overwrite_rate = (float)ranging_overwritten_count / ranging_count *
  //   100.0f; if (overwrite_rate > 20.0f) {  // 覆盖率超过20%
  //     LOG_WRN("High overwrite rate: %.1f%% (%d/%d) - Consider reducing
  //     ranging frequency!",
  //             (double)overwrite_rate, ranging_overwritten_count,
  //             ranging_count);
  //   }
  // }
#endif
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params) {
  if (err) {
    LOG_ERR("MTU exchange failed (err %d)", err);
    return;
  }

  LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
  k_sem_give(&sem_mtu_exchange_done);
}

// 在回调中，完成对服务句柄的分配和初始化。
// 使用信号量 sem_discovery_done 通知主线程服务发现成功。
static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context) {
  int err;

  LOG_INF("The discovery procedure succeeded");

  struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

  bt_gatt_dm_data_print(dm);

  err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
  if (err) {
    LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
  }

  err = bt_gatt_dm_data_release(dm);
  if (err) {
    LOG_ERR("Could not release the discovery data (err %d)", err);
  }

  k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn,
                                           void *context) {
  LOG_INF("The service could not be found during the discovery, disconnecting");
  // 这是用于断开连接的代码
  bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err,
                                     void *context) {
  LOG_INF("The discovery procedure failed (err %d)", err);
  bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_service_not_found_cb,
    .error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
            bt_security_err_to_str(err));
    return;
  }

  LOG_INF("Security changed: %s level %u", addr, level);
  k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param) {
  /* Ignore peer parameter preferences. */
  return false;
}

// 连接成功回调函数。扫描到符合条件的设备后，程序会自动尝试连接。如果连接成功，保存连接对象到全局变量
// connection，并通过信号量 sem_connected 通知主线程
static void connected_cb(struct bt_conn *conn, uint8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];

  (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Connected to %s (err 0x%02X)", addr, err);

  if (err) {
    bt_conn_unref(conn);
    connection = NULL;
    return;
  }

  connection = bt_conn_ref(conn);

  k_sem_give(&sem_connected);
  LOG_INF("sem_connected count after conncb: %d",
          k_sem_count_get(&sem_connected));
  LOG_INF("Now connected, scanning stopped");

  dk_set_led_on(CON_STATUS_LED);

  // 检查连接间隔参数
  struct bt_conn_info info = {0};
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return;
	}

	LOG_INF("Conn. interval is %u units", info.le.interval);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
  LOG_INF("Disconnected (reason 0x%02X)", reason);

  // 释放连接对象
  if (connection) {
    bt_conn_unref(connection);
    connection = NULL;
  }
  // 更新 LED 状态，指示蓝牙已断开
  dk_set_led_off(CON_STATUS_LED); // 关闭连接状态指示灯
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
                                   struct bt_conn_le_cs_capabilities *params) {
  ARG_UNUSED(conn);
  ARG_UNUSED(params);

  if (status == BT_HCI_ERR_SUCCESS) {
    LOG_INF("CS capability exchange completed.");
    k_sem_give(&sem_remote_capabilities_obtained);
  } else {
    LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
  }
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
                             struct bt_conn_le_cs_config *config) {
  ARG_UNUSED(conn);

  if (status == BT_HCI_ERR_SUCCESS) {
    LOG_INF("CS config creation complete. ID: %d", config->id);
    k_sem_give(&sem_config_created);
  } else {
    LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
  }
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status) {
  ARG_UNUSED(conn);

  if (status == BT_HCI_ERR_SUCCESS) {
    LOG_INF("CS security enabled.");
    k_sem_give(&sem_cs_security_enabled);
  } else {
    LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
  }
}

static void
procedure_enable_cb(struct bt_conn *conn, uint8_t status,
                    struct bt_conn_le_cs_procedure_enable_complete *params) {
  ARG_UNUSED(conn);

  if (status == BT_HCI_ERR_SUCCESS) {
    if (params->state == 1) {
      LOG_INF("CS procedures enabled:\n"
              " - config ID: %u\n"
              " - antenna configuration index: %u\n"
              " - TX power: %d dbm\n"
              " - subevent length: %u us\n"
              " - subevents per event: %u\n"
              " - subevent interval: %u\n"
              " - event interval: %u\n"
              " - procedure interval: %u\n"
              " - procedure count: %u\n"
              " - maximum procedure length: %u",
              params->config_id, params->tone_antenna_config_selection,
              params->selected_tx_power, params->subevent_len,
              params->subevents_per_event, params->subevent_interval,
              params->event_interval, params->procedure_interval,
              params->procedure_count, params->max_procedure_len);
    } else {
      LOG_INF("CS procedures disabled.");
    }
  } else {
    LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
  }
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

  LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info) {
  int err;

  LOG_INF("Connecting failed, restarting scanning");

  err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
  if (err) {
    LOG_ERR("Failed to restart scanning (err %i)", err);
    return;
  }
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn) {
  LOG_INF("Connecting");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error,
                scan_connecting);

static int scan_init(void) {
  int err;

  struct bt_scan_init_param param = {.scan_param = NULL,
                                     .conn_param = BT_LE_CONN_PARAM_DEFAULT,
                                     .connect_if_match = 1};

  bt_scan_init(&param);
  bt_scan_cb_register(&scan_cb);
  // 过滤了具有ranging service 的设备
  err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
  if (err) {
    LOG_ERR("Scanning filters cannot be set (err %d)", err);
    return err;
  }

  err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
  if (err) {
    LOG_ERR("Filters cannot be turned on (err %d)", err);
    return err;
  }

  return 0;
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .le_param_req = le_param_req,
    .security_changed = security_changed,
    .le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
    .le_cs_config_complete = config_create_cb,
    .le_cs_security_enable_complete = security_enable_cb,
    .le_cs_procedure_enable_complete = procedure_enable_cb,
    .le_cs_subevent_data_available = subevent_result_cb,
};

// 工作队列任务处理函数。按键0负责启动蓝牙测距
static void button0_work_handler(struct k_work *work) {
  if (!gpio_pin_get(BUTTON0_PORT, BUTTON0_PIN)) { // 按键已释放
    LOG_INF("Button 0 released (debounced)");
    return;
  }

  // 启用CS前检查flash状态并自动处理
  LOG_INF("Preparing flash for new test session...");
  int flash_check_result = flash_check_and_suggest_erase();

  if (flash_check_result == -1) { // 检查失败
    LOG_ERR("Flash status check failed, aborting CS start");
    return;
  }

  // 智能设置flash写入起始位置
  flash_smart_set_start_position();

  LOG_INF("Current write mode: %s",
          FLASH_WRITE_MODE == FLASH_WRITE_MODE_SINGLE ? "SINGLE" : "BATCH");

#if TEST_RANGING_ENABLED
  // 重置测距计数器
  ranging_count = 0;
  ranging_success_count = 0;
  ranging_error_count = 0;
  ranging_overwritten_count = 0;
  LOG_INF("Button 0 pressed: Starting test with %d ranging measurements",
          TEST_RANGING_COUNT);
  LOG_INF(
      "Counters reset - attempts: %d, success: %d, errors: %d, overwritten: %d",
      ranging_count, ranging_success_count, ranging_error_count,
      ranging_overwritten_count);
#endif
  params.enable = 1;
  bt_le_cs_procedure_enable(connection, &params);
  LOG_INF("Button 0 pressed: Start CS procedures");
  // 开启CS指示灯，同时这也意味着正在写入flash
  led_on(0);
  bt_cs_state = BT_CS_STATE_ENABLED; // 更新CS状态为已启用
}
// 按键1负责停止测距流程，
static void button1_work_handler(struct k_work *work) {
  if (!gpio_pin_get(BUTTON1_PORT, BUTTON1_PIN)) { // 按键已释放
    LOG_INF("Button 1 released (debounced)");
    return;
  }
  params.enable = 0;
  bt_le_cs_procedure_enable(connection, &params);
  LOG_INF("Button 1 pressed: Disable CS procedures");
  bt_cs_state = BT_CS_STATE_DISABLED; // 更新CS状态为已禁用
  // 关闭CS指示灯
  led_off(0);

#if TEST_RANGING_ENABLED
  // 重置测距计数器
  ranging_count = 0;
  LOG_INF("Ranging counter reset to 0");
#endif
}

static void _debug_print_report(store_cs_de_report_t *p_rep) {
  LOG_INF("=======log %llu=======", p_rep->report_index);
  LOG_INF("timestamp:%llu ms", p_rep->timestamp_ms);
  // LOG_INF("n_ap:%d", p_rep->report.n_ap);
}
// 按键2负责读取flash已存储的内容，且首先保证如果没停则不操作
static store_cs_de_report_t record; // 用于上传与擦除的结构体。与数据写入的分离

static void button2_work_handler(struct k_work *work) {
  if (!gpio_pin_get(BUTTON2_PORT, BUTTON2_PIN)) { // 按键已释放
    LOG_INF("Button 2 released (debounced)");
    return;
  }
  // 如果测距流程正在运行，则不读取flash
  if (bt_cs_state == BT_CS_STATE_ENABLED) {
    LOG_INF("CS procedures are enabled, cannot read from Flash");
    return;
  }
  LOG_INF("Button 2 pressed: Start reading from Flash");
  // 开启FLASH上传指示灯
  led_on(1);
  // 更新CS状态为已禁用且正在上传
  bt_cs_state = BT_CS_STATE_DISABLED_AND_UPLOADING;
  flash_state = FLASH_STATE_DATA_READING;
  // ========== 读取全部数据并打印 ==========
  uint64_t total = flash_ops_get_index();
  LOG_INF("total: %llu", total);
  int err = 0;
  // LOG_INF("Read %llu records from flash:", total);
  for (uint64_t i = 0; i < total; i++) {
    LOG_INF("Reading from Flash index %llu", i);
    err = flash_read_data_compact(flash_dev, i, &record,
                                  sizeof(store_cs_de_report_t));
    if (err) {
      LOG_ERR("Flash read error at %llu: %d", i, err);
      break;
    }
    LOG_INF("Read Flash data - Idx: %llu, Tstp: %llu ms", record.report_index,
            record.timestamp_ms);
    // print_store_cs_de_report(&record, 80);
    print_store_cs_de_report_basic(&record, 80);
    // _debug_print_report(&record);
  }
  LOG_INF("Flash read finished.");
  bt_cs_state = BT_CS_STATE_IDLE;
  flash_state = FLASH_STATE_IDLE;
  led_off(1);
}
// 按键3负责在上传后擦除数据，保证后续使用。无论是正在CS测距或正在上传时都不应该起效
// 强制擦除指示,如果任何时候连续按3次按键3，则强制擦除
#define FORCE_ERASE_THRESHOLD 3
#define FORCE_ERASE_TIMEOUT_MS 2000

static uint8_t force_erase_count = 0;
static int64_t last_force_erase_tick = 0;

static void button3_work_handler(struct k_work *work) {
  if (!gpio_pin_get(BUTTON3_PORT, BUTTON3_PIN)) { // 按键已释放
    LOG_INF("Button 3 released (debounced)");
    return;
  }
  int64_t now = k_uptime_get();
  // 如果距离上次按下超过超时时间，则重置计数
  if (now - last_force_erase_tick > FORCE_ERASE_TIMEOUT_MS) {
    force_erase_count = 0;
  }
  last_force_erase_tick = now;

  force_erase_count++;
  // 非强制擦除场景：只有在未上传、未测距、未达到3次才拦截
  if (force_erase_count < FORCE_ERASE_THRESHOLD) {
    if (bt_cs_state == BT_CS_STATE_DISABLED_AND_UPLOADING) {
      LOG_INF("CS procedures are disabled and uploading, cannot erase Flash");
      return;
    }
    if (bt_cs_state == BT_CS_STATE_ENABLED) {
      LOG_INF("CS procedures are enabled, cannot erase Flash");
      return;
    }
    LOG_INF("Button 3 pressed (%d/%d). Press %d times quickly to force erase.",
            force_erase_count, FORCE_ERASE_THRESHOLD, FORCE_ERASE_THRESHOLD);
    return;
  }
  // LOG_INF("Button 3 pressed: Erase Flash");
  // 满足3次，强制擦除
  LOG_INF("Force erase triggered after %d presses!", FORCE_ERASE_THRESHOLD);
  force_erase_count = 0; // 擦除后立即复位
  // 关闭FLASH上传指示灯
  led_off(1);
  bt_cs_state = BT_CS_STATE_DATA_ERASING;
  flash_state = FLASH_STATE_DATA_ERASING;
  //   flash_erase_data(); // 假设有一个 flash 擦除接口
  // ========== 擦除所有使用过的数据 ==========
  int err = 0;
  uint64_t flash_size = flash_ops_get_size();
  for (uint64_t i = 0; i < flash_size / SPI_FLASH_SECTOR_SIZE; i++) {
    led_on(1);
    err = flash_sector_needs_erase(i);
    if (err == 0) {
      LOG_INF("flash is empty, skip erase");
      break;
    } else if (err == -1) {
      LOG_ERR("try erase but flash read failed: %d", err);
      break;
    }
    LOG_INF("start earse flash %llu", i);
    err = flash_erase(flash_dev, i * SPI_FLASH_SECTOR_SIZE,
                      SPI_FLASH_SECTOR_SIZE);
    led_off(1);
  }

  flash_state = FLASH_STATE_IDLE;
  bt_cs_state = BT_CS_STATE_IDLE;
  LOG_INF("Force erase finished");
}

// Flash写入工作队列处理函数
// 单次flash写入的工作队列处理函数
static void flash_single_write_work_handler(struct k_work *work) {
  ARG_UNUSED(work);

  flash_state = FLASH_STATE_DATA_WRITING;

  LOG_DBG("Using single write mode");
  int err = flash_write_single_report(&temp_flash_data);
  if (err) {
    LOG_ERR("Single flash write failed: %d", err);
  } else {
    LOG_DBG("Single write successful - Idx: %llu, Time: %llu",
            temp_flash_data.report_index, temp_flash_data.timestamp_ms);
  }

  flash_state = FLASH_STATE_IDLE;
}

static void flash_write_work_handler(struct k_work *work) {
  ARG_UNUSED(work);

  flash_state = FLASH_STATE_DATA_WRITING;

  // 使用分片写入，避免长时间阻塞
  int err = flash_buffer_flush_chunked();
  if (err < 0) {
    LOG_ERR("Flash buffer flush error: %d", err);
    flash_state = FLASH_STATE_IDLE;
  } else if (err > 0) {
    // 还有更多数据需要写入，延迟后继续
    LOG_INF("Flash chunk completed, scheduling next chunk");
    k_work_schedule(&flash_write_work, K_MSEC(FLASH_WRITE_CHUNK_DELAY_MS));
  } else {
    // 所有数据写入完成
    LOG_INF("Flash buffer flush completed");
    flash_state = FLASH_STATE_IDLE;
  }
}
#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
// 定时刷新缓冲区的工作队列处理函数
static void flash_timer_work_handler(struct k_work *work) {
  ARG_UNUSED(work);

  // 检查缓冲区是否有数据需要刷新
  uint32_t buffer_count = flash_buffer_get_count();
  if (buffer_count > 0) {
    LOG_INF("Timer triggered flush, buffer count: %d", buffer_count);
    k_work_schedule(&flash_write_work, K_NO_WAIT);
  }

  // 重新提交定时工作，每10秒检查一次
  k_work_schedule(&flash_timer_work, K_SECONDS(10));
}
#endif
// 按键回调函数,仅用于提交队列
void button0_callback(void) { k_work_submit(&button0_work); }

void button1_callback(void) { k_work_submit(&button1_work); }

void button2_callback(void) { k_work_submit(&button2_work); }

void button3_callback(void) { k_work_submit(&button3_work); }

int main(void) {
  int err;

  //   初始化按键和LED
  button_led_init();
  // 初始化flash
  flash_init(flash_dev);
  // 初始化工作队列任务
  k_work_init(&button0_work, button0_work_handler);
  k_work_init(&button1_work, button1_work_handler);
  k_work_init(&button2_work, button2_work_handler);
  k_work_init(&button3_work, button3_work_handler);
  k_work_init(&flash_single_write_work, flash_single_write_work_handler);
  k_work_init_delayable(&flash_write_work, flash_write_work_handler);
#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
  k_work_init_delayable(&flash_timer_work, flash_timer_work_handler);
#endif

  // 注册按键回调
  button_register_callback(0, button0_callback);
  button_register_callback(1, button1_callback);
  button_register_callback(2, button2_callback);
  button_register_callback(3, button3_callback);

  // 运行flash性能测试，强制使用前全部擦除
  // flash_performance_test();
  flash_check_and_suggest_erase();

  LOG_INF("Starting Channel Sounding Initiator Sample");

  dk_leds_init();
  // 初始化蓝牙子系统
  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 0;
  }
  // 蓝牙扫描与链接，添加一个过滤器，只关注有ranging service 的设备
  err = scan_init();
  if (err) {
    LOG_ERR("Scan init failed (err %d)", err);
    return 0;
  }
  // 启动被动扫描（BT_SCAN_TYPE_SCAN_PASSIVE），不会发送扫描请求。
  // 但扫描到之后会直接尝试连接，一旦成功则进入connected_cb,它会给一个成功信号量sem_connected
  err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
  if (err) {
    LOG_ERR("Scanning failed to start (err %i)", err);
    return 0;
  }
  LOG_INF("Scanning started");
  bt_cs_state = BT_CS_STATE_SCANNING;

  k_sem_take(&sem_connected, K_FOREVER);

  err = bt_conn_set_security(connection, BT_SECURITY_L2);
  if (err) {
    LOG_ERR("Failed to encrypt connection (err %d)", err);
    return 0;
  }

  k_sem_take(&sem_security, K_FOREVER);

  static struct bt_gatt_exchange_params mtu_exchange_params = {
      .func = mtu_exchange_cb};

  bt_gatt_exchange_mtu(connection, &mtu_exchange_params);

  k_sem_take(&sem_mtu_exchange_done, K_FOREVER);

  // （连接后）开始发现目标设备connection上的 GATT 服务。如果有ranging
  // service，则进入discovery_completed_cb
  err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb,
                         NULL);
  if (err) {
    LOG_ERR("Discovery failed (err %d)", err);
    return 0;
  }

  k_sem_take(&sem_discovery_done, K_FOREVER);

  // 此时基本上进入更细致的信道探测功能的配置了，设备已经连接，服务成功发现。
  // 设置默认的信道探测参数：
  // enable_initiator_role = true：启用发起者角色。
  // enable_reflector_role = false：禁用反射器角色。
  // 其他参数包括天线选择、最大功率等。
  // const struct bt_le_cs_set_default_settings_param default_settings = {
  //     .enable_initiator_role = true,
  //     .enable_reflector_role = false,
  //     .cs_sync_antenna_selection =
  //     BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE, .max_tx_power =
  //     BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
  // };

  err = bt_le_cs_set_default_settings(connection, &default_settings);
  if (err) {
    LOG_ERR("Failed to configure default CS settings (err %d)", err);
    return 0;
  }

  err = bt_ras_rreq_rd_overwritten_subscribe(connection,
                                             ranging_data_overwritten_cb);
  if (err) {
    LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
    return 0;
  }

  err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
  if (err) {
    LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
    return 0;
  }

  err = bt_ras_rreq_on_demand_rd_subscribe(connection);
  if (err) {
    LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
    return 0;
  }

  err = bt_ras_rreq_cp_subscribe(connection);
  if (err) {
    LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
    return 0;
  }

  err = bt_le_cs_read_remote_supported_capabilities(connection);
  if (err) {
    LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
    return 0;
  }

  k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

  // struct bt_le_cs_create_config_params config_params = {
  //     .id = CS_CONFIG_ID,
  //     .main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
  //     .sub_mode_type = BT_CONN_LE_CS_SUB_MODE_1,
  //     .min_main_mode_steps = 2,
  //     .max_main_mode_steps = 5,
  //     .main_mode_repetition = 0,
  //     .mode_0_steps = NUM_MODE_0_STEPS,
  //     .role = BT_CONN_LE_CS_ROLE_INITIATOR,
  //     .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
  //     .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
  //     .channel_map_repetition = 3,
  //     .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
  //     .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
  //     .ch3c_jump = 2,
  // };

  bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

  err = bt_le_cs_create_config(connection, &config_params,
                               BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
  if (err) {
    LOG_ERR("Failed to create CS config (err %d)", err);
    return 0;
  }

  k_sem_take(&sem_config_created, K_FOREVER);
  // 启用信道探测的安全性
  err = bt_le_cs_security_enable(connection);
  if (err) {
    LOG_ERR("Failed to start CS Security (err %d)", err);
    return 0;
  }
  k_sem_take(&sem_cs_security_enabled, K_FOREVER);

  // 测距参数：interval应该是最核心的一个。假设一个procedure是50ms，那么采样间隔就是间隔N个50ms
  // const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
  //     .config_id = CS_CONFIG_ID,
  //     .max_procedure_len = 1000,
  //     .min_procedure_interval = 10,
  //     .max_procedure_interval = 10,
  //     .max_procedure_count = 0,
  //     .min_subevent_len = 60000,
  //     .max_subevent_len = 60000,
  //     .tone_antenna_config_selection =
  //         BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
  //     .phy = BT_LE_CS_PROCEDURE_PHY_1M,
  //     .tx_power_delta = 0x80,
  //     .preferred_peer_antenna =
  //     BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1, .snr_control_initiator =
  //     BT_LE_CS_SNR_CONTROL_NOT_USED, .snr_control_reflector =
  //     BT_LE_CS_SNR_CONTROL_NOT_USED,
  // };

  err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
  if (err) {
    LOG_ERR("Failed to set procedure parameters (err %d)", err);
    return 0;
  }

  // struct bt_le_cs_procedure_enable_param params = {
  //     .config_id = CS_CONFIG_ID,
  //     .enable = 1,
  // };
  // 使能信道探测，开始测距
  err = bt_le_cs_procedure_enable(connection, &params);
  if (err) {
    LOG_ERR("Failed to enable CS procedures (err %d)", err);
    return 0;
  }
  //   这是第一次连接
  bt_cs_state = BT_CS_STATE_ENABLED;
  LOG_INF("!!!CS procedures are enabled !!!");
  LOG_INF("cs_de_report size: %d", sizeof(cs_de_report_t));
  LOG_INF("store_cs_de_report_t size: %d", sizeof(store_cs_de_report_t));
#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
  // 启动定时刷新缓冲区
  k_work_schedule(&flash_timer_work, K_SECONDS(10));
#endif
  // // 运行flash性能测试
  // flash_performance_test();

  while (true) {
    k_sleep(K_MSEC(1000));
#if NEED_LOG
    if (buffer_num_valid != 0 && bt_cs_state == BT_CS_STATE_ENABLED) {
      for (uint8_t ap = 0; ap < MAX_AP; ap++) {
        cs_de_dist_estimates_t distance_on_ap = get_distance(ap);

        LOG_INF("Distance estimates on antenna path %u: ifft: %f, "
                "phase_slope: %f, rtt: %f",
                ap, (double)distance_on_ap.ifft,
                (double)distance_on_ap.phase_slope, (double)distance_on_ap.rtt);
      }
    }
#endif
    if (bt_cs_state == BT_CS_STATE_ENABLED) {
#ifdef TEST_RANGING_ENABLED
      // LOG_INF("CS short test enabled - Progress: %d/%d ranging measurements",
      //         ranging_count, TEST_RANGING_COUNT);
#endif
    }
    // LOG_INF("Sleeping for a few seconds...");
  }

  return 0;
}
