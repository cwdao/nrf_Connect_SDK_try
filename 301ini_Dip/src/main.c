/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Direct IQ Pipeline
 *  @author Cheng Wang, cwang199@connect.hkust-gz.edu.cn;
 *  @date 2026-05-12
 *  @version 1.0
 *  @note This project is modified from Nordic Connect SDK Channel Sounding
 *        sample.
 *       Flash storage and key operations are referenced from taki_tooru@163.com
 */

#include <bluetooth/cs_de.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/gpio.h>
/* 以下头文件供 DIP_REPORT_BINARY_OUTPUT：设备绑定、uart_poll_out、LE 编解码、错误码 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
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

/* DIP 第一版：为 1 时在 main() 中跳过 RAS 订阅，仅走本地 HCI subevent 解析，避免 ranging 回调与
 * latest_local_steps 不同步。恢复旧测距链路时改为 0。不影响 legacy 源码保留与编译。 */
#ifndef APP_CS_DIP_BYPASS_RAS
#define APP_CS_DIP_BYPASS_RAS 1
#endif

/* DIP 报告打印：0 仅一行 pc（减轻 UART/deferred log，便于对比 abort 是否减轻）；1 为多信道 IQ 详版 */
#ifndef DIP_REPORT_LOG_VERBOSE
#define DIP_REPORT_LOG_VERBOSE 1
#endif

/*
 * DIP 二进制 UART 输出开关（与 DIP_REPORT_LOG_VERBOSE 互斥，见 dip_parse_local_iq_from_subevent）：
 *   1 — 解析完成后调用 dip_output_local_report_binary()，经 console UART 发 v2 二进制帧；
 *   0 — 保持原有 LOG_INF 文本输出（详版或多信道 IQ 行）。
 * 测试二进制时建议同时将 DIP_REPORT_LOG_VERBOSE 置 0，并降低全局日志等级，避免 ASCII 与二进制混流。
 * 协议与 PC 解析见 doc/DIP_binary_protocol.md、doc/DIP_binary_pc_parser.md。
 */
#ifndef DIP_REPORT_BINARY_OUTPUT
#define DIP_REPORT_BINARY_OUTPUT 1
#endif

/*
 * DIP 二进制 UART 发送路径：
 *   1 — 回调内仅打包并 k_msgq_put(K_NO_WAIT)，由 dip_uart_tx 线程 uart_poll_out（推荐，默认）；
 *   0 — 在 BT RX WQ 内直接 uart_poll_out，实现最简单，可能拉长回调占用时间。
 */
#ifndef DIP_BINARY_USE_THREAD
#define DIP_BINARY_USE_THREAD 1
#endif

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
    // .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
    .max_tx_power = 8,
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
// 功率差：-1 = 0xff, -2 = 0xfe, -3 = 0xfd,-4 = 0xfc,-5 = 0xfb, -6 = 0xfa,-7 =
// 0xf9,-8 = 0xf8
static const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
    .config_id = CS_CONFIG_ID,
    .max_procedure_len = 500,
    .min_procedure_interval = 1,
    .max_procedure_interval = 1,
    .max_procedure_count = 0,
    .min_subevent_len = 10000,
    .max_subevent_len = 40000, // 这个就是us
    .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
    .phy = BT_LE_CS_PROCEDURE_PHY_2M,
    .tx_power_delta = 0x80, // 0x80 means no power difference
    .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
    .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
    .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
};

// 启用测距参数，传给bt_le_cs_procedure_enable
static struct bt_le_cs_procedure_enable_param params = {
    .config_id = CS_CONFIG_ID,
    .enable = 1,
};

uint8_t valid_channels[] = {40, 41, 42, 43, 44, 45, 46, 47,
                            48, 49, 50, 51, 52, 53, 54, 55};

static void set_custom_channel_map(uint8_t channel_map[10]) {
  // 初始化所有信道为无效（全 0）
  memset(channel_map, 0x00, 10);

  // 定义启用的信道集合。例如：启用信道 2、3、10、11
  // uint8_t valid_channels[] =
  // {40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56}; uint8_t
  // valid_channels[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

  // 遍历启用的信道集合，将对应 bit 设置为有效
  for (size_t i = 0; i < sizeof(valid_channels) / sizeof(valid_channels[0]);
       i++) {
    uint8_t channel = valid_channels[i];

    // 检查信道是否在有效范围
    if (channel > 78) {
      printk("Invalid channel: %d\n", channel);
      continue;
    }

    // 设置信道为有效
    channel_map[channel / 8] |= (1 << (channel % 8));
  }

  // 如果有协议限制（如禁用信道
  // 0、1、23、24、25、77、78），可以额外禁用这些信道
  channel_map[0] &= ~((1 << 0) | (1 << 1));            // 禁用信道 0 和 1
  channel_map[2] &= ~((1 << 7) | (1 << 6) | (1 << 5)); // 禁用信道 23、24、25
  channel_map[9] &= ~((1 << 5) | (1 << 6));            // 禁用信道 77、78
}

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
  // uint64_t timestamp_ms_de_c_1 = k_uptime_get();

  cs_de_quality_t quality = cs_de_calc(&cs_de_report);

  // uint64_t timestamp_ms_de_c_2 = k_uptime_get();
  // uint64_t timestamp_ms_de_c = timestamp_ms_de_c_2 - timestamp_ms_de_c_1;

  //  print time cost of cs_de_calc
  // LOG_INF("ct: %llu", timestamp_ms_de_c);

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

#if ENABLE_DIRECT_PRINT
    // 直接打印模式 - 测距完成后直接打印到串口，不写入flash
    // 相当于直接调用button2的打印输出功能
    print_store_cs_de_report_basic(&temp_flash_data, 80);
#elif FLASH_WRITE_MODE == FLASH_WRITE_MODE_SINGLE
    // 单个写入模式 - 使用k_work异步写入flash
    // LOG_DBG("Using single write mode with k_work");
    k_work_submit(&flash_single_write_work);
#else
    // 批量写入模式 - 使用环形缓冲区
    LOG_DBG("Using batch write mode");
    int err = flash_buffer_put(&temp_flash_data, sizeof(store_cs_de_report_t));
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

/* =============================================================================
 * Direct IQ Pipeline (DIP)：从 HCI LE CS Subevent Result 的 step_data 中直接取本地 PCT，
 * 调用栈提供的 bt_le_cs_parse_pct() 转为 IQ 并打印。不依赖 RAS / peer steps，也不调用
 * cs_de_populate_report / cs_de_calc。
 *
 * tone/AP 索引方式与 Nordic cs_de.c::extract_pcts() 一致：按 tone_index 查 permutation
 * 表得到 antenna_path，再读 tone_info[antenna_path]。
 *
 * 注意：bt_le_cs_step_data_parse() 会消耗缓冲区，因此必须先 memcpy 到本地缓存再解析。
 *
 * 信道编号：每个 step 的 step->channel 为 HCI 信道索引；与 cs_de 相同地定义
 * fft 下标 ch = hci_ch - CHANNEL_INDEX_OFFSET（见 DIP_CS_CHANNEL_INDEX_OFFSET）。
 * 单次 subevent 内通常有十余至数十个 step（你提到的约 15～72 个信道），先在 ctx 中
 * 按 (ap, ch) 聚合，再按 print_report_basic 风格分块（每行 8 个信道）打印，格式为仅本地的
 * ch(i,q)，不含对端项以缩短串口输出。
 * ============================================================================= */

/** 与 cs_de 中 CHANNEL_INDEX_OFFSET 一致，IQ[ch] 的 ch 与 cs_de_data_parse 中使用的 fft 下标一致 */
#define DIP_CS_CHANNEL_INDEX_OFFSET 2U

/**
 * 与 cs_de.c 中 NUM_CHANNELS(75) 对齐的 IQ 下标上界；规范下单次报告常见约 15～72 个有效信道，
 * 数组留出完整 fft 格点便于与 print_report_basic 的 IQ[0..max] 窗口一致。
 */
#define DIP_MAX_FFT_CHANNELS 75

/** 每个 (天线, fft 信道) 上一组本地 IQ；valid 表示本报告内该格点有 HIGH quality 的 PCT */
struct dip_ch_sample {
  int16_t i;
  int16_t q;
  bool valid;
};

/** 传给 bt_le_cs_step_data_parse 的解析上下文：解析结束后一次性多信道打印 fill ch[][] */
struct dip_step_parse_ctx {
  /** procedure_counter：本条 subevent 所属的 CS procedure 计数 */
  uint16_t procedure_counter;
  /** 本 subevent HCI 头中的天线路径数，对应 cs_de_report::n_ap / extract_pcts 循环上界 */
  uint8_t n_ap;
  struct dip_ch_sample ch[CONFIG_BT_RAS_MAX_ANTENNA_PATHS][DIP_MAX_FFT_CHANNELS];
};

/*
 * DIP step_data 拷贝缓冲 + 解析上下文必须放在 BSS，不可放在 dip_parse_local_iq_from_subevent()
 * 的栈上：该函数经由 .le_cs_subevent_data_available 在「BT RX WQ」里调用，该线程栈很小
 *（通常约 1k～2k），而 LOCAL_PROCEDURE_MEM  alone 即可达数 KB，叠加大 ctx 与调用链后必然
 * 触发 Usage Fault: Stack overflow。单连接下 subevent 串行投递，静态工作区可安全复用
 * （与同文件 legacy 的 static cs_de_report 同理）。
 */
static uint8_t dip_step_data_copy[LOCAL_PROCEDURE_MEM];
static struct dip_step_parse_ctx dip_parse_work_ctx;

#if DIP_REPORT_BINARY_OUTPUT

/* =============================================================================
 * DIP 二进制 UART 输出（协议 v2：single AP + channel bitmap + int16 IQ）
 *
 * 设计目标：在保留 dip_local_step_iq_cb / ctx->ch[ap][ch] 解析结果的前提下，
 * 用紧凑定长头 + 位图描述有效 fft 信道，再顺序附带 IQ，便于 PC 端高速解析。
 *
 * 线上字节序：除单字节字段外均为 little-endian（与 ARM / x86 PC 一致）。
 *
 * 完整帧布局（按发送顺序）：
 *   [固定同步/类型 4B]
 *     sync1=0x55, sync2=0xAA, version=0x01, type=0x01(DIP IQ)
 *   [payload_len 2B LE]
 *     从 procedure_counter 起至 IQ 区末尾的字节数（不含 sync..type、payload_len 自身、CRC）
 *   [元数据 + 位图 16B]
 *     procedure_counter(2), ap(1), iq_format(1), channel_count(1), reserved(1),
 *     channel_bitmap[10]
 *   [IQ 变长 4*channel_count B]
 *     按 fft ch 升序，仅对 bitmap 对应位为 1 的信道各写 int16 i、int16 q
 *   [crc16 2B LE]
 *     CRC16-CCITT(0x1021)，初值 0xFFFF，覆盖从 sync1 到 IQ 区最后一字节
 *
 * 位图编码：channel_bitmap[k] 的 bit (ch%8) 表示 fft 下标 ch 是否有效；
 *   ch 与 dip_parse 中 ctx->ch[ap][ch]、HCI 信道关系：ch = hci_ch - DIP_CS_CHANNEL_INDEX_OFFSET。
 *   共 DIP_MAX_FFT_CHANNELS(75) 格点，需 ceil(75/8)=10 字节，最高 5 bit 保留未用。
 *
 * 当前限制：优先 n_ap=1；若 n_ap>1 仅发 ap=0 并 LOG_WRN，不拆多帧。
 *
 * --- 发送架构（DIP_BINARY_USE_THREAD，默认，已板级验证）---
 *   Producer：subevent_result_dip_cb → dip_parse_local_iq_from_subevent
 *            → dip_output_local_report_binary → dip_enqueue_local_report_binary
 *            → dip_bin_build_frame（写入 dip_bin_msg_scratch）
 *            → k_msgq_put(K_NO_WAIT)；满则 LOG_WRN 丢帧，不阻塞 BT RX WQ
 *   Consumer：dip_uart_tx 线程 → k_msgq_get(K_FOREVER) → dip_uart_send_bytes
 *   设计参考：快速二进制输出 2.0.md 阶段 2（msgq + 独立线程）；帧格式为 v2 bitmap 而非 sample-list。
 *
 * --- 阻塞回退（DIP_BINARY_USE_THREAD=0）---
 *   同上 Producer 直至 dip_bin_build_frame，随后在回调线程内直接 dip_uart_send_bytes。
 *
 * --- 固件函数一览 ---
 *   dip_bin_fill_channel_bitmap / dip_bin_pack_iq_payload — 位图与 IQ 载荷
 *   dip_bin_build_frame          — 组帧 + CRC（两路径共用）
 *   dip_crc16_ccitt              — PC 端须使用相同算法与覆盖范围
 *   dip_uart_send_bytes          — uart_poll_out 逐字节（消费者线程或阻塞路径）
 *
 * 内存：dip_bin_frame_buf、dip_bin_msg_scratch、msgq 消息均放 BSS/static，
 *       禁止在 BT RX WQ 栈上分配 dip_bin_msg（约 402B）或大帧数组。
 * ============================================================================= */

#if DIP_BINARY_USE_THREAD
/** 待发帧队列深度：可缓存 8 条完整帧；突发超过 8 条/消费速度则触发 drop */
#define DIP_BIN_MSGQ_DEPTH 8U
/** 发送线程栈：含 dip_bin_msg 局部副本（约 400B）+ 调用链余量 */
#define DIP_UART_TX_THREAD_STACK_SIZE 1024
/**
 * 发送线程优先级（数值越小越高，具体语义依 Zephyr 配置）。
 * 低于 BT 栈线程，避免 UART 发送长期抢占蓝牙；高于 idle 以保证及时 drain 队列。
 */
#define DIP_UART_TX_THREAD_PRIORITY 7
#endif

/** 帧同步字节 1，用于 PC 端串口流中定位帧起点 */
#define DIP_BIN_SYNC1 0x55U
/** 帧同步字节 2，与 sync1 组合降低误同步概率 */
#define DIP_BIN_SYNC2 0xAAU
/** 协议版本号；字段含义变更时递增，PC 端据此分支解析 */
#define DIP_BIN_VERSION 0x01U
/** 帧类型：0x01 表示本条为 DIP 本地 IQ 报告（后续可扩展其它 type） */
#define DIP_BIN_TYPE_IQ 0x01U
/** iq_format 字段：0 表示 IQ 载荷为 int16 有符号、小端 */
#define DIP_BIN_IQ_FORMAT_INT16 0U
/** 信道位图字节数：覆盖 75 个 fft 信道需 10 字节（80 bit，高 5 bit 未使用） */
#define DIP_BIN_CHANNEL_BITMAP_BYTES 10U
/**
 * 单帧最大长度上界：头(22) + 全信道 IQ(75*4=300) + CRC(2) ≈ 324，取 400 留余量。
 * 须 ≥ sizeof(dip_bin_header) + DIP_MAX_FFT_CHANNELS*4 + 2。
 */
#define DIP_BIN_MAX_FRAME_SIZE 400U

/**
 * 二进制帧固定头（packed，无编译器填充）。
 * 其后紧接变长 IQ 区，再跟 2 字节 CRC；IQ 区不嵌入本结构体。
 */
struct __packed dip_bin_header {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t version;
  uint8_t type;
  /** 载荷长度：procedure_counter(2) + ap..reserved(4) + bitmap(10) + IQ 字节数 */
  uint16_t payload_len;
  /** 与 dip_step_parse_ctx::procedure_counter 一致，标识所属 CS procedure */
  uint16_t procedure_counter;
  /** 天线路径索引；当前实现固定发 0（单 AP 场景） */
  uint8_t ap;
  /** IQ 数值格式，见 DIP_BIN_IQ_FORMAT_INT16 */
  uint8_t iq_format;
  /** 本帧有效 fft 信道数，等于 channel_bitmap 中置 1 的位数 */
  uint8_t channel_count;
  /** 保留，填 0，供后续扩展（如缩放因子、质量标志等） */
  uint8_t reserved;
  /** 位图：bit ch 为 1 表示该 fft 信道在 IQ 区中占一对 (i,q) */
  uint8_t channel_bitmap[DIP_BIN_CHANNEL_BITMAP_BYTES];
};

/** 与 Zephyr console 共用 UART，二进制与 LOG_* 可能混流，测试时请控制日志量 */
static const struct device *dip_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
/** 整帧组装缓冲；静态分配，供 dip_bin_build_frame() / 阻塞发送路径复用 */
static uint8_t dip_bin_frame_buf[DIP_BIN_MAX_FRAME_SIZE];

#if DIP_BINARY_USE_THREAD
/** 入队消息：已打包好的完整帧，发送线程只负责 uart_poll_out */
struct dip_bin_msg {
  uint16_t len;
  uint8_t data[DIP_BIN_MAX_FRAME_SIZE];
};

/** 深度 DIP_BIN_MSGQ_DEPTH；每条消息 sizeof(dip_bin_msg)；4 字节对齐满足 ARM */
K_MSGQ_DEFINE(dip_bin_msgq, sizeof(struct dip_bin_msg), DIP_BIN_MSGQ_DEPTH, 4);
K_THREAD_STACK_DEFINE(dip_uart_tx_stack, DIP_UART_TX_THREAD_STACK_SIZE);
static struct k_thread dip_uart_tx_thread_data;
/**
 * Producer 侧打包缓冲区（单连接 subevent 串行，可安全复用）。
 * k_msgq_put 会把本结构体 **拷贝** 进队列，故入队后 scratch 可立即用于下一帧。
 */
static struct dip_bin_msg dip_bin_msg_scratch;
#endif /* DIP_BINARY_USE_THREAD */

/**
 * 经 UART 逐字节阻塞发送。
 * 线程版在 dip_uart_tx 中调用；阻塞版在 BT RX WQ 中调用。
 * uart_poll_out 会等待 TX 寄存器空，属阻塞语义，但已移出蓝牙回调（线程版）。
 * @param data 待发送缓冲区首地址
 * @param len  字节数；为 0 或设备未就绪时直接返回
 */
static void dip_uart_send_bytes(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0U) {
    return;
  }

  if (!device_is_ready(dip_uart)) {
    return;
  }

  for (size_t i = 0; i < len; i++) {
    uart_poll_out(dip_uart, data[i]);
  }
}

/**
 * CRC16-CCITT（多项式 0x1021，初值 0xFFFF）。
 * PC 端校验范围须与本函数一致：从帧首 sync1 到 IQ 区最后一字节（不含 CRC 域）。
 *
 * @param data 参与校验的数据
 * @param len  字节长度
 * @return 16 位 CRC，由调用方 sys_put_le16 写入帧尾
 */
static uint16_t dip_crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;

  if (data == NULL) {
    return crc;
  }

  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

/**
 * 根据 ctx->ch[ap][ch].valid 填充信道位图并统计有效信道数。
 * 数据来源与 dip_local_step_iq_cb 写入的格点一致，不二次解析 HCI。
 *
 * @param ctx           已完成 bt_le_cs_step_data_parse 的解析上下文
 * @param ap            天线路径索引（当前调用方固定传 0）
 * @param bitmap        输出至少 DIP_BIN_CHANNEL_BITMAP_BYTES 字节，调用前内容可被覆盖
 * @param channel_count 输出置 1 的位数，与后续 IQ 区样本对数一致
 */
static void dip_bin_fill_channel_bitmap(const struct dip_step_parse_ctx *ctx, uint8_t ap,
                                        uint8_t *bitmap, uint8_t *channel_count)
{
  memset(bitmap, 0, DIP_BIN_CHANNEL_BITMAP_BYTES);

  if (ctx == NULL || bitmap == NULL || channel_count == NULL) {
    if (channel_count != NULL) {
      *channel_count = 0U;
    }
    return;
  }

  uint8_t count = 0U;

  for (uint8_t ch = 0; ch < DIP_MAX_FFT_CHANNELS; ch++) {
    if (!ctx->ch[ap][ch].valid) {
      continue;
    }

    /* 小端位序：ch 0 对应 bitmap[0] 的 bit0，ch 7 对应 bit7，ch 8 对应 bitmap[1] 的 bit0 */
    bitmap[ch / 8U] |= (uint8_t)(1U << (ch % 8U));
    count++;
  }

  *channel_count = count;
}

/**
 * 将有效信道的 i/q 顺序写入 IQ 载荷区（int16 little-endian）。
 * 遍历顺序必须与位图生成时一致：ch 从 0 递增，仅处理 bitmap 中已置位的信道。
 *
 * @param ctx     解析上下文
 * @param ap      天线路径
 * @param bitmap  由 dip_bin_fill_channel_bitmap() 生成
 * @param out     写入起点（通常为 dip_bin_frame_buf + sizeof(dip_bin_header)）
 * @param out_max 允许写入的最大字节数（预留 CRC 空间由调用方扣除）
 * @return 实际写入字节数，正常应为 channel_count * 4
 */
static size_t dip_bin_pack_iq_payload(const struct dip_step_parse_ctx *ctx, uint8_t ap,
                                      const uint8_t *bitmap, uint8_t *out, size_t out_max)
{
  size_t offset = 0U;

  if (ctx == NULL || bitmap == NULL || out == NULL) {
    return 0U;
  }

  for (uint8_t ch = 0; ch < DIP_MAX_FFT_CHANNELS; ch++) {
    if ((bitmap[ch / 8U] & (uint8_t)(1U << (ch % 8U))) == 0U) {
      continue;
    }

    if (offset + sizeof(int16_t) + sizeof(int16_t) > out_max) {
      break;
    }

    /* 强制转为 uint16_t 再写入，保留 int16 负值的位模式 */
    sys_put_le16((uint16_t)ctx->ch[ap][ch].i, &out[offset]);
    offset += sizeof(int16_t);
    sys_put_le16((uint16_t)ctx->ch[ap][ch].q, &out[offset]);
    offset += sizeof(int16_t);
  }

  return offset;
}

/**
 * 将 ctx 打包为完整二进制帧写入 frame（线程版与阻塞版共用）。
 *
 * 组帧顺序：
 *   1. 从 ctx->ch[ap][ch].valid 生成 bitmap 与 channel_count
 *   2. 填写 dip_bin_header（含 payload_len，先按 iq_size 计算）
 *   3. 在 frame[hdr_size..] 写入 IQ 载荷
 *   4. 对 [0 .. hdr_size+iq_size) 计算 CRC 并写入帧尾
 *
 * @param ctx       已完成 step_data 解析的上下文
 * @param frame     输出缓冲区
 * @param frame_cap frame 容量（字节）
 * @param out_len   成功时输出整帧长度（含 CRC）
 * @return 0 成功；-EINVAL 参数无效；-ENOSPC 帧超长；-EIO IQ 打包长度不一致
 */
static int dip_bin_build_frame(const struct dip_step_parse_ctx *ctx, uint8_t *frame,
                               size_t frame_cap, uint16_t *out_len)
{
  if (ctx == NULL || frame == NULL || out_len == NULL) {
    return -EINVAL;
  }

  /* 协议 v2 按单 AP 发帧；多 AP 时只导出 ap=0 的数据 */
  uint8_t ap = 0U;

  if (ctx->n_ap != 1U) {
    LOG_WRN("DIP binary: n_ap=%u, sending ap=0 only", ctx->n_ap);
  }

  uint8_t channel_count = 0U;
  uint8_t bitmap[DIP_BIN_CHANNEL_BITMAP_BYTES];

  dip_bin_fill_channel_bitmap(ctx, ap, bitmap, &channel_count);

  const size_t hdr_size = sizeof(struct dip_bin_header);
  const size_t iq_size = (size_t)channel_count * 2U * sizeof(int16_t);
  const size_t frame_len = hdr_size + iq_size + sizeof(uint16_t);

  if (frame_len > frame_cap) {
    LOG_WRN("DIP binary: frame too large (ch=%u)", channel_count);
    return -ENOSPC;
  }

  struct dip_bin_header *hdr = (struct dip_bin_header *)frame;

  /* --- 固定头与位图（22 字节）--- */
  hdr->sync1 = DIP_BIN_SYNC1;
  hdr->sync2 = DIP_BIN_SYNC2;
  hdr->version = DIP_BIN_VERSION;
  hdr->type = DIP_BIN_TYPE_IQ;
  hdr->procedure_counter = sys_cpu_to_le16(ctx->procedure_counter);
  hdr->ap = ap;
  hdr->iq_format = DIP_BIN_IQ_FORMAT_INT16;
  hdr->channel_count = channel_count;
  hdr->reserved = 0U;
  memcpy(hdr->channel_bitmap, bitmap, DIP_BIN_CHANNEL_BITMAP_BYTES);

  const uint16_t payload_len =
      (uint16_t)(sizeof(hdr->procedure_counter) + sizeof(hdr->ap) + sizeof(hdr->iq_format) +
                 sizeof(hdr->channel_count) + sizeof(hdr->reserved) +
                 DIP_BIN_CHANNEL_BITMAP_BYTES + iq_size);

  hdr->payload_len = sys_cpu_to_le16(payload_len);

  /* --- 变长 IQ 区（4 * channel_count 字节）--- */
  const size_t iq_written = dip_bin_pack_iq_payload(
      ctx, ap, bitmap, &frame[hdr_size], frame_cap - hdr_size - sizeof(uint16_t));

  if (iq_written != iq_size) {
    LOG_WRN("DIP binary: IQ pack mismatch (expect %u got %u)", (unsigned)iq_size,
            (unsigned)iq_written);
    return -EIO;
  }

  /* --- CRC：覆盖 sync1 起至 IQ 末字节；PC 端校验须一致 --- */
  const uint16_t crc = dip_crc16_ccitt(frame, hdr_size + iq_written);

  sys_put_le16(crc, &frame[hdr_size + iq_written]);
  *out_len = (uint16_t)frame_len;

  return 0;
}

#if DIP_BINARY_USE_THREAD

/**
 * DIP 二进制 UART 发送消费者线程（线程名 dip_uart_tx）。
 *
 * 生命周期：main() 中 k_thread_create 启动后永久运行。
 * 职责：仅负责从 dip_bin_msgq 取帧并 dip_uart_send_bytes，不理解协议内容。
 * 栈上 dip_bin_msg 为 k_msgq_get 的接收副本，与队列内存储独立。
 */
static void dip_uart_tx_thread(void *p1, void *p2, void *p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  struct dip_bin_msg msg;

  while (true) {
    if (k_msgq_get(&dip_bin_msgq, &msg, K_FOREVER) == 0) {
      dip_uart_send_bytes(msg.data, msg.len);
    }
  }
}

/**
 * 非阻塞上报路径（Producer，运行于 BT RX WQ）。
 *
 * 约束：禁止 k_msgq_put(..., K_FOREVER)，避免串口慢导致蓝牙回调饿死。
 * 失败策略：打包失败静默返回；队列满则丢弃当前帧并 LOG_WRN（含 pc 便于统计丢包率）。
 */
static void dip_enqueue_local_report_binary(const struct dip_step_parse_ctx *ctx)
{
  if (ctx == NULL) {
    return;
  }

  if (dip_bin_build_frame(ctx, dip_bin_msg_scratch.data, sizeof(dip_bin_msg_scratch.data),
                          &dip_bin_msg_scratch.len) != 0) {
    return;
  }

  const int err = k_msgq_put(&dip_bin_msgq, &dip_bin_msg_scratch, K_NO_WAIT);

  if (err != 0) {
    LOG_WRN("DIP binary msgq full, drop pc=%u", ctx->procedure_counter);
  }
}

#else /* !DIP_BINARY_USE_THREAD */

/**
 * 阻塞上报路径：组帧与 UART 发送均在 subevent/BT RX WQ 内完成。
 * 用于对比 abort 率或快速验证协议；生产采集建议 DIP_BINARY_USE_THREAD=1。
 */
static void dip_send_local_report_binary(const struct dip_step_parse_ctx *ctx)
{
  uint16_t len = 0U;

  if (dip_bin_build_frame(ctx, dip_bin_frame_buf, sizeof(dip_bin_frame_buf), &len) == 0) {
    dip_uart_send_bytes(dip_bin_frame_buf, len);
  }
}

#endif /* DIP_BINARY_USE_THREAD */

/**
 * DIP 二进制报告统一入口（dip_parse_local_iq_from_subevent 在解析后调用）。
 * 根据 DIP_BINARY_USE_THREAD 分派到入队或阻塞发送，上层无需关心 UART 细节。
 */
static void dip_output_local_report_binary(const struct dip_step_parse_ctx *ctx)
{
#if DIP_BINARY_USE_THREAD
  dip_enqueue_local_report_binary(ctx);
#else
  dip_send_local_report_binary(ctx);
#endif
}

#endif /* DIP_REPORT_BINARY_OUTPUT */

/**
 * 分块打印本地 IQ（每行最多 8 个信道，与 print_report_basic 的窗口划分一致）；
 * 每点格式 ch(i,q)，无 remote。某 8 信道窗口内若没有任何有效测量则整行跳过。
 */
static void dip_print_local_report_multichannel(const struct dip_step_parse_ctx *ctx) {
  const int max_output_channels = DIP_MAX_FFT_CHANNELS;

  LOG_INF("DIP local report pc=%u n_ap=%u (IQ[ch]: ch=hci_ch-%u; local-only ch(i,q))",
          ctx->procedure_counter, ctx->n_ap, DIP_CS_CHANNEL_INDEX_OFFSET);

  for (uint8_t ap = 0; ap < ctx->n_ap; ap++) {
    LOG_INF("-- DIP Antenna Path %u --", ap);

    const int channels_per_line = 8;

    for (int ch_start = 0; ch_start < max_output_channels; ch_start += channels_per_line) {
      int ch_end = ch_start + channels_per_line;
      if (ch_end > max_output_channels) {
        ch_end = max_output_channels;
      }

      int valid_ch[8];
      int n_valid = 0;

      for (int ch = ch_start; ch < ch_end && n_valid < channels_per_line; ch++) {
        if (ctx->ch[ap][ch].valid) {
          valid_ch[n_valid++] = ch;
        }
      }

      if (n_valid == 0) {
        continue;
      }

      char iq_buffer[512];
      int offset =
          snprintf(iq_buffer, sizeof(iq_buffer), "AP%u IQ[%d-%d]: ", ap, ch_start, ch_end - 1);

      if (offset < 0 || offset >= (int)sizeof(iq_buffer)) {
        continue;
      }

      for (int i = 0; i < n_valid; i++) {
        int ch = valid_ch[i];
        if (i > 0) {
          offset += snprintf(iq_buffer + offset, sizeof(iq_buffer) - (size_t)offset, " | ");
        }
        offset += snprintf(iq_buffer + offset, sizeof(iq_buffer) - (size_t)offset,
                           "ch%d(i%.1f,q%.1f)", ch, (double)ctx->ch[ap][ch].i,
                           (double)ctx->ch[ap][ch].q);
      }

      LOG_INF("%s", iq_buffer);
    }
  }
}

/**
 * bt_le_cs_step_data_parse 回调：仅处理 main mode 2 / 3（含相位校正项 PCT）；其它 mode 跳过。
 * 将 PCT 填入 ctx->ch[antenna_path][fft_ch]；同一格点重复出现则覆盖为最后一次。
 */
static bool dip_local_step_iq_cb(struct bt_le_cs_subevent_step *step, void *user_data) {
  struct dip_step_parse_ctx *ctx = user_data;

  if (step == NULL || ctx == NULL || ctx->n_ap == 0U) {
    return true;
  }

  if (step->channel < DIP_CS_CHANNEL_INDEX_OFFSET) {
    LOG_WRN("DIP: unexpected hci channel %u (pc=%u)", step->channel, ctx->procedure_counter);
    return true;
  }

  unsigned fft_ch = (unsigned)(step->channel - DIP_CS_CHANNEL_INDEX_OFFSET);
  if (fft_ch >= DIP_MAX_FFT_CHANNELS) {
    LOG_WRN("DIP: fft_ch %u out of range (hci=%u pc=%u)", fft_ch, step->channel,
            ctx->procedure_counter);
    return true;
  }

  if (step->mode == BT_CONN_LE_CS_MAIN_MODE_2) {
    const struct bt_hci_le_cs_step_data_mode_2 *data =
        (const struct bt_hci_le_cs_step_data_mode_2 *)step->data;

    for (uint8_t tone_index = 0; tone_index < ctx->n_ap; tone_index++) {
      int antenna_path = bt_le_cs_get_antenna_path(
          ctx->n_ap, data->antenna_permutation_index, tone_index);
      if (antenna_path < 0) {
        LOG_WRN("DIP: invalid antenna path (mode2 pc=%u hci_ch=%u tone=%u)", ctx->procedure_counter,
                step->channel, tone_index);
        continue;
      }
      if (antenna_path >= CONFIG_BT_RAS_MAX_ANTENNA_PATHS) {
        continue;
      }

      if (data->tone_info[antenna_path].quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
        continue;
      }

      struct bt_le_cs_iq_sample iq =
          bt_le_cs_parse_pct(data->tone_info[antenna_path].phase_correction_term);

      ctx->ch[antenna_path][fft_ch].i = iq.i;
      ctx->ch[antenna_path][fft_ch].q = iq.q;
      ctx->ch[antenna_path][fft_ch].valid = true;
    }
  } else if (step->mode == BT_CONN_LE_CS_MAIN_MODE_3) {
    const struct bt_hci_le_cs_step_data_mode_3 *data =
        (const struct bt_hci_le_cs_step_data_mode_3 *)step->data;

    for (uint8_t tone_index = 0; tone_index < ctx->n_ap; tone_index++) {
      int antenna_path = bt_le_cs_get_antenna_path(
          ctx->n_ap, data->antenna_permutation_index, tone_index);
      if (antenna_path < 0) {
        LOG_WRN("DIP: invalid antenna path (mode3 pc=%u hci_ch=%u tone=%u)", ctx->procedure_counter,
                step->channel, tone_index);
        continue;
      }
      if (antenna_path >= CONFIG_BT_RAS_MAX_ANTENNA_PATHS) {
        continue;
      }

      if (data->tone_info[antenna_path].quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
        continue;
      }

      struct bt_le_cs_iq_sample iq =
          bt_le_cs_parse_pct(data->tone_info[antenna_path].phase_correction_term);

      ctx->ch[antenna_path][fft_ch].i = iq.i;
      ctx->ch[antenna_path][fft_ch].q = iq.q;
      ctx->ch[antenna_path][fft_ch].valid = true;
    }
  }

  return true;
}

/**
 * 拷贝一份 subevent 的 step_data 再解析，避免消费 Host 传入的 net_buf 影响其它逻辑。
 */
static void dip_parse_local_iq_from_subevent(const struct bt_conn_le_cs_subevent_result *result) {
  if (result == NULL || result->step_data_buf == NULL) {
    return;
  }

  if (result->step_data_buf->len == 0U) {
    return;
  }

  struct net_buf_simple buf;

  if (result->step_data_buf->len > sizeof(dip_step_data_copy)) {
    LOG_ERR("DIP: step_data too large (%u)", result->step_data_buf->len);
    return;
  }

  memcpy(dip_step_data_copy, result->step_data_buf->data, result->step_data_buf->len);
  net_buf_simple_init_with_data(&buf, dip_step_data_copy, result->step_data_buf->len);

  uint8_t n_ap = result->header.num_antenna_paths;
  if (n_ap == 0U) {
    n_ap = 1U;
  }
  if (n_ap > CONFIG_BT_RAS_MAX_ANTENNA_PATHS) {
    n_ap = CONFIG_BT_RAS_MAX_ANTENNA_PATHS;
  }

  memset(&dip_parse_work_ctx, 0, sizeof(dip_parse_work_ctx));

  dip_parse_work_ctx.procedure_counter = result->header.procedure_counter;
  dip_parse_work_ctx.n_ap = n_ap;

  bt_le_cs_step_data_parse(&buf, dip_local_step_iq_cb, &dip_parse_work_ctx);

  /*
   * 报告输出三选一（互斥）：
   *   BINARY — UART 原始帧，带宽最小，适合 PC 脚本采集；
   *   VERBOSE — LOG_INF 多信道文本，便于人工对照；
   *   否则 — 仅一行 pc，减轻日志对实时性的影响。
   */
#if DIP_REPORT_BINARY_OUTPUT
  dip_output_local_report_binary(&dip_parse_work_ctx);
#elif DIP_REPORT_LOG_VERBOSE
  dip_print_local_report_multichannel(&dip_parse_work_ctx);
#else
  /* 仅证明收到并完成解析，不刷屏；若需多信道 IQ，将 DIP_REPORT_LOG_VERBOSE 置 1 */
  LOG_INF("DIP ok pc=%u", dip_parse_work_ctx.procedure_counter);
#endif
}

/**
 * HCI subevent abort reason 的简短说明，便于串口对照 Core Spec / hci_types.h
 */
static const char *dip_subevent_abort_reason_str(enum bt_conn_le_cs_subevent_abort_reason r) {
  switch (r) {
  case BT_CONN_LE_CS_SUBEVENT_NOT_ABORTED:
    return "no_abort";
  case BT_CONN_LE_CS_SUBEVENT_ABORT_REQUESTED:
    return "host_or_peer_request";
  case BT_CONN_LE_CS_SUBEVENT_ABORT_NO_CS_SYNC:
    return "no_cs_sync";
  case BT_CONN_LE_CS_SUBEVENT_ABORT_SCHED_CONFLICT:
    return "sched_conflict";
  case BT_CONN_LE_CS_SUBEVENT_ABORT_UNSPECIFIED:
    return "unspecified";
  default:
    return "unknown";
  }
}

/**
 * DIP 专用 HCI subevent 回调：每条本地 report 到达即解析其中全部 step（即 procedure 内所有
 * 已上报信道步进），不等待 RAS、不操作 sem_local_steps / latest_local_steps。
 */
static void subevent_result_dip_cb(struct bt_conn *conn,
                                   struct bt_conn_le_cs_subevent_result *result) {
  ARG_UNUSED(conn);

  if (result == NULL) {
    return;
  }

  if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
    LOG_WRN("DIP: subevent aborted pc=%u reason=%u (%s) n_steps=%u abort_step=%u proc_done_st=%u",
            result->header.procedure_counter,
            (unsigned int)result->header.subevent_abort_reason,
            dip_subevent_abort_reason_str(result->header.subevent_abort_reason),
            result->header.num_steps_reported, result->header.abort_step,
            (unsigned int)result->header.procedure_done_status);
    return;
  }

  if (result->step_data_buf == NULL || result->step_data_buf->len == 0U) {
    LOG_DBG("DIP: no step data (pc=%u)", result->header.procedure_counter);
    return;
  }

  dip_parse_local_iq_from_subevent(result);
}

/* Legacy：原 RAS 对齐 + 缓存 latest_local_steps 流程。DIP 模式下不再注册本回调，仅保留编译与对照。 */
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
      // 这一步把 HCI 上报的原始本地 step data 存起来，此时并没有解析成 IQ。
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
  // 此处首先检查本地和对端数据是否对齐。然后立刻调用Get_ranging_data取回下一步
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
  // simple overwritten report
  // LOG_INF("OW!");
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

  // 我添加了interval_max用于配置连接间隔，40=50ms,16=20ms
  static struct bt_le_conn_param custom_conn_param = {
      .interval_min = 6,  // 最小连接间隔 (7.5ms)
      .interval_max = 40, // 最大连接间隔 (7.5ms，1.25ms unit)
      .latency = 0,       // 从设备延迟
      .timeout = 400,     // 监督超时 (400 * 10ms = 4秒)
  };

  struct bt_scan_init_param param_on_conn = {
      .scan_param = NULL,
      .conn_param = &custom_conn_param, // 使用自定义参数
      .connect_if_match = 1};

  bt_scan_init(&param_on_conn);
  bt_scan_cb_register(&scan_cb);
  // 过滤了具有ranging service 的设备，ini作为client，ref
  // 反而是RAS的服务器。ini会定期取ref发来的数据。
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
    .le_cs_subevent_data_available = subevent_result_dip_cb,
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

#if DIP_REPORT_BINARY_OUTPUT && DIP_BINARY_USE_THREAD
  /*
   * 在 bt_enable 之前启动发送线程：仅依赖 UART 设备，与蓝牙栈无耦合。
   * 线程启动后即阻塞在 k_msgq_get，待首次 subevent 解析入队后才开始发字节。
   */
  k_thread_create(&dip_uart_tx_thread_data, dip_uart_tx_stack,
                  K_THREAD_STACK_SIZEOF(dip_uart_tx_stack), dip_uart_tx_thread, NULL, NULL, NULL,
                  DIP_UART_TX_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&dip_uart_tx_thread_data, "dip_uart_tx");
  LOG_INF("DIP binary UART TX thread started (msgq depth %u)", (unsigned)DIP_BIN_MSGQ_DEPTH);
#endif

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

#if APP_CS_DIP_BYPASS_RAS
  /* DIP：不订阅 RAS，避免 ranging_data_ready_cb 在 latest_local_steps 未由 legacy 回调填充时
   * 仍拉取对端数据。恢复双端测距时将 APP_CS_DIP_BYPASS_RAS 置 0 并改回 subevent_result_cb。 */
  LOG_INF("DIP mode: RAS RREQ subscriptions skipped");
#else
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
#endif

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

  // enable custom or default channel settings
  bt_le_cs_set_valid_chmap_bits(config_params.channel_map);
  // set_custom_channel_map(config_params.channel_map);

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
