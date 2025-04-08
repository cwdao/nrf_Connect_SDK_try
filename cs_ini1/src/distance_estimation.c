/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding distance estimation for Ranging Requestor
 */

#include "zephyr/bluetooth/hci_types.h"
#include <bluetooth/services/ras.h>
#include <math.h>
#include <zephyr/bluetooth/cs.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(app_main, LOG_LEVEL_INF);

#define CS_FREQUENCY_MHZ(ch) (2402u + 1u * (ch))
#define CS_FREQUENCY_HZ(ch) (CS_FREQUENCY_MHZ(ch) * 1000000.0f)
#define SPEED_OF_LIGHT_M_PER_S (299792458.0f)
#define SPEED_OF_LIGHT_NM_PER_S (SPEED_OF_LIGHT_M_PER_S / 1000000000.0f)
#define PI 3.14159265358979323846f
#define MAX_NUM_RTT_SAMPLES 256
#define MAX_NUM_IQ_SAMPLES 256 * CONFIG_BT_RAS_MAX_ANTENNA_PATHS

// 包含每个信道的 IQ 数据和信道信息,
// 具体地说，这是每一个cs包的内容，一次探测只能在一个通道上进行。在多少个通道上采集了，就有多少个这样的样本
struct iq_sample_and_channel {
  bool failed;                               // 是否采样失败
  uint8_t channel;                           // 信道
  uint8_t antenna_permutation;               // 天线排列
  struct bt_le_cs_iq_sample local_iq_sample; // 本地 IQ 采样
  struct bt_le_cs_iq_sample peer_iq_sample;  // 对端 IQ 采样
};

struct rtt_timing {
  bool failed;
  int16_t toa_tod_initiator;
  int16_t tod_toa_reflector;
};

static struct iq_sample_and_channel iq_sample_channel_data[MAX_NUM_IQ_SAMPLES];
static struct rtt_timing rtt_timing_data[MAX_NUM_RTT_SAMPLES];

struct processing_context {
  uint16_t rtt_timing_data_index;
  uint16_t iq_sample_channel_data_index;
  uint8_t n_ap;
  enum bt_conn_le_cs_role role;
};

static void calc_complex_product(int32_t z_a_real, int32_t z_a_imag,
                                 int32_t z_b_real, int32_t z_b_imag,
                                 int32_t *z_out_real, int32_t *z_out_imag) {
  *z_out_real = z_a_real * z_b_real - z_a_imag * z_b_imag;
  *z_out_imag = z_a_real * z_b_imag + z_a_imag * z_b_real;
}

static float linear_regression(float *x_values, float *y_values,
                               uint8_t n_samples) {
  if (n_samples == 0) {
    return 0.0;
  }

  /* Estimates b in y = a + b x */

  float y_mean = 0.0;
  float x_mean = 0.0;

  for (uint8_t i = 0; i < n_samples; i++) {
    y_mean += (y_values[i] - y_mean) / (i + 1);
    x_mean += (x_values[i] - x_mean) / (i + 1);
  }

  float b_est_upper = 0.0;
  float b_est_lower = 0.0;

  for (uint8_t i = 0; i < n_samples; i++) {
    b_est_upper += (x_values[i] - x_mean) * (y_values[i] - y_mean);
    b_est_lower += (x_values[i] - x_mean) * (x_values[i] - x_mean);
  }

  return b_est_upper / b_est_lower;
}

static void bubblesort_2(float *array1, float *array2, uint16_t len) {
  bool swapped;
  float temp;

  for (uint16_t i = 0; i < len - 1; i++) {
    swapped = false;
    for (uint16_t j = 0; j < len - i - 1; j++) {
      if (array1[j] > array1[j + 1]) {
        temp = array1[j];
        array1[j] = array1[j + 1];
        array1[j + 1] = temp;
        temp = array2[j];
        array2[j] = array2[j + 1];
        array2[j + 1] = temp;
        swapped = true;
      }
    }

    if (!swapped) {
      break;
    }
  }
}

static float
estimate_distance_using_phase_slope(struct iq_sample_and_channel *data,
                                    uint8_t len) {
  int32_t combined_i;
  int32_t combined_q;
  uint16_t num_angles = 0;
  static float theta[MAX_NUM_IQ_SAMPLES];
  static float frequencies[MAX_NUM_IQ_SAMPLES];
  static float channels[MAX_NUM_IQ_SAMPLES]; // 用于存储信道信息
  for (uint8_t i = 0; i < len; i++) {
    if (!data[i].failed) {
      // 计算 IQ 数据的复数乘积
      calc_complex_product(data[i].local_iq_sample.i, data[i].local_iq_sample.q,
                           data[i].peer_iq_sample.i, data[i].peer_iq_sample.q,
                           &combined_i, &combined_q);

      // 计算相位角,这就是反三角函数吧
      theta[num_angles] = atan2(1.0 * combined_q, 1.0 * combined_i);

      // 计算频率
      frequencies[num_angles] = 1.0 * CS_FREQUENCY_MHZ(data[i].channel);

      // LOG_INF("Raw Data -> Channel: %d, Frequency: %f MHz, Angle (theta): %f "
      //         "radians",
      //         data[i].channel, (double)frequencies[num_angles],
      //         (double)theta[num_angles]);

      num_angles++;
    }
  }

  // 如果采样点不足，无法计算距离
  if (num_angles < 2) {
    return 0.0;
  }

  // 按频率对相位排序
  bubblesort_2(frequencies, theta, num_angles);
  // 排序完成后，通过频率重新计算信道编号
  // LOG_INF("Sorted Data -> Channel : Theta");
  // for (uint8_t i = 0; i < num_angles; i++) {
  //   uint8_t channel = (uint8_t)(frequencies[i] - 2402); // 从频率计算信道编号
  //   // LOG_INF("ch[%d] : %f radians", channel, (double)theta[i]);
  // }

  /* One-dimensional phase unwrapping */
  // 相位展开（Phase Unwrapping）
  // 遍历所有相位角，确保相位值在 −π 到 π 范围内连续。
  for (uint8_t i = 1; i < num_angles; i++) {
    float difference = theta[i] - theta[i - 1];
    // 如果相邻相位角的差值大于 π，则将后续相位值减去 2π
    if (difference > PI) {
      for (uint8_t j = i; j < num_angles; j++) {
        theta[j] -= 2.0f * PI;
      }
    } else if (difference < -PI) {
      // 如果相邻相位角的差值小于 -π，则将后续相位值加上 2π
      for (uint8_t j = i; j < num_angles; j++) {
        theta[j] += 2.0f * PI;
      }
    }
  }

      // 输出相位展开后的信道和相位
    LOG_INF("Unwrapped Data -> Channel : Theta");
    for (uint8_t i = 0; i < num_angles; i++) {
        uint8_t channel = (uint8_t)(frequencies[i] - 2402); // 从频率计算信道编号
        LOG_INF("ch[%d] : %f radians", channel, (double)theta[i]);
    }

  // 使用线性回归计算相位斜率
  float phase_slope = linear_regression(frequencies, theta, num_angles);

  // 根据相位斜率计算距离
  float distance = -phase_slope * (SPEED_OF_LIGHT_M_PER_S / (4 * PI));

  return distance / 1000000.0f; /* Scale to meters. */
}

static float estimate_distance_using_time_of_flight(uint8_t n_samples) {
  float tof;
  float tof_mean = 0.0;

  /* Cumulative Moving Average */
  for (uint8_t i = 0; i < n_samples; i++) {
    if (!rtt_timing_data[i].failed) {
      tof = (rtt_timing_data[i].toa_tod_initiator -
             rtt_timing_data[i].tod_toa_reflector) /
            2;
      tof_mean += (tof - tof_mean) / (i + 1);
    }
  }

  float tof_mean_ns = tof_mean / 2.0f;

  return tof_mean_ns * SPEED_OF_LIGHT_NM_PER_S;
}

// 将解析出的IQ样本数据存储到iq_sample_channel_data数组
// context - 处理上下文，包含处理过程中的各种索引和参数
// local_tone_info[] - 本地设备的音调信息数组
// peer_tone_info[] - 对端设备的音调信息数组
// channel - 当前处理的蓝牙信道
// antenna_permutation_index - 天线排列索引
static void process_tone_info_data(
    struct processing_context *context,
    struct bt_hci_le_cs_step_data_tone_info local_tone_info[],
    struct bt_hci_le_cs_step_data_tone_info peer_tone_info[], uint8_t channel,
    uint8_t antenna_permutation_index) {

  // 循环处理每个天线路径
  for (uint8_t i = 0; i < (context->n_ap + 1); i++) {
    // 如果本地或对端的音调信息扩展指示符不是BT_HCI_LE_CS_NOT_TONE_EXT_SLOT，则跳过
    if (local_tone_info[i].extension_indicator !=
            BT_HCI_LE_CS_NOT_TONE_EXT_SLOT ||
        peer_tone_info[i].extension_indicator !=
            BT_HCI_LE_CS_NOT_TONE_EXT_SLOT) {
      continue;
    }

    if (context->iq_sample_channel_data_index >= MAX_NUM_IQ_SAMPLES) {
      LOG_WRN("More IQ samples than size of iq_sample_channel_data array");
      return;
    }

    // 将当前处理的信道和天线排列索引存储到IQ样本数据中
    iq_sample_channel_data[context->iq_sample_channel_data_index].channel =
        channel;
    iq_sample_channel_data[context->iq_sample_channel_data_index]
        .antenna_permutation = antenna_permutation_index;

    // 解析并存储IQ样本,使用bt_le_cs_parse_pct函数解析本地和对端的相位校正项（phase
    // correction term），得到IQ样本数据
    iq_sample_channel_data[context->iq_sample_channel_data_index]
        .local_iq_sample =
        bt_le_cs_parse_pct(local_tone_info[i].phase_correction_term);
    iq_sample_channel_data[context->iq_sample_channel_data_index]
        .peer_iq_sample =
        bt_le_cs_parse_pct(peer_tone_info[i].phase_correction_term);
    // 似乎是判断接收信号的质量的,如果本地或对端的音调质量指示器显示为低质量或不可用，则将该IQ样本标记为失败。
    if (local_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
        local_tone_info[i].quality_indicator ==
            BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE ||
        peer_tone_info[i].quality_indicator == BT_HCI_LE_CS_TONE_QUALITY_LOW ||
        peer_tone_info[i].quality_indicator ==
            BT_HCI_LE_CS_TONE_QUALITY_UNAVAILABLE) {
      iq_sample_channel_data[context->iq_sample_channel_data_index].failed =
          true;
    }

    context->iq_sample_channel_data_index++;
  }
}

static void
process_rtt_timing_data(struct processing_context *context,
                        struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
                        struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data) {
  if (context->rtt_timing_data_index >= MAX_NUM_RTT_SAMPLES) {
    LOG_WRN("More RTT samples processed than size of rtt_timing_data array");
    return;
  }

  if (local_rtt_data->packet_quality_aa_check !=
          BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
      local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
      local_rtt_data->tod_toa_reflector ==
          BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
      peer_rtt_data->packet_quality_aa_check !=
          BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
      peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
      peer_rtt_data->tod_toa_reflector ==
          BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
    rtt_timing_data[context->rtt_timing_data_index].failed = true;
  }

  if (context->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
    rtt_timing_data[context->rtt_timing_data_index].toa_tod_initiator =
        local_rtt_data->toa_tod_initiator;
    rtt_timing_data[context->rtt_timing_data_index].tod_toa_reflector =
        peer_rtt_data->tod_toa_reflector;
  } else if (context->role == BT_CONN_LE_CS_ROLE_REFLECTOR) {
    rtt_timing_data[context->rtt_timing_data_index].tod_toa_reflector =
        local_rtt_data->tod_toa_reflector;
    rtt_timing_data[context->rtt_timing_data_index].toa_tod_initiator =
        peer_rtt_data->toa_tod_initiator;
  }

  context->rtt_timing_data_index++;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
                              struct bt_le_cs_subevent_step *peer_step,
                              void *user_data) {
  struct processing_context *context = (struct processing_context *)user_data;

  if (local_step->mode == BT_CONN_LE_CS_MAIN_MODE_2) {
    struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
        (struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
    struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
        (struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

    process_tone_info_data(context, local_step_data->tone_info,
                           peer_step_data->tone_info, local_step->channel,
                           local_step_data->antenna_permutation_index);

  } else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
    struct bt_hci_le_cs_step_data_mode_1 *local_step_data =
        (struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
    struct bt_hci_le_cs_step_data_mode_1 *peer_step_data =
        (struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

    process_rtt_timing_data(context, local_step_data, peer_step_data);

  } else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
    struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
        (struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
    struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
        (struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

    process_rtt_timing_data(
        context, (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
        (struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);

    process_tone_info_data(context, local_step_data->tone_info,
                           peer_step_data->tone_info, local_step->channel,
                           local_step_data->antenna_permutation_index);
  }

  return true;
}

void estimate_distance(struct net_buf_simple *local_steps,
                       struct net_buf_simple *peer_steps, uint8_t n_ap,
                       enum bt_conn_le_cs_role role) {
  struct processing_context context = {
      .rtt_timing_data_index = 0,
      .iq_sample_channel_data_index = 0,
      .n_ap = n_ap,
      .role = role,
  };

  memset(rtt_timing_data, 0, sizeof(rtt_timing_data));
  memset(iq_sample_channel_data, 0, sizeof(iq_sample_channel_data));

  bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, context.role,
                                     NULL, process_step_data, &context);

  float phase_slope_based_distance = estimate_distance_using_phase_slope(
      iq_sample_channel_data, context.iq_sample_channel_data_index);

  float rtt_based_distance =
      estimate_distance_using_time_of_flight(context.rtt_timing_data_index);

  if (rtt_based_distance == 0.0f && phase_slope_based_distance == 0.0f) {
    LOG_INF("A reliable distance estimate could not be computed.");
  } else {
    LOG_INF("Estimated distance to reflector:");
  }

  if (rtt_based_distance != 0.0f) {
    LOG_INF("- Round-Trip Timing method: %f meters (derived from %d samples)",
            (double)rtt_based_distance, context.rtt_timing_data_index);
  }
  if (phase_slope_based_distance != 0.0f) {
    LOG_INF("- Phase-Based Ranging method: %f meters (derived from %d samples)",
            (double)phase_slope_based_distance,
            context.iq_sample_channel_data_index);
  }
}
