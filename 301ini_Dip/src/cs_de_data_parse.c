/**
 * @file cs_de_data_parse.c
 * @brief cs_de 报告的人类可读打印与全功能 CS 二进制 UART 组帧
 *
 * 文本输出：print_report / print_report_fast 等，经 LOG_INF 输出 ASCII。
 * 二进制输出（CS_REPORT_BINARY_OUTPUT）：cs_bin_* 将 cs_de_report_t 编码为
 * type=0x02 帧，再调用 cs_uart_bin_enqueue_frame()。
 *
 * IQ 数值：cs_de 内部以 float 保存 bt_le_cs_parse_pct() 的 int16 结果（含均值），
 * 组帧时用 cs_bin_float_to_iq() 四舍五入回 int16 线上格式。
 */

#include "cs_de_data_parse.h"
#include <bluetooth/cs_de.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#if CS_REPORT_BINARY_OUTPUT
#include "cs_uart_binary.h"
#endif

LOG_MODULE_REGISTER(step_data_parse, CONFIG_LOG_DEFAULT_LEVEL);

void print_report(const cs_de_report_t *r, int max_output_channels) {
  // 内联role、quality、tone的字符串转换
  const char *role_str;
  switch (r->role) {
  case BT_CONN_LE_CS_ROLE_INITIATOR:
    role_str = "INITIATOR";
    break;
  case BT_CONN_LE_CS_ROLE_REFLECTOR:
    role_str = "REFLECTOR";
    break;
  default:
    role_str = "UNKNOWN";
    break;
  }

  LOG_INF("role=%s, n_ap=%d", role_str, r->n_ap);
  LOG_INF("rtt_accumulated_half_ns=%d, rtt_count=%d",
          r->rtt_accumulated_half_ns, r->rtt_count);

  for (int ap = 0; ap < r->n_ap; ap++) {
    const char *tone_str = (r->tone_quality[ap] == CS_DE_TONE_QUALITY_OK) ? "OK"
                           : (r->tone_quality[ap] == CS_DE_TONE_QUALITY_BAD)
                               ? "BAD"
                               : "UNKNOWN";
    LOG_INF("-- Antenna Path %d --", ap);
    LOG_INF("  Tone quality: %s", tone_str);
    LOG_INF("  Distance (ifft/phase_slope/rtt/best): %.3f / %.3f / %.3f / %.3f",
            r->distance_estimates[ap].ifft,
            r->distance_estimates[ap].phase_slope,
            r->distance_estimates[ap].rtt, r->distance_estimates[ap].best);
    for (int ch = 0; ch < max_output_channels; ch++) {
      LOG_INF(
          "  IQ[%d]: i_local=%.5f, q_local=%.5f, i_remote=%.5f, q_remote=%.5f",
          ch, r->iq_tones[ap].i_local[ch], r->iq_tones[ap].q_local[ch],
          r->iq_tones[ap].i_remote[ch], r->iq_tones[ap].q_remote[ch]);
    }
  }
}

void print_store_cs_de_report(const store_cs_de_report_t *s,
                              int max_output_channels) {
  LOG_INF("== store_cs_de_report_t ==");
  LOG_INF("report_index: %u", s->report_index);
  LOG_INF("timestamp_ms: %llu", s->timestamp_ms);
  print_report(&s->report, max_output_channels);
}

void store_cs_de_report(cs_de_report_t *p_report) {
  // 存储原始数据
  // 存储原始数据
}

void print_report_basic(const cs_de_report_t *r, int max_output_channels) {
  // 内联role、quality、tone的字符串转换
  const char *role_str;
  switch (r->role) {
  case BT_CONN_LE_CS_ROLE_INITIATOR:
    role_str = "INITIATOR";
    break;
  case BT_CONN_LE_CS_ROLE_REFLECTOR:
    role_str = "REFLECTOR";
    break;
  default:
    role_str = "UNKNOWN";
    break;
  }

  LOG_INF("role=%s, n_ap=%d, rtt_accumulated_half_ns=%d, rtt_count=%d",
          role_str, r->n_ap, r->rtt_accumulated_half_ns, r->rtt_count);

  for (int ap = 0; ap < r->n_ap; ap++) {
    const char *tone_str = (r->tone_quality[ap] == CS_DE_TONE_QUALITY_OK) ? "OK"
                           : (r->tone_quality[ap] == CS_DE_TONE_QUALITY_BAD)
                               ? "BAD"
                               : "UNKNOWN";
    
    LOG_INF("AP%d: quality=%s, dist(ifft/phase/rtt/best)=%.3f/%.3f/%.3f/%.3f",
            ap, tone_str,
            r->distance_estimates[ap].ifft,
            r->distance_estimates[ap].phase_slope,
            r->distance_estimates[ap].rtt, 
            r->distance_estimates[ap].best);
    
    // 分块打印IQ数据，避免缓冲区溢出
    const int channels_per_line = 8; // 每行8个通道
    const int max_buffer_size = 256; // 安全的缓冲区大小
    
    for (int ch_start = 0; ch_start < max_output_channels; ch_start += channels_per_line) {
      char iq_buffer[max_buffer_size];
      int offset = 0;
      
      offset += snprintf(iq_buffer + offset, sizeof(iq_buffer) - offset, 
                        "AP%d IQ[%d-%d]: ", ap, ch_start, 
                        (ch_start + channels_per_line - 1 < max_output_channels) ? 
                        ch_start + channels_per_line - 1 : max_output_channels - 1);
      
      int ch_end = (ch_start + channels_per_line < max_output_channels) ? 
                   ch_start + channels_per_line : max_output_channels;
      
      for (int ch = ch_start; ch < ch_end; ch++) {
        if (ch > ch_start) {
          offset += snprintf(iq_buffer + offset, sizeof(iq_buffer) - offset, " | ");
        }
        offset += snprintf(iq_buffer + offset, sizeof(iq_buffer) - offset,
                          "ch%d(i%.1f,q%.1f,i%.1f,q%.1f)", 
                          ch,
                          r->iq_tones[ap].i_local[ch],
                          r->iq_tones[ap].q_local[ch],
                          r->iq_tones[ap].i_remote[ch],
                          r->iq_tones[ap].q_remote[ch]);
      }
      
      LOG_INF("%s", iq_buffer);
    }
  }
}

void print_store_cs_de_report_basic(const store_cs_de_report_t *s,
                                   int max_output_channels) {
  LOG_INF("== Basic Report == index:%llu, timestamp:%llu", 
          s->report_index, s->timestamp_ms);
  print_report_fast(&s->report, max_output_channels);
  LOG_INF("== End Report ==");
}


#define IQ_LINE_CHUNK 512     // 单次输出最大字节数，按需调整
#define IQ_PRECISION 1        // 小数位数，1 位更紧凑更快
void print_report_fast(const cs_de_report_t *r, int max_output_channels) {
  // 内联role、quality、tone的字符串转换
  const char *role_str;
  switch (r->role) {
  case BT_CONN_LE_CS_ROLE_INITIATOR:
    role_str = "INITIATOR";
    break;
  case BT_CONN_LE_CS_ROLE_REFLECTOR:
    role_str = "REFLECTOR";
    break;
  default:
    role_str = "UNKNOWN";
    break;
  }

  LOG_INF("role=%s, n_ap=%d", role_str, r->n_ap);
  LOG_INF("rtt_accumulated_half_ns=%d, rtt_count=%d",
          r->rtt_accumulated_half_ns, r->rtt_count);

  for (int ap = 0; ap < r->n_ap; ap++) {
    const char *tone_str = (r->tone_quality[ap] == CS_DE_TONE_QUALITY_OK) ? "OK"
                           : (r->tone_quality[ap] == CS_DE_TONE_QUALITY_BAD)
                               ? "BAD"
                               : "UNKNOWN";
    LOG_INF("-- Antenna Path %d --", ap);
    LOG_INF("Tone=%s Dist(ifft/phase_slope/rtt/best)=%.3f/%.3f/%.3f/%.3f",
            tone_str,
            r->distance_estimates[ap].ifft,
            r->distance_estimates[ap].phase_slope,
            r->distance_estimates[ap].rtt,
            r->distance_estimates[ap].best);

    // 紧凑的 IQ 多信道合并输出
    char buf[IQ_LINE_CHUNK];
    size_t off = 0;

    // 先输出一个前缀，便于解析（可选）
    int n = snprintk(buf + off, sizeof(buf) - off, "IQ: ");
    if (n < 0) {
      // 格式化错误，直接跳过本 ap
      continue;
    }
    off += (size_t)n;

    for (int ch = 0; ch < max_output_channels; ch++) {
      // 每个信道的紧凑片段；为减少体积与格式化开销，默认 1 位小数
      n = snprintk(
          buf + off, sizeof(buf) - off,
          // "ch:%d,il:%.*f,ql:%.*f,ir:%.*f,qr:%.*f;",
          "ch:%d:%.*f,%.*f,%.*f,%.*f;",
          ch,
          IQ_PRECISION, r->iq_tones[ap].i_local[ch],
          IQ_PRECISION, r->iq_tones[ap].q_local[ch],
          IQ_PRECISION, r->iq_tones[ap].i_remote[ch],
          IQ_PRECISION, r->iq_tones[ap].q_remote[ch]);

      if (n < 0) {
        // 格式化错误，提前结束
        break;
      }

      // 如果这次写入会溢出缓冲区，先把已有内容打出去，再把当前信道重写入
      if ((size_t)n >= sizeof(buf) - off) {
        buf[off] = '\0';
        LOG_INF("%s", buf);
        off = 0;

        // 重新写入当前信道
        n = snprintk(
            buf + off, sizeof(buf) - off,
            // "ch:%d,il:%.*f,ql:%.*f,ir:%.*f,qr:%.*f;",
            "ch:%d:%.*f,%.*f,%.*f,%.*f;",
            ch,
            IQ_PRECISION, r->iq_tones[ap].i_local[ch],
            IQ_PRECISION, r->iq_tones[ap].q_local[ch],
            IQ_PRECISION, r->iq_tones[ap].i_remote[ch],
            IQ_PRECISION, r->iq_tones[ap].q_remote[ch]);

        if (n < 0) {
          break;
        }
      }

      off += (size_t)n;
    }

    // flush 剩余内容
    if (off > 0) {
      buf[off] = '\0';
      LOG_INF("%s", buf);
    }
  }
}

#if CS_REPORT_BINARY_OUTPUT

/* =============================================================================
 * 全功能 CS 二进制 UART 输出（协议 type=0x02，双端 IQ）
 *
 * 帧格式与 DIP v2 共用 cs_uart_bin_header；差异：
 *   - type = CS_UART_BIN_TYPE_CS_DUAL_IQ (0x02)
 *   - 每有效信道 8 字节：i_local, q_local, i_remote, q_remote（各 int16 LE）
 *   - procedure_counter 填入 RAS ranging_counter
 *   - timestamp_ms 填入 store_cs_de_report_t::timestamp_ms（k_uptime_get 毫秒）
 *
 * 有效信道：cs_de_populate_report 后 iq_tones 任一分量非零的 fft 格点（0..74）。
 * 发送：cs_uart_bin_enqueue_frame() → cs_uart_tx 线程（见 cs_uart_binary.c）。
 *
 * 协议文档：doc/CS_binary_protocol.md
 * ============================================================================= */

/** 组帧静态缓冲；单连接 ranging 串行完成，可安全复用 */
static uint8_t cs_bin_frame_buf[CS_UART_BIN_MAX_FRAME_SIZE];

/**
 * @brief 将 cs_de 内 float IQ 钳位并四舍五入为 int16 线格式
 * @note cs_de 存的是 PCT 解析后的数值（可能经 cumulate_mean），非物理浮点 IQ
 */
static int16_t cs_bin_float_to_iq(float value)
{
  if (value > 32767.0f) {
    return 32767;
  }
  if (value < -32768.0f) {
    return -32768;
  }

  return (int16_t)lroundf(value);
}

/**
 * @brief 判断某 (ap, fft_ch) 是否在 cs_de 报告中有 IQ 数据
 * @note 未测信道在 populate 后保持 0.0f；全零格点不进入位图
 */
static bool cs_bin_channel_has_iq(const cs_de_report_t *report, uint8_t ap, uint8_t ch)
{
  return report->iq_tones[ap].i_local[ch] != 0.0f ||
         report->iq_tones[ap].q_local[ch] != 0.0f ||
         report->iq_tones[ap].i_remote[ch] != 0.0f ||
         report->iq_tones[ap].q_remote[ch] != 0.0f;
}

/**
 * @brief 根据 iq_tones 非零格点填充 10 字节信道位图
 * @param channel_count 输出置 1 的 bit 数，须与后续 IQ 区样本数一致
 */
static void cs_bin_fill_channel_bitmap(const cs_de_report_t *report, uint8_t ap,
                                       uint8_t *bitmap, uint8_t *channel_count)
{
  memset(bitmap, 0, CS_UART_BIN_CHANNEL_BITMAP_BYTES);

  if (report == NULL || bitmap == NULL || channel_count == NULL) {
    if (channel_count != NULL) {
      *channel_count = 0U;
    }
    return;
  }

  uint8_t count = 0U;

  for (uint8_t ch = 0; ch < CS_UART_BIN_MAX_FFT_CHANNELS; ch++) {
    if (!cs_bin_channel_has_iq(report, ap, ch)) {
      continue;
    }

    bitmap[ch / 8U] |= (uint8_t)(1U << (ch % 8U));
    count++;
  }

  *channel_count = count;
}

/**
 * @brief 按位图顺序写入双端 IQ 载荷（每信道 4×int16，共 8 字节）
 * @return 实际写入字节数；遍历顺序须与 cs_bin_fill_channel_bitmap 一致（ch 升序）
 */
static size_t cs_bin_pack_dual_iq_payload(const cs_de_report_t *report, uint8_t ap,
                                          const uint8_t *bitmap, uint8_t *out, size_t out_max)
{
  size_t offset = 0U;

  if (report == NULL || bitmap == NULL || out == NULL) {
    return 0U;
  }

  for (uint8_t ch = 0; ch < CS_UART_BIN_MAX_FFT_CHANNELS; ch++) {
    if ((bitmap[ch / 8U] & (uint8_t)(1U << (ch % 8U))) == 0U) {
      continue;
    }

    if (offset + 4U * sizeof(int16_t) > out_max) {
      break;
    }

    sys_put_le16((uint16_t)cs_bin_float_to_iq(report->iq_tones[ap].i_local[ch]), &out[offset]);
    offset += sizeof(int16_t);
    sys_put_le16((uint16_t)cs_bin_float_to_iq(report->iq_tones[ap].q_local[ch]), &out[offset]);
    offset += sizeof(int16_t);
    sys_put_le16((uint16_t)cs_bin_float_to_iq(report->iq_tones[ap].i_remote[ch]), &out[offset]);
    offset += sizeof(int16_t);
    sys_put_le16((uint16_t)cs_bin_float_to_iq(report->iq_tones[ap].q_remote[ch]), &out[offset]);
    offset += sizeof(int16_t);
  }

  return offset;
}

/**
 * @brief 将 cs_de_report 打包为完整 type=0x02 二进制帧（含 CRC）
 * @param ranging_counter 写入帧头 procedure_counter（LE）
 * @param out_len         成功时输出整帧字节数
 * @return 0 成功；-EINVAL / -ENOSPC / -EIO 见实现内 LOG_WRN
 */
static int cs_bin_build_dual_iq_frame(const cs_de_report_t *report, uint16_t ranging_counter,
                                      uint64_t timestamp_ms, uint8_t *frame, size_t frame_cap,
                                      uint16_t *out_len)
{
  if (report == NULL || frame == NULL || out_len == NULL) {
    return -EINVAL;
  }

  uint8_t ap = 0U;

  if (report->n_ap != 1U) {
    LOG_WRN("CS binary: n_ap=%u, sending ap=0 only", report->n_ap);
  }

  uint8_t channel_count = 0U;
  uint8_t bitmap[CS_UART_BIN_CHANNEL_BITMAP_BYTES];

  cs_bin_fill_channel_bitmap(report, ap, bitmap, &channel_count);

  const size_t hdr_size = sizeof(struct cs_uart_bin_header);
  const size_t iq_size = (size_t)channel_count * 4U * sizeof(int16_t);
  const size_t frame_len = hdr_size + iq_size + sizeof(uint16_t);

  if (frame_len > frame_cap) {
    LOG_WRN("CS binary: frame too large (ch=%u)", channel_count);
    return -ENOSPC;
  }

  struct cs_uart_bin_header *hdr = (struct cs_uart_bin_header *)frame;

  hdr->sync1 = CS_UART_BIN_SYNC1;
  hdr->sync2 = CS_UART_BIN_SYNC2;
  hdr->version = CS_UART_BIN_VERSION;
  hdr->type = CS_UART_BIN_TYPE_CS_DUAL_IQ;
  hdr->procedure_counter = sys_cpu_to_le16(ranging_counter);
  hdr->timestamp_ms = sys_cpu_to_le64(timestamp_ms);
  hdr->ap = ap;
  hdr->iq_format = CS_UART_BIN_IQ_FORMAT_INT16;
  hdr->channel_count = channel_count;
  hdr->reserved = 0U;
  memcpy(hdr->channel_bitmap, bitmap, CS_UART_BIN_CHANNEL_BITMAP_BYTES);

  const uint16_t payload_len =
      (uint16_t)(sizeof(hdr->procedure_counter) + sizeof(hdr->timestamp_ms) + sizeof(hdr->ap) +
                 sizeof(hdr->iq_format) + sizeof(hdr->channel_count) + sizeof(hdr->reserved) +
                 CS_UART_BIN_CHANNEL_BITMAP_BYTES + iq_size);

  hdr->payload_len = sys_cpu_to_le16(payload_len);

  const size_t iq_written = cs_bin_pack_dual_iq_payload(
      report, ap, bitmap, &frame[hdr_size], frame_cap - hdr_size - sizeof(uint16_t));

  if (iq_written != iq_size) {
    LOG_WRN("CS binary: IQ pack mismatch (expect %u got %u)", (unsigned)iq_size,
            (unsigned)iq_written);
    return -EIO;
  }

  const uint16_t crc = cs_uart_bin_crc16_ccitt(frame, hdr_size + iq_written);

  sys_put_le16(crc, &frame[hdr_size + iq_written]);
  *out_len = (uint16_t)frame_len;

  return 0;
}

/**
 * @brief 全功能 CS 二进制报告统一入口（由 ranging_data_get_complete_cb 调用）
 *
 * 失败策略：组帧失败或队列满时 LOG_WRN 并丢帧，不阻塞 ranging 回调。
 */
void cs_output_store_report_binary(const store_cs_de_report_t *store, uint16_t ranging_counter)
{
  if (store == NULL) {
    return;
  }

  uint16_t frame_len = 0U;
  const int err = cs_bin_build_dual_iq_frame(&store->report, ranging_counter, store->timestamp_ms,
                                             cs_bin_frame_buf, sizeof(cs_bin_frame_buf),
                                             &frame_len);

  if (err != 0) {
    LOG_WRN("CS binary build failed rc=%d rcounter=%u", err, ranging_counter);
    return;
  }

  const int tx_err = cs_uart_bin_enqueue_frame(cs_bin_frame_buf, frame_len);

  if (tx_err == -ENOMEM) {
    LOG_WRN("CS binary msgq full, drop rcounter=%u", ranging_counter);
  }
}

#endif /* CS_REPORT_BINARY_OUTPUT */