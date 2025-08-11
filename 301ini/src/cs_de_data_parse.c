#include "cs_de_data_parse.h"
#include <bluetooth/cs_de.h>
#include <zephyr/logging/log.h>

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