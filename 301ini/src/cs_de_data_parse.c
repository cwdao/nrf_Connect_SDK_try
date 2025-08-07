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