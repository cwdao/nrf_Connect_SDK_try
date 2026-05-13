#ifndef CS_DE_DATA_PARSE_H
#define CS_DE_DATA_PARSE_H

#include <bluetooth/cs_de.h>

typedef struct {
  uint64_t report_index; // 测距结果的索引
  uint64_t timestamp_ms; // 记录写入时的毫秒数
  cs_de_report_t report; // 原始测距结果
} store_cs_de_report_t;

void store_cs_de_report(cs_de_report_t *p_report);
void print_report(const cs_de_report_t *r, int max_output_channels);
void print_store_cs_de_report(const store_cs_de_report_t *s,
                              int max_output_channels);
void print_store_cs_de_report_basic(const store_cs_de_report_t *s,
                                   int max_output_channels);
void print_report_fast(const cs_de_report_t *r, int max_output_channels);

#endif