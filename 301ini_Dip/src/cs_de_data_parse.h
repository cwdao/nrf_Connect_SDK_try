/**
 * @file cs_de_data_parse.h
 * @brief cs_de 测距报告的展示、存储封装与全功能 CS 二进制 UART 输出
 *
 * 模块职责（按数据流）：
 *
 *   Nordic cs_de_populate_report / cs_de_calc
 *        → cs_de_report_t
 *        → [文本] print_report_* / print_store_cs_de_report_*
 *        → [二进制] cs_output_store_report_binary()  （CS_REPORT_BINARY_OUTPUT=1）
 *        → cs_uart_binary 传输层
 *
 * 注意：本文件不负责解析 HCI step_data 或 RAS 原始字节（由 Nordic cs_de 完成）。
 * 文件名中的 parse 为历史命名，实质是「报告输出 / 格式化」层。
 *
 * DIP 本地直采的二进制组帧在 main.c（dip_bin_*），同样经 cs_uart_binary 发送。
 *
 * @see doc/CS_binary_protocol.md
 * @see doc/firmware_uart_binary_architecture.md
 */

#ifndef CS_DE_DATA_PARSE_H
#define CS_DE_DATA_PARSE_H

#include <bluetooth/cs_de.h>
#include <stdint.h>

/**
 * 全功能 CS（Legacy + RAS）二进制 UART 输出开关。
 * 1 — ranging 完成后经 console UART 发双端 IQ 二进制帧（见 doc/CS_binary_protocol.md）；
 * 0 — 保持 LOG_INF 文本输出（print_store_cs_de_report_basic）。
 * 与 DIP_REPORT_BINARY_OUTPUT 独立；仅 APP_CS_DIP_BYPASS_RAS=0 时生效。
 */
#ifndef CS_REPORT_BINARY_OUTPUT
#define CS_REPORT_BINARY_OUTPUT 1
#endif

/**
 * 1 — 组帧后 k_msgq 入队，由 cs_uart_tx 线程发送（推荐）；
 * 0 — 回调内阻塞 uart_poll_out。
 * 须与 main.c 中 CS_UART_BIN_USE_THREAD / DIP_BINARY_USE_THREAD 保持一致。
 */
#ifndef CS_BINARY_USE_THREAD
#define CS_BINARY_USE_THREAD 1
#endif

#ifndef CS_UART_BIN_USE_THREAD
#define CS_UART_BIN_USE_THREAD CS_BINARY_USE_THREAD
#endif

/** Flash / 串口输出用的测距记录包装（索引 + 时间戳 + 原始 cs_de 报告） */
typedef struct {
  uint64_t report_index;
  uint64_t timestamp_ms;
  cs_de_report_t report;
} store_cs_de_report_t;

void store_cs_de_report(cs_de_report_t *p_report);
void print_report(const cs_de_report_t *r, int max_output_channels);
void print_store_cs_de_report(const store_cs_de_report_t *s,
                              int max_output_channels);
void print_store_cs_de_report_basic(const store_cs_de_report_t *s,
                                   int max_output_channels);
void print_report_fast(const cs_de_report_t *r, int max_output_channels);

#if CS_REPORT_BINARY_OUTPUT
/**
 * @brief 将 store_cs_de_report 打包为 CS 双端 IQ 二进制帧并经 UART 输出
 *
 * 组帧 type=CS_UART_BIN_TYPE_CS_DUAL_IQ(0x02)；每有效 fft 信道 8 字节 IQ。
 * 调用上下文：ranging_data_get_complete_cb（Legacy 路径，ENABLE_DIRECT_PRINT=1）。
 *
 * @param store            含 cs_de_report 的存储结构（report 字段为 IQ 数据源）
 * @param ranging_counter  RAS ranging 计数，写入帧头 procedure_counter 字段
 */
void cs_output_store_report_binary(const store_cs_de_report_t *store,
                                   uint16_t ranging_counter);
#endif

#endif
