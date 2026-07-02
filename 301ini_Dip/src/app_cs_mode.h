/**
 * @file app_cs_mode.h
 * @brief DIP / 全功能 CS 与 UART 输出的统一编译期开关
 *
 * 通常只需修改本文件顶部两个宏即可在模式间切换；其余宏由下方自动推导。
 * 若需细调单项（如仅关 DIP 二进制但保留文本详版），可在 #include 本头文件之前
 * 手动 #define 覆盖对应宏。
 *
 * @see README.md「模式切换清单」
 */

#ifndef APP_CS_MODE_H
#define APP_CS_MODE_H

/**
 * 1 — DIP：本地 subevent 直采，跳过 RAS（type=0x01 帧）
 * 0 — 全功能 CS：RAS + cs_de 双端测距（type=0x02 帧）
 */
#ifndef APP_CS_MODE_DIP
#define APP_CS_MODE_DIP 1
#endif

/**
 * 1 — console UART 输出二进制帧（推荐 PC 采集）
 * 0 — LOG_INF 文本报告
 */
#ifndef APP_CS_UART_BINARY
#define APP_CS_UART_BINARY 1
#endif

/* --- 推导宏（一般无需手改）--- */

#ifndef APP_CS_DIP_BYPASS_RAS
#define APP_CS_DIP_BYPASS_RAS APP_CS_MODE_DIP
#endif

#ifndef DIP_REPORT_BINARY_OUTPUT
#define DIP_REPORT_BINARY_OUTPUT (APP_CS_MODE_DIP && APP_CS_UART_BINARY)
#endif

#ifndef DIP_REPORT_LOG_VERBOSE
#define DIP_REPORT_LOG_VERBOSE (APP_CS_MODE_DIP && !APP_CS_UART_BINARY)
#endif

#ifndef CS_REPORT_BINARY_OUTPUT
#define CS_REPORT_BINARY_OUTPUT (!APP_CS_MODE_DIP && APP_CS_UART_BINARY)
#endif

/**
 * Legacy 路径串口直出（二进制或文本）须为 1；为 0 时改写 Flash。
 * DIP 二进制不依赖此项。
 */
#ifndef ENABLE_DIRECT_PRINT
#define ENABLE_DIRECT_PRINT 1
#endif

#endif /* APP_CS_MODE_H */
