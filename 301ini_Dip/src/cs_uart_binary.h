/**
 * @file cs_uart_binary.h
 * @brief DIP / 全功能 CS 共用的 UART 二进制传输层（与业务组帧解耦）
 *
 * 职责边界：
 *   - 本模块只负责「已组好帧」的 CRC 辅助、入队与 uart_poll_out 发送；
 *   - 不解析蓝牙 HCI、不访问 cs_de_report、不决定 IQ 语义。
 *
 * 组帧方（上层）：
 *   - DIP 本地 IQ（type 0x01）：main.c 中 dip_bin_build_frame()
 *   - CS 双端 IQ（type 0x02）：cs_de_data_parse.c 中 cs_bin_build_dual_iq_frame()
 *
 * 线上协议字段定义见 doc/DIP_binary_protocol.md、doc/CS_binary_protocol.md。
 * 模块架构说明见 doc/firmware_uart_binary_architecture.md。
 */

#ifndef CS_UART_BINARY_H
#define CS_UART_BINARY_H

#include <stddef.h>
#include <stdint.h>

/** 帧同步字节 1；与 sync2 组合供 PC 端在串口流中定位帧起点 */
#define CS_UART_BIN_SYNC1 0x55U
/** 帧同步字节 2 */
#define CS_UART_BIN_SYNC2 0xAAU
/** 协议版本号；帧头字段含义变更时递增，PC 端可据此分支解析 */
#define CS_UART_BIN_VERSION 0x01U

/** 信道位图字节数：75 个 fft 格点 → ceil(75/8)=10 字节（80 bit，高 5 bit 保留） */
#define CS_UART_BIN_CHANNEL_BITMAP_BYTES 10U
/** fft 信道下标上界（不含）；与 cs_de.c NUM_CHANNELS、DIP_MAX_FFT_CHANNELS 对齐 */
#define CS_UART_BIN_MAX_FFT_CHANNELS 75U

/**
 * 单帧最大长度上界。
 * 头(22) + 全信道双端 IQ(75×8=600) + CRC(2) = 624，取 640 留余量。
 * DIP 本地帧（75×4）亦在此上限内。
 */
#define CS_UART_BIN_MAX_FRAME_SIZE 640U

/** 帧类型：DIP 本地 IQ，每有效信道 4 字节（int16 i + int16 q） */
#define CS_UART_BIN_TYPE_DIP_LOCAL_IQ 0x01U
/** 帧类型：全功能 CS 双端 IQ，每有效信道 8 字节（i_local,q_local,i_remote,q_remote） */
#define CS_UART_BIN_TYPE_CS_DUAL_IQ 0x02U

/** iq_format 字段：0 表示 IQ 载荷为有符号 int16、little-endian */
#define CS_UART_BIN_IQ_FORMAT_INT16 0U

/**
 * 发送路径选择（须在编译 cs_uart_binary.c 前与各路径宏保持一致）：
 *   1 — Producer 仅 k_msgq_put(K_NO_WAIT)，Consumer 线程 cs_uart_tx 负责 uart_poll_out；
 *   0 — cs_uart_bin_enqueue_frame() 内直接阻塞发送（便于对比，不推荐量产采集）。
 */
#ifndef CS_UART_BIN_USE_THREAD
#define CS_UART_BIN_USE_THREAD 1
#endif

/**
 * @brief 二进制 UART 帧固定头（packed，无编译器填充）
 *
 * 内存布局（22 字节）后紧接变长 IQ 区，再跟 2 字节 CRC16-CCITT（LE）。
 * IQ 区不嵌入本结构体；组帧时先写头与 IQ，再对 [sync1 .. IQ末] 算 CRC。
 *
 * payload_len：从 procedure_counter 首字节到 IQ 区最后一字节的长度（不含 sync..type、
 *               payload_len 自身、CRC）。
 */
struct __packed cs_uart_bin_header {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t version;
  uint8_t type;
  uint16_t payload_len;
  /** DIP：CS procedure_counter；CS 双端：RAS ranging_counter */
  uint16_t procedure_counter;
  uint8_t ap;
  uint8_t iq_format;
  uint8_t channel_count;
  uint8_t reserved;
  uint8_t channel_bitmap[CS_UART_BIN_CHANNEL_BITMAP_BYTES];
};

/**
 * @brief CRC16-CCITT（多项式 0x1021，初值 0xFFFF）
 * @param data 参与校验的数据首地址
 * @param len  字节长度
 * @return 16 位 CRC；data 为 NULL 时返回初值 0xFFFF
 *
 * 覆盖范围须与 PC 端一致：从帧首 sync1 到 IQ 区最后一字节（不含 CRC 两字节）。
 */
uint16_t cs_uart_bin_crc16_ccitt(const uint8_t *data, size_t len);

/**
 * @brief 经 console UART 逐字节阻塞发送
 * @param data 待发送缓冲区；NULL 或 len==0 时直接返回
 * @param len  字节数
 *
 * 使用 Zephyr uart_poll_out；设备未就绪时静默返回。
 * 线程版在 cs_uart_tx 中调用；阻塞回退路径在 Producer 上下文调用。
 */
void cs_uart_bin_send_bytes(const uint8_t *data, size_t len);

/**
 * @brief 将完整帧交给传输层（入队或直发）
 * @param frame 已组好的帧（含 CRC）
 * @param len   帧长度，须 ≤ CS_UART_BIN_MAX_FRAME_SIZE
 * @return 0 成功；-EINVAL 参数无效；-ENOMEM 队列满（线程模式下丢帧，不阻塞）
 */
int cs_uart_bin_enqueue_frame(const uint8_t *frame, uint16_t len);

/**
 * @brief 启动 cs_uart_tx 发送线程
 *
 * 须在 main() 中 bt_enable() 之前调用一次；重复调用无害。
 * CS_UART_BIN_USE_THREAD=0 时仅打日志，不创建线程。
 */
void cs_uart_bin_tx_thread_start(void);

#endif /* CS_UART_BINARY_H */
