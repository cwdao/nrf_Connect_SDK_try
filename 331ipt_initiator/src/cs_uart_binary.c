/**
 * @file cs_uart_binary.c
 * @brief UART 二进制传输层实现：CRC、消息队列、cs_uart_tx 消费者线程
 *
 * 架构（CS_UART_BIN_USE_THREAD=1，默认）：
 *
 *   [Producer，运行于 BT 回调 / ranging 完成回调]
 *        cs_uart_bin_enqueue_frame(完整帧)
 *             → k_msgq_put(K_NO_WAIT)   // 满则返回 -ENOMEM，不阻塞蓝牙
 *
 *   [Consumer：线程 cs_uart_tx，main() 启动]
 *        k_msgq_get(K_FOREVER)
 *             → cs_uart_bin_send_bytes()
 *             → uart_poll_out(zephyr_console)
 *
 * 设计约束：
 *   - 禁止在 Producer 路径使用 K_FOREVER 入队，避免串口慢导致 BT RX WQ 饿死；
 *   - 帧缓冲由上层（main.c / cs_de_data_parse.c）静态分配，本模块队列消息内嵌
 *     data[CS_UART_BIN_MAX_FRAME_SIZE] 拷贝副本；
 *   - 与 Zephyr LOG_* 共用 console UART，采集时须降低日志等级。
 *
 * @see cs_uart_binary.h
 * @see doc/firmware_uart_binary_architecture.md
 */

#include "cs_uart_binary.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(app_main);

/** 待发帧队列深度；突发超过「深度 × 消费速度」时 Producer 丢帧 */
#ifndef CS_UART_BIN_MSGQ_DEPTH
#define CS_UART_BIN_MSGQ_DEPTH 8U
#endif

/** 发送线程栈：含 cs_uart_bin_msg 局部副本 + uart_poll_out 调用链 */
#ifndef CS_UART_BIN_TX_THREAD_STACK_SIZE
#define CS_UART_BIN_TX_THREAD_STACK_SIZE 1024
#endif

/**
 * 发送线程优先级：低于 BT 栈线程，避免长期抢占；高于 idle 以保证及时 drain。
 * 具体数值语义依 Zephyr CONFIG_NUM_PREEMPT_PRIORITIES 配置。
 */
#ifndef CS_UART_BIN_TX_THREAD_PRIORITY
#define CS_UART_BIN_TX_THREAD_PRIORITY 7
#endif

/** 与 devicetree chosen zephyr_console 绑定，二进制与日志可能混流 */
static const struct device *cs_uart_bin_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#if CS_UART_BIN_USE_THREAD

/** 入队消息：已打包的完整帧；k_msgq 会拷贝整个结构体 */
struct cs_uart_bin_msg {
  uint16_t len;
  uint8_t data[CS_UART_BIN_MAX_FRAME_SIZE];
};

K_MSGQ_DEFINE(cs_uart_bin_msgq, sizeof(struct cs_uart_bin_msg), CS_UART_BIN_MSGQ_DEPTH, 4);
K_THREAD_STACK_DEFINE(cs_uart_bin_tx_stack, CS_UART_BIN_TX_THREAD_STACK_SIZE);
static struct k_thread cs_uart_bin_tx_thread_data;
static bool cs_uart_bin_thread_started;

/**
 * @brief UART 发送消费者线程入口（线程名 cs_uart_tx）
 *
 * 永久循环：出队 → 逐字节 uart_poll_out。不理解帧 type 与 IQ 语义。
 */
static void cs_uart_bin_tx_thread(void *p1, void *p2, void *p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  struct cs_uart_bin_msg msg;

  while (true) {
    if (k_msgq_get(&cs_uart_bin_msgq, &msg, K_FOREVER) == 0) {
      cs_uart_bin_send_bytes(msg.data, msg.len);
    }
  }
}
#endif /* CS_UART_BIN_USE_THREAD */

uint16_t cs_uart_bin_crc16_ccitt(const uint8_t *data, size_t len)
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

void cs_uart_bin_send_bytes(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0U) {
    return;
  }

  if (!device_is_ready(cs_uart_bin_dev)) {
    return;
  }

  for (size_t i = 0; i < len; i++) {
    uart_poll_out(cs_uart_bin_dev, data[i]);
  }
}

int cs_uart_bin_enqueue_frame(const uint8_t *frame, uint16_t len)
{
  if (frame == NULL || len == 0U || len > CS_UART_BIN_MAX_FRAME_SIZE) {
    return -EINVAL;
  }

#if CS_UART_BIN_USE_THREAD
  struct cs_uart_bin_msg msg;

  msg.len = len;
  memcpy(msg.data, frame, len);

  if (k_msgq_put(&cs_uart_bin_msgq, &msg, K_NO_WAIT) != 0) {
    return -ENOMEM;
  }

  return 0;
#else
  cs_uart_bin_send_bytes(frame, len);
  return 0;
#endif
}

void cs_uart_bin_tx_thread_start(void)
{
#if CS_UART_BIN_USE_THREAD
  if (cs_uart_bin_thread_started) {
    return;
  }

  k_thread_create(&cs_uart_bin_tx_thread_data, cs_uart_bin_tx_stack,
                  K_THREAD_STACK_SIZEOF(cs_uart_bin_tx_stack), cs_uart_bin_tx_thread, NULL, NULL,
                  NULL, CS_UART_BIN_TX_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&cs_uart_bin_tx_thread_data, "cs_uart_tx");
  cs_uart_bin_thread_started = true;
  LOG_INF("CS UART binary TX thread started (msgq depth %u)", (unsigned)CS_UART_BIN_MSGQ_DEPTH);
#else
  LOG_INF("CS UART binary: direct (blocking) TX mode");
#endif
}
