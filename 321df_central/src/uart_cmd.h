#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*uart_cmd_handler_t)(const char *cmd, int argc, const char *argv[]);

/* 初始化命令接收线程；handler 处理 "$CMD,<cmd>,k=v,k=v..." */
int uart_cmd_init(uart_cmd_handler_t handler);

/* 控制面输出（同一个串口，走 printk） */
void uart_cmd_send_ok(const char *cmd, const char *fmt, ...);
void uart_cmd_send_err(const char *cmd, int code, const char *fmt, ...);
void uart_cmd_send_evt(const char *topic, const char *fmt, ...);

#ifdef __cplusplus
}
#endif