#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 命令处理函数类型定义
 * @param cmd 命令名称（如 "PING", "BLE_SCAN" 等）
 * @param argc 参数个数（不包括命令名本身）
 * @param argv 参数数组，每个参数可能是 "key=value" 格式或普通字符串
 */
typedef void (*uart_cmd_handler_t)(const char *cmd, int argc, const char *argv[]);

/**
 * @brief 初始化串口命令接收系统
 * 
 * 该函数会：
 * 1. 初始化UART设备（使用设备树中定义的zephyr_console）
 * 2. 创建并启动一个后台线程用于解析命令
 * 3. 启用UART接收中断，将接收到的数据放入ring buffer
 * 4. 注册命令处理回调函数
 * 
 * @param handler 命令处理回调函数，当解析到有效命令时会调用此函数
 * @return 0 成功，负数表示错误码
 */
int uart_cmd_init(uart_cmd_handler_t handler);

/**
 * @brief 发送成功响应
 * 
 * 输出格式：$OK,<cmd>,<fmt内容>\r\n
 * 例如：$OK,PING,pong\r\n
 * 
 * @param cmd 命令名称
 * @param fmt 格式化字符串（可选，可为NULL或空字符串）
 * @param ... 格式化参数
 */
void uart_cmd_send_ok(const char *cmd, const char *fmt, ...);

/**
 * @brief 发送错误响应
 * 
 * 输出格式：$ERR,<cmd>,<code>,<fmt内容>\r\n
 * 例如：$ERR,BLE_SCAN,1,MISSING_PARAM:action\r\n
 * 
 * @param cmd 命令名称
 * @param code 错误码（整数）
 * @param fmt 错误描述格式化字符串
 * @param ... 格式化参数
 */
void uart_cmd_send_err(const char *cmd, int code, const char *fmt, ...);

/**
 * @brief 发送事件通知
 * 
 * 用于主动上报系统状态变化，如连接建立、断开等
 * 输出格式：$EVT,<topic>,<fmt内容>\r\n
 * 例如：$EVT,BLE,connected,addr=AA:BB:CC:DD:EE:FF\r\n
 * 
 * @param topic 事件主题（如 "BLE", "DF", "UART" 等）
 * @param fmt 事件描述格式化字符串（可选）
 * @param ... 格式化参数
 */
void uart_cmd_send_evt(const char *topic, const char *fmt, ...);

/* ---------- 参数解析辅助函数（通用工具） ---------- */

/**
 * @brief 从参数数组中查找key=value格式的参数
 * 
 * 在argv数组中查找格式为"key=value"的参数，返回value部分的指针
 * 例如：argv = ["action=start", "interval=100"]
 *      uart_cmd_kv_find(2, argv, "action") 返回 "start"
 * 
 * @param argc 参数个数
 * @param argv 参数数组
 * @param key 要查找的键名
 * @return 如果找到，返回value部分的指针；否则返回NULL
 */
const char *uart_cmd_kv_find(int argc, const char *argv[], const char *key);

/**
 * @brief 解析字符串为32位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u32(const char *s, uint32_t *out);

/**
 * @brief 解析字符串为16位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u16(const char *s, uint16_t *out);

/**
 * @brief 解析字符串为8位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u8(const char *s, uint8_t *out);

/**
 * @brief 解析字符串为32位有符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_i32(const char *s, int32_t *out);

#ifdef __cplusplus
}
#endif