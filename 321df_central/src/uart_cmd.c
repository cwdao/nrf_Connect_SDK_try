#include "uart_cmd.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <limits.h>

#define UART_DEV_NODE DT_CHOSEN(zephyr_console)

#define CMD_MAX_LINE     192
#define CMD_MAX_TOKENS   24

/* RX ring buffer：存原始字节流 */
#define RX_RING_SZ 512

/* 调试开关：1=打印每次解析到的整行 */
#define UART_CMD_DEBUG_LINE 1

static const struct device *g_uart;
static uart_cmd_handler_t g_handler;

K_THREAD_STACK_DEFINE(uart_cmd_stack, 1024);
static struct k_thread uart_cmd_tid;

static uint8_t rx_ring_mem[RX_RING_SZ];
static struct ring_buf rx_ring;
static struct k_sem rx_sem;

/**
 * @brief 检查字符串是否以指定前缀开头
 * @param s 待检查的字符串
 * @param pfx 前缀字符串
 * @return true 如果s以pfx开头，否则返回false
 */
static bool starts_with(const char *s, const char *pfx)
{
    return (strncmp(s, pfx, strlen(pfx)) == 0);
}

/**
 * @brief 将CSV格式字符串按逗号分割成多个token
 * 
 * 该函数会原地修改字符串s，将逗号替换为'\0'，并在out数组中存储每个token的起始地址
 * 例如："PING,key1=val1,key2=val2" 会被分割为：
 *   out[0] = "PING"
 *   out[1] = "key1=val1"
 *   out[2] = "key2=val2"
 * 
 * @param s 待分割的字符串（会被原地修改）
 * @param out 输出数组，存储每个token的指针
 * @param max_tokens out数组的最大容量
 * @return 实际分割出的token数量
 */
static int split_csv(char *s, const char *out[], int max_tokens)
{
    int n = 0;
    char *p = s;

    while (p && *p && n < max_tokens) {
        /* 跳过前导空白字符 */
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        out[n++] = p;

        /* 查找下一个逗号 */
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        /* 将逗号替换为字符串结束符 */
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

/**
 * @brief 处理一行完整的命令字符串
 * 
 * 命令格式：$CMD,<命令名>,<参数1>,<参数2>,...
 * 例如：$CMD,PING 或 $CMD,BLE_SCAN,action=start
 * 
 * 处理流程：
 * 1. 检查是否以"$CMD,"开头
 * 2. 提取命令体（去掉"$CMD,"前缀）
 * 3. 按逗号分割成多个token
 * 4. 第一个token作为命令名，其余作为参数
 * 5. 调用注册的命令处理函数
 * 
 * @param line 完整的命令行（以'\0'结尾）
 */
static void handle_line(char *line)
{
    /* 空行检查 */
    if (!line || !line[0]) {
        return;
    }

    /* 检查命令前缀：必须以"$CMD,"开头 */
    if (!starts_with(line, "$CMD,")) {
        return;
    }

    /* 提取命令体（跳过"$CMD,"这5个字符） */
    char *payload = line + 5;

    /* 分割命令体为多个token */
    const char *tokens[CMD_MAX_TOKENS];
    int ntok = split_csv(payload, tokens, CMD_MAX_TOKENS);

    /* 至少需要一个token（命令名） */
    if (ntok < 1) {
        uart_cmd_send_err("CMD", 1, "BAD_FORMAT");
        return;
    }

    /* 第一个token是命令名，其余是参数 */
    const char *cmd = tokens[0];
    const char **argv = &tokens[1];
    int argc = ntok - 1;

    /* 调用注册的命令处理函数 */
    if (g_handler) {
        g_handler(cmd, argc, argv);
    } else {
        uart_cmd_send_err(cmd, 2, "NO_HANDLER");
    }
}

/**
 * @brief UART接收中断回调函数
 * 
 * 该函数在UART接收到数据时被硬件中断调用，主要职责：
 * 1. 从UART硬件FIFO中读取接收到的字节
 * 2. 将字节数据放入ring buffer（环形缓冲区）
 * 3. 通过信号量唤醒命令解析线程
 * 
 * 注意：如果ring buffer已满，新数据会被丢弃，但仍会唤醒线程处理已有数据
 * 
 * @param dev UART设备指针
 * @param user_data 用户数据（未使用）
 */
static void uart_isr_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    /* 检查并处理所有待处理的UART中断 */
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        /* 检查是否有接收数据就绪 */
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            /* 从UART FIFO读取数据（最多64字节） */
            int r = uart_fifo_read(dev, buf, sizeof(buf));
            if (r > 0) {
                /* 将数据放入ring buffer（如果满了会丢弃部分数据） */
                uint32_t wrote = ring_buf_put(&rx_ring, buf, (uint32_t)r);
                (void)wrote;  /* 忽略实际写入的字节数 */
                /* 唤醒命令解析线程 */
                k_sem_give(&rx_sem);
            }
        }
    }
}

/**
 * @brief 串口命令解析线程主函数
 * 
 * 该线程负责：
 * 1. 从ring buffer中读取接收到的字节
 * 2. 按行组装命令（以'\n'为行结束符）
 * 3. 调用handle_line处理完整的命令行
 * 
 * 工作流程：
 * - 等待信号量（表示有数据到达）
 * - 从ring buffer逐个读取字节
 * - 忽略'\r'字符（回车符）
 * - 遇到'\n'时，将已积累的字符作为一行命令处理
 * - 如果行长度超过限制，丢弃该行并报错
 * 
 * @param a 线程参数1（未使用）
 * @param b 线程参数2（未使用）
 * @param c 线程参数3（未使用）
 */
static void uart_cmd_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[CMD_MAX_LINE];  /* 命令行缓冲区 */
    size_t len = 0;            /* 当前行已积累的字符数 */

    uart_cmd_send_evt("UART", "cmd_thread_started");

    while (1) {
        /* 等待信号量：表示有新的接收数据到达 */
        k_sem_take(&rx_sem, K_FOREVER);

        /* 从ring buffer中逐个读取字节 */
        uint8_t ch;
        while (ring_buf_get(&rx_ring, &ch, 1) == 1) {

            /* 忽略回车符（Windows换行是\r\n，Linux是\n） */
            if (ch == '\r') {
                continue;
            }

            /* 遇到换行符，表示一行命令结束 */
            if (ch == '\n') {
                if (len > 0) {
                    /* 添加字符串结束符 */
                    line[len] = '\0';
#if UART_CMD_DEBUG_LINE
                    printk("[LINE len=%u] '%s'\r\n", (unsigned)len, line);
#endif
                    /* 处理这一行命令 */
                    handle_line(line);
                    len = 0;  /* 重置行长度，准备接收下一行 */
                }
                continue;
            }

            /* 积累字符到行缓冲区 */
            if (len < (CMD_MAX_LINE - 1)) {
                line[len++] = (char)ch;
            } else {
                /* 行太长：丢弃本行直到下一个 '\n' */
                len = 0;
                uart_cmd_send_err("CMD", 3, "LINE_TOO_LONG");
            }
        }
    }
}

/**
 * @brief 初始化串口命令接收系统
 * 
 * 初始化步骤：
 * 1. 获取UART设备（从设备树中获取zephyr_console节点）
 * 2. 检查设备是否就绪
 * 3. 保存命令处理回调函数
 * 4. 初始化ring buffer和信号量
 * 5. 设置UART中断回调并启用接收中断
 * 6. 创建并启动命令解析线程
 * 
 * @param handler 命令处理回调函数
 * @return 0 成功，-ENODEV 设备不存在或未就绪
 */
int uart_cmd_init(uart_cmd_handler_t handler)
{
    /* 检查设备树中是否定义了UART设备节点 */
#if !DT_NODE_HAS_STATUS(UART_DEV_NODE, okay)
    return -ENODEV;
#endif

    /* 获取UART设备指针 */
    g_uart = DEVICE_DT_GET(UART_DEV_NODE);
    if (!device_is_ready(g_uart)) {
        return -ENODEV;
    }

    /* 保存命令处理回调函数 */
    g_handler = handler;

    /* 初始化ring buffer（用于存储接收到的原始字节流） */
    ring_buf_init(&rx_ring, sizeof(rx_ring_mem), rx_ring_mem);
    /* 初始化信号量（初始值为0，最大值为1） */
    k_sem_init(&rx_sem, 0, 1);

    /* 设置UART中断回调函数 */
    uart_irq_callback_set(g_uart, uart_isr_cb);
    /* 启用UART接收中断 */
    uart_irq_rx_enable(g_uart);

    /* 创建命令解析线程 */
    k_thread_create(&uart_cmd_tid, uart_cmd_stack,
                    K_THREAD_STACK_SIZEOF(uart_cmd_stack),
                    uart_cmd_thread, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);  /* 优先级7，立即启动 */
    k_thread_name_set(&uart_cmd_tid, "uart_cmd");

    return 0;
}

/* ---------- 输出封装 ---------- */

/**
 * @brief 内部函数：发送成功响应的可变参数版本
 * @param cmd 命令名称
 * @param fmt 格式化字符串（可选）
 * @param ap 可变参数列表
 */
static void vprint_ok(const char *cmd, const char *fmt, va_list ap)
{
    if (fmt && fmt[0]) {
        char buf[160];
        vsnprintk(buf, sizeof(buf), fmt, ap);
        printk("$OK,%s,%s\r\n", cmd, buf);
    } else {
        printk("$OK,%s\r\n", cmd);
    }
}

/**
 * @brief 发送成功响应（公共接口）
 * 
 * 输出格式：$OK,<cmd>,<内容>\r\n
 * 示例：$OK,PING,pong\r\n
 * 
 * @param cmd 命令名称
 * @param fmt 格式化字符串（可选，可为NULL）
 * @param ... 格式化参数
 */
void uart_cmd_send_ok(const char *cmd, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprint_ok(cmd, fmt, ap);
    va_end(ap);
}

/**
 * @brief 发送错误响应
 * 
 * 输出格式：$ERR,<cmd>,<code>,<错误描述>\r\n
 * 示例：$ERR,BLE_SCAN,1,MISSING_PARAM:action\r\n
 * 
 * @param cmd 命令名称
 * @param code 错误码（整数）
 * @param fmt 错误描述格式化字符串
 * @param ... 格式化参数
 */
void uart_cmd_send_err(const char *cmd, int code, const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintk(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    printk("$ERR,%s,%d,%s\r\n", cmd, code, buf);
}

/**
 * @brief 发送事件通知
 * 
 * 用于主动上报系统状态变化，如连接建立、断开、扫描开始等
 * 输出格式：$EVT,<topic>,<内容>\r\n
 * 示例：$EVT,BLE,connected,addr=AA:BB:CC:DD:EE:FF\r\n
 * 
 * @param topic 事件主题（如 "BLE", "DF", "UART" 等）
 * @param fmt 事件描述格式化字符串（可选）
 * @param ... 格式化参数
 */
void uart_cmd_send_evt(const char *topic, const char *fmt, ...)
{
    if (fmt && fmt[0]) {
        char buf[180];
        va_list ap;
        va_start(ap, fmt);
        vsnprintk(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        printk("$EVT,%s,%s\r\n", topic, buf);
    } else {
        printk("$EVT,%s\r\n", topic);
    }
}

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
const char *uart_cmd_kv_find(int argc, const char *argv[], const char *key)
{
    size_t klen = strlen(key);
    for (int i = 0; i < argc; i++) {
        const char *s = argv[i];
        if (!s) continue;
        /* 检查是否以"key="开头 */
        if (!strncmp(s, key, klen) && s[klen] == '=') {
            return s + klen + 1;  /* 返回'='后面的value部分 */
        }
    }
    return NULL;
}

/**
 * @brief 解析字符串为32位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u32(const char *s, uint32_t *out)
{
    if (!s || !out) return -EINVAL;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    /* 检查是否完全解析（end指向字符串末尾） */
    if (end == s || *end != '\0') return -EINVAL;
    *out = (uint32_t)v;
    return 0;
}

/**
 * @brief 解析字符串为16位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u16(const char *s, uint16_t *out)
{
    if (!s || !out) return -EINVAL;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') return -EINVAL;
    if (v > 0xFFFF) return -ERANGE;  /* 超出uint16_t范围 */
    *out = (uint16_t)v;
    return 0;
}

/**
 * @brief 解析字符串为8位无符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_u8(const char *s, uint8_t *out)
{
    if (!s || !out) return -EINVAL;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') return -EINVAL;
    if (v > 0xFF) return -ERANGE;  /* 超出uint8_t范围 */
    *out = (uint8_t)v;
    return 0;
}

/**
 * @brief 解析字符串为32位有符号整数
 * 
 * @param s 待解析的字符串
 * @param out 输出参数，存储解析结果
 * @return 0 成功，-EINVAL 参数无效或解析失败
 */
int uart_cmd_parse_i32(const char *s, int32_t *out)
{
    if (!s || !out) return -EINVAL;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return -EINVAL;
    if (v < INT32_MIN || v > INT32_MAX) return -ERANGE;
    *out = (int32_t)v;
    return 0;
}