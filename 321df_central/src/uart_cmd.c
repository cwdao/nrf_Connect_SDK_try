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

static bool starts_with(const char *s, const char *pfx)
{
    return (strncmp(s, pfx, strlen(pfx)) == 0);
}

/* 把 "a,b,c" 按逗号切成 token；会原地写 '\0' */
static int split_csv(char *s, const char *out[], int max_tokens)
{
    int n = 0;
    char *p = s;

    while (p && *p && n < max_tokens) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        out[n++] = p;

        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static void handle_line(char *line)
{
    if (!line || !line[0]) {
        return;
    }

    if (!starts_with(line, "$CMD,")) {
        return;
    }

    char *payload = line + 5;

    const char *tokens[CMD_MAX_TOKENS];
    int ntok = split_csv(payload, tokens, CMD_MAX_TOKENS);

    if (ntok < 1) {
        uart_cmd_send_err("CMD", 1, "BAD_FORMAT");
        return;
    }

    const char *cmd = tokens[0];
    const char **argv = &tokens[1];
    int argc = ntok - 1;

    if (g_handler) {
        g_handler(cmd, argc, argv);
    } else {
        uart_cmd_send_err(cmd, 2, "NO_HANDLER");
    }
}

/* UART IRQ 回调：尽快把 RX FIFO 里的字节搬进 ring buffer */
static void uart_isr_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            int r = uart_fifo_read(dev, buf, sizeof(buf));
            if (r > 0) {
                /* ring_buf_put 可能放不下：放不下就丢（并且唤醒线程处理） */
                uint32_t wrote = ring_buf_put(&rx_ring, buf, (uint32_t)r);
                (void)wrote;
                k_sem_give(&rx_sem);
            }
        }
    }
}

static void uart_cmd_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[CMD_MAX_LINE];
    size_t len = 0;

    uart_cmd_send_evt("UART", "cmd_thread_started");

    while (1) {
        /* 等待有 RX 数据 */
        k_sem_take(&rx_sem, K_FOREVER);

        /* 把 ring buffer 里的数据都取出来 */
        uint8_t ch;
        while (ring_buf_get(&rx_ring, &ch, 1) == 1) {

            if (ch == '\r') {
                continue;
            }

            if (ch == '\n') {
                if (len > 0) {
                    line[len] = '\0';
#if UART_CMD_DEBUG_LINE
                    printk("[LINE len=%u] '%s'\r\n", (unsigned)len, line);
#endif
                    handle_line(line);
                    len = 0;
                }
                continue;
            }

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

int uart_cmd_init(uart_cmd_handler_t handler)
{
#if !DT_NODE_HAS_STATUS(UART_DEV_NODE, okay)
    return -ENODEV;
#endif

    g_uart = DEVICE_DT_GET(UART_DEV_NODE);
    if (!device_is_ready(g_uart)) {
        return -ENODEV;
    }

    g_handler = handler;

    ring_buf_init(&rx_ring, sizeof(rx_ring_mem), rx_ring_mem);
    k_sem_init(&rx_sem, 0, 1);

    /* 启用 UART RX IRQ */
    uart_irq_callback_set(g_uart, uart_isr_cb);
    uart_irq_rx_enable(g_uart);

    k_thread_create(&uart_cmd_tid, uart_cmd_stack,
                    K_THREAD_STACK_SIZEOF(uart_cmd_stack),
                    uart_cmd_thread, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);
    k_thread_name_set(&uart_cmd_tid, "uart_cmd");

    return 0;
}

/* ---------- 输出封装 ---------- */

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

void uart_cmd_send_ok(const char *cmd, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprint_ok(cmd, fmt, ap);
    va_end(ap);
}

void uart_cmd_send_err(const char *cmd, int code, const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintk(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    printk("$ERR,%s,%d,%s\r\n", cmd, code, buf);
}

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