# DIP 二进制 UART 帧协议（v2）

本文档描述固件在 `DIP_REPORT_BINARY_OUTPUT=1` 时，经 **console UART** 输出的 **DIP 本地 IQ 二进制帧**格式。实现见 `src/main.c` 中 `dip_bin_*` 与 `dip_output_local_report_binary()`。

**状态**：已在 nRF54L15 DK 上完成端到端验证（固件组帧 + PC 脚本 `dip_parse_uart.py` 解析/CRC 通过）。

---

## 1. 设计概要

| 项目 | 说明 |
|------|------|
| 用途 | 将一次 subevent 解析后的本地 IQ（PCT→int16）发给 PC，替代逐样本 `ap/ch/i/q` 列表 |
| 数据源 | `dip_local_step_iq_cb` 写入的 `ctx->ch[ap][ch].valid / i / q`，解析逻辑不变 |
| 字节序 | 多字节字段均为 **little-endian** |
| 单 AP | 当前优先 `n_ap=1`；若 `n_ap>1` 仅发 `ap=0` 并打 `LOG_WRN` |
| 发送方式 | `DIP_BINARY_USE_THREAD=1`（默认）：回调内打包入 `k_msgq`，独立线程 `uart_poll_out`；`=0`：回调内直接发送 |

---

## 2. 帧结构（按字节顺序）

```
偏移   长度    字段                 说明
----   ----    ----                 ----
0      1       sync1                固定 0x55
1      1       sync2                固定 0xAA
2      1       version              当前 0x01
3      1       type                 0x01 = DIP IQ 帧
4      2       payload_len          LE；见下文
6      2       procedure_counter    LE；CS procedure 计数
8      1       ap                   天线路径，当前多为 0
9      1       iq_format            0 = int16 I/Q
10     1       channel_count        有效 fft 信道数 N
11     1       reserved             填 0
12     10      channel_bitmap[10]   见 §3
22     4×N     IQ payload           按 ch 升序，每信道 int16 i + int16 q
22+4N  2       crc16                LE；CRC16-CCITT，见 §4
```

**整帧长度** = `22 + 4×N + 2` = `24 + 4×N` 字节。

固定头（含 bitmap）共 **22 字节**（`sizeof(dip_bin_header)`）。

---

## 3. payload_len 与信道位图

### 3.1 payload_len

从 **`procedure_counter` 第一个字节** 起，到 **IQ 区最后一字节** 止的字节数（**不含** sync1～type、payload_len 自身、CRC）：

```
payload_len = 2 + 4 + 10 + (4 × channel_count)
            = 16 + 4×N
```

### 3.2 channel_bitmap

- 共 10 字节 = 80 bit，覆盖 fft 下标 `ch ∈ [0, 74]`（`DIP_MAX_FFT_CHANNELS = 75`）。
- **bit 编码**：`channel_bitmap[ch / 8]` 的 **bit (ch % 8)** 为 1 表示该信道有效。
- **bit0 = LSB**（ch=0 → `bitmap[0]` bit0；ch=8 → `bitmap[1]` bit0）。
- `channel_count` 必须等于位图中置 1 的位数。

### 3.3 fft 信道与 HCI

与 `main.c` 中 DIP 解析一致：

```
fft_ch = hci_channel - DIP_CS_CHANNEL_INDEX_OFFSET   // OFFSET = 2
```

---

## 4. IQ 载荷

- 仅对 `channel_bitmap` 中为 1 的 `ch`，按 **ch 从小到大** 依次写入：
  - `int16 i`（LE）
  - `int16 q`（LE）
- `iq_format = 0` 时表示有符号 int16；负值按二补码位模式传输。

---

## 5. CRC16

| 项目 | 值 |
|------|-----|
| 算法 | CRC16-CCITT |
| 多项式 | 0x1021 |
| 初值 | 0xFFFF |
| 覆盖范围 | 从 **sync1** 到 **IQ 区最后一字节**（不含 CRC 两字节） |
| 帧尾 | CRC 以 uint16 **little-endian** 附加 |

固件实现：`dip_crc16_ccitt()`。

---

## 6. 固件宏（`src/main.c`）

| 宏 | 默认 | 说明 |
|----|------|------|
| `DIP_REPORT_BINARY_OUTPUT` | 1 | 1=二进制；0=文本/精简日志 |
| `DIP_BINARY_USE_THREAD` | 1 | 1=msgq+发送线程；0=回调内阻塞 UART |
| `DIP_REPORT_LOG_VERBOSE` | 1 | 与二进制互斥；二进制开启时不走文本 IQ 打印 |

**测试建议**：二进制采集时设 `DIP_REPORT_LOG_VERBOSE=0`，并降低 `LOG_INF` 等级，避免 ASCII 日志与二进制混流。

---

## 7. 线程化发送（非阻塞路径，默认）

架构与《快速二进制输出 2.0.md》**阶段 2** 一致（`k_msgq` + 独立发送线程），帧载荷为本文 **v2 bitmap** 格式，而非该文档中的 sample-list 示例。

```
subevent_result_dip_cb (BT RX WQ)
  → dip_parse_local_iq_from_subevent
  → dip_output_local_report_binary
  → dip_enqueue_local_report_binary
  → dip_bin_build_frame → dip_bin_msg_scratch
  → k_msgq_put(..., K_NO_WAIT)   // 满则 LOG_WRN 丢帧，不阻塞

dip_uart_tx 线程（main 中 k_thread_create，名 dip_uart_tx）
  → k_msgq_get(..., K_FOREVER)
  → dip_uart_send_bytes → uart_poll_out
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 消息结构 | `struct dip_bin_msg` | `uint16_t len` + `uint8_t data[400]` |
| 队列 | `dip_bin_msgq` | `K_MSGQ_DEFINE(..., 8, 4)` |
| 打包缓冲 | `dip_bin_msg_scratch` | 静态 BSS，避免 BT RX WQ 栈上 ~402B |
| 线程栈 | 1024 B | `dip_uart_tx_stack` |
| 线程优先级 | 7 | 低于 BT、高于 idle（依 Zephyr 配置） |
| 阻塞回退 | `DIP_BINARY_USE_THREAD=0` | 回调内 `dip_send_local_report_binary` |

### 7.1 固件符号对照

| 符号 | 角色 |
|------|------|
| `dip_bin_fill_channel_bitmap` | valid → bitmap + channel_count |
| `dip_bin_pack_iq_payload` | 按 ch 升序写 int16 i,q |
| `dip_bin_build_frame` | 组帧 + CRC（两路径共用） |
| `dip_crc16_ccitt` | CRC16-CCITT，初值 0xFFFF |
| `dip_enqueue_local_report_binary` | Producer：打包入队 |
| `dip_uart_tx_thread` | Consumer：出队发 UART |
| `dip_output_local_report_binary` | 解析完成后的统一入口 |

### 7.2 丢帧与调优

- 串口出现 `DIP binary msgq full, drop pc=…` 表示 Producer 快于 Consumer。
- 可增大 `DIP_BIN_MSGQ_DEPTH`、降低 CS 报告率，或后续改用独立 UART / 更高波特率。
- 采集时建议 `DIP_REPORT_LOG_VERBOSE=0`，减少 ASCII 与二进制混流。

---

## 8. 示例（N=2，ch=3 与 ch=10 有效）

假设 `procedure_counter=100`，`ap=0`，ch3 的 i=100,q=-50，ch10 的 i=200,q=300：

- `channel_count = 2`
- `bitmap[0]=0x08`（bit3），`bitmap[1]=0x04`（bit2 即 ch10）
- `payload_len = 16 + 8 = 24`
- 帧长约 `22 + 8 + 2 = 32` 字节（含 CRC）

PC 端解析步骤见 [DIP_binary_pc_parser.md](./DIP_binary_pc_parser.md)；参考脚本 [dip_parse_uart.py](./dip_parse_uart.py)。

---

## 9. 与 v1 sample-list 的差异

| 项目 | v1（设计稿 sample-list） | v2（当前固件） |
|------|--------------------------|----------------|
| 有效信道 | 每样本 1B ap + 1B ch | 10B channel_bitmap |
| IQ 区 | 每样本 4B | 仅有效信道，4×N B |
| 头部 | n_ap + sample_count | ap + channel_count + iq_format |
| 典型帧长 | 随样本线性增长 | 22 + 4N + 2，N≤75 |

---

## 10. 版本历史

| version | 说明 |
|---------|------|
| 0x01 | single AP + bitmap + int16 IQ + CRC16；默认 msgq 非阻塞发送 |
