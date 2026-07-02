# 全功能 CS 二进制 UART 帧协议（v1）

本文档描述固件在 **`APP_CS_DIP_BYPASS_RAS=0`** 且 **`CS_REPORT_BINARY_OUTPUT=1`** 时，经 **console UART** 输出的 **全功能 CS 双端 IQ 二进制帧**格式。实现见 `src/cs_de_data_parse.c` 中 `cs_bin_*` 与 `cs_output_store_report_binary()`；UART 发送与 DIP 共用 `src/cs_uart_binary.c`。

**与 DIP 的关系**：帧头布局与 [DIP_binary_protocol.md](./DIP_binary_protocol.md) v2 **完全相同**，仅 `type` 与 IQ 载荷宽度不同。

---

## 1. 设计概要

| 项目 | 说明 |
|------|------|
| 用途 | RAS 对端数据到齐、`cs_de_populate_report` 完成后，将本地+远端 IQ 以二进制帧发给 PC，替代 `print_store_cs_de_report_basic` 文本输出 |
| 数据源 | `cs_de_report_t::iq_tones[ap]`（float 存 int16 PCT，组帧时四舍五入为 int16） |
| 触发点 | `ranging_data_get_complete_cb` → `cs_output_store_report_binary()` |
| 字节序 | 多字节字段均为 **little-endian** |
| 单 AP | 当前优先 `n_ap=1`；若 `n_ap>1` 仅发 `ap=0` 并打 `LOG_WRN` |
| 发送方式 | `CS_UART_BIN_USE_THREAD=1`（默认）：入 `k_msgq`，`cs_uart_tx` 线程 `uart_poll_out` |

---

## 2. 帧类型对照

| type | 名称 | IQ 载荷（每有效信道） | 文档 |
|------|------|----------------------|------|
| `0x01` | DIP 本地 IQ | `int16 i` + `int16 q`（4 B） | [DIP_binary_protocol.md](./DIP_binary_protocol.md) |
| `0x02` | CS 双端 IQ | `int16 i_local` + `q_local` + `i_remote` + `q_remote`（8 B） | 本文 |

同步字、版本、CRC、位图编码与 DIP v2 一致：`sync1=0x55`, `sync2=0xAA`, `version=0x02`。

---

## 3. 帧结构（type = 0x02）

```
偏移   长度    字段                 说明
----   ----    ----                 ----
0      1       sync1                固定 0x55
1      1       sync2                固定 0xAA
2      1       version              当前 0x02（0x01 为无 timestamp 的旧版）
3      1       type                 0x02 = CS 双端 IQ 帧
4      2       payload_len          LE；见 §4
6      2       procedure_counter    LE；RAS ranging_counter
8      8       timestamp_ms         LE；k_uptime_get() 毫秒（与文本/Flash 路径一致）
16     1       ap                   天线路径，当前多为 0
17     1       iq_format            0 = int16 I/Q
18     1       channel_count        有效 fft 信道数 N
19     1       reserved             填 0
20     10      channel_bitmap[10]   与 DIP 相同，见 DIP 文档 §3
30     8×N     IQ payload           按 ch 升序，每信道 4×int16
30+8N  2       crc16                LE；CRC16-CCITT，与 DIP 相同
```

**整帧长度** = `30 + 8×N + 2` = `32 + 8×N` 字节。

N=75（满信道）时约 **624** 字节（`CS_UART_BIN_MAX_FRAME_SIZE=640`）。

---

## 4. payload_len

从 **`procedure_counter` 第一个字节** 起，到 **IQ 区最后一字节** 止（**不含** sync～type、payload_len 自身、CRC）：

```
payload_len = 2 + 8 + 4 + 10 + (8 × channel_count)
            = 24 + 8×N
```

---

## 5. 信道位图与 fft 下标

与 DIP 完全一致：

- 10 字节位图，覆盖 fft 下标 `ch ∈ [0, 74]`。
- `channel_bitmap[ch/8]` 的 **bit (ch%8)** 为 1 表示该信道在 IQ 区有条目（**bit0 = LSB**）。
- `fft_ch = hci_channel - 2`（与 `cs_de.c` 中 `CHANNEL_INDEX_OFFSET` 一致）。
- 有效信道判定：该格点 `i_local/q_local/i_remote/q_remote` 任一非零（`cs_de_populate_report` 后未测信道保持 0）。

---

## 6. IQ 载荷（type = 0x02）

仅对 `channel_bitmap` 中为 1 的 `ch`，按 **ch 从小到大** 依次写入：

| 顺序 | 类型 | 说明 |
|------|------|------|
| 1 | int16 LE | i_local |
| 2 | int16 LE | q_local |
| 3 | int16 LE | i_remote |
| 4 | int16 LE | q_remote |

数值来自 `bt_le_cs_parse_pct()` 的 int16，经 `cs_de` 以 float 累积均值后再 **四舍五入** 回 int16。

---

## 7. CRC16

与 DIP 相同：CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`，覆盖 **sync1** 至 IQ 区末字节。实现：`cs_uart_bin_crc16_ccitt()`。

---

## 8. 固件宏

| 宏 | 位置 | 默认 | 说明 |
|----|------|------|------|
| `CS_REPORT_BINARY_OUTPUT` | `cs_de_data_parse.h` | 1 | 1=二进制；0=文本 `print_store_cs_de_report_basic` |
| `CS_BINARY_USE_THREAD` | `cs_de_data_parse.h` | 1 | 别名，应等于 `CS_UART_BIN_USE_THREAD` |
| `CS_UART_BIN_USE_THREAD` | `cs_uart_binary.h` / `main.c` | 1 | DIP/CS 共用发送线程 |
| `ENABLE_DIRECT_PRINT` | `flash_ops.h` | 1 | 须为 1 才会走串口直出（含二进制） |
| `APP_CS_DIP_BYPASS_RAS` | `main.c` | 1 | **须为 0** 才启用本协议路径 |

**Legacy 采集建议**：

```c
#define APP_CS_DIP_BYPASS_RAS 0
#define CS_REPORT_BINARY_OUTPUT 1
#define ENABLE_DIRECT_PRINT 1
```

并降低 `CONFIG_LOG` 等级，避免 ASCII 日志与 `0x55 0xAA` 混流。

---

## 9. 数据流

```
subevent_result_cb → latest_local_steps
ranging_data_ready_cb → bt_ras_rreq_cp_get_ranging_data
ranging_data_get_complete_cb
  → cs_de_populate_report + cs_de_calc
  → cs_output_store_report_binary
  → cs_bin_build_dual_iq_frame
  → cs_uart_bin_enqueue_frame
cs_uart_tx 线程 → uart_poll_out
```

---

## 10. 带宽对比（示意）

假设 N=72 个有效信道：

| 输出方式 | 约每帧字节 | 说明 |
|----------|-----------|------|
| CS 二进制 type=0x02 | 32 + 576 ≈ **608** | 固定帧，无 ASCII 开销 |
| DIP 二进制 type=0x01 | 32 + 288 ≈ **320** | 仅本地 IQ |
| 文本 `print_report_fast` | **数千～上万** | 每信道 ASCII 浮点 + LOG 前缀 |

---

## 11. PC 解析

参考脚本 [cs_parse_uart.py](./cs_parse_uart.py)；解析步骤与 [DIP_binary_pc_parser.md](./DIP_binary_pc_parser.md) 相同，按 `type` 分支 IQ 区宽度（4B 或 8B 每信道）。

---

## 12. 版本历史

| version | 说明 |
|---------|------|
| 0x02 | 增加 `timestamp_ms`（uint64 LE）；固定头 30 字节 |
| 0x01 | 首版：与 DIP v2 共用帧头；type=0x02；双端 int16 IQ + CRC16（无 timestamp） |
