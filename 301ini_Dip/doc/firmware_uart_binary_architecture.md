# 固件 UART 二进制模块架构

本文描述工程中 **DIP** 与 **全功能 CS** 两条路径如何共用串口二进制输出，以及 `cs_uart_binary` 与 `cs_de_data_parse` / `main.c` 的职责划分。面向固件维护者与上位机协议开发者。

---

## 1. 分层总览

```
┌─────────────────────────────────────────────────────────────────┐
│  Layer 3：业务数据源                                              │
├────────────────────────────┬────────────────────────────────────┤
│ DIP（APP_CS_DIP_BYPASS_RAS=1）│ Legacy CS（APP_CS_DIP_BYPASS_RAS=0）│
│ HCI subevent → PCT → IQ    │ RAS + cs_de_populate_report        │
│ dip_step_parse_ctx         │ cs_de_report_t                     │
└─────────────┬──────────────┴──────────────────┬─────────────────┘
              │                                  │
┌─────────────▼──────────────┐    ┌──────────────▼─────────────────┐
│ Layer 2：组帧（协议语义）     │    │ Layer 2：组帧（协议语义）       │
│ main.c                     │    │ cs_de_data_parse.c             │
│ dip_bin_build_frame()      │    │ cs_bin_build_dual_iq_frame()   │
│ type 0x01, 4B/ch           │    │ type 0x02, 8B/ch               │
└─────────────┬──────────────┘    └──────────────┬─────────────────┘
              │                                  │
              └──────────────┬───────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ Layer 1：传输（cs_uart_binary.c / cs_uart_binary.h）            │
│   cs_uart_bin_crc16_ccitt()                                     │
│   cs_uart_bin_enqueue_frame()  →  k_msgq (K_NO_WAIT)            │
│   cs_uart_tx 线程 → cs_uart_bin_send_bytes() → uart_poll_out    │
└─────────────────────────────────────────────────────────────────┘
                             ▼
                    Zephyr console UART
```

**原则**：传输层不理解 IQ 含义；组帧层不关心 UART 硬件与线程细节。

---

## 2. 模块对照表

| 模块 | 文件 | 职责 | 不负责 |
|------|------|------|--------|
| **cs_uart_binary** | `src/cs_uart_binary.c` `.h` | 帧头常量、`cs_uart_bin_header`、CRC、msgq、发送线程、入队 API | 蓝牙解析、cs_de、IQ 来源 |
| **cs_de_data_parse** | `src/cs_de_data_parse.c` `.h` | `cs_de_report` 文本打印；type `0x02` 双端 IQ 组帧；`store_cs_de_report_t` | HCI/RAS 原始解析；DIP 组帧 |
| **main.c（DIP 段）** | `src/main.c` | DIP 解析、`dip_step_parse_ctx`；type `0x01` 本地 IQ 组帧 | 全功能 CS 双端 IQ |
| **Nordic cs_de** | SDK | `cs_de_populate_report` / `cs_de_calc` | UART 输出 |

---

## 3. 帧类型与调用链

### 3.1 DIP 本地 IQ（type = 0x01）

```
subevent_result_dip_cb
  → dip_parse_local_iq_from_subevent
  → dip_output_local_report_binary
  → dip_bin_build_frame
  → cs_uart_bin_enqueue_frame
  → cs_uart_tx → UART
```

- 宏：`DIP_REPORT_BINARY_OUTPUT`
- 文档：[DIP_binary_protocol.md](./DIP_binary_protocol.md)

### 3.2 全功能 CS 双端 IQ（type = 0x02）

```
subevent_result_cb → latest_local_steps
ranging_data_ready_cb → RAS 拉取对端
ranging_data_get_complete_cb
  → cs_de_populate_report / cs_de_calc
  → cs_output_store_report_binary
  → cs_bin_build_dual_iq_frame
  → cs_uart_bin_enqueue_frame
  → cs_uart_tx → UART
```

- 宏：`CS_REPORT_BINARY_OUTPUT`、`ENABLE_DIRECT_PRINT`
- 文档：[CS_binary_protocol.md](./CS_binary_protocol.md)

---

## 4. 共用帧头（30 字节，version 0x02）

定义于 `cs_uart_binary.h` 的 `struct cs_uart_bin_header`。  
DIP 与 CS 仅在下述字段有差异：

| 字段 | DIP (0x01) | CS (0x02) |
|------|------------|-----------|
| `type` | `0x01` | `0x02` |
| `procedure_counter` | CS procedure 计数 | RAS `ranging_counter` |
| `timestamp_ms` | 组帧时 `k_uptime_get()` | `store_cs_de_report_t::timestamp_ms` |
| IQ 区 | 每信道 4 B | 每信道 8 B |
| `payload_len` | `24 + 4×N` | `24 + 8×N` |

CRC、位图编码、字节序规则相同。

---

## 5. 线程与宏配置

| 宏 | 典型位置 | 说明 |
|----|----------|------|
| `CS_UART_BIN_USE_THREAD` | `cs_uart_binary.h` / `main.c` | 1=msgq+线程；0=回调内阻塞发送 |
| `DIP_BINARY_USE_THREAD` | `main.c` | 默认映射到 `CS_UART_BIN_USE_THREAD` |
| `CS_BINARY_USE_THREAD` | `cs_de_data_parse.h` | 同上，须保持一致 |
| `DIP_REPORT_BINARY_OUTPUT` | `main.c` | DIP 路径二进制开关 |
| `CS_REPORT_BINARY_OUTPUT` | `cs_de_data_parse.h` | Legacy 路径二进制开关 |

`main()` 在 `bt_enable()` 前调用 `cs_uart_bin_tx_thread_start()`（当任一路径启用线程模式时）。

---

## 6. 内存与实时性约束

1. **大缓冲放 BSS**：`dip_bin_frame_buf`、`cs_bin_frame_buf`、`dip_step_data_copy` 等不得放在 BT RX WQ 栈上。  
2. **非阻塞入队**：`k_msgq_put(..., K_NO_WAIT)`；满则丢帧并 `LOG_WRN`。  
3. **队列深度**：默认 8（`CS_UART_BIN_MSGQ_DEPTH`），可在 `cs_uart_binary.c` 调整。  
4. **日志混流**：`LOG_INF` 与二进制共用 console；采集时降低日志级别。

---

## 7. 相关文档

| 文档 | 内容 |
|------|------|
| [DIP_binary_protocol.md](./DIP_binary_protocol.md) | type 0x01 线格式 |
| [CS_binary_protocol.md](./CS_binary_protocol.md) | type 0x02 线格式 |
| [host_acquisition_guide.md](./host_acquisition_guide.md) | 上位机开发索引 |
| [DIP_binary_pc_parser.md](./DIP_binary_pc_parser.md) | PC 解析与排错 |
| [README.md](../README.md) | 工程宏与模式切换 |

---

## 8. 版本

| 日期 | 说明 |
|------|------|
| 2026-07 | 初版：记录 cs_uart_binary 与组帧层拆分及双路径调用链 |
