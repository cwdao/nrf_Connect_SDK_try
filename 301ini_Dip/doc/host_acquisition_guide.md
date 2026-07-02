# 上位机串口采集协议开发指南

本文说明如何基于本工程固件的 **console UART 二进制输出**，开发 PC 端（上位机）采集与解析程序。与固件协议文档、参考脚本、源码位置一一对应，便于独立实现或移植到其他语言。

**适用对象**：需要接收 DIP 本地 IQ 或全功能 CS 双端 IQ 二进制帧的上位机开发者。

---

## 1. 开发前：确认固件运行模式

固件通过条件编译在两条路径间切换，**同一串口、同一帧头**，但 `type` 字段与 IQ 载荷宽度不同：

| 模式 | 关键宏 | 帧 `type` | 每有效信道 IQ 字节数 | 协议文档 |
|------|--------|-----------|---------------------|----------|
| **DIP 本地直采** | `APP_CS_DIP_BYPASS_RAS=1` | `0x01` | 4（`i`, `q`） | [DIP_binary_protocol.md](./DIP_binary_protocol.md) |
| **全功能 CS（RAS + cs_de）** | `APP_CS_DIP_BYPASS_RAS=0` | `0x02` | 8（`i_local`, `q_local`, `i_remote`, `q_remote`） | [CS_binary_protocol.md](./CS_binary_protocol.md) |

工程总览、宏默认值与模式切换步骤见仓库根目录 [README.md](../README.md)。

**上位机建议**：实现**一套**字节流解析状态机，根据帧内 `type` 分支处理 IQ 区（推荐直接参考 [cs_parse_uart.py](./cs_parse_uart.py)）。

---

## 2. 文档阅读顺序

### 2.1 必读（字段定义与帧格式）

1. **[DIP_binary_protocol.md](./DIP_binary_protocol.md)**  
   定义共用帧头（30 字节，version 0x02）、`timestamp_ms`、`payload_len` 计算、10 字节信道位图、CRC16-CCITT、线程化发送架构。  
   DIP（type `0x01`）的 IQ 区格式以本文为准。

2. **[CS_binary_protocol.md](./CS_binary_protocol.md)**  
   在 DIP v2 帧头基础上，说明 type `0x02` 的双端 IQ 载荷（每信道 8 字节）、`procedure_counter` 与 RAS `ranging_counter` 的对应关系、与文本输出的带宽对比。

### 2.2 实操与排错

3. **[DIP_binary_pc_parser.md](./DIP_binary_pc_parser.md)**  
   串口参数、同步搜帧流程、CRC 校验步骤、ASCII 日志混流、队列丢帧等常见问题。  
   全功能 CS 二进制采集同样适用（同步/CRC/位图逻辑一致）。

### 2.3 固件模块架构（维护者）

4. **[firmware_uart_binary_architecture.md](./firmware_uart_binary_architecture.md)**  
   `cs_uart_binary` 传输层与 `cs_de_data_parse` / `main.c` 组帧层的职责划分、调用链与宏配置。

### 2.4 工程配置速查

5. **[README.md](../README.md)**  
   宏表（`DIP_REPORT_BINARY_OUTPUT`、`CS_REPORT_BINARY_OUTPUT`、`ENABLE_DIRECT_PRINT` 等）、DIP/Legacy 切换、代码导航表。

---

## 3. 参考代码（可直接移植）

### 3.1 Python 脚本（推荐起点）

| 文件 | 用途 |
|------|------|
| [cs_parse_uart.py](./cs_parse_uart.py) | **通用**：同时解析 type `0x01`（DIP）与 `0x02`（CS 双端） |
| [dip_parse_uart.py](./dip_parse_uart.py) | 仅 DIP（type `0x01`） |

移植到其他语言时，建议对照以下函数保持行为一致：

| 函数 | 作用 |
|------|------|
| `crc16_ccitt()` | CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF` |
| `channels_from_bitmap()` | 10 字节位图 → 升序 fft 信道列表 |
| `iq_bytes_per_channel(ftype)` | `0x01`→4，`0x02`→8 |
| `parse_frame()` | 定长头 + 变长 IQ + CRC 校验 |
| `FrameReader.feed()` | 字节流缓冲、搜 `0x55 0xAA`、按 `payload_len` 定帧长 |

快速试用：

```bash
cd doc
pip install pyserial
python cs_parse_uart.py COM3
python cs_parse_uart.py COM3 --csv out.csv
```

### 3.2 固件源码（权威实现，文档歧义时以代码为准）

#### 共用：帧头、CRC、UART 发送

| 文件 | 关注点 |
|------|--------|
| [src/cs_uart_binary.h](../src/cs_uart_binary.h) | `CS_UART_BIN_*` 常量、`struct cs_uart_bin_header` 内存布局 |
| [src/cs_uart_binary.c](../src/cs_uart_binary.c) | `cs_uart_bin_crc16_ccitt()` — CRC 覆盖范围（sync1 至 IQ 区末字节，不含 CRC 两字节） |

#### DIP 路径（type = 0x01）

| 文件 | 函数 / 宏 |
|------|-----------|
| [src/main.c](../src/main.c) | `dip_bin_build_frame()`、`dip_bin_fill_channel_bitmap()`、`dip_bin_pack_iq_payload()`、`dip_output_local_report_binary()` |
| [src/main.c](../src/main.c) | `DIP_REPORT_BINARY_OUTPUT`、`DIP_REPORT_LOG_VERBOSE`（二进制采集建议后者为 `0`） |

触发链：`subevent_result_dip_cb` → `dip_parse_local_iq_from_subevent` → `dip_output_local_report_binary`。

#### 全功能 CS 路径（type = 0x02）

| 文件 | 函数 / 宏 |
|------|-----------|
| [src/cs_de_data_parse.c](../src/cs_de_data_parse.c) | `cs_bin_build_dual_iq_frame()`、`cs_bin_pack_dual_iq_payload()`、`cs_output_store_report_binary()` |
| [src/cs_de_data_parse.h](../src/cs_de_data_parse.h) | `CS_REPORT_BINARY_OUTPUT`、`CS_BINARY_USE_THREAD` |
| [src/main.c](../src/main.c) | `ranging_data_get_complete_cb` 内调用 `cs_output_store_report_binary()` |

触发链：RAS 数据到齐 → `cs_de_populate_report` / `cs_de_calc` → `cs_output_store_report_binary`。

---

## 4. 上位机解析流程（推荐实现）

```
打开串口（通常 115200 8N1，与板级 zephyr console 一致）
    │
    ▼
维护接收环形缓冲 / 字节队列
    │
    ▼
扫描同步字 0x55 0xAA
    │
    ▼
读取 version(1)、type(1)、payload_len(2 LE)
    │
    ▼
根据 version 与 type 计算 IQ 区长度与整帧长度：
  version 0x02: hdr=30, payload_fixed=24
  version 0x01: hdr=22, payload_fixed=16（旧版，无 timestamp_ms）
  iq_len   = payload_len - payload_fixed
  type 0x01: iq_len 须为 4 的倍数，每信道 4 字节
  type 0x02: iq_len 须为 8 的倍数，每信道 8 字节
  frame_len = hdr + iq_len + 2
    │
    ▼
收满 frame_len 字节后做 CRC16-CCITT（覆盖 buf[0 .. hdr+iq_len-1]）
    │
    ▼
解析 procedure_counter、timestamp_ms（v0x02）、ap、channel_count、channel_bitmap[10]
    │
    ▼
按 bitmap 中置位信道升序读取 IQ：
  type 0x01: int16 i, int16 q（各 LE）
  type 0x02: int16 i_local, q_local, i_remote, q_remote（各 LE）
    │
    ▼
输出结构化记录（建议字段见 §5）
```

**`payload_len` 定义**（与固件一致）：从 `procedure_counter` 首字节起，到 IQ 区最后一字节止，**不含** sync～type、payload_len 自身、CRC。

```
payload_len = 24 + (每信道字节数 × channel_count)
            = 24 + 4×N   （type 0x01）
            = 24 + 8×N   （type 0x02）
```

---

## 5. 建议的上位机数据结构

解析成功后，每条帧可映射为：

| 字段 | 来源 | 说明 |
|------|------|------|
| `version` | 帧头 | 当前固件为 `0x02` |
| `type` | 帧头 | `0x01` DIP / `0x02` CS 双端 |
| `procedure_counter` | 帧头 uint16 LE | DIP：CS procedure 计数；CS：`ranging_counter` |
| `timestamp_ms` | 帧头 uint64 LE | `k_uptime_get()` 毫秒（v0x02 起） |
| `ap` | 帧头 | 天线路径，当前多为 `0` |
| `channel_count` | 帧头 | 须等于位图中置 1 的位数 |
| `channels` | 位图展开 | fft 下标 `0..74` |
| `iq` | 变长载荷 | 按信道索引的字典/数组 |

**信道下标与 HCI 关系**（与 Nordic `cs_de.c` 一致）：

```
fft_ch = hci_channel - 2
```

**当前二进制帧未包含的字段**（若上位机需要，需另辟通道或扩展协议）：

- `report_index`（仅文本/Flash 路径有）
- 距离估计（`ifft` / `phase_slope` / `rtt` / `best`）
- `tone_quality`、`role` 等元数据

---

## 6. 固件宏与采集环境

### 6.1 DIP 二进制采集

```c
// main.c
#define APP_CS_DIP_BYPASS_RAS 1
#define DIP_REPORT_BINARY_OUTPUT 1
#define DIP_REPORT_LOG_VERBOSE 0
```

### 6.2 全功能 CS 二进制采集

```c
// main.c
#define APP_CS_DIP_BYPASS_RAS 0
// cs_de_data_parse.h
#define CS_REPORT_BINARY_OUTPUT 1
// flash_ops.h
#define ENABLE_DIRECT_PRINT 1
```

### 6.3 串口与日志

- 二进制与 Zephyr `LOG_*` **共用 console UART**，采集时建议降低 `CONFIG_LOG` 级别，避免 ASCII 与 `0x55 0xAA` 混流导致误同步。
- 固件队列满时会丢帧（日志：`DIP binary msgq full` / `CS binary msgq full`），上位机可用 `procedure_counter` 连续性检测缺帧。

---

## 7. 单模式 vs 通用采集器

| 场景 | 最少阅读材料 |
|------|----------------|
| 只做 DIP 采集 | `DIP_binary_protocol.md` + `dip_parse_uart.py` + `main.c` 中 `dip_bin_*` |
| 只做全功能 CS 采集 | `CS_binary_protocol.md` + `cs_parse_uart.py` + `cs_de_data_parse.c` 中 `cs_bin_*` |
| 通用采集器（推荐） | 两篇协议文档 + `cs_parse_uart.py` + `cs_uart_binary.h` |

---

## 8. 相关文档索引

| 文档 | 内容 |
|------|------|
| [firmware_uart_binary_architecture.md](./firmware_uart_binary_architecture.md) | 固件 UART 二进制分层架构 |
| [DIP_binary_protocol.md](./DIP_binary_protocol.md) | DIP 帧 type `0x01` |
| [CS_binary_protocol.md](./CS_binary_protocol.md) | CS 双端帧 type `0x02` |
| [DIP_binary_pc_parser.md](./DIP_binary_pc_parser.md) | PC 解析步骤与 FAQ |
| [cs_parse_uart.py](./cs_parse_uart.py) | 通用 Python 参考实现 |
| [dip_parse_uart.py](./dip_parse_uart.py) | DIP 专用 Python 参考实现 |
| [README.md](../README.md) | 工程总览与宏说明 |

---

## 9. 版本

| 日期 | 说明 |
|------|------|
| 2026-07 | 初版：汇总上位机开发所需的文档、脚本与固件符号对照 |
