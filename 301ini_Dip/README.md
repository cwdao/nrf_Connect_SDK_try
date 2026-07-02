# 301ini_Dip

基于 **Nordic Connect SDK（NCS）Channel Sounding** 的 **Initiator** 示例演进：**在保留传统「CS + RAS + cs_de」双端测距链路源码的前提下**，通过 **绕开 RAS（不对对端 GATT 测距数据做订阅与拉取）**，走 **DIP（Direct IQ Pipeline）**，在本地 **HCI LE CS Subevent Result** 到达后 **立刻** 解析 **PCT → 本地 IQ**，从而 **加快单侧测量结果的可用时间**，避免等待对端 reflector 经 RAS 回传后再处理。

- **DIP**：不等 RAS、不调用 `cs_de_populate_report` / `cs_de_calc`，适合单边 IQ 采集、链路验证与调试。  
- **Legacy**：仍为 `subevent_result_cb` → 缓冲 `latest_local_steps` → RAS → `ranging_data_get_complete_cb` → `cs_de` 距离估计；需显式改回调与宏后才会重新启用。

实现与宏默认值以 **`src/app_cs_mode.h`** 为准（统一推导 `main.c`、`cs_de_data_parse.h`、`flash_ops.h` 中的相关开关）。

**与本文同步的当前仓库默认（请改动后自行对照提交）：**

| 宏（`src/app_cs_mode.h`） | 当前默认 | 说明 |
|---------------------------|----------|------|
| `APP_CS_MODE_DIP` | **1** | **主开关**：`1`=DIP 本地直采，`0`=全功能 CS（RAS + cs_de） |
| `APP_CS_UART_BINARY` | **1** | **输出开关**：`1`=UART 二进制帧，`0`=LOG 文本 |
| `DIP_BINARY_USE_THREAD`（`main.c`） | **1** | msgq + `cs_uart_tx` 发送线程 |
| `FLASH_WRITE_MODE`（`flash_ops.h`） | **SINGLE** | Legacy 写 Flash 方式（`ENABLE_DIRECT_PRINT=0` 时生效） |

由上面两项自动推导（一般无需手改）：`APP_CS_DIP_BYPASS_RAS`、`DIP_REPORT_BINARY_OUTPUT`、`DIP_REPORT_LOG_VERBOSE`、`CS_REPORT_BINARY_OUTPUT`、`ENABLE_DIRECT_PRINT`，以及 `subevent` 回调注册（`#if APP_CS_MODE_DIP`）。

**CS procedure 参数**（`bt_le_cs_set_procedure_parameters` / `default_settings`）亦在 `app_cs_mode.h` 中按模式分块配置，见下文「CS 参数配置」。

---

## CS 参数配置（`app_cs_mode.h`）

除模式与 UART 开关外，可在同一文件内为 **DIP** 与 **全功能 CS** 分别设置 CS 时序/功率。`main.c` 根据 `APP_CS_MODE_DIP` 自动选用 `APP_CS_DIP_*` 或 `APP_CS_LEGACY_*` 参数块。

### 日志字段与配置宏对照

启动后 `procedure_enable_cb` 会打印协商结果，与配置宏的对应关系如下：

| 日志字段 | 配置宏（DIP 前缀 `APP_CS_DIP_`，Legacy 前缀 `APP_CS_LEGACY_`） | 说明 |
|----------|----------------------------------------------------------------|------|
| config ID | （固定 `CS_CONFIG_ID=0`） | 配置 ID |
| antenna configuration index | `TONE_ANTENNA_CONFIG_INDEX` | 0 = A1_B1 |
| TX power | `MAX_TX_POWER_DBM` | `bt_le_cs_set_default_settings` |
| subevent length | `MIN_SUBEVENT_LEN_US` / `MAX_SUBEVENT_LEN_US` | 控制器在范围内选取实际值 |
| procedure interval | `MIN_PROCEDURE_INTERVAL` / `MAX_PROCEDURE_INTERVAL` | 连接事件间隔 |
| procedure count | `MAX_PROCEDURE_COUNT` | 0 = 不限制 |
| maximum procedure length | `MAX_PROCEDURE_LEN` | procedure 最大时长 |
| （config）Mode-0 steps | `MODE_0_STEPS` | `bt_le_cs_create_config` |
| （PHY） | `PHY_2M` | 1=2M，0=1M |

**无法直接写入的日志项**（由控制器协商决定，只能间接影响）：

- `subevents per event`
- `subevent interval`
- `event interval`

蓝牙规范中 `min_subevent_len` / `procedure_interval` 等为**建议值**，SDC 可能忽略或与请求不同（例如单天线下 subevent length 常见上限约 21334 μs）。以 `procedure_enable_cb` 打印为准。

### 示例：DIP 放宽 subevent、降低 procedure 频率（缓解 no_cs_sync）

```c
/* app_cs_mode.h — DIP 参数块 */
#define APP_CS_DIP_MIN_PROCEDURE_INTERVAL      2U
#define APP_CS_DIP_MAX_PROCEDURE_INTERVAL      4U
#define APP_CS_DIP_MIN_SUBEVENT_LEN_US         15000U
#define APP_CS_DIP_MAX_SUBEVENT_LEN_US         40000U
```

改完后重新编译烧录，对照日志中的 `CS procedures enabled:` 确认协商结果。

---

## 模式切换清单（推荐）

**只需改 `src/app_cs_mode.h`**（模式开关 + 可选 CS 参数块），重新编译烧录即可。回调、RAS 订阅、二进制/文本输出均由编译期自动对齐。

### 快速对照

| 目标 | `APP_CS_MODE_DIP` | `APP_CS_UART_BINARY` | 串口帧 `type` | PC 脚本 |
|------|-------------------|----------------------|---------------|---------|
| **DIP + 二进制**（当前默认） | `1` | `1` | `0x01` 本地 IQ | `doc/dip_parse_uart.py` 或 `cs_parse_uart.py` |
| **全功能 CS + 二进制** | `0` | `1` | `0x02` 双端 IQ | `doc/cs_parse_uart.py` |
| DIP + 文本调试 | `1` | `0` | —（LOG 多信道） | 串口终端 |
| 全功能 CS + 文本调试 | `0` | `0` | —（`print_report_fast`） | 串口终端 |

### DIP + 二进制采集

编辑 `src/app_cs_mode.h`：

```c
#define APP_CS_MODE_DIP      1
#define APP_CS_UART_BINARY   1
```

自动生效项：

| 项目 | 值 |
|------|-----|
| subevent 回调 | `subevent_result_dip_cb` |
| RAS 订阅 | 跳过（`APP_CS_DIP_BYPASS_RAS=1`） |
| 输出函数 | `dip_output_local_report_binary()` |
| 帧格式 | version `0x02`，含 `timestamp_ms`；见 `doc/DIP_binary_protocol.md` |

采集建议：降低 `CONFIG_LOG` 级别，避免 ASCII 与 `0x55 0xAA` 混流。

```bash
cd doc
python dip_parse_uart.py COM3
```

### 全功能 CS + 二进制采集

编辑 `src/app_cs_mode.h`：

```c
#define APP_CS_MODE_DIP      0
#define APP_CS_UART_BINARY   1
```

自动生效项：

| 项目 | 值 |
|------|-----|
| subevent 回调 | `subevent_result_cb` |
| RAS 订阅 | 启用四个 `bt_ras_rreq_*_subscribe` |
| 输出函数 | `cs_output_store_report_binary()`（须 `ENABLE_DIRECT_PRINT=1`，已自动推导） |
| 数据流 | 本地 + 对端 step → `cs_de_populate_report` → 双端 IQ 帧 |
| 帧格式 | type `0x02`；见 `doc/CS_binary_protocol.md` |

**硬件要求**：对端 reflector 须支持 RAS。

```bash
cd doc
python cs_parse_uart.py COM3
```

### 切回 DIP（从全功能 CS）

将 `APP_CS_MODE_DIP` 改回 `1`；若继续二进制采集，保持 `APP_CS_UART_BINARY=1`。重新编译烧录即可，无需再改 `main.c` 里的回调名。

### 细调（可选，覆盖推导宏）

若只需改单项，可在 `#include "app_cs_mode.h"` **之前**手动 `#define`，例如：

```c
#define DIP_REPORT_LOG_VERBOSE 1   /* 强制 DIP 文本详版，即使 APP_CS_UART_BINARY=1 */
```

或继续直接改 `main.c` 中的 `DIP_BINARY_USE_THREAD`、`NEED_LOG` 等高级选项。

### 切换后检查清单

- [ ] 修改 `src/app_cs_mode.h` 中 `APP_CS_MODE_DIP` / `APP_CS_UART_BINARY`
- [ ] 全功能 CS 时确认对端 RAS 可用
- [ ] 二进制采集时降低 `CONFIG_LOG` 级别
- [ ] 重新 **build + flash**
- [ ] PC 端使用对应解析脚本（`type` 0x01 vs 0x02）
- [ ] 日志中确认模式：DIP 启动见 `DIP mode: RAS RREQ subscriptions skipped`；Legacy 无此行且 RAS 订阅成功

---

## DIP 启用与禁用（历史说明）

> **现已由 `app_cs_mode.h` 统一推导**，下列步骤仅作原理说明；日常切换请用上文「模式切换清单」。

DIP 由 **subevent 回调** 与 **`APP_CS_DIP_BYPASS_RAS`** 共同决定（二者均由 `APP_CS_MODE_DIP` 控制）。

### 启用 DIP（`APP_CS_MODE_DIP=1`）

1. subevent 回调为 `subevent_result_dip_cb`（`main.c` 中 `#if APP_CS_MODE_DIP` 自动选择）
2. `APP_CS_DIP_BYPASS_RAS=1`：不注册 `bt_ras_rreq_*_subscribe`
3. 二进制：`DIP_REPORT_BINARY_OUTPUT=1`；文本：`DIP_REPORT_LOG_VERBOSE=1`

### 恢复 Legacy 双端测距（`APP_CS_MODE_DIP=0`）

1. subevent 回调为 `subevent_result_cb`
2. `APP_CS_DIP_BYPASS_RAS=0`：恢复四个 RAS 订阅
3. 二进制：`CS_REPORT_BINARY_OUTPUT=1` + `ENABLE_DIRECT_PRINT=1`

`subevent_result_cb`、`ranging_data_*` 等仍保留在工程中。

---

## DIP 实现原理（数据流）

| 路径 | 数据流 |
|------|--------|
| Legacy | `subevent_result_cb` → 缓存 `latest_local_steps` → 等 RAS → `ranging_data_get_complete_cb` → `cs_de_populate_report` / `cs_de_calc` |
| DIP | `subevent_result_dip_cb` → 拷贝 `step_data_buf` → `bt_le_cs_step_data_parse` → mode 2/3 的 **PCT** → `bt_le_cs_parse_pct()` → 聚合到 `dip_parse_work_ctx` → 二进制 UART 或文本打印 |

要点：

1. **线程**：回调跑在 **BT RX WQ**，栈很小。`step_data` 拷贝与解析上下文使用 **静态** `dip_step_data_copy[]`、`dip_parse_work_ctx`，避免 **栈溢出**（`LOCAL_PROCEDURE_MEM` 量级不可放在该线程栈上）。  
2. **二进制 UART（默认开启）**：解析后经 `dip_output_local_report_binary()` 输出 v2 帧；默认 **`DIP_BINARY_USE_THREAD=1`**，在回调内 **入队**、由 **`cs_uart_tx`** 线程 `uart_poll_out`，避免在 BT RX WQ 内长时间发串口。详见 `doc/DIP_binary_protocol.md`。  
3. **Abort**：若 `subevent_done_status` 为 **aborted**，会打印 `pc`、`subevent_abort_reason`、`num_steps_reported`、`abort_step`、`procedure_done_status` 等，不解析 IQ。  
4. **IQ 与天线索引**：与 Nordic `cs_de.c::extract_pcts` 一致——`bt_le_cs_get_antenna_path` + `tone_info[antenna_path]`；**信道格点** `fft_ch = step->channel - 2`（`DIP_CS_CHANNEL_INDEX_OFFSET`），宽度 `DIP_MAX_FFT_CHANNELS`（与 cs_de 通道格点思路对齐）。  
5. **配置**：`CONFIG_BT_RAS_MAX_ANTENNA_PATHS`、`header.num_antenna_paths` 决定 `n_ap`；`LOCAL_PROCEDURE_MEM` / reassembly 相关配置限制单包 `step_data` 最大长度，超限则丢弃并 `LOG_ERR`。

---

## Legacy 流程概要（RAS + cs_de）

传统 Initiator 端到端流程：

1. 初始化蓝牙，被动扫描带 **Ranging Service** 的设备并连接。  
2. 加密、MTU、GATT 发现（含 RAS 句柄分配）。  
3. 配置 CS 默认能力、创建 config、安全使能、procedure 参数；**`main()` 末尾在成功后即调用 `bt_le_cs_procedure_enable`（`params.enable == 1`）**，连接建立后 CS procedure 会按配置运行（按键逻辑仍可另行启停）。  
4. 本地 subevent 数据由 `subevent_result_cb` 写入 `latest_local_steps`；对端数据经 RAS 就绪后在 `ranging_data_get_complete_cb` 中与本地对齐并 **populate + cs_de 计算**。  
5. 结果可写入 Flash 或串口打印（视 `flash_ops.h` 中 `ENABLE_DIRECT_PRINT` / `FLASH_WRITE_MODE` 等）。  
   **`CS_REPORT_BINARY_OUTPUT=1`** 时，`ranging_data_get_complete_cb` 经 `cs_output_store_report_binary()` 输出 **type=0x02** 双端 IQ 二进制帧（每信道 local+remote），替代文本 `print_report_fast`；详见 `doc/CS_binary_protocol.md`。

若需自定义落盘格式，可在 `ranging_data_get_complete_cb` 中针对 `cs_de_populate_report` / 报告结构做重构（见原工程注释）。

---

## 条件编译说明

以下宏影响 **Legacy 路径**或系统行为；DIP 路径不依赖 RAS 完成回调，但 Flash/日志仍可能与主线程交互。

### 1. FLASH_WRITE_MODE（`src/flash/flash_ops.h`）

- **SINGLE (0)**：每次测距完成单次异步写 Flash。  
- **BATCH (1)**：环形缓冲批量写入 + 定时刷新。  

影响 `ranging_data_get_complete_cb` 及是否启用 `flash_timer_work`。

### 2. TEST_RANGING_ENABLED（`src/main.c`）

- **0**（默认）：关闭固定次数自动测距统计。  
- **1**：固定次数测距测试、成功/失败/覆盖计数，结束后可自动关 CS。  

### 3. ENABLE_DIRECT_PRINT（`src/flash/flash_ops.h`）

- **0**：Legacy 测距完成后按 `FLASH_WRITE_MODE` 写入 Flash。  
- **1**：Legacy 完成路径下直接串口打印报告，不写 Flash。  

**当前仓库**该宏多为 **1**（以 `flash_ops.h` 为准）。与 **DIP** 的 `DIP_REPORT_LOG_VERBOSE` 相互独立：DIP 日志仅在 `subevent_result_dip_cb` / 解析路径侧。

### 4. NEED_LOG（`src/main.c`）

控制主循环是否周期性打印滑动窗口距离估计；**0** 可降低开销。

### 使用建议

- **生产 / 性能**：`FLASH_WRITE_MODE_BATCH`、`TEST_RANGING_ENABLED=0`、`ENABLE_DIRECT_PRINT=0`、`NEED_LOG=0`。  
- **调试 Legacy 测距**：可开 `ENABLE_DIRECT_PRINT`、`NEED_LOG`。  
- **调试 DIP**：用 `DIP_REPORT_LOG_VERBOSE` 控制 IQ 输出量；过密 UART 可能影响 **BT RX WQ** 实时性，可先试 `0` 对比 subevent abort 率。
- **DIP 二进制采集（已验证）**：`DIP_REPORT_BINARY_OUTPUT=1`、`DIP_BINARY_USE_THREAD=1`、`DIP_REPORT_LOG_VERBOSE=0`；PC 用 `doc/dip_parse_uart.py`。

---

## 文档（`doc/`）

| 文档 | 内容 |
|------|------|
| [doc/firmware_uart_binary_architecture.md](doc/firmware_uart_binary_architecture.md) | 固件 UART 二进制分层架构（cs_uart_binary / 组帧层） |
| [doc/host_acquisition_guide.md](doc/host_acquisition_guide.md) | **上位机采集开发指南**（文档/代码索引与解析流程） |
| [doc/DIP_binary_protocol.md](doc/DIP_binary_protocol.md) | DIP 二进制帧（type=0x01）、bitmap、CRC |
| [doc/CS_binary_protocol.md](doc/CS_binary_protocol.md) | 全功能 CS 双端 IQ 二进制帧（type=0x02） |
| [doc/DIP_binary_pc_parser.md](doc/DIP_binary_pc_parser.md) | PC 端串口接收、解析步骤与常见问题 |
| [doc/dip_parse_uart.py](doc/dip_parse_uart.py) | DIP 专用 Python 解析（type=0x01） |
| [doc/cs_parse_uart.py](doc/cs_parse_uart.py) | DIP + CS 通用 Python 解析（type=0x01/0x02） |

快速试用：

```bash
cd doc
python dip_parse_uart.py COM3
```

---

## 代码导航

| 内容 | 位置 |
|------|------|
| DIP 入口、abort 日志 | `subevent_result_dip_cb` |
| DIP 解析与输出 | `dip_parse_local_iq_from_subevent`、`dip_local_step_iq_cb`、`dip_output_local_report_binary`、`dip_print_local_report_multichannel` |
| DIP 二进制 | `dip_bin_build_frame`、`cs_uart_bin_enqueue_frame`（`src/cs_uart_binary.c`） |
| CS 二进制 | `cs_bin_build_dual_iq_frame`、`cs_output_store_report_binary`（`cs_de_data_parse.c`） |
| UART 传输层 | `cs_uart_binary.c`（CRC、msgq、`cs_uart_tx` 线程） |
| DIP / RAS / CS 宏 | **`src/app_cs_mode.h`**（`APP_CS_MODE_DIP`、`APP_CS_UART_BINARY`） |
| Legacy subevent / RAS | `subevent_result_cb`、`main()` 中 `#if APP_CS_DIP_BYPASS_RAS` |
| Zephyr API | `bt_le_cs_step_data_parse`、`bt_le_cs_parse_pct`、`bt_le_cs_get_antenna_path`（`zephyr/bluetooth/cs.h`） |
| 双端距离与 IQ 融合 | Nordic `cs_de` / `cs_de_populate_report`（DIP **不调用**） |

---

## 文档维护

- **本 `README.md`**：工程总览、**模式切换清单**、`app_cs_mode.h` 说明。
- **`doc/`**：DIP / CS 二进制协议与 PC 解析（见上表）；与 `src/main.c`、`src/cs_de_data_parse.c` 实现同步维护。
