# 301ini_Dip

基于 **Nordic Connect SDK（NCS）Channel Sounding** 的 **Initiator** 示例演进：**在保留传统「CS + RAS + cs_de」双端测距链路源码的前提下**，通过 **绕开 RAS（不对对端 GATT 测距数据做订阅与拉取）**，走 **DIP（Direct IQ Pipeline）**，在本地 **HCI LE CS Subevent Result** 到达后 **立刻** 解析 **PCT → 本地 IQ**，从而 **加快单侧测量结果的可用时间**，避免等待对端 reflector 经 RAS 回传后再处理。

- **DIP**：不等 RAS、不调用 `cs_de_populate_report` / `cs_de_calc`，适合单边 IQ 采集、链路验证与调试。  
- **Legacy**：仍为 `subevent_result_cb` → 缓冲 `latest_local_steps` → RAS → `ranging_data_get_complete_cb` → `cs_de` 距离估计；需显式改回调与宏后才会重新启用。

实现与宏默认值以 **`src/main.c`**（及 **`src/flash/flash_ops.h`** 中与 Flash/打印相关的宏）为准。

**与本文同步的当前仓库默认（请改动后自行对照提交）：**

| 宏 | 当前默认 | 说明 |
|----|----------|------|
| `APP_CS_DIP_BYPASS_RAS` | **1** | DIP 模式下跳过 RAS 订阅 |
| `DIP_REPORT_BINARY_OUTPUT` | **1** | UART 二进制 IQ 帧；**0** 则走文本/精简日志 |
| `DIP_BINARY_USE_THREAD` | **1** | **1**：msgq + 发送线程（非阻塞入队）；**0**：回调内阻塞 `uart_poll_out` |
| `DIP_REPORT_LOG_VERBOSE` | **1** | 多信道 `ch(i,q)` 分块打印（与二进制互斥）；二进制测试建议 **0** |
| `FLASH_WRITE_MODE`（`flash_ops.h`） | **SINGLE** | 单次异步写 Flash |
| `ENABLE_DIRECT_PRINT`（`flash_ops.h`） | **1** | Legacy 完成路径优先串口打印；生产可改为 **0** 写 Flash |

---

## DIP 启用与禁用

DIP 由 **连接回调** 与 **`APP_CS_DIP_BYPASS_RAS`** 共同决定，两处需一致。

### 启用 DIP（当前默认方向）

1. **`BT_CONN_CB_DEFINE(conn_cb)`** 中：

   ```c
   .le_cs_subevent_data_available = subevent_result_dip_cb,
   ```

2. **旁路 RAS 订阅**（`main.c`）：

   ```c
   #define APP_CS_DIP_BYPASS_RAS 1
   ```

   为 `1` 时不注册 `bt_ras_rreq_*_subscribe`，避免未走 `subevent_result_cb` 时 RAS 仍去拉取数据而 **`latest_local_steps` 未同步**。

3. **报告输出**（`main.c`，三选一，见 `dip_parse_local_iq_from_subevent()`）：

   ```c
   #define DIP_REPORT_BINARY_OUTPUT 1   /* UART 二进制帧（推荐采集） */
   #define DIP_REPORT_BINARY_OUTPUT 0
   #define DIP_REPORT_LOG_VERBOSE 1       /* LOG_INF 多信道 ch(i,q) */
   #define DIP_REPORT_LOG_VERBOSE 0       /* 每报告仅 DIP ok pc=… */
   ```

4. **二进制发送路径**（仅当 `DIP_REPORT_BINARY_OUTPUT=1`）：

   ```c
   #define DIP_BINARY_USE_THREAD 1   /* 回调入队，dip_uart_tx 线程发串口（默认） */
   #define DIP_BINARY_USE_THREAD 0   /* 回调内直接发，便于对比 */
   ```

### 恢复 Legacy 双端测距

1. 将 subevent 回调改回：

   ```c
   .le_cs_subevent_data_available = subevent_result_cb,
   ```

2. 关闭 RAS 旁路：

   ```c
   #define APP_CS_DIP_BYPASS_RAS 0
   ```

3. 确认 `main()` 中四个 `bt_ras_rreq_*_subscribe` 均恢复执行。

`subevent_result_cb`、`ranging_data_*` 等仍保留在工程中，仅在未注册 `subevent_result_dip_cb` 时参与运行。

---

## DIP 实现原理（数据流）

| 路径 | 数据流 |
|------|--------|
| Legacy | `subevent_result_cb` → 缓存 `latest_local_steps` → 等 RAS → `ranging_data_get_complete_cb` → `cs_de_populate_report` / `cs_de_calc` |
| DIP | `subevent_result_dip_cb` → 拷贝 `step_data_buf` → `bt_le_cs_step_data_parse` → mode 2/3 的 **PCT** → `bt_le_cs_parse_pct()` → 聚合到 `dip_parse_work_ctx` → 二进制 UART 或文本打印 |

要点：

1. **线程**：回调跑在 **BT RX WQ**，栈很小。`step_data` 拷贝与解析上下文使用 **静态** `dip_step_data_copy[]`、`dip_parse_work_ctx`，避免 **栈溢出**（`LOCAL_PROCEDURE_MEM` 量级不可放在该线程栈上）。  
2. **二进制 UART（默认开启）**：解析后经 `dip_output_local_report_binary()` 输出 v2 帧；默认 **`DIP_BINARY_USE_THREAD=1`**，在回调内 **入队**、由 **`dip_uart_tx`** 线程 `uart_poll_out`，避免在 BT RX WQ 内长时间发串口。详见 `doc/DIP_binary_protocol.md`。  
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
| [doc/DIP_binary_protocol.md](doc/DIP_binary_protocol.md) | 二进制帧字段、bitmap、CRC、固件宏与线程发送 |
| [doc/DIP_binary_pc_parser.md](doc/DIP_binary_pc_parser.md) | PC 端串口接收、解析步骤与常见问题 |
| [doc/dip_parse_uart.py](doc/dip_parse_uart.py) | 可运行的 Python 解析示例（`pip install pyserial`） |

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
| DIP 二进制 | `dip_bin_build_frame`、`dip_uart_tx_thread`（`DIP_BINARY_USE_THREAD=1`） |
| DIP / RAS 宏 | `APP_CS_DIP_BYPASS_RAS`、`DIP_REPORT_BINARY_OUTPUT`、`DIP_BINARY_USE_THREAD`、`DIP_REPORT_LOG_VERBOSE` |
| Legacy subevent / RAS | `subevent_result_cb`、`main()` 中 `#if APP_CS_DIP_BYPASS_RAS` |
| Zephyr API | `bt_le_cs_step_data_parse`、`bt_le_cs_parse_pct`、`bt_le_cs_get_antenna_path`（`zephyr/bluetooth/cs.h`） |
| 双端距离与 IQ 融合 | Nordic `cs_de` / `cs_de_populate_report`（DIP **不调用**） |

---

## 文档维护

- **本 `README.md`**：工程总览、宏默认值、DIP/Legacy 切换。
- **`doc/`**：DIP 二进制协议与 PC 解析专文（见上表）；与 `src/main.c` 实现同步维护。
