# IPT 二进制 UART 帧协议（type `0x03`）

本文面向**上位机开发**：说明 `331ipt_initiator` 经 console UART 输出的 **IPT 已合成 IQ** 帧格式、物理含义与解析要点。

固件实现：`ipt_bin_*` / `ipt_output_procedure_binary()`（`331ipt_initiator/src/main.c`）；CRC 与发送线程见同目录 `cs_uart_binary.c`（与 `301ini_Dip` 同源）。

背景与测距语义见 [IPT_数据特性与上位机适配方案.md](./IPT_数据特性与上位机适配方案.md)。  
共用帧头细节可对照 `301ini_Dip/doc/DIP_binary_protocol.md`。

**状态**：固件已实现并完成编译；PC 参考脚本已支持 `0x03`（`301ini_Dip/doc/cs_parse_uart.py`）。

---

## 1. 一句话结论

| 项目 | 说明 |
|------|------|
| 帧类型 | `type = 0x03` |
| 载荷宽度 | 与 DIP 相同：每有效信道 **4 字节**（`int16 i` + `int16 q`） |
| 相位含义 | 已是往返信道相位 \(2\theta_{CH}\)（控制器内完成 IPT 相位补偿） |
| 上位机处理 | **直接**做相位斜率 / IFFT；**不要**再调用双端复数相乘 |
| 一帧粒度 | **一次完成的 CS procedure**（多 subevent 攒齐），不是每个 subevent |

不要把本帧当 DIP（`0x01`，单侧原始相位）或 CS 双端（`0x02`，相乘前两端 PCT）解析。

---

## 2. 与现有类型对照

| type | 名称 | 每有效信道 | 相位语义 | 典型工程 |
|------|------|------------|----------|----------|
| `0x01` | DIP 本地 IQ | 4 B：`i, q` | \(\theta_{CH}-\Delta\theta_{LO}\)，缺对端不可测距 | `301ini_Dip` |
| `0x02` | CS 双端 IQ | 8 B：`i_l,q_l,i_r,q_r` | 两端原始 PCT，上位机需先复数相乘 | `301ini` / Dip legacy |
| **`0x03`** | **IPT 已合成 IQ** | **4 B：`i, q`** | **已是 \(2\theta_{CH}\)** | **`331ipt_initiator`** |

同步字、版本、CRC、位图编码与 DIP/CS 完全一致：

- `sync1 = 0x55`，`sync2 = 0xAA`
- `version = 0x02`
- CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`
- 多字节字段均为 **little-endian**

---

## 3. 串口参数

| 项目 | 值 |
|------|-----|
| 设备 | 发起端 DK console（overlay 将 `zephyr,console` 绑到 **uart30**） |
| 波特率 | **230400**（与 `301ini_Dip` 采集口一致；勿用 115200） |
| 数据位 / 校验 / 停止位 | 8N1 |
| 流控 | 无 |
| 混流 | 固件默认日志等级为 WARN；测距时同口可能仍有少量 ASCII，解析器须能跳过非同步字节 |

反射端（`331ipt_reflector`）**不**输出本协议帧。

---

## 4. 帧结构（type = `0x03`）

```
偏移   长度    字段                 说明
----   ----    ----                 ----
0      1       sync1                固定 0x55
1      1       sync2                固定 0xAA
2      1       version              当前 0x02
3      1       type                 0x03 = IPT 已合成 IQ
4      2       payload_len          LE；见 §5
6      2       procedure_counter    LE；HCI CS procedure counter
8      8       timestamp_ms         LE；k_uptime_get() 毫秒（自启动）
16     1       ap                   当前固定 0（A1/B1，tone_info[0]）
17     1       iq_format            0 = int16 I/Q
18     1       channel_count        有效 fft 信道数 N
19     1       reserved             填 0
20     10      channel_bitmap[10]   见 §6
30     4×N     IQ payload           按 ch 升序，int16 i + int16 q
30+4N  2       crc16                LE；见 §8
```

固定头（含 bitmap）**30 字节**，与 `struct cs_uart_bin_header` 一致。

| 量 | 公式 |
|----|------|
| `payload_len` | `24 + 4×N` |
| 整帧长度 | `32 + 4×N` |

满信道约 N≈70～72（75 格点去掉广播 23–25 等未测点）时，整帧约 **312～320** 字节；230400 下约 **14 ms**，短于典型 procedure 间隔。

---

## 5. payload_len

从 **`procedure_counter` 首字节** 到 **IQ 区最后一字节**（**不含** sync～type、payload_len 自身、CRC）：

```
payload_len = 2 + 8 + 4 + 10 + (4 × channel_count)
            = 24 + 4×N
```

解析校验：`iq_len = payload_len - 24`，且 `iq_len == 4×channel_count`，且 `iq_len % 4 == 0`。

---

## 6. 信道位图与 fft 下标

与 DIP / CS 完全一致：

- 10 字节 = 80 bit，覆盖 fft 下标 `ch ∈ [0, 74]`（`CS_DE_NUM_CHANNELS = 75`）。
- `channel_bitmap[ch / 8]` 的 **bit (ch % 8)** 为 1 → 该信道在 IQ 区有一条 4 字节记录（**bit0 = LSB**）。
- `channel_count` 必须等于位图中置 1 的位数。
- 映射：`fft_ch = hci_channel - 2`（信道 2 → 2404 MHz … 信道 76 → 2479 MHz）。
- 广播信道 23–25 对应格点通常无测量，位图不置位，IQ 区不出现这些 ch。

固件仅对 **Tone Quality = HIGH** 的 Mode-2 音调置位并写入 IQ；非 HIGH 格点保持 0 且不置位。

---

## 7. IQ 载荷

仅对位图为 1 的 `ch`，按 **ch 从小到大** 依次写入：

| 顺序 | 类型 | 说明 |
|------|------|------|
| 1 | int16 LE | I（实部） |
| 2 | int16 LE | Q（虚部） |

来源：`bt_le_cs_parse_pct(tone_info[0].phase_correction_term)` 的原始 int16，**不经 float 再四舍五入**。

布局约定与 Nordic `cs_de.c` 一致：交织缓冲中偶数下标为 I、奇数下标为 Q。上位机勿按 `cs_de.h` 里可能写反的注释对调。

### 7.1 物理含义（上位机必读）

发起端这一组 `(I, Q)` 已经等价于全功能 CS 里 `cs_de_combined_iq_calculate()`（逐信道复数相乘）的**输出**，不是 DIP 那种还缺对端的单侧 PCT。

因此：

1. **禁止**再与“远端 IQ”相乘一次。
2. **禁止**把本帧填进 type `0x02` 的 `i_remote/q_remote` 字段当远端原始 PCT。
3. 可直接喂给与 SDK 相同的相位斜率与 IFFT 路径。

相位斜率（与 SDK `cs_de_phase_slope` 同类，相邻有效信道按 1 MHz）：

\[
d = -\frac{c \cdot \mathrm{atan2}\big(\sum Q,\, \sum I\big)}{4\pi \cdot 10^{6}}
\]

IFFT：若要与固件侧对照，**NFFT = 1024**（`CONFIG_BT_CS_DE_1024_NFFT`）。固件在组帧**之后**才对本机 float 缓冲调用 `cs_de_ifft()`；串口帧中的 IQ 是 IFFT **之前**的原始合成 PCT。

幅度注意：IPT 只有发起端收到的回程幅度，不是全功能 CS 的 \(A_{INIT}\cdot A_{REFL}\)；视距单径峰位主要看相位，多径时峰可能与 RAS 路径有差别。本协议**无 RTT / NADM**。

---

## 8. CRC16

| 项目 | 值 |
|------|-----|
| 算法 | CRC16-CCITT |
| 多项式 | `0x1021` |
| 初值 | `0xFFFF` |
| 覆盖 | 从 **sync1** 到 **IQ 区最后一字节**（不含 CRC 两字节） |
| 帧尾 | uint16 **little-endian** |

实现：`cs_uart_bin_crc16_ccitt()`；PC 侧见 `cs_parse_uart.py` 的 `crc16_ccitt()`。

---

## 9. 何时发一帧 / 何时不发

| 条件 | 行为 |
|------|------|
| `procedure_done_status == COMPLETE` 且未 abort，且 `channel_count > 0` | 发一帧 `0x03` |
| procedure / subevent abort | 不发 |
| 有效信道数为 0 | 不发 |
| 发送队列满（`K_NO_WAIT`） | 丢帧，固件 `LOG_WRN` |

同一 procedure 的多个 subevent 先累加到同一 IQ 缓冲；`procedure_counter` 变化时清零再累加。后到的同信道 step **覆盖**先前值。

---

## 10. 设备侧操作（与上位机配合）

| 操作 | 行为 |
|------|------|
| 上电 / 断线 | 自动扫描名称为 `Nordic CS IPT Reflector` 的设备；建链、加密、CS 配置与 security 自动完成 |
| **Button 1**（DK 第一键，文档称键 0） | 打开 CS procedure，点亮测距 LED，开始产出 `0x03` |
| **Button 2**（DK 第二键，文档称键 1） | 关闭 procedure，熄灯，停止组帧 |
| 断线 | 熄灯，重新扫描（不冷启动） |

本期**无** Flash、无键 2/3、无 `$CMD` 文本命令。不要在同一条 UART 上混跑测向 `$CMD` 协议。

---

## 11. 上位机实现清单

1. **串口**：230400 8N1，打开发起端 console 口。
2. **搜帧**：字节流中找 `0x55 0xAA`；读 `version/type/payload_len`；按 type 定每信道字节数（`0x03`→4）。
3. **定长**：`total = 30 + (payload_len - 24) + 2`；收齐后校验 CRC。
4. **分支**：`type==0x03` → 解析位图 → 升序 ch → 每点 `(i,q)`。
5. **测距**：跳过 `combined_iq_calculate`；直接相位斜率 +（可选）1024 点 IFFT。
6. **兼容**：若旧上位机只能吃 `0x02`，固件未提供自动伪装；应新增 `0x03` 分支，或短期自行把 `(i,q)` 当作已合成结果处理。

参考脚本：

```bash
cd 301ini_Dip/doc
pip install pyserial
python cs_parse_uart.py COM3 --baud 230400
python cs_parse_uart.py COM3 --baud 230400 --csv ipt_out.csv --dump-hex
```

成功时日志形如：`IPT-combined pc=… N=70 ch=[0..74]`（起止 ch 随位图变化）。

CSV 中 `type=3` 行的 `i_local,q_local` 列即本帧 I/Q；`i_remote,q_remote` 为空。

---

## 12. 数据流（固件）

```
le_cs_subevent_data_available
  → 累加 Mode-2 HIGH PCT → iq_int16[] + float iq[]
  → procedure COMPLETE → sem → main
distance_estimates_update
  → ipt_bin_build_frame (type 0x03)     ← 必须在 IFFT 之前
  → cs_uart_bin_enqueue_frame
  → cs_de_ifft(float 缓冲，会破坏原 IQ)
cs_uart_tx 线程 → uart_poll_out(console)
```

---

## 13. 源码与文档索引

| 内容 | 位置 |
|------|------|
| 组帧 / 按键 / procedure 启停 | `331ipt_initiator/src/main.c` |
| 传输层 | `331ipt_initiator/src/cs_uart_binary.h` / `.c` |
| 类型宏 `CS_UART_BIN_TYPE_IPT_COMBINED_IQ` | 同上；`301ini_Dip/src/cs_uart_binary.h` 已同步登记 |
| UART overlay 230400 | `331ipt_initiator/boards/nrf54l15dk_nrf54l15_cpuapp.overlay` |
| 语义与方案 | [IPT_数据特性与上位机适配方案.md](./IPT_数据特性与上位机适配方案.md) |
| PC 解析 | `301ini_Dip/doc/cs_parse_uart.py` |
| DIP / CS 帧头对照 | `301ini_Dip/doc/DIP_binary_protocol.md`、`CS_binary_protocol.md` |

---

## 14. 版本历史

| version | 说明 |
|---------|------|
| 0x02 | 与 DIP/CS 共用帧头（含 `timestamp_ms`）；新增 **type=`0x03`** IPT 已合成 IQ（4 B/信道） |
