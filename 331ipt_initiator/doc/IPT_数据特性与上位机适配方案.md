# 331 IPT 数据特性与上位机适配方案

本文说明 `331ipt_initiator` / `331ipt_reflector` 最终拿到的测量数据从哪来、和全功能 Channel Sounding（CS）以及 DIP 有什么差别，以及如何接到现有上位机接口上。

对照工程：

| 工程 | 角色 |
|------|------|
| `331ipt_initiator`、`331ipt_reflector` | 官方 BLE Channel Sounding Inline PCT Transfer（IPT）示例，尚未接上位机 |
| `301ini` / `301ref` | 全功能 CS：启用 Ranging Service（RAS），上传双端 IQ |
| `301ini_Dip` | 跳过 RAS，只上传发起端本地 IQ；帧格式是现有上位机接口 |
| `321df_central` / `321df_peripheral` | 测向工程；文本命令协议，不是 CS 二进制帧 |

本地 SDK 核对：`D:\n\v3.3.1\nrf\subsys\bluetooth\cs_de\cs_de.c` 与 `include/bluetooth/cs_de.h`。`CS_DE_NUM_CHANNELS` 为 75；双端合成是逐信道复数相乘。

---

## 结论

发起端最终只有一组本地 PCT。它的相位已经是往返信道相位 \(2\theta_{CH}\)。

这组数在测距意义上，对应全功能 CS 做完双端复数相乘之后的结果。线上格式应接近 DIP 的每信道 4 字节，不能按全功能 CS 的双端 8 字节去填两端原始 PCT。

距离不是控制器给出的。应用在 procedure 完成后对这组 IQ 调用 `cs_de_ifft()`，再做 9 点中值，然后打日志。

反射端不解析、不上传 PCT。相位补偿在控制器内完成。

---

## 1. IPT 最终数据从哪来

### 1.1 两端各自做什么

`331ipt_initiator` 不订阅 Ranging Service，也不保存对端 step data。一次 procedure 的原始量和距离都来自本机 HCI 子事件 `le_cs_subevent_data_available`。

`331ipt_reflector` 只打开 reflector 角色并等待对端建配置。它没有该子事件回调，不解析 PCT，也不上传测量值。

IPT 由发起端在创建 CS 配置时打开：

```c
/* 331ipt_initiator/src/main.c, cs_config_get() */
config_params->cs_enhancements_1 = 1;
```

打开之后，反射端控制器按刚收到的音调相位发射回程音调。发起端测到的相位里已经包含反射端测到的相位和回程路径相位。反射端的原始相位不必再经 ACL / RAS 送回。

### 1.2 原始 PCT

控制器在每个 Mode-2 step 的本地 step data 里给出 `tone_info[0].phase_correction_term`。应用用 `bt_le_cs_parse_pct()` 解成 int16 的 I/Q，再写入缓冲：

```c
/* 331ipt_initiator/src/main.c */
static void pcts_parse(uint8_t channel_index,
                       struct bt_hci_le_cs_step_data_tone_info *local_tone_info)
{
    struct bt_le_cs_iq_sample local_iq =
        bt_le_cs_parse_pct(local_tone_info[0].phase_correction_term);

    iq.values[channel_index - CHANNEL_INDEX_OFFSET].i = local_iq.i;
    iq.values[channel_index - CHANNEL_INDEX_OFFSET].q = local_iq.q;
}
```

`CHANNEL_INDEX_OFFSET` 为 2。`fft_ch = hci_channel - 2`。信道 2 对应 2404 MHz，信道 76 对应 2479 MHz，下标 0..74，共 75 点，与 `CS_DE_NUM_CHANNELS` 以及 DIP 的位图宽度一致。信道 23–25 留给广播，这些格点保持 0。

`phase_correction_term` 仍是 HCI 上那 3 字节 PCT（12 bit 有符号 I、12 bit 有符号 Q）。IPT 改变的是反射端发射相位，不是给发起端的 step data 多塞一组远端 PCT 字段。

解析范围：

- 只处理 Mode 2。Mode 0 的 CS_SYNC 被跳过。
- 同一 procedure 的多个 subevent 累加到同一块缓冲；procedure counter 变化时先清零。
- 同一信道后到的 step 覆盖先前的值。
- 当前单天线 A1/B1，`tone_info[0]` 就是唯一路径。
- Tone Quality Indicator 没有过滤，上报的音调全部写入。

### 1.3 这组 PCT 的相位

全功能 CS 在数字域把两端相位相加，消掉本振差：

\[
\theta_{INIT} = \theta_{CH} - \Delta\theta_{LO}
\]

\[
\theta_{REFL} = \theta_{CH} + \Delta\theta_{LO}
\]

\[
\theta_{INIT} + \theta_{REFL} = 2\theta_{CH}
\]

Nordic `cs_de_combined_iq_calculate()`（`D:\n\v3.3.1\nrf\subsys\bluetooth\cs_de\cs_de.c`）是逐信道复数相乘，相位相加：

\[
I = I_L I_R - Q_L Q_R
\]

\[
Q = I_L Q_R + I_R Q_L
\]

内存布局与实现一致：偶数下标是 I，奇数下标是 Q。`cs_de.h` 里“偶数是虚部、奇数是实部”那句和 `.c` 相反。发起端现有 `struct { float i; float q; }` 与 `.c` 一致，不要按头文件注释对调。

IPT 把这一步挪到反射端模拟前端。反射端用刚测到的相位校准回程音调，发起端下变频后的基带相位直接是 \(2\theta_{CH}\)。规范侧同时规定：反射端再上报的 PCT 虚部为 0，实部只剩幅度。这组“远端 PCT”到不了当前发起端，也不再含有可用于相加的相位。

因此发起端这一组 I/Q 已经是全功能 CS 里复数相乘的输出，不是 `i_local/q_local` 那一半，也不是 DIP 那种还缺对端的原始相位。

### 1.4 距离

控制器不给距离。`procedure_done_status == COMPLETE` 后，主循环调用 `distance_estimates_update()`：

```c
float distance_ifft = cs_de_ifft(iq.scratch_mem);
```

`cs_de_ifft()` 对该 IQ 做 IFFT，在幅度谱上找最短路径峰，把峰下标换成米。`iq.values[]` 与 `iq.scratch_mem[]` 是同一块 union，布局就是 IFFT 要求的交织 I/Q。

随后把有限值放进长度 9 的环形缓冲，取中值，`LOG_INF` 打出 `median` 和当次 `update`。样本没有调用 `cs_de_phase_slope()`，也没有 RTT。

`cs_de_ifft()` 会把输入缓冲覆盖成幅度谱。若要上传原始 IQ，必须在这次调用之前拷贝。

`rtt_type` 虽配置为 AA only，Mode-0 的时延没有解析。Core 6.3 也写明 IPT 只覆盖相位测距（PBR）；Mode-3 的 RTT 仍要靠 RAS 回传。

---

## 2. 与全功能 CS、DIP 的比较

| 项目 | 全功能 CS（`301ini` + RAS） | DIP（`301ini_Dip`） | IPT（`331ipt_*`） |
|------|------------------------------|---------------------|-------------------|
| 发起端 PCT 相位 | \(\theta_{CH}-\Delta\theta_{LO}\) | 同左，只此一组 | 已是 \(2\theta_{CH}\) |
| 反射端 PCT | \(\theta_{CH}+\Delta\theta_{LO}\)，经 RAS 取回 | 不取 | 控制器内相位为 0、只剩幅度；本工程不取 |
| 设备上的合成 | `cs_de_calc`：复数相乘后再 IFFT、相位斜率、RTT | 不做 | 本地 IQ 直接 `cs_de_ifft` |
| 串口载荷 | type `0x02`，每信道 8 字节 | type `0x01`，每信道 4 字节 | 目前只有日志，没有帧 |
| 一帧对应 | 一次 procedure（本地 step 收齐且 RAS 返回） | 每个 subevent 一帧 | 应是一次完成的 procedure |
| 距离后处理 | IFFT / phase slope / RTT，质量门控 | 无 | 仅 IFFT + 9 点中值 |
| NFFT | `301ini` 为 512 | 不估算距离 | 1024 |
| TQI | `cs_de` 质量判断 | 只留 HIGH | 全部送入 |
| RTT / NADM | 可有 | 无 | 无。Mode 2、无 RAS |
| 天线 | A1/B1 | A1/B1 | A1/B1，且写死 `tone_info[0]` |

相位测量可以和全功能 CS 对齐：把 IPT 的本地 I/Q 直接送给现在用在“相乘之后”的相位斜率和 IFFT，得到的就是同一类 \(2\theta_{CH}(f)\)。官方示例日志里约 0.60 m 的结果说明这条链路是通的。

测量上仍有三处不能当成同一份原始数据：

1. 幅度不是两端幅度之积。全功能 CS 的 IFFT 权重是 \(A_{INIT}\cdot A_{REFL}\)；IPT 只有发起端收到的回程幅度。视距单径时峰位置主要由相位决定，多径时峰会有差别。
2. 没有 RTT，也没有对端 tone quality。官方说明写明只用 Mode 2，抗测距攻击弱于 RAS 上 Mode 1+2 的组合。
3. 采样网格不完全一样。IPT 的 `min/max_main_mode_steps` 是 2..5，`301ini` 现行配置是 1/1。IPT 的 procedure interval 固定为 2，subevent 固定 11000 µs，发射功率用控制器最大档。`301ini` 的 interval 为 1..10，subevent 10–40 ms，功率 8 dBm，NFFT 是 512 对 1024。同一段 IQ 用不同 NFFT，量化后的米数会不同。可比的是 IQ，不是日志里的米数。

DIP 虽然同样跳过 RAS、同样每信道 4 字节，物理含义相反：DIP 的 I/Q 仍是单侧 \(\theta_{CH}-\Delta\theta_{LO}\)，缺对端就不能测距。不能把 IPT 帧标成 `0x01` 交给现有 DIP 解析逻辑。

### 2.1 配置差异（便于以后对齐实验条件）

| 参数 | IPT 发起端 | `301ini` 现行配置 |
|------|------------|-------------------|
| 主模式 | Mode 2，无子模式 | Mode 2，子模式未用 |
| main mode steps | min 2，max 5 | min 1，max 1 |
| Mode-0 steps | 3 | 3 |
| RTT 类型 | AA only（未解析） | AA only |
| CS sync PHY | LE 1M | LE 1M |
| 信道选择 | 算法 3b | 算法 3b |
| IPT | `cs_enhancements_1 = 1` | 无此字段（3.0.1 代） |
| procedure interval | 固定 2 | 1 .. 10 |
| subevent 长度 | 11000 µs | 10000 .. 40000 µs |
| 天线 | A1/B1 | A1/B1 |
| procedure PHY | LE 2M | LE 2M |
| 发射功率 | 控制器最大档 | 8 dBm |
| NFFT | 1024 | 512 |
| RAS | 关闭 | 开启 |
| 扫描过滤 | 设备名 `Nordic CS IPT Reflector` | Ranging Service UUID |
| 断线 | 冷启动 | 只清连接，不重启 |
| 启动 procedure | `main()` 里自动打开 | 建链后也会打开；另有按键再启停 |

---

## 3. 上位机适配方案

本期只做发起端实时上传，不做 Flash，不做键 2 / 键 3，不做 `$CMD`。

要保证的是一次 procedure 里大约 70 个信道的 IQ 用二进制帧送出。DIP 已经把这条链路做成 `0x55 0xAA` 帧加独立发送线程，IPT 复用传输层，帧类型用 `0x03`（每信道仍是 4 字节，和 DIP 一样快）。按键只保留键 0 开始测量、键 1 停止测量。

满信道大约 N=72（75 个格点去掉广播信道 23–25）。整帧约 `32 + 4×72 = 320` 字节。`301ini` 的采集串口是 230400。按 10 bit/字节估算，一帧约 14 ms，短于官方示例里大约 30 ms 的测距间隔。若仍用 115200，一帧约 28 ms，和 procedure 间隔贴在一起，队列容易丢帧。采集口沿用 230400，并关掉 `LOG_INF`，避免 ASCII 把帧拆开。

现有 CS 上位机接口在 `301ini_Dip`：

- 同步字 `0x55 0xAA`
- version `0x02`
- 30 字节帧头（含 `procedure_counter`、`timestamp_ms`、天线、10 字节信道位图）
- CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`，覆盖从 sync 到 IQ 区末字节
- 传输层：`301ini_Dip/src/cs_uart_binary.c`，入队后由 `cs_uart_tx` 线程 `uart_poll_out`

帧类型：

| type | 含义 | 每有效信道 |
|------|------|------------|
| `0x01` | DIP 本地原始 IQ | 4 字节：`i, q` |
| `0x02` | 全功能 CS 双端原始 IQ | 8 字节：`i_local, q_local, i_remote, q_remote` |
| `0x03`（建议新增） | IPT 已合成 IQ | 4 字节：`i, q` |

`321df` 的 `$CMD` / `$OK` / `$EVT` 是测向文本协议。CS 工程本身没有这套命令解析。不要把它和 `0x55 0xAA` 混在同一条 UART 上当 CS 数据通道。

数据出口只做在发起端。反射端保持 IPT 对端，不启用 RAS，不上传 PCT。

### 3.1 帧定义（type `0x03`）

帧头与 `0x01` / `0x02` 相同，定义见 `301ini_Dip/src/cs_uart_binary.h` 的 `struct cs_uart_bin_header`。

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | sync1 | `0x55` |
| 1 | 1 | sync2 | `0xAA` |
| 2 | 1 | version | `0x02` |
| 3 | 1 | type | `0x03` |
| 4 | 2 | payload_len | 小端；从 `procedure_counter` 到 IQ 末字节，不含 CRC |
| 6 | 2 | procedure_counter | HCI procedure counter |
| 8 | 8 | timestamp_ms | `k_uptime_get()` |
| 16 | 1 | ap | 当前为 0 |
| 17 | 1 | iq_format | 0 = int16 |
| 18 | 1 | channel_count | 有效 fft 信道数 N |
| 19 | 1 | reserved | 0 |
| 20 | 10 | channel_bitmap | 与 DIP 相同，bit0 = LSB |
| 30 | 4×N | IQ | 按 ch 升序，`int16 i`、`int16 q`，小端 |
| 30+4N | 2 | crc16 | 小端 |

`payload_len = 24 + 4×N`。整帧长度 `32 + 4×N`。

位图：`fft_ch = hci_channel - 2`，`channel_bitmap[ch/8]` 的 bit `(ch%8)` 表示该信道有效。只对置位信道写 IQ。

上位机对 `0x03` 应直接做相位斜率和 IFFT，跳过 `cs_de_combined_iq_calculate()`。NFFT 若要和固件日志对照，用 1024。相位斜率公式与 SDK 相同，输入就是这份已合成 IQ：

\[
d = -\frac{c \cdot \mathrm{atan2}(\sum Q, \sum I)}{4\pi \cdot 10^{6}}
\]

相邻信道间隔按 1 MHz。未测信道为 0 时，跨过 0 的一对不会贡献斜率，等效于只使用间隔 1 MHz 的相邻有效点。

不要把这组数填进 type `0x02` 的 `i_remote/q_remote`。那四个数在现有协议里是相乘前的两端原始 PCT。

若短期不能改上位机，兼容做法是仍发 type `0x02`，每个有效信道的远端固定为 `(1, 0)`。现有复数相乘会原样得到发起端 IQ，相位和相对幅度都保留。代价是带宽翻倍，并且帧内容对外表示“有远端测量”。反射端真实幅度无法在发起端恢复，不应编造。优先用 `0x03`。

### 3.2 发起端改动

1. 传输层复用 `301ini_Dip/src/cs_uart_binary.c` 与 `cs_uart_binary.h`。组帧放在发起端。
2. 在 `cs_de_ifft()` 之前把 int16 I/Q 拷出再入队。`bt_le_cs_parse_pct()` 的结果本来就是 int16，直接保存，不必先转 float 再四舍五入。
3. 聚合粒度跟现有 IPT 和全功能 CS：多个 subevent 攒满，仅当 `procedure_done_status == COMPLETE` 且未 abort 时发一帧。abort 或空 step 不发。不要学 DIP 每个 subevent 发一帧，否则上位机会看到残缺信道图。
4. 只保留 `quality_indicator == HIGH` 的音调。其余格点不置位图，送给 IFFT 的值保持 0。这是 DIP 已有、官方 IPT 示例省略的过滤。
5. 单天线继续读 `tone_info[0]`。若以后 `n_ap > 1`，改用 `bt_le_cs_get_antenna_path()`。
6. `prj.conf` 保持 IPT 专用项：`CONFIG_BT_CTLR_EXTENDED_FEAT_SET`、不启用 `CONFIG_BT_RAS`、`CONFIG_BT_CS_DE_1024_NFFT`、initiator-only、按名字扫描 `Nordic CS IPT Reflector`。不要改回 RAS UUID 过滤，反射端没有 Ranging Service。
7. 二进制采集时把 `LOG_INF` 降到警告以下。现在的距离日志和 `0x55 0xAA` 走同一 console，会拆帧。固件距离若仍要给上位机，放进帧内（后续可加毫米整型字段），不要继续打 ASCII。

`main()` 在 `bt_enable()` 之前调用一次 `cs_uart_bin_tx_thread_start()`。组帧缓冲放 BSS，不要放在蓝牙接收工作队列的栈上。入队使用 `K_NO_WAIT`，队列满则丢帧并打警告。这些约束与 `301ini_Dip/doc/firmware_uart_binary_architecture.md` 相同。

### 3.3 按键

本期只移植键 0 和键 1，加在 `331ipt_initiator`。键 2、键 3 和 Flash 先不做。反射端不加按键。

| 按键 | 行为 |
|------|------|
| 键 0 | 打开 procedure，LED 表示正在测距，开始发送 `0x03` |
| 键 1 | 关闭 procedure，熄灭该灯，停止组帧 |

不要带上 `301ini` 键 0 里的 `flash_check_and_suggest_erase()` 和写入起点设置。建链、加密和 CS security 仍可自动完成，procedure 等键 0。官方示例目前是建链后自动开测并在断线时冷启动；改成按键控制后，断线重新扫描即可。

GPIO 用开发板 `button0`、`button1`，可复用 `301ini/src/interface/button_led.c` 里对应的两个回调。

### 3.4 命令

现有 CS 上位机只吃 `0x55 0xAA`。不要在同一 UART 上同时跑 `321df` 的 `$CMD` 行协议和二进制帧。

若确实要上位机启停，用第二条 UART，或仅在 procedure 关闭时解析少量命令，例如：

```text
$CMD,PING
$CMD,CS_START
$CMD,CS_STOP
```

应答仍用 `$OK` / `$ERR`。测距进行中只输出二进制帧。

### 3.5 反射端

保持广播名 `Nordic CS IPT Reflector`、reflector-only、`CONFIG_BT_CTLR_EXTENDED_FEAT_SET`。CS 配置由发起端写入，其中包含 IPT 位。

反射端没有可上传的双端 PCT。若把反射端本地 PCT 打出来，相位应接近 0，只剩幅度。不能拿来和发起端再相加一次，否则会把已经补偿过的 \(2\theta_{CH}\) 再算错。

反射端不需要 RAS，也不需要二进制组帧。连接灯可保持现状。

### 3.6 落地顺序

1. 发起端在 IFFT 前按 procedure 组 `0x03` 帧，串口 230400，关掉 `LOG_INF`。PC 侧按 4 字节 IQ 校验 CRC 和位图，确认一次 procedure 能收到约 70 个信道。解析可在 `301ini_Dip/doc/cs_parse_uart.py` 上增加 `0x03` 分支，宽度与 `0x01` 相同。
2. 加上键 0 / 键 1，只控制 procedure 启停。
3. 上位机对 `0x03` 直接做相位斜率与 1024 点 IFFT，再与 `301ini` 的 type `0x02` 对照。Flash、键 2、键 3、命令口以后再加。

---

## 4. 参考位置

| 内容 | 位置 |
|------|------|
| IPT 解析与 IFFT | `331ipt_initiator/src/main.c`：`pcts_parse`、`subevent_steps_parse`、`subevent_result_cb`、`distance_estimates_update`、`cs_config_get` |
| IPT 反射端 | `331ipt_reflector/src/main.c` |
| 官方行为说明 | `331ipt_initiator/README.rst`「How CS IPT works」 |
| 双端合成与 IFFT | `D:\n\v3.3.1\nrf\subsys\bluetooth\cs_de\cs_de.c`：`cs_de_combined_iq_calculate`、`cs_de_ifft`、`cs_de_phase_slope` |
| 信道数 75 | `D:\n\v3.3.1\nrf\include\bluetooth\cs_de.h`：`CS_DE_NUM_CHANNELS` |
| 二进制帧与传输 | `301ini_Dip/src/cs_uart_binary.h`、`cs_uart_binary.c` |
| DIP 组帧 | `301ini_Dip/src/main.c`：`dip_bin_build_frame` |
| 全功能 CS 组帧 | `301ini_Dip/src/cs_de_data_parse.c`：`cs_bin_build_dual_iq_frame` |
| 协议说明 | `301ini_Dip/doc/DIP_binary_protocol.md`、`CS_binary_protocol.md`、`firmware_uart_binary_architecture.md` |
| 按键 | `301ini/src/main.c` 的 `button0_work_handler` / `button1_work_handler` |
| 文本命令（测向，非 CS 帧） | `321df_central/docs/uart_command_format.md` |
