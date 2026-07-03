# 这些代码是执行BLE CS测距示例的代码，角色是initiator。代码从HCI中获取本地report之后，再利用RAS服务获取对端数据，从而结合两者的测量结果计算距离。我想知道的是，ranging_data_get_complete_cb 里面最终能获得一个cs_de_report。里面有local iq和peer iq。我想知道的是，local iq 的获取是否依赖对端提供某些校正项？HCI的report在subevent_result_cb 已经报告了step PCT，如何由这个PCT转成local iq ？

是的，**`cs_de_report` 里的 `local iq` 不是“纯本地原始IQ采样”**，而是**由本地 HCI report 里的 step PCT（Phase Correction Term）解析出来的复数表示**；它**不依赖对端再额外提供一个“给本地IQ做校正”的量**。
 对端提供的是**它自己的 step PCT / RTT 数据**，也就是 `peer iq` 对应的数据。最后算法把 **local PCT** 和 **peer PCT** 结合起来做距离估计。

你这套代码链路里，关键关系是：

1. **本地控制器**通过 HCI 上报 CS subevent result
    → `subevent_result_cb()`
2. 这些本地 step data 被存到 `latest_local_steps`
3. **对端 reflector** 的 RAS 数据被取回
    → `ranging_data_get_complete_cb()`
4. 在 `ranging_data_get_complete_cb()` 里调用
    `cs_de_populate_report(&latest_local_steps, &latest_peer_steps, ...)`
5. `cs_de_populate_report()` 解析本地和对端 step data
6. 对于 mode 2 / mode 3，分别从 **local step data** 和 **peer step data** 中取出 tone info 里的 PCT
7. 再调用 `bt_le_cs_parse_pct()`，把 3 字节 PCT 转成 `I/Q`

------

## 先回答你的核心问题

### 1. `local iq` 的获取是否依赖对端提供某些校正项？

**不直接依赖。**

`local iq` 是从**本地 HCI report** 里的 `local_tone_info[antenna_path].phase_correction_term` 解析出来的。
 对应代码在 `cs_de.c`：

```c
struct bt_le_cs_iq_sample local_iq =
    bt_le_cs_parse_pct(local_tone_info[antenna_path].phase_correction_term);
```

这里的 `local_tone_info` 来自：

```c
struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
    (struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
```

也就是说：

- `local_step` 来自 `latest_local_steps`
- `latest_local_steps` 来自 `subevent_result_cb()`
- `subevent_result_cb()` 里的数据来自 HCI event `bt_hci_le_cs_subevent_result`

所以 **local iq 完全是由本端 controller 上报的 step data 里的 PCT 解析得到的**。

------

### 2. HCI report 在 `subevent_result_cb` 已经报告了 step PCT，如何由这个 PCT 转成 local iq？

转换函数就在 `cs.c` 里：

```c
struct bt_le_cs_iq_sample bt_le_cs_parse_pct(const uint8_t pct[3])
{
    uint32_t pct_u32 = sys_get_le24(pct);

    /* Extract I and Q. */
    uint16_t i_u16 = pct_u32 & BT_HCI_LE_CS_PCT_I_MASK;
    uint16_t q_u16 = (pct_u32 & BT_HCI_LE_CS_PCT_Q_MASK) >> 12;

    /* Convert from 12-bit 2's complement to int16_t */
    int16_t i = (i_u16 ^ BIT(11)) - BIT(11);
    int16_t q = (q_u16 ^ BIT(11)) - BIT(11);

    return (struct bt_le_cs_iq_sample){.i = i, .q = q};
}
```

它做了三件事：

1. 把 **3字节 little-endian PCT** 读成一个 24-bit 数
2. 低 12 bit 取作 `I`
3. 高 12 bit 取作 `Q`
4. 两者都按 **12-bit two's complement** 转成有符号 `int16_t`

也就是说，PCT 本质上就是一个压缩编码的复数：

$\text{PCT} = I + jQ$

其中：

- `I` 占 12 bit
- `Q` 占 12 bit
- 都是补码有符号数

------

## 从你的代码路径完整串一下

------

### 一、本地 HCI report 是怎么进来的

在 `main.c` 里：

```c
.le_cs_subevent_data_available = subevent_result_cb,
```

说明当 controller 有 CS subevent result 时，会调用：

```c
static void subevent_result_cb(struct bt_conn *conn,
                               struct bt_conn_le_cs_subevent_result *result)
```

这里：

```c
if (result->step_data_buf) {
    ...
    uint16_t len = result->step_data_buf->len;
    uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

    net_buf_simple_add_mem(&latest_local_steps, step_data, len);
}
```

这一步只是**把 HCI 上报的原始本地 step data 存起来**，此时并没有解析成 IQ。

------

### 二、什么时候开始解析本地 / 对端数据

在 `ranging_data_get_complete_cb()` 里：

```c
cs_de_populate_report(&latest_local_steps, &latest_peer_steps,
                      BT_CONN_LE_CS_ROLE_INITIATOR, &cs_de_report);
```

此时本地和对端两份 step 数据都齐了，才开始组合解析。

------

### 三、`cs_de_populate_report()` 怎么解析

`cs_de_populate_report()` 里调用：

```c
bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, role,
                                   process_ranging_header,
                                   NULL, process_step_data, p_report);
```

这里做的是：

- 先解析 peer ranging header
- 然后按 subevent / step 对齐地遍历
- 每一步拿到：
  - `local_step`
  - `peer_step`

之后进入：

```c
static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
                              struct bt_le_cs_subevent_step *peer_step, void *user_data)
```

如果是 mode 2：

```c
struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
    (struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
    (struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

extract_pcts(..., local_step_data->tone_info, peer_step_data->tone_info);
```

如果是 mode 3，也会走 `extract_pcts(...)`。

------

### 四、`extract_pcts()` 里真正把 PCT 变成 IQ

关键代码：

```c
struct bt_le_cs_iq_sample local_iq =
    bt_le_cs_parse_pct(local_tone_info[antenna_path].phase_correction_term);
struct bt_le_cs_iq_sample remote_iq =
    bt_le_cs_parse_pct(remote_tone_info[antenna_path].phase_correction_term);
```

然后保存到 report：

```c
p_report->iq_tones[antenna_path].i_local[channel_index] = local_iq.i;
p_report->iq_tones[antenna_path].q_local[channel_index] = local_iq.q;
p_report->iq_tones[antenna_path].i_remote[channel_index] = remote_iq.i;
p_report->iq_tones[antenna_path].q_remote[channel_index] = remote_iq.q;
```

所以：

- `local iq` ← 本地 HCI step data 的 PCT
- `peer iq` ← RAS peer step data 的 PCT

------

## 所以“local iq 是否依赖对端校正项”要分两层理解

### 层 1：代码数据来源层面

**不依赖。**

本地 `local iq` 的数值来自本地 controller 已经生成好的 `phase_correction_term`。
 你的 host 代码只是**解包**它，不是自己根据对端信息去算出来的。

------

### 层 2：协议/物理意义层面

**PCT 本身是 controller 根据底层测量和规范定义生成的“相位校正项”表示。**
 它不是“原始ADC IQ样本”，而是**控制器已经处理过的、适合上层进行双端组合的复数项**。

因此更准确地说：

- `local iq` **不依赖 peer step data 才能被解析**
- 但 `local iq` 本身也**不是未经处理的本地原始RF IQ**
- 它是**本地控制器基于本地测量生成的 PCT，再被 host 解析成 I/Q**

------

## 为什么最后要把 local/peer IQ 组合？

在 `cs_de_calc()` 里：

```c
calculate_vec_cmac_f(m_iq_scratch_mem, p_report->iq_tones[ap].i_remote,
                     p_report->iq_tones[ap].q_remote,
                     p_report->iq_tones[ap].i_local,
                     p_report->iq_tones[ap].q_local);
```

这个函数：

```c
iq_result[2 * n]     = i_1[n] * i_2[n] - q_1[n] * q_2[n];
iq_result[2 * n + 1] = i_1[n] * q_2[n] + i_2[n] * q_1[n];
```

这其实是在做复数乘法：

$(I_1 + jQ_1)(I_2 + jQ_2)$

也就是把本地和对端的复数项组合起来，得到后续 phase slope / IFFT 距离估计所需的量。

所以本地 IQ 自己能单独解析出来，但**距离估计**需要双边数据联合。

------

## 你可以把这件事理解成：

### `subevent_result_cb()`

拿到的是：

- 本端 controller 的 CS step data
- 其中包含本端每个 tone 的 PCT

### `ranging_data_get_complete_cb()`

再拿到：

- 对端 reflector 的 CS step data
- 其中包含对端每个 tone 的 PCT

### `cs_de_populate_report()`

做的事：

- 对齐本端和对端 step
- 把双方的 PCT 都解析成 IQ
- 放到 `cs_de_report`

### `cs_de_calc()`

做的事：

- 用 local IQ 和 peer IQ 做复数融合
- 算 phase slope / IFFT / RTT distance

------

## 如果你问“PCT为什么能直接当 IQ 用？”

因为在这份 Nordic 代码里，PCT 被解释成一个复数样本，具体格式是：

- 24 bit
- 低 12 bit = I
- 高 12 bit = Q
- 都是 signed 12-bit

因此 `bt_le_cs_parse_pct()` 返回：

```c
struct bt_le_cs_iq_sample { .i = i, .q = q }
```

后续算法直接把这个当作每个 channel / antenna path 上的复平面点来处理。

------

## 一个简单例子

假设某个 tone 的 `phase_correction_term` 是 3 字节：

```c
pct[0], pct[1], pct[2]
```

先拼成：

```c
pct_u32 = pct[0] | (pct[1] << 8) | (pct[2] << 16)
```

然后：

```c
i_u16 = pct_u32 & 0x0FFF;
q_u16 = (pct_u32 >> 12) & 0x0FFF;
```

再把 12 位补码转成有符号整数：

```c
i = sign_extend_12bit(i_u16);
q = sign_extend_12bit(q_u16);
```

最后得到：

```c
local_iq = i + jq
```

这就是 `local iq`。

------

## 结论

### 直接回答你：

**1. `local iq` 的获取是否依赖对端提供某些校正项？**
 **不依赖。** `local iq` 直接来自本地 HCI report 中的 step PCT，经 `bt_le_cs_parse_pct()` 解析得到。

**2. HCI report 在 `subevent_result_cb` 已经报告了 step PCT，如何由这个 PCT 转成 local iq？**
 通过 `bt_le_cs_parse_pct()`：

- 把 3-byte little-endian PCT 读成 24-bit
- 低 12 bit 取 `I`
- 高 12 bit 取 `Q`
- 按 12-bit 补码扩展成 `int16_t`
- 返回 `struct bt_le_cs_iq_sample { i, q }`

------

如果你愿意，我下一条可以继续给你画一张**从 HCI step_data 到 cs_de_report.i_local/q_local 的完整字段流转图**，包括：

- `bt_hci_evt_le_cs_subevent_result`
- `bt_conn_le_cs_subevent_result.step_data_buf`
- `bt_le_cs_subevent_step`
- `bt_hci_le_cs_step_data_mode_2/3`
- `tone_info[]`
- `phase_correction_term[3]`
- `bt_le_cs_parse_pct()`
- `cs_de_report.iq_tones[ap].i_local/q_local`

这样你会一眼看清每一层结构体之间的对应关系。

下面给你一张**从本地 HCI report 到 `cs_de_report.i_local/q_local`，以及对端 RAS 数据到 `i_remote/q_remote` 的完整流转图**。

------

# 1. 总体流程图

```text
┌─────────────────────────────────────────────────────────────────────┐
│                     Controller / Link Layer                         │
│   CS procedure done -> generate local CS subevent result (HCI evt)  │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  HCI Event: LE CS Subevent Result                   │
│                 bt_hci_le_cs_subevent_result() in cs.c              │
│                                                                     │
│  parse HCI event header                                             │
│  result.step_data_buf = pointer to raw local step data              │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         main.c : subevent_result_cb()               │
│                                                                     │
│  result->step_data_buf                                               │
│      └─ copy raw bytes into latest_local_steps                      │
│                                                                     │
│  latest_local_steps = accumulated LOCAL step data buffer            │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  │   (later, when peer RD is ready)
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│               RAS peer ranging data fetched successfully            │
│               main.c : ranging_data_get_complete_cb()               │
│                                                                     │
│  latest_local_steps  +  latest_peer_steps                           │
│      └─ passed into cs_de_populate_report()                         │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    cs_de.c : cs_de_populate_report()                │
│                                                                     │
│  bt_ras_rreq_rd_subevent_data_parse(peer_steps, local_steps, ...)   │
│                                                                     │
│  step-by-step aligned parsing:                                      │
│      local_step  <---- from latest_local_steps                      │
│      peer_step   <---- from latest_peer_steps                       │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     cs_de.c : process_step_data()                   │
│                                                                     │
│  if mode 2 / mode 3:                                                │
│      local_step->data -> bt_hci_le_cs_step_data_mode_2/3            │
│      peer_step->data  -> bt_hci_le_cs_step_data_mode_2/3            │
│                                                                     │
│      extract_pcts(..., local_step_data->tone_info,                  │
│                         peer_step_data->tone_info)                  │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        cs_de.c : extract_pcts()                     │
│                                                                     │
│  for each antenna path:                                             │
│      local tone_info[ap].phase_correction_term  ----┐               │
│      peer  tone_info[ap].phase_correction_term  ----┼--> parse PCT  │
│                                                      │               │
│  bt_le_cs_parse_pct(pct[3])                          │               │
│      -> { i, q }                                     │               │
│                                                      │               │
│  store into p_report->iq_tones[ap][channel]:         │               │
│      i_local, q_local  <-----------------------------┘               │
│      i_remote, q_remote <---------------------------- peer           │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       final: cs_de_report_t                         │
│                                                                     │
│  iq_tones[ap].i_local[channel]                                      │
│  iq_tones[ap].q_local[channel]                                      │
│  iq_tones[ap].i_remote[channel]                                     │
│  iq_tones[ap].q_remote[channel]                                     │
└─────────────────────────────────────────────────────────────────────┘
```

------

# 2. 本地数据路径：从 HCI 到 `i_local/q_local`

```text
[Controller generates local CS measurement]
                │
                ▼
HCI LE CS Subevent Result Event
                │
                ▼
cs.c : bt_hci_le_cs_subevent_result(struct net_buf *buf)
                │
                │  parse HCI event
                │  result.step_data_buf = raw local steps
                ▼
main.c : subevent_result_cb(struct bt_conn *conn,
                            struct bt_conn_le_cs_subevent_result *result)
                │
                │  copy result->step_data_buf bytes
                ▼
latest_local_steps   (net_buf_simple)
                │
                ▼
cs_de.c : cs_de_populate_report(&latest_local_steps, ...)
                │
                ▼
bt_ras_rreq_rd_subevent_data_parse(...)
                │
                ▼
local_step : struct bt_le_cs_subevent_step
                │
                │  local_step->data
                ▼
struct bt_hci_le_cs_step_data_mode_2
or
struct bt_hci_le_cs_step_data_mode_3
                │
                │  local_step_data->tone_info[antenna_path]
                ▼
tone_info[ap].phase_correction_term[3]
                │
                ▼
bt_le_cs_parse_pct(pct)
                │
                ▼
struct bt_le_cs_iq_sample { i, q }
                │
                ▼
p_report->iq_tones[ap].i_local[channel] = i
p_report->iq_tones[ap].q_local[channel] = q
```

------

# 3. 对端数据路径：从 RAS 到 `i_remote/q_remote`

```text
[Peer reflector stores its ranging data]
                │
                ▼
RAS On-demand Ranging Data
                │
                ▼
main.c : ranging_data_ready_cb()
                │
                ▼
bt_ras_rreq_cp_get_ranging_data(...)
                │
                ▼
ras_rreq.c receives segmented peer ranging data
                │
                ▼
latest_peer_steps  (net_buf_simple)
                │
                ▼
main.c : ranging_data_get_complete_cb()
                │
                ▼
cs_de.c : cs_de_populate_report(&latest_local_steps, &latest_peer_steps, ...)
                │
                ▼
bt_ras_rreq_rd_subevent_data_parse(...)
                │
                ▼
peer_step : struct bt_le_cs_subevent_step
                │
                │  peer_step->data
                ▼
struct bt_hci_le_cs_step_data_mode_2
or
struct bt_hci_le_cs_step_data_mode_3
                │
                │  peer_step_data->tone_info[antenna_path]
                ▼
tone_info[ap].phase_correction_term[3]
                │
                ▼
bt_le_cs_parse_pct(pct)
                │
                ▼
struct bt_le_cs_iq_sample { i, q }
                │
                ▼
p_report->iq_tones[ap].i_remote[channel] = i
p_report->iq_tones[ap].q_remote[channel] = q
```

------

# 4. 结构体层级图

这个更像“内存结构展开图”。

## 4.1 本地 HCI 数据侧

```text
bt_conn_le_cs_subevent_result
└── step_data_buf : net_buf_simple *
    └── raw bytes
        └── bt_le_cs_subevent_step #0
            ├── mode
            ├── channel
            ├── data_len
            └── data
                └── bt_hci_le_cs_step_data_mode_2 / mode_3
                    ├── antenna_permutation_index
                    └── tone_info[...]
                        └── tone_info[antenna_path]
                            ├── quality_indicator
                            └── phase_correction_term[3]
                                └── bt_le_cs_parse_pct()
                                    └── { i, q }
```

## 4.2 对端 RAS 数据侧

```text
RAS ranging data payload
└── ras_ranging_header
└── ras_subevent_header
└── peer step bytes
    └── bt_le_cs_subevent_step #0
        ├── mode
        ├── channel
        ├── data_len
        └── data
            └── bt_hci_le_cs_step_data_mode_2 / mode_3
                ├── antenna_permutation_index
                └── tone_info[...]
                    └── tone_info[antenna_path]
                        ├── quality_indicator
                        └── phase_correction_term[3]
                            └── bt_le_cs_parse_pct()
                                └── { i, q }
```

------

# 5. `PCT -> IQ` 解析图

`bt_le_cs_parse_pct()` 的逻辑可以画成这样：

```text
phase_correction_term[3]
    │
    │ little-endian 24-bit
    ▼
pct_u32 = b0 | (b1 << 8) | (b2 << 16)
    │
    ├── low  12 bits -> I field
    └── high 12 bits -> Q field
    ▼
I = sign_extend_12bit(pct_u32 & 0x0FFF)
Q = sign_extend_12bit((pct_u32 >> 12) & 0x0FFF)
    ▼
bt_le_cs_iq_sample {
    .i = I,
    .q = Q
}
```

更具体一点：

```text
24-bit PCT layout:

 bit 23                                  bit 0
 +----------------+------------------------+
 |   Q[11:0]      |       I[11:0]         |
 +----------------+------------------------+

 little-endian bytes:
 pct[0] = bits  7:0
 pct[1] = bits 15:8
 pct[2] = bits 23:16
```

然后：

```text
I_raw = pct_u32 & 0x0FFF
Q_raw = (pct_u32 >> 12) & 0x0FFF

I = 12-bit signed value
Q = 12-bit signed value
```

也就是：

$\text{IQ} = I + jQ$

------

# 6. 在你的代码里的关键函数调用链

```text
subevent_result_cb()
    └── latest_local_steps <- result->step_data_buf

ranging_data_ready_cb()
    └── bt_ras_rreq_cp_get_ranging_data(...)

ranging_data_get_complete_cb()
    └── cs_de_populate_report(&latest_local_steps, &latest_peer_steps, ...)

cs_de_populate_report()
    └── bt_ras_rreq_rd_subevent_data_parse(..., process_step_data, ...)

process_step_data()
    └── extract_pcts(...)

extract_pcts()
    ├── local_iq  = bt_le_cs_parse_pct(local_tone_info[ap].phase_correction_term)
    ├── remote_iq = bt_le_cs_parse_pct(remote_tone_info[ap].phase_correction_term)
    ├── p_report->iq_tones[ap].i_local[channel]  = local_iq.i
    ├── p_report->iq_tones[ap].q_local[channel]  = local_iq.q
    ├── p_report->iq_tones[ap].i_remote[channel] = remote_iq.i
    └── p_report->iq_tones[ap].q_remote[channel] = remote_iq.q
```

------

# 7. 一个“你最关心的问题”版本

## `local iq` 是怎么来的？

```text
本地 controller
   └─ 生成 local step PCT
       └─ HCI 上报给 host
           └─ latest_local_steps
               └─ local_step_data->tone_info[].phase_correction_term
                   └─ bt_le_cs_parse_pct()
                       └─ local iq
```

## 它是否需要对端先提供校正项？

```text
不需要
```

因为：

- `local iq` 的解析输入就是本地 HCI 里的 `phase_correction_term`
- host 不需要拿 peer 数据来“修正后才能解出 local iq”

但是：

- **距离计算** 需要 local 和 peer 两边一起参与

------

# 8. 最后给你一个简化版“单个 step / 单个 tone”的图

```text
LOCAL SIDE
==========
HCI event
  └── step_data_buf
      └── step
          └── mode_2/mode_3 data
              └── tone_info[path]
                  └── phase_correction_term[3]
                      └── bt_le_cs_parse_pct()
                          └── local_iq = (I_local, Q_local)

PEER SIDE
=========
RAS peer data
  └── step
      └── mode_2/mode_3 data
          └── tone_info[path]
              └── phase_correction_term[3]
                  └── bt_le_cs_parse_pct()
                      └── peer_iq = (I_peer, Q_peer)

REPORT
======
cs_de_report.iq_tones[path]
  ├── i_local[channel]  = I_local
  ├── q_local[channel]  = Q_local
  ├── i_remote[channel] = I_peer
  └── q_remote[channel] = Q_peer
```

如果你愿意，我下一条可以继续给你补一张：

1. **`extract_pcts()` 内部按 `antenna_path` / `channel_index` 写数组的图**
    或者
2. **`cs_de_calc()` 如何把 `local_iq` 和 `peer_iq` 做复数乘法得到后续 phase slope / IFFT 输入的图**

这两张会更接近“算法输入到底是什么”。



# 如果我想从subevent_result_cb 开始，直接从subevent report里拿到本地PCT，然后计算本地IQ，不再走RAS取对端的数据了。我现在只需要本地IQ，不计算距离。理想状态下，我希望可以不断重复执行CS subevent，然后只取本地IQ。那相比现在的代码，流程上会有什么区别？需要怎么修改？

可以，而且**改动不需要很大**。你的目标已经从：

- **双端融合求距离**

变成了：

- **只取 initiator 本地 HCI subevent report 里的 PCT**
- **直接解析成本地 IQ**
- **不再通过 RAS 向对端拉取 peer data**
- **不再等待 `ranging_data_ready` / `get_ranging_data_complete`**
- **不再调用 `cs_de_populate_report()` / `cs_de_calc()`**

这会让流程**明显简化**。

------

# 一、先说结论

如果你**只要本地 IQ**，那么：

## 你不再需要的部分

- `latest_peer_steps`
- `ranging_data_ready_cb()`
- `ranging_data_overwritten_cb()`
- `ranging_data_get_complete_cb()`
- `bt_ras_rreq_cp_get_ranging_data()`
- `bt_ras_rreq_on_demand_rd_subscribe()`
- `bt_ras_rreq_rd_ready_subscribe()`
- `bt_ras_rreq_rd_overwritten_subscribe()`
- `bt_ras_rreq_cp_subscribe()`
- `cs_de_populate_report()`
- `cs_de_calc()`
- `sem_local_steps`
- `most_recent_local_ranging_counter` / `dropped_ranging_counter` 这套“等对端数据对齐”的逻辑

## 你保留的核心

- `bt_enable()`
- 扫描 / 连接 / 配对 / MTU / discovery
- `bt_le_cs_*` 配置和启用流程
- `subevent_result_cb()` 回调
- `bt_le_cs_parse_pct()` 这个 PCT→IQ 解析函数思路
- 对 `step_data_buf` 的遍历解析

## 新的核心处理点

你应该在 **`subevent_result_cb()` 内部** 直接：

1. 遍历 `result->step_data_buf`
2. 找出 mode 2 / mode 3 step
3. 取出其中的 `tone_info[].phase_correction_term`
4. 调用与 `bt_le_cs_parse_pct()` 等价的逻辑
5. 得到本地 IQ
6. 打印、缓存或写 flash

------

# 二、和现在流程相比，最大的流程区别

------

## 现在的流程

你当前的流程本质上是：

```text
本地 controller 产生 subevent report
    -> subevent_result_cb() 收集 local step data
对端 RAS 通知 ready
    -> 发 GET_RD
    -> 收到 peer ranging data
    -> ranging_data_get_complete_cb()
    -> cs_de_populate_report(local + peer)
    -> cs_de_calc()
    -> 得到距离
```

也就是说，**subevent_result_cb() 只是暂存本地数据，不立即做最终处理**。

------

## 你要的新流程

你现在要的是：

```text
本地 controller 产生 subevent report
    -> subevent_result_cb()
    -> 直接解析 step_data_buf 中的本地 PCT
    -> PCT 转本地 IQ
    -> 输出 / 存储本地 IQ
    -> 等下一次 subevent
```

也就是说：

- **不等对端**
- **不做 local/peer 对齐**
- **不做 RAS 请求**
- **每来一个 subevent result 就本地处理一次**

------

# 三、需要修改的核心思路

------

## 方案建议：保留 CS procedure，删除 RAS 依赖

最适合你的方式是：

### 保留

- BLE 连接
- CS config / security / procedure enable
- `subevent_result_cb()`

### 删除或绕过

- RAS subscribe
- RAS peer 数据获取
- 距离计算路径

------

# 四、你需要改哪些地方

下面按 `main.c` 来说。

------

## 1. 删除 peer 相关全局变量

现在有：

```c
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static K_SEM_DEFINE(sem_local_steps, 1, 1);
```

如果只做本地 IQ，建议：

### 保留

```c
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
```

### 删除

```c
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static K_SEM_DEFINE(sem_local_steps, 1, 1);
```

甚至如果你想更直接，**连 `latest_local_steps` 都不一定需要长期缓存**，可以在 `subevent_result_cb()` 中就地解析 `result->step_data_buf`。

------

## 2. 删除 main 里 RAS 订阅流程

当前你在 `main()` 中做了这些：

```c
err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
err = bt_ras_rreq_on_demand_rd_subscribe(connection);
err = bt_ras_rreq_cp_subscribe(connection);
```

如果你只要本地 IQ，这些都可以去掉。

严格来说，连 discovery 后的：

```c
err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
```

也不再必要，除非你还想保留 RAS 服务发现做别的用途。

------

## 3. `discovery_completed_cb()` 里不再需要 RAS handle 分配

当前：

```c
err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
if (err) {
  LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
}
```

如果你只要本地 IQ，可以删掉。
 甚至如果你的连接目标不再依赖 Ranging Service，你连 `bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, ...)` 这一步都可以重新考虑。

不过这里要注意：

### 如果你的对端依然必须是支持 CS 的 reflector

你可能仍然需要正常连接这个设备，但**不需要 RAS 服务发现**。
 也就是说你可以：

- 不做 GATT discovery
- 或至少不做 RAS service discovery

因为本地 IQ 来自 HCI 的 CS subevent result，不来自 GATT。

------

## 4. 重写 `subevent_result_cb()`

这是最关键的。

你当前的 `subevent_result_cb()` 是为“等待 peer 数据后做距离计算”服务的，所以有很多对齐与丢弃逻辑：

```c
if (dropped_ranging_counter == result->header.procedure_counter) ...
if (most_recent_local_ranging_counter != ...) ...
k_sem_take(&sem_local_steps, K_NO_WAIT) ...
...
net_buf_simple_add_mem(&latest_local_steps, step_data, len);
...
if (procedure aborted) ...
```

这些逻辑大部分都与“要和 peer data 配对”有关。

------

## 新的 `subevent_result_cb()` 应该做什么

新版本建议做成：

```text
收到 subevent result
  -> 若 aborted / 无 step_data_buf，则返回
  -> 遍历 result->step_data_buf 内每个 step
  -> 如果 step.mode 是 mode 2 或 mode 3
      -> 解析 step.data
      -> 拿到 tone_info[]
      -> 对每个 antenna path:
           如果 quality_indicator == HIGH
              -> parse_pct()
              -> 得到 local I/Q
              -> 保存或打印
```

------

# 五、你需要一个“本地 step data 解析函数”

你现在现成有：

- `bt_le_cs_step_data_parse()` in `cs.c`
- `bt_le_cs_parse_pct()` in `cs.c`

这两个逻辑就够用了。

你可以在应用层自己实现一个简化版，比如：

## 示例思路

```c
static void parse_local_iq_from_subevent(struct bt_conn_le_cs_subevent_result *result)
{
    struct net_buf_simple step_buf_copy;
    uint8_t temp_buf[LOCAL_PROCEDURE_MEM];

    if (!result->step_data_buf || result->step_data_buf->len == 0) {
        return;
    }

    if (result->step_data_buf->len > sizeof(temp_buf)) {
        LOG_ERR("step data too large");
        return;
    }

    memcpy(temp_buf, result->step_data_buf->data, result->step_data_buf->len);
    net_buf_simple_init_with_data(&step_buf_copy, temp_buf, result->step_data_buf->len);

    bt_le_cs_step_data_parse(&step_buf_copy, local_step_cb, NULL);
}
```

这里之所以复制一份，是因为 `bt_le_cs_step_data_parse()` 会 `pull` 消耗 buffer。

------

## `local_step_cb()` 里做什么

伪代码：

```c
static bool local_step_cb(struct bt_le_cs_subevent_step *step, void *user_data)
{
    if (step->mode == BT_CONN_LE_CS_MAIN_MODE_2) {
        parse_mode2_local_iq(step);
    } else if (step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
        parse_mode3_local_iq(step);
    }

    return true;
}
```

------

## mode 2 / mode 3 的处理

和 `cs_de.c` 类似，但你只取 local side：

```c
static void parse_mode2_local_iq(struct bt_le_cs_subevent_step *step)
{
    struct bt_hci_le_cs_step_data_mode_2 *data =
        (struct bt_hci_le_cs_step_data_mode_2 *)step->data;

    for (uint8_t tone_index = 0; tone_index < N_AP; tone_index++) {
        ...
        struct bt_le_cs_iq_sample iq =
            bt_le_cs_parse_pct(data->tone_info[path].phase_correction_term);

        ...
    }
}
```

但这里有个关键点：

------

# 六、你仍然需要 `n_ap` 和 antenna path 映射

在 `cs_de.c` 中，PCT 提取不是直接 `tone_info[0..]` 就完，而是通过：

```c
int antenna_path = bt_le_cs_get_antenna_path(p_report->n_ap,
                                             antenna_permutation_index, tone_index);
```

也就是：

- `tone_info` 的排列和 `antenna path` 的逻辑有关
- 不能简单把数组下标直接当最终 AP

所以如果你要正确提取本地 IQ，仍然建议保留这个映射逻辑：

1. 知道当前 step 的 `antenna_permutation_index`
2. 知道系统当前 `n_ap`
3. 用 `bt_le_cs_get_antenna_path()` 求真实 AP

------

## 但 `n_ap` 从哪来？

以前 `n_ap` 是从 peer RAS header 里解析的：

```c
p_report->n_ap = ((ranging_header->antenna_paths_mask & BIT(0)) + ...)
```

现在没有 peer RAS header 了，这里需要新来源。

### 有几种办法：

#### 办法 A：你自己已知配置

如果你的实验固定就是单天线或固定天线路径数，比如 1 或 2，那么可以直接写死：

```c
static uint8_t local_n_ap = 1; // or 2, 3, 4
```

#### 办法 B：从 capability / config 推出

如果 controller 或 config 完全确定可用 antenna path 数，也可以在初始化后保存。

#### 办法 C：先不做 antenna permutation，还原最原始 tone_info 下标

如果你暂时只是想看 PCT->IQ，可以先直接打印 `tone_info[idx]`，以后再补 AP 映射。

------

# 七、如果你只是想“拿到本地 IQ”，最小改法是什么？

最小改法是：

## 在 `subevent_result_cb()` 里直接解析，不再缓存到 `latest_local_steps`

也就是说，把现在这段：

```c
if (result->step_data_buf) {
    ...
    net_buf_simple_add_mem(&latest_local_steps, step_data, len);
}
```

替换成：

```c
if (result->step_data_buf) {
    parse_local_iq_from_subevent(result);
}
```

这样你不需要：

- `latest_local_steps`
- `latest_peer_steps`
- `sem_local_steps`
- RAS callback 链

------

# 八、推荐的新流程图

```text
连接成功
  -> 配置 CS
  -> enable CS procedures
  -> controller 周期性产生 subevent result
  -> subevent_result_cb()
      -> 遍历 step_data_buf
      -> 找到 mode2/mode3 step
      -> 读取 tone_info[].phase_correction_term
      -> bt_le_cs_parse_pct()
      -> 得到 local IQ
      -> 打印/缓存/写flash
  -> 下一次 subevent
```

------

# 九、示意代码结构

下面给你一个“应用层改造骨架”。

## 1. 本地 PCT 解析函数

```c
static struct bt_le_cs_iq_sample parse_pct_local(const uint8_t pct[3])
{
    uint32_t pct_u32 = pct[0] | (pct[1] << 8) | (pct[2] << 16);

    uint16_t i_u16 = pct_u32 & 0x0FFF;
    uint16_t q_u16 = (pct_u32 >> 12) & 0x0FFF;

    int16_t i = (i_u16 ^ BIT(11)) - BIT(11);
    int16_t q = (q_u16 ^ BIT(11)) - BIT(11);

    return (struct bt_le_cs_iq_sample){ .i = i, .q = q };
}
```

------

## 2. step 回调

```c
static uint8_t local_n_ap = 1;  // 根据你的实际配置修改

static bool local_step_iq_cb(struct bt_le_cs_subevent_step *step, void *user_data)
{
    if (step->mode == BT_CONN_LE_CS_MAIN_MODE_2) {
        struct bt_hci_le_cs_step_data_mode_2 *data =
            (struct bt_hci_le_cs_step_data_mode_2 *)step->data;

        for (uint8_t tone_index = 0; tone_index < local_n_ap; tone_index++) {
            int ap = bt_le_cs_get_antenna_path(local_n_ap,
                                               data->antenna_permutation_index,
                                               tone_index);
            if (ap < 0) {
                continue;
            }

            if (data->tone_info[ap].quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
                continue;
            }

            struct bt_le_cs_iq_sample iq =
                parse_pct_local(data->tone_info[ap].phase_correction_term);

            LOG_INF("CH=%u AP=%d MODE2 I=%d Q=%d",
                    step->channel, ap, iq.i, iq.q);
        }
    } else if (step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
        struct bt_hci_le_cs_step_data_mode_3 *data =
            (struct bt_hci_le_cs_step_data_mode_3 *)step->data;

        for (uint8_t tone_index = 0; tone_index < local_n_ap; tone_index++) {
            int ap = bt_le_cs_get_antenna_path(local_n_ap,
                                               data->antenna_permutation_index,
                                               tone_index);
            if (ap < 0) {
                continue;
            }

            if (data->tone_info[ap].quality_indicator != BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
                continue;
            }

            struct bt_le_cs_iq_sample iq =
                parse_pct_local(data->tone_info[ap].phase_correction_term);

            LOG_INF("CH=%u AP=%d MODE3 I=%d Q=%d",
                    step->channel, ap, iq.i, iq.q);
        }
    }

    return true;
}
```

------

## 3. 从 subevent result 解析本地 IQ

```c
static void parse_local_iq_from_subevent(struct bt_conn_le_cs_subevent_result *result)
{
    if (!result->step_data_buf || result->step_data_buf->len == 0) {
        return;
    }

    uint8_t tmp[LOCAL_PROCEDURE_MEM];
    struct net_buf_simple buf;

    if (result->step_data_buf->len > sizeof(tmp)) {
        LOG_ERR("step_data too large");
        return;
    }

    memcpy(tmp, result->step_data_buf->data, result->step_data_buf->len);
    net_buf_simple_init_with_data(&buf, tmp, result->step_data_buf->len);

    bt_le_cs_step_data_parse(&buf, local_step_iq_cb, NULL);
}
```

------

## 4. 简化后的 `subevent_result_cb()`

```c
static void subevent_result_cb(struct bt_conn *conn,
                               struct bt_conn_le_cs_subevent_result *result)
{
    ARG_UNUSED(conn);

    if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
        LOG_WRN("Subevent aborted");
        return;
    }

    if (!result->step_data_buf || result->step_data_buf->len == 0) {
        return;
    }

    parse_local_iq_from_subevent(result);
}
```

------

# 十、main 里要删哪些调用

建议删掉这些：

```c
err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
err = bt_ras_rreq_on_demand_rd_subscribe(connection);
err = bt_ras_rreq_cp_subscribe(connection);
```

如果完全不需要 RAS，也删：

```c
bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
```

前提是后续没有地方再依赖 RAS discovery 结果。

------

# 十一、还需不需要连接对端？

**需要。**

因为你还是在跑 BLE CS procedure，initiator 必须和 reflector 建立连接并完成：

- security
- CS capability exchange
- create config
- security enable
- procedure parameter set
- procedure enable

只是你不需要再通过 GATT/RAS 去把对端测量结果取回来。

------

# 十二、几个实际注意点

## 1. 你拿到的是 PCT 解析后的 IQ，不是原始 RF ADC IQ

这点你已经理解得差不多了。这里再次强调：

- 这是 controller 输出的 phase correction term
- host 侧只是把它转成复数形式的 `I/Q`
- 它适合作为算法输入或观测值，但不是底层原始采样流

------

## 2. `step_data_parse()` 会消耗 buffer

所以别直接对 `result->step_data_buf` 原地 pull，除非你确认后面再也不用它。
 复制一份临时 buffer 是最稳妥的。

------

## 3. mode 选择

你当前配置：

```c
.main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
```

这意味着主要是 mode 2。
 如果你将来改成 mode 3，也可以沿用同一套解析。

------

## 4. 输出频率可能很高

如果你“不断重复执行 CS subevent，然后只取本地IQ”，日志量会非常大。建议：

- 不要每个 IQ 都 `LOG_INF`
- 用 ring buffer / flash / UART binary output
- 或按 channel / step 抽样输出

否则日志会成为瓶颈。

------

# 十三、最推荐的修改策略

我建议你分两步做，而不是一次大改：

## 第一步：保留现有 CS 配置，只在 `subevent_result_cb()` 中额外打印本地 IQ

这样最安全。
 你先确认：

- 能稳定拿到本地 IQ
- PCT 解析正确
- channel / AP 映射正确

## 第二步：再移除 RAS 相关流程

确认本地 IQ 路径稳定后，再删掉：

- RAS subscribe
- peer steps
- distance calculation

这样调试成本最低。

------

# 十四、简化后的新旧对比

## 旧版

```text
subevent_result_cb
  -> 保存 local steps
ranging_data_ready_cb
  -> 发 GET_RD
ranging_data_get_complete_cb
  -> local + peer
  -> cs_de_populate_report
  -> cs_de_calc
  -> distance
```

## 新版

```text
subevent_result_cb
  -> 直接解析 local step PCT
  -> PCT -> local IQ
  -> 输出/保存 local IQ
```

------

如果你愿意，我下一条可以直接帮你做两件事之一：

1. **把你现在的 `main.c` 按“只取本地IQ”改成一个可编译的精简版本**
    或者
2. **只给你一组最小 patch：具体删哪些函数、改哪些回调、加哪些新函数**

如果你想快速上手，我建议我直接给你第 2 种：**面向你这份 `main.c` 的最小改动版 patch 思路**。