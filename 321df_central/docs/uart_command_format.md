## UART 命令格式说明（Command Format Spec）

### 1. 传输与分隔规则

- **一条命令占一行**，以 `\n` 作为结束符；兼容 `CRLF`，接收端会忽略 `\r`。
- 命令帧为 **CSV 逗号分隔**，格式为：

```text
$CMD,<cmd>[,<k>=<v>[,<k>=<v>...]]\n
```

- 解析逻辑（来自 `uart_cmd.c`）：
  - 只处理以 `"$CMD,"` 开头的行，其它行直接忽略。
  - 去掉前缀 `$CMD,` 后按 `,` 分割成 tokens：
    - `tokens[0]` 为命令名 `<cmd>`
    - `tokens[1..]` 为参数字符串数组 `argv[]`，每个参数形如 `k=v`
  - 参数值不做自动解码；空格/Tab 会在每个 token 开始处被跳过。

### 2. 输出格式

同一串口输出（走 `printk`），统一为三类帧：

- 成功应答：

```text
$OK,<cmd>[,<msg>]\r\n
```

- 错误应答：

```text
$ERR,<cmd>,<code>,<msg>\r\n
```

- 事件上报：

```text
$EVT,<topic>[,<msg>]\r\n
```

### 3. 参数编码规则（当前实现）

- 参数以 `k=v` 形式出现，且彼此用逗号分隔。
- `channels` 参数内部使用 `|` 分隔多个信道号，如：`channels=3|10|25`。
- `interval_ms` 必须能精确换算为 BLE 连接间隔单位（1.25 ms）：
  - `units = interval_ms * 100 / 125`
  - 必须整除，否则报参数非法
  - 范围要求满足 BLE 规范：7.5ms..4s（units: 6..3200）

---

## 命令表（Command Table）

> 命令名大小写敏感（代码中用 `strcmp`），请按下表使用大写。

### 1) `PING`

- **用途**：连通性测试
- **请求**：

```text
$CMD,PING
```

- **成功回复**：

```text
$OK,PING,pong
```

- **示例**

发送：

```text
$CMD,PING\n
```

收到：

```text
$OK,PING,pong\r\n
```

---

### 2) `BLE_SCAN`

- **用途**：控制扫描
- **参数**
  - `action=start|stop`（必填）

#### 2.1 开始扫描

- 请求：

```text
$CMD,BLE_SCAN,action=start
```

- 可能输出：
  - 成功应答：`$OK,BLE_SCAN,action=start`
  - 事件：`$EVT,BLE,scan_started`
  - 若失败：`$EVT,BLE,scan_start_failed,err=<err>` + `$OK` 仍可能先后出现（取决于调用路径；当前实现 `start_scan()` 内会发 EVT，命令处理会发 OK）

#### 2.2 停止扫描

- 请求：

```text
$CMD,BLE_SCAN,action=stop
```

- 成功回复：

```text
$OK,BLE_SCAN,action=stop
$EVT,BLE,scan_stopped
```

- 失败回复：

```text
$ERR,BLE_SCAN,2,scan_stop_failed,err=<err>
```

---

### 3) `BLE_CONN`

- **用途**：连接控制（当前只实现断开）
- **参数**
  - `action=disconnect`（必填）

#### 3.1 断开连接

- 请求：

```text
$CMD,BLE_CONN,action=disconnect
```

- 成功回复：

```text
$OK,BLE_CONN,action=disconnect
```

- 失败情况：
  - 未连接：

    ```text
    $ERR,BLE_CONN,2,NOT_CONNECTED
    ```

  - 断开调用失败：

    ```text
    $ERR,BLE_CONN,3,disconnect_failed,err=<err>
    ```

---

### 4) `DF_START`

- **用途**：配置 DF（Direction Finding）参数，并在需要时启动扫描/重新使能 CTE
- **参数（全可选）**
  - `channels=<list>`：自定义数据信道列表，`|` 分隔；每个信道 `0..36`
    - 例：`channels=3|10|25`
  - `interval_ms=<n>`：连接间隔（毫秒），必须可换算成 1.25ms units 且满足范围
    - 例：`interval_ms=25`（25ms 对应 units=20）
  - `cte_len=<n>`：CTE 长度（单位 8us），`0..255`
    - 例：`cte_len=2`
  - `cte_type=aod1|aod2|aoa`
    - `aoa` 仅在编译启用 `CONFIG_BT_DF_CTE_RX_AOA` 时可用

- **成功回复**（回显当前配置）：

```text
$OK,DF_START,channels_cnt=<n>,interval_units=<u>,cte_len=<l>,cte_type=<t>
```

- **行为说明（来自 `main.c`）**
  - 如果当前未连接：会调用 `start_scan()`，等待连接后在 `connected()` 回调里应用信道图并 `enable_cte_request()`
  - 如果当前已连接：会立即应用信道图并 `enable_cte_request()`

- **完整示例 1：只启动，使用默认参数**

```text
$CMD,DF_START
```

可能输出（示例）：

```text
$OK,DF_START,channels_cnt=1,interval_units=20,cte_len=2,cte_type=2
$EVT,BLE,scan_started
```

- **完整示例 2：配置多个信道 + 25ms 间隔 + AOD2**

发送：

```text
$CMD,DF_START,channels=3|10|25,interval_ms=25,cte_len=20,cte_type=aod2
```

收到：

```text
$OK,DF_START,channels_cnt=3,interval_units=20,cte_len=20,cte_type=2
```

连接成功后还可能出现：

```text
$EVT,BLE,connected,addr=xx:xx:xx:xx:xx:xx
$EVT,DF,cte_enabled,len=20,type=2,intv=1
```

- **参数错误示例**

```text
$CMD,DF_START,interval_ms=23
```

（23ms 不能整除换算到 1.25ms units）：

```text
$ERR,DF_START,2,INVALID_PARAM:interval_ms
```

---

### 5) `DF_CONFIG`

- **用途**：在DF运行时动态修改参数（不重新enable CTE REQ，不中断CTE接收）
- **前置条件**
  - 必须已建立BLE连接
  - CTE必须已启用（需先使用`DF_START`）
- **参数（全可选）**
  - `channels=<list>`：自定义数据信道列表，`|` 分隔；每个信道 `0..36`
    - **立即生效**：直接更新信道映射，无需重新enable CTE REQ
    - 例：`channels=7` 或 `channels=3|10|25`
  - `interval_ms=<n>`：连接间隔（毫秒），必须可换算成 1.25ms units 且满足范围
    - **下次连接生效**：更新连接参数值，需要重新连接才能生效
    - 例：`interval_ms=25`
  - `cte_len=<n>`：CTE 长度（单位 8us），`0..255`
    - **需要DF_START生效**：只更新参数值，需要`DF_START`才能生效
    - 例：`cte_len=4`
  - `cte_type=aod1|aod2|aoa`
    - **需要DF_START生效**：只更新参数值，需要`DF_START`才能生效
    - `aoa` 仅在编译启用 `CONFIG_BT_DF_CTE_RX_AOA` 时可用

- **成功回复**（回显当前配置）：

```text
$OK,DF_CONFIG,channels_cnt=<n>,interval_units=<u>,cte_len=<l>,cte_type=<t>
```

- **错误情况**
  - 未连接：
    ```text
    $ERR,DF_CONFIG,1,NOT_CONNECTED
    ```
  - CTE未启用：
    ```text
    $ERR,DF_CONFIG,2,CTE_NOT_ENABLED,use_DF_START_first
    ```
  - 参数无效（错误码3-6）：
    ```text
    $ERR,DF_CONFIG,3,INVALID_PARAM:channels
    $ERR,DF_CONFIG,4,INVALID_PARAM:interval_ms
    $ERR,DF_CONFIG,5,INVALID_PARAM:cte_len
    $ERR,DF_CONFIG,6,INVALID_PARAM:cte_type
    ```
  - 信道映射更新失败：
    ```text
    $ERR,DF_CONFIG,7,channel_map_update_failed,err=<err>
    ```

- **行为说明**
  - 此命令**不会重新enable CTE REQ**，因此不会中断CTE数据接收
  - `channels`参数会立即生效，直接更新信道映射
  - `interval_ms`参数只更新值，需要重新连接才能生效
  - `cte_len`和`cte_type`参数只更新值，需要`DF_START`才能生效（会重新enable CTE REQ）

- **完整示例 1：动态切换信道（不中断）**

发送：
```text
$CMD,DF_CONFIG,channels=7
```

收到：
```text
$OK,DF_CONFIG,channels_cnt=1,interval_units=20,cte_len=2,cte_type=1
```

CTE数据会继续接收，但信道从之前的信道切换到信道7。

- **完整示例 2：修改多个参数**

发送：
```text
$CMD,DF_CONFIG,channels=10|25,interval_ms=30
```

收到：
```text
$OK,DF_CONFIG,channels_cnt=2,interval_units=24,cte_len=2,cte_type=1
Note: interval_ms change will take effect on next connection
```

- **完整示例 3：修改cte_len（需要DF_START生效）**

发送：
```text
$CMD,DF_CONFIG,cte_len=4
```

收到：
```text
$OK,DF_CONFIG,channels_cnt=1,interval_units=20,cte_len=4,cte_type=1
Note: cte_len/cte_type changes require DF_START to take effect
```

此时参数已更新，但需要执行`DF_START`才能生效。

- **与DF_START的区别**

| 特性 | DF_START | DF_CONFIG |
|------|----------|-----------|
| 用途 | 启动/重新启动DF功能 | 在DF运行时动态修改参数 |
| CTE REQ | 会重新enable（可能短暂中断） | 不重新enable（不中断） |
| channels | 立即生效 | 立即生效 |
| interval_ms | 立即生效（如果已连接） | 下次连接生效 |
| cte_len/cte_type | 立即生效 | 需要DF_START才能生效 |
| 前置条件 | 无（可启动扫描） | 必须已连接且CTE已启用 |

---

### 6) `DF_STOP`

- **用途**：停止/禁用 CTE request/rx（若已连接）
- **请求**：

```text
$CMD,DF_STOP
```

- **回复**：

```text
$OK,DF_STOP,done
```

---

### 7) 未知命令

- 请求：

```text
$CMD,FOO
```

- 回复：

```text
$ERR,FOO,99,UNKNOWN_CMD
```

---

## 完整交互示例（从启动到 DF）

```text
(boot)
$EVT,UART,cmd_thread_started

(host)
$CMD,PING

(device)
$OK,PING,pong

(host)
$CMD,DF_START,channels=3|10|25,interval_ms=25,cte_len=20,cte_type=aod2

(device)
$OK,DF_START,channels_cnt=3,interval_units=20,cte_len=20,cte_type=2
$EVT,BLE,scan_started

(device after connect)
$EVT,BLE,connected,addr=xx:xx:xx:xx:xx:xx
$EVT,DF,cte_enabled,len=20,type=2,intv=1
$DF,1,<chan>,<seq>,<ts_ms>,<p_avg>
...

(host - 在DF运行时动态切换信道，不中断)
$CMD,DF_CONFIG,channels=7

(device)
$OK,DF_CONFIG,channels_cnt=1,interval_units=20,cte_len=20,cte_type=2
$DF,1,7,<seq>,<ts_ms>,<p_avg>  (信道已切换到7，CTE数据继续接收)
...
```
