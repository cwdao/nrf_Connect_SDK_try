# 串口命令处理系统工作流程详解

## 概述

本系统实现了一个基于串口的命令接收、解析和执行机制。上位机通过串口发送命令，单片机接收后解析并执行相应的操作，然后通过串口返回执行结果。

## 系统架构

系统分为三个主要模块：

1. **串口接收模块** (`uart_cmd.c/h`)：负责串口数据的接收、解析和分发
   - 包含通用的参数解析工具函数（`uart_cmd_kv_find()`, `uart_cmd_parse_u32()` 等）
2. **命令处理模块** (`main.c` 中的 `on_cmd`)：负责具体命令的执行
   - 包含业务相关的解析函数（如 `parse_channels()`, `interval_ms_to_units()`）
3. **响应输出模块** (`uart_cmd.c` 中的输出函数)：负责格式化输出响应

## 详细工作流程

### 1. 系统初始化阶段

```
main() 
  └─> uart_cmd_init(on_cmd)
      ├─> 获取UART设备（从设备树获取zephyr_console）
      ├─> 初始化ring buffer（512字节，用于存储接收到的原始字节流）
      ├─> 初始化信号量（用于线程间通信）
      ├─> 设置UART中断回调函数（uart_isr_cb）
      ├─> 启用UART接收中断
      └─> 创建并启动命令解析线程（uart_cmd_thread）
```

**关键数据结构：**
- `rx_ring`: 环形缓冲区，存储从UART接收到的原始字节
- `rx_sem`: 信号量，用于通知解析线程有新数据到达
- `g_handler`: 命令处理回调函数指针

### 2. 数据接收阶段（中断上下文）

```
UART硬件接收数据
  └─> 触发UART接收中断
      └─> uart_isr_cb() [中断上下文]
          ├─> 从UART FIFO读取数据（最多64字节）
          ├─> 将数据放入ring buffer
          └─> 发送信号量唤醒解析线程
```

**特点：**
- 在中断上下文中执行，必须快速完成
- 如果ring buffer已满，新数据会被丢弃（但仍会唤醒线程）
- 使用ring buffer避免数据丢失，提供缓冲能力

### 3. 命令解析阶段（线程上下文）

```
uart_cmd_thread() [后台线程]
  └─> 等待信号量（阻塞，直到有数据到达）
      └─> 从ring buffer逐个读取字节
          ├─> 忽略'\r'字符（回车符）
          ├─> 遇到'\n'时：
          │   ├─> 将积累的字符作为一行命令
          │   ├─> 添加'\0'结束符
          │   └─> 调用handle_line()处理
          └─> 如果行长度超过192字节，丢弃并报错
```

**行组装逻辑：**
- 以`\n`（换行符）作为行结束标志
- 忽略`\r`（回车符），兼容Windows的`\r\n`格式
- 最大行长度：192字节（`CMD_MAX_LINE`）

### 4. 命令处理阶段

```
handle_line(line)
  ├─> 检查是否以"$CMD,"开头
  ├─> 提取命令体（去掉"$CMD,"前缀）
  ├─> 按逗号分割成多个token（split_csv）
  │   └─> 第一个token = 命令名
  │   └─> 其余token = 参数（可能是"key=value"格式）
  └─> 调用g_handler(cmd, argc, argv)
      └─> on_cmd() [main.c中实现]
          ├─> 根据命令名分发到不同的处理逻辑
          ├─> 解析参数（使用uart_cmd_kv_find()查找key=value参数）
          ├─> 使用uart_cmd_parse_*()函数解析数值参数
          ├─> 执行相应操作
          └─> 发送响应（OK/ERR/EVT）
```

**命令格式：**
```
$CMD,<命令名>,<参数1>,<参数2>,...
```

**示例命令：**
- `$CMD,PING` - 心跳测试
- `$CMD,BLE_SCAN,action=start` - 启动BLE扫描
- `$CMD,DF_START,channels=3|10|25,interval_ms=25,cte_len=2` - 启动DF功能

### 5. 响应输出阶段

系统提供三种响应格式：

**成功响应：**
```
$OK,<命令名>,<附加信息>\r\n
示例：$OK,PING,pong\r\n
```

**错误响应：**
```
$ERR,<命令名>,<错误码>,<错误描述>\r\n
示例：$ERR,BLE_SCAN,1,MISSING_PARAM:action\r\n
```

**事件通知：**
```
$EVT,<主题>,<事件描述>\r\n
示例：$EVT,BLE,connected,addr=AA:BB:CC:DD:EE:FF\r\n
```

所有响应都通过`printk()`输出到串口（与接收使用同一个UART）。

## 数据流图

```
上位机
  │
  │ 发送: $CMD,PING\r\n
  ▼
UART硬件FIFO
  │
  │ 中断读取
  ▼
ring buffer (512字节)
  │
  │ 信号量通知
  ▼
uart_cmd_thread (解析线程)
  │
  │ 按行组装
  ▼
handle_line()
  │
  │ 分割token
  ▼
on_cmd() (命令处理)
  │
  │ 执行操作
  ▼
uart_cmd_send_ok/err/evt()
  │
  │ printk输出
  ▼
UART硬件
  │
  │ 响应: $OK,PING,pong\r\n
  ▼
上位机
```

## 关键函数说明

### uart_cmd.c 中的函数

| 函数名 | 功能 | 调用上下文 |
|--------|------|-----------|
| `uart_cmd_init()` | 初始化串口命令系统 | main函数（初始化时） |
| `uart_isr_cb()` | UART中断回调，接收数据 | 中断上下文 |
| `uart_cmd_thread()` | 命令解析线程主函数 | 独立线程 |
| `handle_line()` | 处理一行命令 | 解析线程 |
| `split_csv()` | 按逗号分割字符串 | 解析线程 |
| `uart_cmd_send_ok()` | 发送成功响应 | 任意上下文 |
| `uart_cmd_send_err()` | 发送错误响应 | 任意上下文 |
| `uart_cmd_send_evt()` | 发送事件通知 | 任意上下文 |
| `uart_cmd_kv_find()` | 查找key=value参数 | 命令处理函数中 |
| `uart_cmd_parse_u32()` | 解析32位无符号整数 | 命令处理函数中 |
| `uart_cmd_parse_u16()` | 解析16位无符号整数 | 命令处理函数中 |
| `uart_cmd_parse_u8()` | 解析8位无符号整数 | 命令处理函数中 |
| `uart_cmd_parse_i32()` | 解析32位有符号整数 | 命令处理函数中 |

### main.c 中的函数

| 函数名 | 功能 | 说明 |
|--------|------|------|
| `on_cmd()` | 命令处理主函数 | 由uart_cmd模块调用 |
| `parse_channels()` | 解析信道列表 | BLE DF特定，业务相关 |
| `interval_ms_to_units()` | 毫秒转BLE单位 | BLE协议特定，业务相关 |

## 支持的命令

### PING
- **格式**: `$CMD,PING`
- **功能**: 心跳测试，验证串口通信
- **响应**: `$OK,PING,pong`

### BLE_SCAN
- **格式**: `$CMD,BLE_SCAN,action=<start|stop>`
- **功能**: 控制BLE扫描
- **示例**: `$CMD,BLE_SCAN,action=start`
- **响应**: `$OK,BLE_SCAN,action=start` 或错误响应

### BLE_CONN
- **格式**: `$CMD,BLE_CONN,action=disconnect`
- **功能**: 断开BLE连接
- **响应**: `$OK,BLE_CONN,action=disconnect` 或错误响应

### DF_START
- **格式**: `$CMD,DF_START[,channels=<列表>][,interval_ms=<值>][,cte_len=<值>][,cte_type=<类型>]`
- **功能**: 启动Direction Finding功能
- **参数说明**:
  - `channels`: 信道列表，格式"3|10|25"（管道符分隔）
  - `interval_ms`: 连接间隔（毫秒，必须是1.25ms的整数倍）
  - `cte_len`: CTE长度（8us单位）
  - `cte_type`: CTE类型（"aod1"/"aod2"/"aoa"）
- **示例**: `$CMD,DF_START,channels=3|10|25,interval_ms=25,cte_len=2,cte_type=aod1`

### DF_STOP
- **格式**: `$CMD,DF_STOP`
- **功能**: 停止Direction Finding功能
- **响应**: `$OK,DF_STOP,done`

## 错误处理

系统在以下情况会发送错误响应：

1. **命令格式错误**: 不以"$CMD,"开头或格式不正确
   - 错误码: 1
   - 响应: `$ERR,CMD,1,BAD_FORMAT`

2. **行长度超限**: 单行命令超过192字节
   - 错误码: 3
   - 响应: `$ERR,CMD,3,LINE_TOO_LONG`

3. **缺少参数**: 命令缺少必需参数
   - 错误码: 1
   - 响应: `$ERR,<命令名>,1,MISSING_PARAM:<参数名>`

4. **参数无效**: 参数格式或值不正确
   - 错误码: 2-4（根据命令不同）
   - 响应: `$ERR,<命令名>,<错误码>,INVALID_PARAM:<参数名>`

5. **未知命令**: 不支持的命令名
   - 错误码: 99
   - 响应: `$ERR,<命令名>,99,UNKNOWN_CMD`

## 线程安全

- **中断上下文** (`uart_isr_cb`): 只操作ring buffer，使用原子操作
- **解析线程** (`uart_cmd_thread`): 从ring buffer读取数据，处理命令
- **命令处理** (`on_cmd`): 可能在任何线程上下文中被调用
- **响应输出**: 使用`printk()`，Zephyr内核保证线程安全

## 性能考虑

1. **ring buffer大小**: 512字节，可缓冲一定量的接收数据
2. **行缓冲区**: 192字节，限制单行命令的最大长度
3. **中断处理**: 快速将数据从UART FIFO转移到ring buffer，避免FIFO溢出
4. **线程优先级**: 解析线程优先级为7，确保及时处理命令

## 调试功能

通过定义 `UART_CMD_DEBUG_LINE` 宏（当前设置为1），可以在解析到每行命令时打印调试信息：
```
[LINE len=15] '$CMD,PING'
```

## 模块化设计

重构后的系统采用模块化设计：

- **uart_cmd 模块**（通用框架）：
  - 串口数据接收和解析
  - 通用的参数解析工具函数
  - 响应输出函数
  - 可在其他项目中复用

- **main.c**（业务逻辑）：
  - 具体的命令处理逻辑
  - 业务特定的解析函数（如BLE DF相关）
  - 与具体应用场景相关

这种设计提高了代码的可移植性和可维护性。更多细节请参考 `uart_cmd_module_refactor.md`。

## 扩展新命令

要添加新命令，只需在 `on_cmd()` 函数中添加新的条件分支：

```c
if (!strcmp(cmd, "NEW_CMD")) {
    // 使用通用工具函数解析参数
    const char *param = uart_cmd_kv_find(argc, argv, "param_name");
    
    // 解析数值参数
    uint32_t value = 0;
    if (uart_cmd_parse_u32(param, &value) != 0) {
        uart_cmd_send_err("NEW_CMD", 1, "INVALID_PARAM");
        return;
    }
    
    // 执行操作
    // ...
    
    // 发送响应
    uart_cmd_send_ok("NEW_CMD", "result");
    return;
}
```

**注意**：重构后，通用的参数解析函数（如 `uart_cmd_kv_find()`, `uart_cmd_parse_u32()` 等）已移到 `uart_cmd` 模块中，可以在任何命令处理函数中使用。业务特定的解析函数（如 `parse_channels()`）仍保留在 `main.c` 中。

