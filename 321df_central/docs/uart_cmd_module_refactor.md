# 串口命令模块重构说明

## 重构目标

将串口命令处理系统模块化，提高代码的可移植性和复用性。将通用的参数解析函数移到 `uart_cmd.c` 模块中，使该模块成为一个独立的、可复用的串口命令框架。

## 重构内容

### 1. 移到 `uart_cmd.c` 的通用函数

以下函数已从 `main.c` 移到 `uart_cmd.c`，并在 `uart_cmd.h` 中导出：

| 原函数名 | 新函数名 | 功能 |
|---------|---------|------|
| `kv_find()` | `uart_cmd_kv_find()` | 查找key=value格式的参数 |
| `parse_u32()` | `uart_cmd_parse_u32()` | 解析32位无符号整数 |
| - | `uart_cmd_parse_u16()` | 解析16位无符号整数（新增） |
| - | `uart_cmd_parse_u8()` | 解析8位无符号整数（新增） |
| - | `uart_cmd_parse_i32()` | 解析32位有符号整数（新增） |

**为什么这些函数适合移到 `uart_cmd.c`：**
- 这些函数是通用的参数解析工具，不依赖具体的业务逻辑
- 可以在任何使用 `uart_cmd` 模块的项目中复用
- 函数命名统一，都带有 `uart_cmd_` 前缀，避免命名冲突

### 2. 保留在 `main.c` 的业务相关函数

以下函数保留在 `main.c` 中，因为它们与具体的业务逻辑（BLE DF功能）相关：

| 函数名 | 功能 | 保留原因 |
|--------|------|---------|
| `parse_channels()` | 解析管道符分隔的信道列表 | 特定于BLE DF功能，解析格式"3\|10\|25" |
| `interval_ms_to_units()` | 毫秒转BLE连接间隔单位 | 特定于BLE协议，将毫秒转换为1.25ms单位 |
| `on_cmd()` | 命令处理主函数 | 包含所有具体的业务逻辑（BLE扫描、连接、DF等） |

**为什么这些函数不适合移到 `uart_cmd.c`：**
- `parse_channels()` 和 `interval_ms_to_units()` 是特定于BLE DF功能的，其他项目不需要
- `on_cmd()` 包含了所有业务逻辑，每个项目的命令处理逻辑都不同
- 保持 `uart_cmd` 模块的通用性，不包含业务相关代码

## 模块化后的架构

### uart_cmd 模块（通用框架）

```
uart_cmd.h / uart_cmd.c
├── 初始化函数
│   └── uart_cmd_init()
├── 响应输出函数
│   ├── uart_cmd_send_ok()
│   ├── uart_cmd_send_err()
│   └── uart_cmd_send_evt()
├── 参数解析工具函数（新增）
│   ├── uart_cmd_kv_find()
│   ├── uart_cmd_parse_u32()
│   ├── uart_cmd_parse_u16()
│   ├── uart_cmd_parse_u8()
│   └── uart_cmd_parse_i32()
└── 内部实现
    ├── UART中断处理
    ├── 命令解析线程
    └── 行解析和token分割
```

### main.c（业务逻辑）

```
main.c
├── 业务相关的解析函数
│   ├── parse_channels()      (BLE DF特定)
│   └── interval_ms_to_units() (BLE协议特定)
├── 命令处理函数
│   └── on_cmd()               (所有业务逻辑)
└── BLE相关函数
    ├── start_scan()
    ├── enable_cte_request()
    └── ...
```

## 使用示例

### 在命令处理函数中使用新的API

**重构前：**
```c
static void on_cmd(const char *cmd, int argc, const char *argv[])
{
    const char *act = kv_find(argc, argv, "action");
    uint32_t value = 0;
    parse_u32(some_str, &value);
    // ...
}
```

**重构后：**
```c
static void on_cmd(const char *cmd, int argc, const char *argv[])
{
    const char *act = uart_cmd_kv_find(argc, argv, "action");
    uint32_t value = 0;
    uart_cmd_parse_u32(some_str, &value);
    // ...
}
```

### 在其他项目中使用 uart_cmd 模块

现在 `uart_cmd` 模块可以作为一个独立的模块被其他项目复用：

```c
// 1. 包含头文件
#include "uart_cmd.h"

// 2. 实现命令处理函数
static void my_cmd_handler(const char *cmd, int argc, const char *argv[])
{
    if (!strcmp(cmd, "MY_CMD")) {
        // 使用通用工具函数解析参数
        const char *param = uart_cmd_kv_find(argc, argv, "param");
        uint32_t value = 0;
        if (uart_cmd_parse_u32(param, &value) == 0) {
            // 处理命令...
            uart_cmd_send_ok("MY_CMD", "value=%u", value);
        } else {
            uart_cmd_send_err("MY_CMD", 1, "INVALID_PARAM");
        }
    }
}

// 3. 初始化
int main(void)
{
    uart_cmd_init(my_cmd_handler);
    // ...
}
```

## 优势

1. **可移植性**：`uart_cmd` 模块现在是一个通用的串口命令框架，可以在任何Zephyr/NCS项目中使用
2. **模块化**：清晰的模块边界，通用功能与业务逻辑分离
3. **可维护性**：业务相关的代码集中在 `main.c`，框架代码集中在 `uart_cmd.c`
4. **可扩展性**：新增了 `parse_u16`、`parse_u8`、`parse_i32` 等函数，提供更丰富的解析能力
5. **命名规范**：所有导出的函数都带有 `uart_cmd_` 前缀，避免命名冲突

## 注意事项

1. **函数命名**：所有从 `uart_cmd` 模块导出的函数都使用 `uart_cmd_` 前缀
2. **头文件依赖**：`uart_cmd.h` 现在包含 `<stdint.h>`，以支持类型定义
3. **向后兼容**：如果其他代码使用了旧的函数名，需要更新为新的函数名
4. **业务逻辑**：业务相关的解析函数（如 `parse_channels`）仍然保留在 `main.c` 中

## 迁移指南

如果要将此模块用于其他项目：

1. **复制文件**：
   - `src/uart_cmd.h`
   - `src/uart_cmd.c`

2. **实现命令处理函数**：
   - 创建自己的 `on_cmd()` 函数
   - 使用 `uart_cmd_kv_find()` 和 `uart_cmd_parse_*()` 函数解析参数

3. **初始化**：
   - 在 `main()` 中调用 `uart_cmd_init(your_handler)`

4. **可选**：
   - 如果需要特定的解析函数（如 `parse_channels`），在业务代码中实现

