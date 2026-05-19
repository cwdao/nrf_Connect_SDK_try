# PC 端解析 DIP 二进制 UART 帧

本文说明如何在 PC 上从串口接收并解析固件发出的 DIP IQ 二进制帧。协议字段定义见 [DIP_binary_protocol.md](./DIP_binary_protocol.md)。

**验证**：使用 `dip_parse_uart.py` 对接默认二进制固件（`DIP_REPORT_BINARY_OUTPUT=1`、`DIP_BINARY_USE_THREAD=1`）已实测可连续收帧并通过 CRC。

---

## 1. 环境准备

### 1.1 串口参数

与 nRF Connect SDK / DK 默认 console 一致（以板级 `prj.conf` / overlay 为准），常见为：

| 参数 | 典型值 |
|------|--------|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | None |
| 停止位 | 1 |

Windows：COMx；Linux：`/dev/ttyACM0` 等。

### 1.2 Python 依赖

```bash
pip install pyserial
```

---

## 2. 解析流程概览

```
打开串口 → 循环读字节 → 状态机搜 0x55 0xAA
  → 读满固定头 22 字节 → 根据 payload_len 读 IQ + CRC
  → 校验 CRC → 按 bitmap 展开 IQ 字典
```

**注意**：若固件仍输出 `LOG_INF` 文本，串口流中会夹杂 ASCII。采集时请将 `DIP_REPORT_LOG_VERBOSE=0` 并降低日志级别，或后续将 logger 改到 RTT。

---

## 3. 固定头解析（22 字节）

读完 `sync1,sync2,version,type` 后，继续读取直至凑满 22 字节头（含 10 字节 bitmap）：

| 字段 | 解析 |
|------|------|
| `payload_len` | `struct.unpack_from("<H", buf, 4)[0]` |
| `procedure_counter` | `unpack("<H", buf, 6)` |
| `ap` | `buf[8]` |
| `iq_format` | `buf[9]`，当前应为 0 |
| `channel_count` | `buf[10]` |
| `reserved` | `buf[11]` |
| `channel_bitmap` | `buf[12:22]` |

**整帧长度**：

```python
frame_len = 22 + (payload_len - 16) + 2   # 等价于 24 + 4*channel_count
# 或：frame_len = 22 + 4 * channel_count + 2
```

其中 `payload_len - 16 == 4 * channel_count`（IQ 区字节数）。

---

## 4. CRC 校验（与固件一致）

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
```

校验：`crc16_ccitt(frame[0 : 22 + iq_len]) == struct.unpack_from("<H", frame, 22 + iq_len)[0]`  
其中 `iq_len = payload_len - 16`。

---

## 5. 从 bitmap 展开 IQ

```python
import struct

def channels_from_bitmap(bitmap: bytes):
    chs = []
    for ch in range(75):
        if bitmap[ch // 8] & (1 << (ch % 8)):
            chs.append(ch)
    return chs

def parse_iq_payload(iq_bytes: bytes, bitmap: bytes):
    chs = channels_from_bitmap(bitmap)
    out = {}
    off = 0
    for ch in chs:
        i, q = struct.unpack_from("<hh", iq_bytes, off)
        off += 4
        out[ch] = (i, q)
    return out
```

**一致性检查**：`len(chs) == channel_count`，且 `len(iq_bytes) == 4 * channel_count`。

---

## 6. 使用自带脚本

仓库提供参考实现：

```bash
cd doc
python dip_parse_uart.py COM3
# Linux:
python dip_parse_uart.py /dev/ttyACM0
```

可选参数：

```text
--baud 115200
--dump-hex        # 打印每帧十六进制
--csv out.csv     # 追加写入 pc,ch,i,q
```

脚本逻辑：同步字搜索 → 读头 → 读剩余 → CRC → 打印/存盘。

---

## 7. 常见问题

| 现象 | 可能原因 |
|------|----------|
| 搜不到帧 | 波特率错误；或日志 ASCII 干扰同步 |
| CRC 失败 | 帧边界错位；需重新对齐 0x55 0xAA |
| channel_count 与 bitmap 不符 | 帧损坏或协议版本不一致 |
| IQ 全 0 | 尚未收到有效 subevent；或 abort 后无解析输出 |
| 队列丢帧 | 固件 `DIP binary msgq full`；可提高 `DIP_BIN_MSGQ_DEPTH` 或降低 procedure 率 |

---

## 8. 与其它工具对接

- **numpy**：`iq = np.array([(i,q) for ch in sorted(d.keys())])`
- **matplotlib**：按 `procedure_counter` 画 `|I+jQ|` 随 ch 曲线
- **实时**：脚本每帧打印一行摘要；大批量请用 `--csv` 落盘后再离线分析

---

## 9. 推荐采集配置（固件侧）

与 PC 脚本配合时，建议在 `src/main.c` 使用：

```c
#define DIP_REPORT_BINARY_OUTPUT 1
#define DIP_BINARY_USE_THREAD      1
#define DIP_REPORT_LOG_VERBOSE     0
```

并在 `prj.conf` 中适当降低 `CONFIG_LOG` 级别，减少 console 上 ASCII 干扰 `0x55 0xAA` 同步。

---

## 10. 相关文件

| 文件 | 说明 |
|------|------|
| [DIP_binary_protocol.md](./DIP_binary_protocol.md) | 协议字段、线程架构、固件符号表 |
| [dip_parse_uart.py](./dip_parse_uart.py) | 可运行解析示例（与 `dip_crc16_ccitt` 一致） |
| `../src/main.c` | `dip_bin_build_frame`、`dip_uart_tx_thread` |
| `../README.md` | 工程总览与宏默认值 |
