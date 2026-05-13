# nrf_Connect_SDK_try

本仓库收集在 **nRF Connect SDK（NCS）** 上做 **Channel Sounding（CS）**、**Direction Finding（DF）** 以及周边能力（Flash、UART 等）的示例与实验工程。各子目录多为独立 Zephyr 应用，请在其目录内用 `west build` 等方式单独编译。

---

## 命名与配对规则

### Channel Sounding（CS）

- **需要两台设备**：一台做 **Initiator**，一台做 **Reflector**。
- 常见目录是**成对出现**的，例如 `301ini` / `301ref`：前缀数字表示 **NCS 主版本号**（如 **301** ≈ **SDK 3.0.1**）。
- **建议同一数字前缀、同一 SDK 版本**的两工程互配，避免 API/行为差异带来的问题。

### Direction Finding（DF）

- **需要两台设备**：一台做 **Central**，一台做 **Peripheral**。
- 同样常用**数字前缀**表示 SDK 代际（如 `301df_*`、`321df_*`），**配对时优先同前缀、同版本**。

### SDK 版本与功能的关系

- **并不是「SDK 越新 = 功能越全」**。本仓库里大致情况是：
  - **CS** 相关可工作、可对照的示例主要集中在 **301（3.0.1）** 一代工程上。
  - **DF** 最早在 **301** 上搭过（`301df_central` / `301df_peripheral`），后续主要维护与验证迁移到了 **321**（`321df_central` / `321df_peripheral`）。跨版本组合曾在开发中试过（以 **git 提交说明**为准）。
- 若需核对某一目录最初基于哪套 SDK、何时切到 321，请在本仓库根目录执行 `git log`，结合各子目录的 `west.yml` / `VERSION` 等再确认。

### 建议从哪一对开始测

| 目标 | 建议使用的目录对 |
|------|------------------|
| **测 CS** | `301ini` + `301ref`（及同代的 `301ini_Dip` 等变体见下表） |
| **测 DF** | `321df_central` + `321df_peripheral` |

---

## 子文件夹说明

以下为当前根目录下各子文件夹的**用途概括**（按名称分组，便于查找）。

### CS（301 代际，Initiator / Reflector）

| 目录 | 说明 |
|------|------|
| **301ini** | CS **Initiator** 侧，301（3.0.1）代际；与 `301ref` 配对测试 CS。 |
| **301ref** | CS **Reflector** 侧，与 `301ini` 配对。 |
| **301ini_Dip** | 在 301ini 思路上的变体（如 Direct IQ / 管线相关实验），仍属 CS 一侧 Initiator 类工程，配对关系以目录内说明或编译选项为准。 |

### DF（Central / Peripheral）

| 目录 | 说明 |
|------|------|
| **301df_central** | DF **Central**，301 代际；可与 `301df_peripheral` 同代配对，或与后续 321 工程做跨代实验（见 git 历史）。 |
| **301df_peripheral** | DF **Peripheral**，301 代际。 |
| **321df_central** | DF **Central**，321 代际；**当前更推荐与 `321df_peripheral` 一起测 DF**。 |
| **321df_peripheral** | DF **Peripheral**，321 代际。 |

### 早期或其它命名的 CS / DF 实验

以下多为**非常早期的 CS/DF 命名或单次试验**，结构或 SDK 版本不一定与 `301ini`/`301ref` 一致，适合对照历史实现，**新实验优先用上面「301 / 321」成对目录**。

| 目录 | 说明 |
|------|------|
| **cs_ini1** / **cs_ref1** | 早期 CS initiator / reflector 命名风格。 |
| **cs_ini1_fast_try** | CS initiator 侧与「更快更新」等相关的尝试工程。 |
| **cs_test_ini** / **cs_test_ref** | 早期 CS 测试用 initiator / reflector。 |
| **ncs301_cs_ini** | 与 NCS 3.0.1 相关的 CS initiator 实验目录。 |

### 特殊功能与参考（非 CS/DF 主命名）

| 目录 | 说明 |
|------|------|
| **09-spi_flash-v1.0** | 通过 SPI 等进行 **Flash 读写** 的参考/实验工程（编号风格表示独立小专题）。 |
| **flash-test** | Flash 相关通用测试或草稿。 |
| **sensing_save_flash** | 传感/采集与 **写入 Flash** 结合的实验。 |
| **peripheral_uart** | UART **Peripheral** 侧示例或串口联调参考（非 CS/DF 主流程命名）。 |

---

## 使用提示

- 每个子目录通常自带 `CMakeLists.txt` / `prj.conf`，在对应目录按 NCS 文档执行 `west build -b <board>`。
- **板型、射频与天线**需与各工程 `README` 或 `prj.conf` 一致；跨板组合以实际连接与 git 记录为准。
- 更新本表时，若新增成对目录，请保持 **CS（ini/ref）**、**DF（central/peripheral）** 与 **数字前缀 = SDK 代际** 的说明一致。
