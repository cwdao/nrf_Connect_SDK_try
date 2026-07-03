/**
 * @file app_cs_mode.h
 * @brief DIP / 全功能 CS、UART 输出与 CS procedure 参数的统一编译期配置
 *
 * ## 快速上手
 *
 * 1. 模式切换：改 APP_CS_MODE_DIP（1=DIP / 0=全功能 CS）、APP_CS_UART_BINARY（1=二进制 / 0=文本）
 * 2. CS 时序/功率：改本文件 APP_CS_DIP_* 或 APP_CS_LEGACY_* 参数块（两套独立，互不影响）
 * 3. 重新编译烧录后，在串口查看 procedure_enable_cb 打印的「CS procedures enabled:」确认协商结果
 *
 * ## 与启动日志的对应关系
 *
 * main.c 中 procedure_enable_cb() 在 CS 启用成功后会打印类似：
 *
 *   CS procedures enabled:
 *    - config ID: 0
 *    - antenna configuration index: 0
 *    - TX power: 8 dbm
 *    - subevent length: 21334 us
 *    - subevents per event: 1
 *    - subevent interval: 0
 *    - event interval: 2
 *    - procedure interval: 1
 *    - procedure count: 0
 *    - maximum procedure length: 500
 *
 * 其中可由本文件直接配置的项见各宏注释；subevents per event / subevent interval /
 * event interval 由控制器协商，无法在此直接写入。
 *
 * ## 控制器「建议值」说明
 *
 * bt_le_cs_set_procedure_parameters() 中的 min/max 字段在 Core Spec 中为对控制器的建议，
 * SDC 可能忽略或与请求不同（例如单天线常见 subevent length 上限约 21334 μs）。
 * 调参后务必以 procedure_enable_cb 日志为准，而非假定与宏完全一致。
 *
 * @see README.md「CS 参数配置」
 */

#ifndef APP_CS_MODE_H
#define APP_CS_MODE_H

/* =============================================================================
 * 一、模式与 UART 输出（主开关，日常切换改这里）
 * ============================================================================= */

/**
 * @brief 运行路径选择
 *
 * 1 — DIP（Direct IQ Pipeline）
 *     - subevent 回调：subevent_result_dip_cb
 *     - 跳过 RAS 订阅；本地 HCI 到达即解析 IQ
 *     - UART 二进制帧 type=0x01
 *
 * 0 — 全功能 CS（Legacy：RAS + cs_de）
 *     - subevent 回调：subevent_result_cb
 *     - 等待对端 RAS 数据后 cs_de_populate_report / cs_de_calc
 *     - UART 二进制帧 type=0x02（须对端支持 RAS）
 */
#ifndef APP_CS_MODE_DIP
#define APP_CS_MODE_DIP 0
#endif

/**
 * @brief UART 报告格式
 *
 * 1 — 二进制帧（0x55 0xAA，推荐 PC 脚本采集；DIP 时自动关闭文本 IQ 刷屏）
 * 0 — LOG_INF 文本（DIP 为多信道 ch(i,q)；Legacy 为 print_report_fast）
 *
 * 二进制采集时建议同时降低 prj.conf 中 CONFIG_LOG_DEFAULT_LEVEL，避免 ASCII 与帧同步字混流。
 */
#ifndef APP_CS_UART_BINARY
#define APP_CS_UART_BINARY 1
#endif

/* =============================================================================
 * 二、DIP 模式 CS 参数（APP_CS_MODE_DIP=1 时由 main.c 选用）
 *
 * 下发 API：
 *   - bt_le_cs_set_default_settings()  ← MAX_TX_POWER_DBM
 *   - bt_le_cs_create_config()          ← MODE_0_STEPS
 *   - bt_le_cs_set_procedure_parameters() ← 其余 PROCEDURE_* / SUBEVENT_* 宏
 * ============================================================================= */

/**
 * @brief 单次 CS procedure 最大时长
 *
 * 日志：maximum procedure length
 * 单位：N × 0.625 ms（HCI 定义）；有效范围 0x0001–0xFFFF
 * 默认 500 ≈ 312.5 ms 上限
 */
#ifndef APP_CS_DIP_MAX_PROCEDURE_LEN
#define APP_CS_DIP_MAX_PROCEDURE_LEN 500U
#endif

/**
 * @brief 相邻两次 CS procedure 之间的连接事件间隔（下限 / 上限）
 *
 * 日志：procedure interval（控制器在 [min, max] 内选取）
 * 单位：连接事件个数（非毫秒）
 * 值越小 procedure 越密集、吞吐越高，但更易触发 scheduling conflict / no_cs_sync
 *
 * 若偶发「DIP: subevent aborted reason=no_cs_sync」，可尝试改为 min=2、max=4 放宽间隔。
 */
#ifndef APP_CS_DIP_MIN_PROCEDURE_INTERVAL
#define APP_CS_DIP_MIN_PROCEDURE_INTERVAL 1U
#endif
#ifndef APP_CS_DIP_MAX_PROCEDURE_INTERVAL
#define APP_CS_DIP_MAX_PROCEDURE_INTERVAL 1U
#endif

/**
 * @brief 计划执行的 CS procedure 总次数上限
 *
 * 日志：procedure count
 * 0 — 不限制（持续测距直到 Host 禁用）
 * 非 0 — 达到次数后停止 scheduling 新 procedure
 */
#ifndef APP_CS_DIP_MAX_PROCEDURE_COUNT
#define APP_CS_DIP_MAX_PROCEDURE_COUNT 0U
#endif

/**
 * @brief 每个 CS subevent 的建议时长范围
 *
 * 日志：subevent length（μs，控制器在 [min, max] 内选取实际值）
 * 规范范围：1250 μs – 4 s
 *
 * subevent 内需容纳 Mode-0（CS_SYNC）+ Main Mode tone 步；时长过短可能导致
 * abort_step=0、reason=no_cs_sync。单天线配置下控制器常见上限约 21334 μs。
 */
#ifndef APP_CS_DIP_MIN_SUBEVENT_LEN_US
#define APP_CS_DIP_MIN_SUBEVENT_LEN_US 10000U
#endif
#ifndef APP_CS_DIP_MAX_SUBEVENT_LEN_US
#define APP_CS_DIP_MAX_SUBEVENT_LEN_US 40000U
#endif

/**
 * @brief Tone 天线配置索引
 *
 * 日志：antenna configuration index
 * 0 — BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1（当前 prj.conf 单天线默认）
 * 须与 CONFIG_BT_RAS_MAX_ANTENNA_PATHS / 对端能力一致
 */
#ifndef APP_CS_DIP_TONE_ANTENNA_CONFIG_INDEX
#define APP_CS_DIP_TONE_ANTENNA_CONFIG_INDEX 0U
#endif

/**
 * @brief CS procedure 所用 PHY
 *
 * 1 — LE 2M（默认，吞吐更高）
 * 0 — LE 1M（链路边际较差时可尝试）
 */
#ifndef APP_CS_DIP_PHY_2M
#define APP_CS_DIP_PHY_2M 1
#endif

/**
 * @brief CS 默认发射功率
 *
 * 日志：TX power（dBm）
 * 传入 bt_le_cs_set_default_settings_param.max_tx_power
 * 须在本机支持的功率等级范围内（见 Kconfig / 板级限制）
 */
#ifndef APP_CS_DIP_MAX_TX_POWER_DBM
#define APP_CS_DIP_MAX_TX_POWER_DBM 8
#endif

/**
 * @brief 每个 subevent 内 Mode-0（CS_SYNC）步数
 *
 * 传入 bt_le_cs_create_config_params.mode_0_steps（非 procedure_enable 日志直接打印）
 * 与 abort 日志中 n_steps 相关；增大可加强同步冗余，但占用 subevent 时间预算
 */
#ifndef APP_CS_DIP_MODE_0_STEPS
#define APP_CS_DIP_MODE_0_STEPS 3U
#endif

/* =============================================================================
 * 三、全功能 CS（Legacy）模式 CS 参数（APP_CS_MODE_DIP=0 时由 main.c 选用）
 *
 * 语义与 DIP 块相同，可单独调参而不影响 DIP 配置。
 * 例如：Legacy 可拉长 subevent、降低 procedure 频率以换取双端 RAS 稳定性。
 * APP_CS_LEGACY_MAX_PROCEDURE_INTERVAL 单位是ACL间隔，SDK默认是40units,50ms，
 *一般全功能CS的间隔在400ms左右，也就是此参数为8-10
 * ============================================================================= */

/** @copydoc APP_CS_DIP_MAX_PROCEDURE_LEN */
#ifndef APP_CS_LEGACY_MAX_PROCEDURE_LEN
#define APP_CS_LEGACY_MAX_PROCEDURE_LEN 500U
#endif

/** @copydoc APP_CS_DIP_MIN_PROCEDURE_INTERVAL */
#ifndef APP_CS_LEGACY_MIN_PROCEDURE_INTERVAL
#define APP_CS_LEGACY_MIN_PROCEDURE_INTERVAL 1U
#endif
/** @copydoc APP_CS_DIP_MAX_PROCEDURE_INTERVAL */
#ifndef APP_CS_LEGACY_MAX_PROCEDURE_INTERVAL
#define APP_CS_LEGACY_MAX_PROCEDURE_INTERVAL 5U
#endif

/** @copydoc APP_CS_DIP_MAX_PROCEDURE_COUNT */
#ifndef APP_CS_LEGACY_MAX_PROCEDURE_COUNT
#define APP_CS_LEGACY_MAX_PROCEDURE_COUNT 0U
#endif

/** @copydoc APP_CS_DIP_MIN_SUBEVENT_LEN_US */
#ifndef APP_CS_LEGACY_MIN_SUBEVENT_LEN_US
#define APP_CS_LEGACY_MIN_SUBEVENT_LEN_US 10000U
#endif
/** @copydoc APP_CS_DIP_MAX_SUBEVENT_LEN_US */
#ifndef APP_CS_LEGACY_MAX_SUBEVENT_LEN_US
#define APP_CS_LEGACY_MAX_SUBEVENT_LEN_US 40000U
#endif

/** @copydoc APP_CS_DIP_TONE_ANTENNA_CONFIG_INDEX */
#ifndef APP_CS_LEGACY_TONE_ANTENNA_CONFIG_INDEX
#define APP_CS_LEGACY_TONE_ANTENNA_CONFIG_INDEX 0U
#endif

/** @copydoc APP_CS_DIP_PHY_2M */
#ifndef APP_CS_LEGACY_PHY_2M
#define APP_CS_LEGACY_PHY_2M 1
#endif

/** @copydoc APP_CS_DIP_MAX_TX_POWER_DBM */
#ifndef APP_CS_LEGACY_MAX_TX_POWER_DBM
#define APP_CS_LEGACY_MAX_TX_POWER_DBM 8
#endif

/** @copydoc APP_CS_DIP_MODE_0_STEPS */
#ifndef APP_CS_LEGACY_MODE_0_STEPS
#define APP_CS_LEGACY_MODE_0_STEPS 3U
#endif

/* =============================================================================
 * 四、当前模式生效别名（main.c 引用，一般无需手改）
 *
 * 根据 APP_CS_MODE_DIP 在 DIP / LEGACY 两套参数间切换。
 * ============================================================================= */

#if APP_CS_MODE_DIP
#define APP_CS_ACTIVE_MAX_PROCEDURE_LEN APP_CS_DIP_MAX_PROCEDURE_LEN
#define APP_CS_ACTIVE_MIN_PROCEDURE_INTERVAL APP_CS_DIP_MIN_PROCEDURE_INTERVAL
#define APP_CS_ACTIVE_MAX_PROCEDURE_INTERVAL APP_CS_DIP_MAX_PROCEDURE_INTERVAL
#define APP_CS_ACTIVE_MAX_PROCEDURE_COUNT APP_CS_DIP_MAX_PROCEDURE_COUNT
#define APP_CS_ACTIVE_MIN_SUBEVENT_LEN_US APP_CS_DIP_MIN_SUBEVENT_LEN_US
#define APP_CS_ACTIVE_MAX_SUBEVENT_LEN_US APP_CS_DIP_MAX_SUBEVENT_LEN_US
#define APP_CS_ACTIVE_TONE_ANTENNA_CONFIG_INDEX APP_CS_DIP_TONE_ANTENNA_CONFIG_INDEX
#define APP_CS_ACTIVE_PHY_2M APP_CS_DIP_PHY_2M
#define APP_CS_ACTIVE_MAX_TX_POWER_DBM APP_CS_DIP_MAX_TX_POWER_DBM
#define APP_CS_ACTIVE_MODE_0_STEPS APP_CS_DIP_MODE_0_STEPS
#else
#define APP_CS_ACTIVE_MAX_PROCEDURE_LEN APP_CS_LEGACY_MAX_PROCEDURE_LEN
#define APP_CS_ACTIVE_MIN_PROCEDURE_INTERVAL APP_CS_LEGACY_MIN_PROCEDURE_INTERVAL
#define APP_CS_ACTIVE_MAX_PROCEDURE_INTERVAL APP_CS_LEGACY_MAX_PROCEDURE_INTERVAL
#define APP_CS_ACTIVE_MAX_PROCEDURE_COUNT APP_CS_LEGACY_MAX_PROCEDURE_COUNT
#define APP_CS_ACTIVE_MIN_SUBEVENT_LEN_US APP_CS_LEGACY_MIN_SUBEVENT_LEN_US
#define APP_CS_ACTIVE_MAX_SUBEVENT_LEN_US APP_CS_LEGACY_MAX_SUBEVENT_LEN_US
#define APP_CS_ACTIVE_TONE_ANTENNA_CONFIG_INDEX APP_CS_LEGACY_TONE_ANTENNA_CONFIG_INDEX
#define APP_CS_ACTIVE_PHY_2M APP_CS_LEGACY_PHY_2M
#define APP_CS_ACTIVE_MAX_TX_POWER_DBM APP_CS_LEGACY_MAX_TX_POWER_DBM
#define APP_CS_ACTIVE_MODE_0_STEPS APP_CS_LEGACY_MODE_0_STEPS
#endif

/* =============================================================================
 * 五、UART / RAS 推导宏（由模式主开关自动设置，可在 include 前 #define 覆盖）
 * ============================================================================= */

/** 1=DIP 时跳过 bt_ras_rreq_*_subscribe，避免 latest_local_steps 不同步 */
#ifndef APP_CS_DIP_BYPASS_RAS
#define APP_CS_DIP_BYPASS_RAS APP_CS_MODE_DIP
#endif

#ifndef DIP_REPORT_BINARY_OUTPUT
#define DIP_REPORT_BINARY_OUTPUT (APP_CS_MODE_DIP && APP_CS_UART_BINARY)
#endif

#ifndef DIP_REPORT_LOG_VERBOSE
#define DIP_REPORT_LOG_VERBOSE (APP_CS_MODE_DIP && !APP_CS_UART_BINARY)
#endif

#ifndef CS_REPORT_BINARY_OUTPUT
#define CS_REPORT_BINARY_OUTPUT (!APP_CS_MODE_DIP && APP_CS_UART_BINARY)
#endif

/**
 * Legacy 路径测距完成后是否直出串口（1）或写 Flash（0）。
 * DIP 二进制输出不依赖此项。
 */
#ifndef ENABLE_DIRECT_PRINT
#define ENABLE_DIRECT_PRINT 1
#endif

#endif /* APP_CS_MODE_H */
