#ifndef FLASH_OPS_H
#define FLASH_OPS_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <bluetooth/cs_de.h>
#include "../cs_de_data_parse.h"

// flash设备定义
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#define SPI_FLASH_SECTOR_SIZE 4096
// 数据记录大小 - 使用sizeof确保与实际数据结构一致
#define RECORD_SIZE sizeof(store_cs_de_report_t)
// 每个扇区可以存储的记录数
#define RECORDS_PER_SECTOR (SPI_FLASH_SECTOR_SIZE / RECORD_SIZE)

// 环形缓冲区配置 - 安全利用剩余内存
#define FLASH_BUFFER_SIZE 45   // 45 * 1328 = 59.8KB，安全利用剩余内存
#define FLASH_BUFFER_MASK (FLASH_BUFFER_SIZE - 1)
#define FLASH_BUFFER_WRITE_TRIGGER_COUNT 27  // 9个扇区（约1.35秒数据）

// 分片写入配置 - 使用3的倍数以优化扇区对齐
#define FLASH_WRITE_CHUNK_SIZE 3   // 每次写入3个记录（1个扇区）
#define FLASH_WRITE_CHUNK_DELAY_MS 50  // 每次写入后延迟50ms

// 写入模式配置
#define FLASH_WRITE_MODE_SINGLE 0    // 单个写入模式
#define FLASH_WRITE_MODE_BATCH  1    // 批量写入模式
#define FLASH_WRITE_MODE FLASH_WRITE_MODE_SINGLE  // 当前使用的写入模式

// 直接打印模式配置（测距完成后直接打印到串口，不写入flash）
#define ENABLE_DIRECT_PRINT 0  // 设置为1启用直接打印模式，0则写入flash

// 获取 Flash 设备
extern const struct device *flash_dev;

// 是在读还是在写？这是状态指示
typedef enum {
  RUN_STATE_IDLE = 0xff,
  RUN_STATE_WRITE = 0,
  RUN_STATE_READ = 2,
} RunState_t;

// 环形缓冲区结构
typedef struct {
    uint8_t buffer[FLASH_BUFFER_SIZE][RECORD_SIZE];
    uint32_t write_index;
    uint32_t read_index;
    uint32_t count;
    bool is_full;
} flash_ring_buffer_t;

// 运行状态接口
RunState_t flash_ops_get_state(void);
void flash_ops_set_state(RunState_t state);

// flash写入的索引接口
uint64_t flash_ops_get_index(void);
void flash_ops_set_index(uint64_t index);
void flash_ops_increment_index(void);
// flash_size 存储 Flash 总大小，通常在初始化时确定。
// 它是一个只读变量，外部模块只需要获取其值。
uint64_t flash_ops_get_size(void);

void flash_init(const struct device *flash_dev);

// 环形缓冲区操作函数
int flash_buffer_init(void);
int flash_buffer_put(const void *data, size_t size);
int flash_buffer_flush(void);
int flash_buffer_flush_chunked(void);  // 分片写入
uint32_t flash_buffer_get_count(void);
bool flash_buffer_is_full(void);
bool flash_buffer_is_empty(void);
void flash_buffer_debug_print(void);

// 紧凑存储的读写函数
int flash_write_data_compact(const struct device *flash_dev, uint64_t index,
                           const void *data, size_t size);
int flash_read_data_compact(const struct device *flash_dev, uint64_t index, 
                          void *data, size_t size);
int flash_erase_sector_if_needed(const struct device *flash_dev, uint64_t index);
// 检查当前扇区是否有我们存储的数据，如果有，则返回1
int flash_sector_needs_erase(uint64_t sector_index);

// 单个写入函数
int flash_write_single_report(const store_cs_de_report_t *report);

// Flash状态检查函数
int flash_check_and_suggest_erase(void);

// 智能设置flash写入起始位置
void flash_smart_set_start_position(void);



// 兼容性函数（保持向后兼容）
int flash_write_data(const struct device *flash_dev, uint64_t index,
                     const void *data, size_t size);
int flash_read_data(const struct device *flash_dev, uint64_t index, void *data,
                    size_t size);
int flash_erase_data(const struct device *flash_dev, uint64_t start,
                     size_t size);

// 性能测试函数
void flash_performance_test(void);

#endif // FLASH_OPS_H