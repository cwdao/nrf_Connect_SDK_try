#ifndef FLASH_OPS_H
#define FLASH_OPS_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>

// flash设备定义
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#define SPI_FLASH_SECTOR_SIZE 4096
// 获取 Flash 设备
extern const struct device *flash_dev;

// 是在读还是在写？这是状态指示
typedef enum {
  RUN_STATE_IDLE = 0xff,
  RUN_STATE_WRITE = 0,
  RUN_STATE_READ = 2,
} RunState_t;
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
int flash_write_data(const struct device *flash_dev, uint64_t index,
                     const void *data, size_t size);
int flash_read_data(const struct device *flash_dev, uint64_t index, void *data,
                    size_t size);
int flash_erase_data(const struct device *flash_dev, uint64_t start,
                     size_t size);

#endif // FLASH_OPS_H