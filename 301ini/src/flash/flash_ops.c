#include "flash_ops.h"
#include "../interface/button_led.h"
#include <bluetooth/cs_de.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>


// 获取 Flash 设备
const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));

LOG_MODULE_REGISTER(flash_ops, LOG_LEVEL_DBG);

// 状态指示
static __IO RunState_t run_state = RUN_STATE_IDLE;
// 写入的索引
static __IO uint64_t flash_index = 0;
// flash大小
static uint64_t flash_size = 0;

// 获取运行状态，这两个函数可以提供接口，避免直接调用 run_state这个变量
RunState_t flash_ops_get_state(void) { return run_state; }
// 设置运行状态
void flash_ops_set_state(RunState_t state) { run_state = state; }
// flash index相关的操作
uint64_t flash_ops_get_index(void) { return flash_index; }

void flash_ops_set_index(uint64_t index) { flash_index = index; }

void flash_ops_increment_index(void) { flash_index++; }
// 获取flash大小
uint64_t flash_ops_get_size(void) { return flash_size; }
// 初始化 Flash 设备
void flash_init(const struct device *flash_dev) {
  if (!device_is_ready(flash_dev)) {
    LOG_ERR("Flash device not ready: %s", flash_dev->name);
  } else {
    LOG_INF("Flash device ready: %s", flash_dev->name);
  }
  flash_get_size(flash_dev, &flash_size);
  // 8M 2616 bytes
  LOG_INF("==> SPI flash[%s] Ready: %lldMB rec:%d", flash_dev->name,
          flash_size / (1024 * 1024), sizeof(cs_de_report_t));
}

int flash_write_data(const struct device *flash_dev, uint64_t index,
                     const void *data, size_t size) {
  return flash_write(flash_dev, index * SPI_FLASH_SECTOR_SIZE, data, size);
}

int flash_read_data(const struct device *flash_dev, uint64_t index, void *data,
                    size_t size) {
  return flash_read(flash_dev, index * SPI_FLASH_SECTOR_SIZE, data, size);
}

int flash_erase_data(const struct device *flash_dev, uint64_t start,
                     size_t size) {
  return flash_erase(flash_dev, start, size);
}