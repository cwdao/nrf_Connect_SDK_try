#include "flash_ops.h"
#include "../interface/button_led.h"
#include <bluetooth/cs_de.h>
#include <string.h>
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

// 环形缓冲区实例
static flash_ring_buffer_t flash_buffer;

// 分片写入状态
static struct {
  bool is_flushing;
  uint32_t remaining_records;
  uint32_t current_chunk;
} chunk_write_state = {false, 0, 0};

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

// 环形缓冲区初始化
int flash_buffer_init(void) {
  memset(&flash_buffer, 0, sizeof(flash_buffer));
  flash_buffer.write_index = 0;
  flash_buffer.read_index = 0;
  flash_buffer.count = 0;
  flash_buffer.is_full = false;
  LOG_INF("Flash buffer initialized with size: %d", FLASH_BUFFER_SIZE);
  return 0;
}

// 向环形缓冲区添加数据
int flash_buffer_put(const void *data, size_t size) {
  if (size != RECORD_SIZE) {
    LOG_ERR("Invalid data size: %d, expected: %d", size, RECORD_SIZE);
    return -1;
  }

  if (flash_buffer.is_full) {
    LOG_WRN("Flash buffer is full, dropping oldest record");
    // 缓冲区满时，覆盖最旧的数据
    flash_buffer.read_index = (flash_buffer.read_index + 1) & FLASH_BUFFER_MASK;
    flash_buffer.count--;
  }

  // 复制数据到缓冲区
  memcpy(flash_buffer.buffer[flash_buffer.write_index], data, size);
  flash_buffer.write_index = (flash_buffer.write_index + 1) & FLASH_BUFFER_MASK;
  flash_buffer.count++;

  // 检查是否满
  if (flash_buffer.count >= FLASH_BUFFER_SIZE) {
    flash_buffer.is_full = true;
  }

  LOG_DBG("Added data to buffer, count: %d", flash_buffer.count);
  return 0;
}

// 将缓冲区数据批量写入flash
int flash_buffer_flush(void) {
  if (flash_buffer.count == 0) {
    LOG_DBG("Buffer is empty, nothing to flush");
    return 0;
  }

  LOG_INF("Flushing %d records to flash", flash_buffer.count);

  int err = 0;
  uint32_t records_to_write = flash_buffer.count;

  for (uint32_t i = 0; i < records_to_write; i++) {
    err = flash_write_data_compact(flash_dev, flash_index,
                                   flash_buffer.buffer[flash_buffer.read_index],
                                   RECORD_SIZE);
    if (err) {
      LOG_ERR("Flash write failed at index %llu: %d", flash_index, err);
      return err;
    }

    flash_index++;
    flash_buffer.read_index = (flash_buffer.read_index + 1) & FLASH_BUFFER_MASK;
    flash_buffer.count--;
  }

  flash_buffer.is_full = false;
  LOG_INF("Successfully flushed %d records to flash", records_to_write);
  return 0;
}

// 分片写入flash - 每次只写入一小部分，避免长时间阻塞
int flash_buffer_flush_chunked(void) {
  if (flash_buffer.count == 0) {
    LOG_DBG("Buffer is empty, nothing to flush");
    chunk_write_state.is_flushing = false;
    return 0;
  }

  // 如果是第一次调用，初始化分片写入状态
  if (!chunk_write_state.is_flushing) {
    chunk_write_state.is_flushing = true;
    chunk_write_state.remaining_records = flash_buffer.count;
    chunk_write_state.current_chunk = 0;
    LOG_INF("Starting chunked flush of %d records", flash_buffer.count);
  }

  // 安全检查：如果remaining_records已经是负数或0，说明已经完成
  if (chunk_write_state.remaining_records <= 0) {
    flash_buffer.is_full = false;
    chunk_write_state.is_flushing = false;
    LOG_INF("Chunked flush completed, total %d chunks",
            chunk_write_state.current_chunk);
    return 0;
  }

  // 计算本次要写入的记录数，确保是3的倍数以优化扇区对齐
  uint32_t max_chunk =
      MIN(FLASH_WRITE_CHUNK_SIZE, chunk_write_state.remaining_records);
  uint32_t chunk_size =
      (max_chunk / RECORDS_PER_SECTOR) * RECORDS_PER_SECTOR; // 确保是3的倍数
  if (chunk_size == 0 && max_chunk > 0) {
    chunk_size = RECORDS_PER_SECTOR; // 至少写入3个记录
  }

  // 安全检查：确保不会写入超过剩余记录数
  if (chunk_size > chunk_write_state.remaining_records) {
    chunk_size = chunk_write_state.remaining_records;
  }

  LOG_INF("Writing chunk %d: %d records (%d sectors)",
          chunk_write_state.current_chunk + 1, chunk_size, chunk_size / 3);

  int err = 0;
  for (uint32_t i = 0; i < chunk_size; i++) {
    err = flash_write_data_compact(flash_dev, flash_index,
                                   flash_buffer.buffer[flash_buffer.read_index],
                                   RECORD_SIZE);
    if (err) {
      LOG_ERR("Flash write failed at index %llu: %d", flash_index, err);
      chunk_write_state.is_flushing = false;
      return err;
    }

    flash_index++;
    flash_buffer.read_index = (flash_buffer.read_index + 1) & FLASH_BUFFER_MASK;
    flash_buffer.count--;
    chunk_write_state.remaining_records--;
  }

  chunk_write_state.current_chunk++;

  // 检查是否还有剩余记录需要写入
  if (chunk_write_state.remaining_records > 0) {
    LOG_INF("Chunk %d completed, %d records remaining",
            chunk_write_state.current_chunk,
            chunk_write_state.remaining_records);
    return 1; // 返回1表示还有更多数据需要写入
  } else {
    // 所有数据写入完成
    flash_buffer.is_full = false;
    chunk_write_state.is_flushing = false;
    LOG_INF("Chunked flush completed, total %d chunks",
            chunk_write_state.current_chunk);
    return 0;
  }
}

// 获取缓冲区中的数据数量
uint32_t flash_buffer_get_count(void) { return flash_buffer.count; }

// 检查缓冲区是否满
bool flash_buffer_is_full(void) { return flash_buffer.is_full; }

// 检查缓冲区是否空
bool flash_buffer_is_empty(void) { return flash_buffer.count == 0; }

// 计算扇区索引和扇区内偏移
static void calculate_sector_and_offset(uint64_t record_index,
                                        uint64_t *sector_index,
                                        uint64_t *offset_in_sector) {
  *sector_index = record_index / RECORDS_PER_SECTOR;
  *offset_in_sector = (record_index % RECORDS_PER_SECTOR) * RECORD_SIZE;
}

// 紧凑存储的写入函数
int flash_write_data_compact(const struct device *flash_dev, uint64_t index,
                             const void *data, size_t size) {
  uint64_t sector_index, offset_in_sector;
  int err;

  // 计算扇区索引和偏移
  calculate_sector_and_offset(index, &sector_index, &offset_in_sector);

  // 如果是扇区的第一个记录，需要先擦除扇区
  if (offset_in_sector == 0) {
    LOG_INF("Erasing sector %llu for new data", sector_index);
    err = flash_erase(flash_dev, sector_index * SPI_FLASH_SECTOR_SIZE,
                      SPI_FLASH_SECTOR_SIZE);
    if (err) {
      LOG_ERR("Flash erase failed: %d", err);
      return err;
    }
    LOG_INF("Sector %llu erased successfully", sector_index);
  }

  // 写入数据到指定位置
  LOG_INF("Writing to Flash - Sector: %llu, Offset: %llu, Size: %d",
          sector_index, offset_in_sector, size);

  err = flash_write(flash_dev,
                    sector_index * SPI_FLASH_SECTOR_SIZE + offset_in_sector,
                    data, size);
  if (err) {
    LOG_ERR("Flash write failed: %d", err);
  } else {
    LOG_INF("Flash write successful - Index: %llu", index);
  }

  return err;
}

// 紧凑存储的读取函数
int flash_read_data_compact(const struct device *flash_dev, uint64_t index,
                            void *data, size_t size) {
  uint64_t sector_index, offset_in_sector;

  // 计算扇区索引和偏移
  calculate_sector_and_offset(index, &sector_index, &offset_in_sector);

  // 从指定位置读取数据
  return flash_read(flash_dev,
                    sector_index * SPI_FLASH_SECTOR_SIZE + offset_in_sector,
                    data, size);
}

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
  LOG_INF("Records per sector: %d, Record size: %d bytes", RECORDS_PER_SECTOR,
          RECORD_SIZE);

  // 检查DMA配置
  LOG_INF("=== Flash DMA Configuration ===");
#ifdef CONFIG_DMA
  LOG_INF("CONFIG_DMA: ENABLED");
#else
  LOG_INF("CONFIG_DMA: DISABLED");
#endif

#ifdef CONFIG_NRFX_QSPI
  LOG_INF("CONFIG_NRFX_QSPI: ENABLED");
#else
  LOG_INF("CONFIG_NRFX_QSPI: DISABLED");
#endif

  // 检查设备属性
  const struct flash_parameters *params = flash_get_parameters(flash_dev);
  if (params) {
    LOG_INF("Flash write block size: %d bytes", params->write_block_size);
    LOG_INF("Flash erase value: 0x%02X", params->erase_value);
  }

  // 初始化环形缓冲区
  flash_buffer_init();
}

// 兼容性函数（保持向后兼容）
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

// Flash写入性能测试
void flash_performance_test(void) {
  uint8_t test_data[RECORD_SIZE];
  uint32_t start_time, end_time, elapsed_ms;
  uint64_t test_start_index = flash_index;
  uint64_t test_end_index;

  // 填充测试数据
  for (int i = 0; i < RECORD_SIZE; i++) {
    test_data[i] = i & 0xFF;
  }

  LOG_INF("=== Flash Performance Test ===");

  // 测试单个记录写入时间
  start_time = k_uptime_get();
  int err =
      flash_write_data_compact(flash_dev, flash_index, test_data, RECORD_SIZE);
  end_time = k_uptime_get();
  elapsed_ms = end_time - start_time;

  if (err == 0) {
    LOG_INF("Single record write: %d ms", elapsed_ms);
    flash_index++;
  } else {
    LOG_ERR("Flash write test failed: %d", err);
    return;
  }

  // 测试批量写入时间
  start_time = k_uptime_get();
  for (int i = 0; i < 10; i++) {
    err = flash_write_data_compact(flash_dev, flash_index, test_data,
                                   RECORD_SIZE);
    if (err) {
      LOG_ERR("Batch write failed at record %d: %d", i, err);
      break;
    }
    flash_index++;
  }
  end_time = k_uptime_get();
  elapsed_ms = end_time - start_time;

  LOG_INF("Batch write (10 records): %d ms", elapsed_ms);
  LOG_INF("Average per record: %d ms", elapsed_ms / 10);

  // 记录测试结束的索引
  test_end_index = flash_index;

  // 清理测试数据 - 擦除所有测试写入的扇区
  LOG_INF("Cleaning up test data...");
  uint64_t start_sector = test_start_index / RECORDS_PER_SECTOR;
  uint64_t end_sector = (test_end_index - 1) / RECORDS_PER_SECTOR;

  for (uint64_t sector = start_sector; sector <= end_sector; sector++) {
    LOG_INF("Erasing test sector %llu", sector);
    err = flash_erase(flash_dev, sector * SPI_FLASH_SECTOR_SIZE,
                      SPI_FLASH_SECTOR_SIZE);
    if (err) {
      LOG_ERR("Failed to erase test sector %llu: %d", sector, err);
    } else {
      LOG_INF("Test sector %llu erased successfully", sector);
    }
  }

  // 恢复flash_index到测试前的状态
  flash_index = test_start_index;
  LOG_INF("Flash performance test completed and cleaned up");
}