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

#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
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
    flash_buffer.read_index = (flash_buffer.read_index + 1) % FLASH_BUFFER_SIZE;
    flash_buffer.count--;
    LOG_DBG("Buffer full - read_index moved to %d", flash_buffer.read_index);
  }

  // 复制数据到缓冲区
  memcpy(flash_buffer.buffer[flash_buffer.write_index], data, size);
  
  // 打印刚刚写入的数据的timestamp
//   LOG_DBG("->> buffer, time: %llu, write_idx: %d, count: %d",
//           ((store_cs_de_report_t *)flash_buffer.buffer[flash_buffer.write_index])
//               ->timestamp_ms,
//           flash_buffer.write_index,
//           flash_buffer.count);
  
  flash_buffer.write_index = (flash_buffer.write_index + 1) % FLASH_BUFFER_SIZE;
  flash_buffer.count++;

  // 检查是否满
  if (flash_buffer.count >= FLASH_BUFFER_SIZE) {
    flash_buffer.is_full = true;
    LOG_DBG("Buffer became full - count: %d, write_idx: %d", flash_buffer.count, flash_buffer.write_index);
  }
  return 0;
}

// 将缓冲区数据批量写入flash
int flash_buffer_flush(void) {
  if (flash_buffer.count == 0) {
    LOG_DBG("Buffer is empty, nothing to flush");
    return 0;
  }

  LOG_INF("Flushing %d records to flash", flash_buffer.count);
  
  // 调试：打印缓冲区内容
  flash_buffer_debug_print();

  int err = 0;
  uint32_t records_to_write = flash_buffer.count;

  for (uint32_t i = 0; i < records_to_write; i++) {
    // 打印正在读取的数据的timestamp
    LOG_DBG("Flushing record %d: time: %llu, read_idx: %d",
            i,
            ((store_cs_de_report_t *)flash_buffer.buffer[flash_buffer.read_index])
                ->timestamp_ms,
            flash_buffer.read_index);
    
    err = flash_write_data_compact(flash_dev, flash_index,
                                   flash_buffer.buffer[flash_buffer.read_index],
                                   RECORD_SIZE);
    if (err) {
      LOG_ERR("Flash write failed at index %llu: %d", flash_index, err);
      return err;
    }

    flash_index++;
    flash_buffer.read_index = (flash_buffer.read_index + 1) % FLASH_BUFFER_SIZE;
    flash_buffer.count--;
  }

  flash_buffer.is_full = false;
  LOG_INF("Successfully flushed %d records to flash", records_to_write);
  return 0;
}
#endif
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
    
    // 调试：打印缓冲区内容
    // flash_buffer_debug_print();
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
    // 打印正在读取的数据的timestamp
    LOG_DBG("Chunk flushing record %d: time: %llu, read_idx: %d",
            i,
            ((store_cs_de_report_t *)flash_buffer.buffer[flash_buffer.read_index])
                ->timestamp_ms,
            flash_buffer.read_index);
    
    err = flash_write_data_compact(flash_dev, flash_index,
                                   flash_buffer.buffer[flash_buffer.read_index],
                                   RECORD_SIZE);
    if (err) {
      LOG_ERR("Flash write failed at index %llu: %d", flash_index, err);
      chunk_write_state.is_flushing = false;
      return err;
    }

    flash_index++;
    flash_buffer.read_index = (flash_buffer.read_index + 1) % FLASH_BUFFER_SIZE;
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

// 调试函数：打印缓冲区内容
void flash_buffer_debug_print(void) {
  LOG_INF("=== Flash Buffer Debug Info ===");
  LOG_INF("Buffer count: %d, read_idx: %d, write_idx: %d, is_full: %s",
          flash_buffer.count, flash_buffer.read_index, flash_buffer.write_index,
          flash_buffer.is_full ? "true" : "false");
  
  if (flash_buffer.count > 0) {
    LOG_INF("Buffer contents:");
    uint32_t idx = flash_buffer.read_index;
    for (uint32_t i = 0; i < flash_buffer.count; i++) {
      LOG_INF("  [%d] time: %llu, report_idx: %llu, write_pos: %d, read_pos: %d",
              idx,
              ((store_cs_de_report_t *)flash_buffer.buffer[idx])->timestamp_ms,
              ((store_cs_de_report_t *)flash_buffer.buffer[idx])->report_index,
              flash_buffer.write_index,
              flash_buffer.read_index);
      idx = (idx + 1) % FLASH_BUFFER_SIZE;
    }
  } else {
    LOG_INF("Buffer is empty");
  }
  LOG_INF("=== End Flash Buffer Debug ===");
}

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

  // 检查闪存容量边界
  uint64_t flash_size = flash_ops_get_size();
  uint64_t max_records = flash_size / RECORD_SIZE;
  
  if (index >= max_records) {
    LOG_ERR("Flash write boundary exceeded! Index: %llu, Max records: %llu", 
            index, max_records);
    return -1;  // 返回错误，表示超出边界
  }

  // 计算扇区索引和偏移
  calculate_sector_and_offset(index, &sector_index, &offset_in_sector);

  // 如果是扇区的第一个记录，需要先考虑擦除扇区。首先读取第一个数据，如果report_index为FF，则说明扇区是空的，需要擦除扇区
  if (offset_in_sector == 0 && flash_sector_needs_erase(sector_index) == 1) {
    LOG_INF("Erasing sector %llu for new sector", sector_index);
    err = flash_erase(flash_dev, sector_index * SPI_FLASH_SECTOR_SIZE,
                      SPI_FLASH_SECTOR_SIZE);
    if (err) {
      LOG_ERR("Flash erase failed: %d", err);
      return err;
    }
    LOG_INF("Sector %llu erased successfully", sector_index);
  }

  // 写入数据到指定位置
  // LOG_INF("Writing to Flash - Sector: %llu, Offset: %llu, Size: %d",
  //         sector_index, offset_in_sector, size);

  err = flash_write(flash_dev,
                    sector_index * SPI_FLASH_SECTOR_SIZE + offset_in_sector,
                    data, size);
  if (err) {
    LOG_ERR("Flash write failed: %d", err);
  } else {
    // LOG_INF("Flash write successful - Index: %llu", index);
    // 打印写入数据的timestamp以确保成功
    // LOG_INF("->> flash write, time: %llu, idx: %llu",
    //         ((store_cs_de_report_t *)data)->timestamp_ms,
    //         ((store_cs_de_report_t *)data)->report_index);
  }

  return err;
}

// 检查扇区是否需要擦除
int flash_sector_needs_erase(uint64_t sector_index) {
  static store_cs_de_report_t erase_tmp;
  int err = flash_read(flash_dev, sector_index * SPI_FLASH_SECTOR_SIZE, &erase_tmp,
    sizeof(store_cs_de_report_t));
  if (err) {
    LOG_ERR("Flash read failed when checking sector %llu: %d", sector_index, err);
    return -1;
  }
  
  // 如果扇区是空的（所有位都是1），则不需要擦除
  if (erase_tmp.report_index == 0xFFFFFFFFFFFFFFFF) {
    // LOG_DBG("Sector %llu is empty, no erase needed", sector_index);
    return 0;  // 不需要擦除
  } else {
    // LOG_DBG("Sector %llu has data (index: %llu), needs erase", 
    //         sector_index, erase_tmp.report_index);
    return 1;  // 需要擦除
  }
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
  LOG_INF("==> SPI flash[%s] Ready: %lldMB Record Size:%d", flash_dev->name,
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
#if FLASH_WRITE_MODE == FLASH_WRITE_MODE_BATCH
  // 初始化环形缓冲区
  flash_buffer_init();
#endif
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

// 单个写入函数 - 直接写入一个report到flash，不经过缓冲区
int flash_write_single_report(const store_cs_de_report_t *report) {
  if (!report) {
    LOG_ERR("Invalid report pointer");
    return -1;
  }
  
  uint64_t current_index = flash_ops_get_index();
  
  // LOG_DBG("Writing single report - Index: %llu, Timestamp: %llu", 
  //         current_index, report->timestamp_ms);
  
  // 计算扇区信息用于调试
  // uint64_t sector_index = current_index / RECORDS_PER_SECTOR;
  // uint64_t offset_in_sector = (current_index % RECORDS_PER_SECTOR) * sizeof(store_cs_de_report_t);
  // LOG_DBG("Target: Sector %llu, Offset %llu", sector_index, offset_in_sector);
  
  int err = flash_write_data_compact(flash_dev, current_index, 
                                     report, sizeof(store_cs_de_report_t));
  if (err) {
    LOG_ERR("Single flash write failed at index %llu: %d", current_index, err);
    return err;
  }
  
  // 成功写入后递增索引
  flash_ops_increment_index();
  
  // LOG_DBG("Single flash write successful - Index: %llu", current_index);
  return 0;
}

// Flash状态检查函数 - 测试模式下自动处理flash状态
int flash_check_and_suggest_erase(void) {
  uint64_t flash_size = flash_ops_get_size();
  uint64_t total_sectors = flash_size / SPI_FLASH_SECTOR_SIZE;
  uint64_t used_sectors = 0;
  uint64_t erased_sectors = 0;
  int erase_errors = 0;
  
  // 进度条相关变量
  const uint64_t progress_step = 256;  // 每64个扇区更新一次进度条
  const uint64_t max_progress_bars = 16;  // 最大32个进度条字符
  uint64_t last_progress_sector = 0;
  
  LOG_INF("🔍 Scanning %llu sectors...", total_sectors);
  
  // 检查已使用的扇区数
  for (uint64_t sector = 0; sector < total_sectors; sector++) {
    int erase_status = flash_sector_needs_erase(sector);
    if (erase_status == 1) {  // 扇区有数据
      used_sectors++;
      int err = flash_erase(flash_dev, sector * SPI_FLASH_SECTOR_SIZE, SPI_FLASH_SECTOR_SIZE);
      if (err) {
        LOG_ERR("Failed to erase sector %llu: %d", sector, err);
        erase_errors++;
      } else {
        LOG_DBG("Sector %llu has data, Erased.", sector);
        erased_sectors ++;
      }
    } else if (erase_status == -1) {  // 读取错误
      LOG_ERR("Flash read error at sector %llu", sector);
      return -1;
    }
    // erase_status == 0 表示扇区为空
    
    // 更新进度条
    if (sector - last_progress_sector >= progress_step || sector == total_sectors - 1) {
      uint64_t progress_bars = (sector + 1) * max_progress_bars / total_sectors;
      if (progress_bars > max_progress_bars) {
        progress_bars = max_progress_bars;
      }
      
      // 构建进度条字符串
      char progress_str[33] = {0};  // 32个字符 + 结束符
      for (uint64_t i = 0; i < progress_bars; i++) {
        progress_str[i] = '>';
      }
      
      uint64_t percentage = (sector + 1) * 100 / total_sectors;
      LOG_INF("[%s] %llu%% (%llu/%llu sectors)", progress_str, percentage, sector + 1, total_sectors);
      last_progress_sector = sector;
    }
  }
  
  // 如果flash是空的，不输出信息，直接返回
  if (used_sectors == 0) {
    return 0;  // Flash干净，无需处理
  }
  
  // 如果有数据，在测试模式下自动擦除
  LOG_INF("🔍 Flash contains data (%llu/%llu sectors used, %.1f%%)", 
          used_sectors, total_sectors, 
          (double)used_sectors / total_sectors * 100.0);
  LOG_INF("Erased sectors: %llu", erased_sectors);
  
  // 擦除所有已使用的扇区
  // int erase_errors = 0;
  // for (uint64_t sector = 0; sector < total_sectors; sector++) {
  //   int erase_status = flash_sector_needs_erase(sector);
  //   if (erase_status == 0) {  // 扇区有数据，需要擦除
  //     int err = flash_erase(flash_dev, sector * SPI_FLASH_SECTOR_SIZE, SPI_FLASH_SECTOR_SIZE);
  //     if (err) {
  //       LOG_ERR("Failed to erase sector %llu: %d", sector, err);
  //       erase_errors++;
  //     } else {
  //       LOG_DBG("Erased sector %llu", sector);
  //     }
  //   }
  // }
  
  if (erase_errors == 0) {
    LOG_INF("✅ Flash erased successfully, ready for new test");
    // 重置flash索引到开头
    flash_index = 0;
    return 0;  // 成功擦除
  } else {
    LOG_ERR("❌ Flash erase completed with %d errors", erase_errors);
    return -1;  // 擦除有错误
  }
}

// 智能设置flash写入起始位置
void flash_smart_set_start_position(void) {
  uint64_t flash_size = flash_ops_get_size();
  uint64_t total_records = flash_size / sizeof(store_cs_de_report_t);
  uint64_t last_valid_index = 0;
  bool found_data = false;
  
  LOG_INF("🔍 Scanning flash to determine start position...");
  
  // 从后往前扫描，找到最后一个有效数据的位置
  for (int64_t index = total_records - 1; index >= 0; index--) {
    store_cs_de_report_t temp_record;
    int err = flash_read_data_compact(flash_dev, index, &temp_record, sizeof(store_cs_de_report_t));
    
    if (err == 0 && temp_record.report_index != 0xFFFFFFFFFFFFFFFF) {
      // 找到有效数据
      last_valid_index = index;
      found_data = true;
      LOG_INF("📍 Found last valid data at index %llu (report_index: %llu)", 
              index, temp_record.report_index);
      break;
    }
  }
  
  if (found_data) {
    // 从最后一个有效数据的下一个位置开始写入
    flash_index = last_valid_index + 1;
    LOG_INF("📝 Continuing from index %llu (after existing data)", flash_index);
  } else {
    // 没有找到有效数据，从头开始
    flash_index = 0;
    LOG_INF("📝 No existing data found, starting from index 0");
  }
  
  LOG_INF("✅ Flash start position set to: %llu", flash_index);
}