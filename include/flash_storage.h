/*
 * flash_storage.h
 *
 * Slot-Based Circular Buffer on Raw Flash.
 */

#ifndef FLASH_STORAGE_H__
#define FLASH_STORAGE_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include "model_handler.h" 

// Maximum entries to buffer in RAM for batch operations
#define FLASH_BATCH_SIZE 10

/**
 * @brief Initialize the flash storage system.
 * Scans metadata to restore the write cursor.
 * @return 0 on success, negative errno on failure.
 */
int flash_storage_init(void);

/**
 * @brief Store a single reading.
 * Writes to the current sector. If full, moves to next sector and erases it.
 * @param msg The sensor reading to store.
 */
void flash_storage_write(const struct sensor_message *msg);

/**
 * @brief Read readings from storage.
 * Reads 'count' items from the ACTIVE slot (newest data).
 * @param msgs Output buffer.
 * @param count Max number to read.
 * @return Number of items read.
 */
int flash_storage_read(struct sensor_message *msgs, int count);

/**
 * @brief Get total number of stored records across all slots.
 */
uint32_t flash_storage_get_count(void);

/**
 * @brief Wipe the entire storage partition.
 */
int flash_storage_clear(void);

#endif /* FLASH_STORAGE_H__ */