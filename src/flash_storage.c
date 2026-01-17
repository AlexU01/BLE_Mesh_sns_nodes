/*
 * flash_storage.c
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include "flash_storage.h"
#include <pm_config.h> 

LOG_MODULE_REGISTER(flash_storage, LOG_LEVEL_INF);

// Flash layout configs
#define SECTOR_SIZE             CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE
#define STORAGE_PARTITION_ID    PM_DATA_STORAGE_ID
#define STORAGE_START_OFFSET    PM_DATA_STORAGE_OFFSET
#define STORAGE_TOTAL_SIZE      PM_DATA_STORAGE_SIZE
const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(nordic_pm_ext_flash));

/** 
 * Layout:
 * Slots 0..N-1: Data
 * Slot N: Metadata
 */
#define META_SECTOR_SIZE        SECTOR_SIZE
#define MAX_SLOTS               ((STORAGE_TOTAL_SIZE / SECTOR_SIZE) - 1)
#define USED_SLOTS              2
#define META_REGION_ADDR        (STORAGE_START_OFFSET + (USED_SLOTS * SECTOR_SIZE))

// Align entry to 4 bytes
#define ENTRY_SIZE              ROUND_UP(sizeof(struct sensor_message), 4)
#define ENTRIES_PER_SECTOR      (SECTOR_SIZE / ENTRY_SIZE)

// Persistent Context
#define CTX_MAGIC               0xDEADBEEF

struct flash_ctx {
    uint32_t magic;
    uint32_t current_slot;           /* Index 0 to USED_SLOTS-1 */
    uint32_t write_offset;           /* Byte offset within the current slot */
    uint32_t total_count;            /* Total records stored (approx) */
    uint8_t  slot_valid[USED_SLOTS]; /* 1 if slot is full */
};

// In-Memory State
static struct flash_ctx ctx;
static uint32_t meta_write_offset = 0;

static bool is_blank(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != 0xFF) return false;
    }
    return true;
}

static int save_context(void)
{
    ctx.magic = CTX_MAGIC;

    // Check if meta sector is full
    if (meta_write_offset + sizeof(struct flash_ctx) > META_SECTOR_SIZE) {
        LOG_INF("Meta sector full, erasing");
        int err = flash_erase(flash_dev, META_REGION_ADDR, META_SECTOR_SIZE);
        if (err != 0) return err;
        meta_write_offset = 0;
    }

    int err = flash_write(flash_dev, META_REGION_ADDR + meta_write_offset, &ctx, sizeof(struct flash_ctx));
    if (err == 0) {
        meta_write_offset += sizeof(struct flash_ctx);
    }
    return err;
}

/* --- Public API --- */

int flash_storage_init(void)
{
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }

    // Restore Context
    struct flash_ctx temp;
    uint32_t off = 0;
    bool found = false;

    while (off + sizeof(struct flash_ctx) <= META_SECTOR_SIZE) {
        int err = flash_read(flash_dev, META_REGION_ADDR + off, &temp, sizeof(temp));
        if (err != 0) return err;

        if (temp.magic == CTX_MAGIC) {
            ctx = temp;
            found = true;
        } else if (is_blank((uint8_t *)&temp, sizeof(temp))) {
            meta_write_offset = off;
            break;
        }
        off += sizeof(struct flash_ctx);
    }

    if (!found) {
        LOG_INF("No context found. Formatting");
        ctx.magic = CTX_MAGIC;
        ctx.current_slot = 0;
        ctx.write_offset = 0;
        ctx.total_count = 0;
        memset(ctx.slot_valid, 0, sizeof(ctx.slot_valid));
        meta_write_offset = 0;
        
        // Erase first data slot to be ready
        flash_erase(flash_dev, STORAGE_START_OFFSET, SECTOR_SIZE);
        save_context();
    } else {
        if (off >= META_SECTOR_SIZE) meta_write_offset = META_SECTOR_SIZE;
    }

    LOG_INF("Storage Ready: Slot %d, Offset %d, Count %d", 
            ctx.current_slot, ctx.write_offset, ctx.total_count);
    return 0;
}

void flash_storage_write(const struct sensor_message *msg)
{
    // Check if current slot is full
    if (ctx.write_offset + ENTRY_SIZE > SECTOR_SIZE) {
        // Move to next slot
        ctx.slot_valid[ctx.current_slot] = 1; // Mark old slot as full/valid
        
        ctx.current_slot = (ctx.current_slot + 1) % USED_SLOTS;
        LOG_INF("Switching to slot %d", ctx.current_slot);
        ctx.write_offset = 0;
        
        
        // Check if the new target slot already has data (Wrapping around)
        if (ctx.slot_valid[ctx.current_slot]) {
            // A full sector of old data is getting erased
            if (ctx.total_count >= ENTRIES_PER_SECTOR) {
                ctx.total_count -= ENTRIES_PER_SECTOR;
            } else {
                ctx.total_count = 0; // Should not happen if logic is correct
            }
        }
        
        // Erase the new target slot
        uint32_t sector_addr = STORAGE_START_OFFSET + (ctx.current_slot * SECTOR_SIZE);
        flash_erase(flash_dev, sector_addr, SECTOR_SIZE);

        // Mark new slot as technically valid (in use)
        ctx.slot_valid[ctx.current_slot] = 1; 
    }

    // Write to Flash
    uint32_t addr = STORAGE_START_OFFSET + (ctx.current_slot * SECTOR_SIZE) + ctx.write_offset;
    int err = flash_write(flash_dev, addr, msg, sizeof(struct sensor_message));
    
    if (err == 0) {
        ctx.write_offset += ENTRY_SIZE;
        ctx.total_count++;
        save_context();
    } else {
        LOG_ERR("Write failed: %d", err);
    }
}

int flash_storage_read(struct sensor_message *msgs, int count)
{
    /** 
     * Simple Read: Reads 'count' messages from the CURRENT active slot (newest data).
     * Does not iterate backwards through previous slots yet.
     */
    int read = 0;
    uint32_t off = 0;
    uint32_t base_addr = STORAGE_START_OFFSET + (ctx.current_slot * SECTOR_SIZE);

    while (read < count && off < ctx.write_offset) {
        flash_read(flash_dev, base_addr + off, &msgs[read], sizeof(struct sensor_message));
        read++;
        off += ENTRY_SIZE;
    }
    return read;
}

uint32_t flash_storage_get_count(void)
{
    uint32_t cnt = 0;
    for (int i = 0; i < USED_SLOTS; i++) {
        if (ctx.slot_valid[i]) {
            if (i == ctx.current_slot) {
                cnt += (ctx.write_offset / ENTRY_SIZE);
            } else {
                cnt += ENTRIES_PER_SECTOR;
            }
        }
    }
    return cnt;
}

int flash_storage_clear(void)
{
    LOG_INF("Clearing flash");
    int err = flash_erase(flash_dev, STORAGE_START_OFFSET, (USED_SLOTS + 1) * SECTOR_SIZE);
    if (err == 0) {
        ctx.current_slot = 0;
        ctx.write_offset = 0;
        ctx.total_count = 0;
        memset(ctx.slot_valid, 0, sizeof(ctx.slot_valid));
        meta_write_offset = 0;
        save_context();
        LOG_INF("Cleared.");
    }
    return err;
}