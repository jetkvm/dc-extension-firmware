#include "flash_store.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>

#define FLASH_TARGET_OFFSET     (512 * 1024)           // Offset where entries are stored
#define ENTRY_SIZE              2                      // 1 byte for each value
#define ENTRIES_PER_SECTOR      (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)  // 16 entries
#define INVALID_BYTE            0xFF

typedef struct {
    uint8_t gpio_state;
    uint8_t mode_value;
} FlashEntry;

static FlashEntry current_entry;

// Search the last valid entry in the flash sector
static FlashEntry flash_read_last_entry(void) {
    const uint8_t *flash = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    FlashEntry entry;

    // Scan backwards to find the last valid entry
    for (int i = ENTRIES_PER_SECTOR - 1; i >= 0; i--) {
        const uint8_t *entry_ptr = flash + i * FLASH_PAGE_SIZE;

        // Check if the first byte is valid (not 0xFF)
        if (entry_ptr[0] != INVALID_BYTE) {
            memcpy(&entry, entry_ptr, ENTRY_SIZE);

            // Sanity check
            if (entry.gpio_state <= POWER_ON && entry.mode_value <= RESTORE_MODE_LAST_STATE) {
                return entry;
            }
        }
    }

    // No valid entry found – return default
    FlashEntry default_entry = { POWER_OFF, RESTORE_MODE_OFF };
    return default_entry;
}

static void flash_write_entry(uint8_t gpio_state, uint8_t mode_value) {
    const uint8_t *flash = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

    // Find the next free slot
    int entry_index = 0;
    for (; entry_index < ENTRIES_PER_SECTOR; entry_index++) {
        if (flash[entry_index * FLASH_PAGE_SIZE] == INVALID_BYTE) {
            break;
        }
    }

    // Sector full – erase and reset
    if (entry_index >= ENTRIES_PER_SECTOR) {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        entry_index = 0;
    }

    FlashEntry entry = { gpio_state, mode_value };

    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0xFF, FLASH_PAGE_SIZE);
    memcpy(buffer, &entry, ENTRY_SIZE);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + entry_index * FLASH_PAGE_SIZE, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

// Initializes the RAM copy from flash
void flash_store_init(void) {
    current_entry = flash_read_last_entry();
}

// Public API

void set_power_state(uint8_t state) {
    if (state != current_entry.gpio_state) {
        current_entry.gpio_state = state;
        flash_write_entry(current_entry.gpio_state, current_entry.mode_value);
    }
}

uint8_t get_power_state(void) {
    return current_entry.gpio_state;
}

void set_restore_mode(uint8_t mode) {
    if (mode != current_entry.mode_value) {
        current_entry.mode_value = mode;
        flash_write_entry(current_entry.gpio_state, current_entry.mode_value);
    }
}

uint8_t get_restore_mode(void) {
    return current_entry.mode_value;
}
