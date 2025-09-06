#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

// GPIO state values
#define POWER_OFF   0
#define POWER_ON    1

// Mode values
#define RESTORE_MODE_OFF            0
#define RESTORE_MODE_ON             1
#define RESTORE_MODE_LAST_STATE     2

// Public API
void flash_store_init(void);

void set_power_state(uint8_t state);
uint8_t get_power_state(void);

void set_restore_mode(uint8_t mode);
uint8_t get_restore_mode(void);

#endif // FLASH_STORE_H
