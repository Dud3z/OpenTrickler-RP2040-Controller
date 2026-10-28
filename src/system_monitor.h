#ifndef SYSTEM_MONITOR_H_
#define SYSTEM_MONITOR_H_

/*
 * Watchdog, error log and the once-a-second housekeeping.
 *
 * The watchdog restarts the trickler when the firmware hangs. That matters
 * for safety too: the motors are stepped by the PIO, which keeps running at
 * the last speed when the CPU hangs; the restart stops them.
 * Crashes (assert, panic, stack overflow, out of memory, hard fault) are
 * written to RAM that survives the restart and logged in the EEPROM on the
 * next start, together with hangs caught by the watchdog.
 */

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

#define EEPROM_ERROR_LOG_REV                1       // 16 bit
#define ERROR_LOG_LEN                       10

typedef enum {
    ERROR_NONE = 0,
    ERROR_WATCHDOG = 1,         // Hang, the watchdog restarted the trickler
    ERROR_ASSERT = 2,
    ERROR_PANIC = 3,
    ERROR_STACK_OVERFLOW = 4,
    ERROR_OUT_OF_MEMORY = 5,
    ERROR_HARD_FAULT = 6,
} error_code_t;

typedef struct {
    uint16_t boot;              // Start number the error ended
    uint8_t code;               // error_code_t
    uint8_t reserved;
    uint32_t detail;            // Line (assert), address (hard fault)
    uint32_t uptime_s;          // How long the trickler ran before
    char text[16];              // File, task or message
} error_log_entry_t;

typedef struct {
    uint16_t error_log_rev;
    uint16_t boot_count;
    uint8_t next;
    uint8_t count;
    uint8_t reserved[2];
    error_log_entry_t entries[ERROR_LOG_LEN];
} eeprom_error_log_t;

#ifdef __cplusplus
extern "C" {
#endif

// Early in main(): classify the last restart and log it
bool system_monitor_init(void);

// After the scheduler started tasks may be created: watchdog and housekeeping
void system_monitor_start(void);

// Call right before a wanted restart (reboot button, firmware update)
void system_monitor_planned_reboot(void);

// Crash handlers: remember what happened and restart
void __attribute__((noreturn)) error_log_fatal(error_code_t code, uint32_t detail, const char * text);
void __attribute__((noreturn)) error_log_assert(const char * file, int line);

bool http_rest_error_log(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_MONITOR_H_
