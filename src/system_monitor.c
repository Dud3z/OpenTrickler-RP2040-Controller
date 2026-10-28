#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <FreeRTOS.h>
#include <task.h>

#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"

#include "system_monitor.h"
#include "common.h"
#include "eeprom.h"
#include "feature_config.h"
#include "neopixel_led.h"
#include "charge_mode.h"
#include "scale.h"
#include "fw_update.h"


extern charge_mode_config_t charge_mode_config;

#define WATCHDOG_TIMEOUT_MS         8000    // RP2040 allows at most 8.3 s
#define MONITOR_PERIOD_MS           1000
#define ACTIVITY_WEIGHT_CHANGE_GR   0.5f    // Something put on or taken off the scale
#define UPDATE_CHECK_FIRST_S        120     // Look for new firmware after the start ...
#define UPDATE_CHECK_EVERY_S        (24 * 3600)     // ... and then once a day

// What the firmware was doing, kept in RAM that survives a watchdog restart
#define CRASH_RECORD_MAGIC          0x4F54C0DEu

typedef enum {
    RECORD_RUNNING = 1,
    RECORD_PLANNED = 2,     // Restart on purpose
    RECORD_FATAL = 3,
} record_state_t;

typedef struct {
    uint32_t magic;
    uint32_t state;
    uint32_t code;
    uint32_t detail;
    uint32_t uptime_s;
    char text[16];
    uint32_t early_crashes;     // Crashes in a row shortly after the start
    uint32_t check;
} crash_record_t;

// The same crash at every start would restart the trickler endlessly (and
// write the log each time): after a few in a row it stops instead
#define EARLY_CRASH_UPTIME_S        30
#define EARLY_CRASH_LIMIT           3

static uint32_t early_crashes = 0;

static crash_record_t __uninitialized_ram(crash_record);

static eeprom_error_log_t error_log;
static uint8_t last_restart_code = ERROR_NONE;
static bool last_restart_watchdog = false;      // Restarted by the watchdog (any reason)
static bool watchdog_running = false;
static volatile bool reboot_planned = false;

static const eeprom_error_log_t default_error_log = {
    .error_log_rev = 0,
};


static uint32_t record_check(const crash_record_t * r) {
    uint32_t check = r->magic ^ (r->state << 3) ^ (r->code << 7) ^ r->detail ^ (r->uptime_s << 1) ^ (r->early_crashes << 11);
    for (size_t i = 0; i < sizeof(r->text); i += 1) {
        check = (check << 5) ^ (check >> 27) ^ (uint8_t) r->text[i];
    }
    return check;
}


static void record_set(record_state_t state, uint32_t code, uint32_t detail, const char * text) {
    crash_record.magic = CRASH_RECORD_MAGIC;
    crash_record.state = state;
    crash_record.code = code;
    crash_record.detail = detail;
    crash_record.uptime_s = to_ms_since_boot(get_absolute_time()) / 1000;
    memset(crash_record.text, 0x0, sizeof(crash_record.text));
    if (text) {
        strncpy(crash_record.text, text, sizeof(crash_record.text) - 1);
    }
    crash_record.early_crashes = early_crashes;
    crash_record.check = record_check(&crash_record);
}


static bool record_valid(void) {
    return crash_record.magic == CRASH_RECORD_MAGIC && crash_record.check == record_check(&crash_record);
}


static void error_log_add(uint8_t code, uint32_t detail, uint32_t uptime_s, const char * text) {
    error_log_entry_t * e = &error_log.entries[error_log.next];
    memset(e, 0x0, sizeof(*e));
    e->boot = error_log.boot_count - 1;
    e->code = code;
    e->detail = detail;
    e->uptime_s = uptime_s;
    strncpy(e->text, text, sizeof(e->text) - 1);
    error_log.next = (error_log.next + 1) % ERROR_LOG_LEN;
    if (error_log.count < ERROR_LOG_LEN) {
        error_log.count += 1;
    }
}


static bool error_log_save(void) {
    return save_config(EEPROM_ERROR_LOG_BASE_ADDR, &error_log, sizeof(error_log));
}


bool system_monitor_init(void) {
    bool is_ok = load_config(EEPROM_ERROR_LOG_BASE_ADDR, &error_log, &default_error_log, sizeof(error_log),
                             EEPROM_ERROR_LOG_REV);
    if (!is_ok || error_log.next >= ERROR_LOG_LEN || error_log.count > ERROR_LOG_LEN) {
        error_log = default_error_log;
    }
    error_log.boot_count += 1;

    // Only a watchdog restart keeps the RAM record; a power cycle or the
    // reset button leaves garbage that fails the check
    last_restart_watchdog = watchdog_caused_reboot();
    if (last_restart_watchdog && record_valid()) {
        crash_record.text[sizeof(crash_record.text) - 1] = '\0';
        if (crash_record.state == RECORD_FATAL) {
            // Count crashes in a row that happen right after the start
            early_crashes = crash_record.uptime_s < EARLY_CRASH_UPTIME_S ? crash_record.early_crashes + 1 : 0;
            last_restart_code = (uint8_t) crash_record.code;
            error_log_add(last_restart_code, crash_record.detail, crash_record.uptime_s, crash_record.text);
        }
        else if (crash_record.state == RECORD_RUNNING) {
            early_crashes = crash_record.uptime_s < EARLY_CRASH_UPTIME_S ? crash_record.early_crashes + 1 : 0;
            last_restart_code = ERROR_WATCHDOG;
            error_log_add(ERROR_WATCHDOG, 0, crash_record.uptime_s, "");
        }
    }
    error_log_save();

    record_set(RECORD_RUNNING, ERROR_NONE, 0, NULL);
    return is_ok;
}


void system_monitor_planned_reboot(void) {
    reboot_planned = true;
    record_set(RECORD_PLANNED, ERROR_NONE, 0, NULL);
}


void __attribute__((noreturn)) error_log_fatal(error_code_t code, uint32_t detail, const char * text) {
    // May run in an interrupt or with the scheduler broken: no RTOS calls
    save_and_disable_interrupts();
    record_set(RECORD_FATAL, code, detail, text);
    if (early_crashes + 1 >= EARLY_CRASH_LIMIT) {
        // Crashing right after every start: stay stopped (the motors are
        // only enabled in charge mode), the log tells what happened after a
        // power cycle
        hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
        while (true) {
            tight_loop_contents();
        }
    }
    // Restart at once; the reset stops the motors
    watchdog_reboot(0, 0, 1);
    while (true) {
        tight_loop_contents();
    }
}


void __attribute__((noreturn)) error_log_assert(const char * file, int line) {
    // Keep the end of the path, the file name is what helps
    const char * name = file ? file : "";
    size_t len = strlen(name);
    if (len > 15) {
        name += len - 15;
    }
    error_log_fatal(ERROR_ASSERT, (uint32_t) line, name);
}


// newlib assert()
void __assert_func(const char * file, int line, const char * func, const char * expr) {
    (void) func;
    (void) expr;
    error_log_assert(file, line);
}


// panic() of the Pico SDK (PICO_PANIC_FUNCTION in CMakeLists.txt)
void __attribute__((noreturn)) error_log_panic(const char * fmt, ...) {
    // The format string tells which panic it was
    error_log_fatal(ERROR_PANIC, 0, fmt ? fmt : "");
}


// Hard fault: the stacked PC says where it happened
void __attribute__((used, noreturn)) error_log_hard_fault(uint32_t * frame) {
    error_log_fatal(ERROR_HARD_FAULT, frame[6], "");
}

void __attribute__((naked)) isr_hardfault(void) {
    pico_default_asm_volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1:\n"
        "mrs r0, msp\n"
        "2:\n"
        "ldr r1, =error_log_hard_fault\n"
        "bx r1\n"
    );
}


// FreeRTOS hooks (were endless asserts before)
void vApplicationMallocFailedHook(void) {
    error_log_fatal(ERROR_OUT_OF_MEMORY, xPortGetFreeHeapSize(), "");
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char * pcTaskName) {
    (void) pxTask;
    error_log_fatal(ERROR_STACK_OVERFLOW, 0, pcTaskName ? pcTaskName : "");
}


static void watchdog_set(bool enable) {
    // Hanging right after every start: no more restarts (see EARLY_CRASH_LIMIT)
    if (early_crashes >= EARLY_CRASH_LIMIT) {
        enable = false;
    }
    if (enable && !watchdog_running) {
        // Paused while a debugger halts the CPU
        watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
        watchdog_running = true;
    }
    else if (!enable && watchdog_running) {
        hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
        watchdog_running = false;
    }
}


// Lowest priority above idle: when this task starves, the firmware is stuck
static void system_monitor_task(void * p) {
    (void) p;
    float last_weight = NAN;
    charge_mode_state_t last_state = CHARGE_MODE_EXIT;
    uint32_t next_update_check_s = UPDATE_CHECK_FIRST_S;

    while (true) {
        // A wanted restart is on its way (it uses the watchdog itself)
        if (reboot_planned) {
            vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
            continue;
        }

        watchdog_set(feature_config.watchdog);
        if (watchdog_running) {
            watchdog_update();
        }
        if (!reboot_planned) {
            record_set(RECORD_RUNNING, ERROR_NONE, 0, NULL);
        }

        // Dimming: a charge, a change of the charge state or something put
        // on the scale counts as use
        charge_mode_state_t state = charge_mode_config.charge_mode_state;
        bool busy = state == CHARGE_MODE_WAIT_FOR_COMPLETE;
        if (state != last_state) {
            neopixel_led_activity();
            last_state = state;
        }
        float weight = scale_get_current_measurement();
        if (!isnan(weight) && !isinf(weight)) {
            if (!isnan(last_weight) && fabsf(weight - last_weight) > ACTIVITY_WEIGHT_CHANGE_GR) {
                neopixel_led_activity();
            }
            last_weight = weight;
        }
        neopixel_led_idle_check(busy);

        // New firmware on GitHub (Pico 2 W): shown in the menu. Never while
        // charging, and not when the display menu checks itself
        uint32_t uptime_s = to_ms_since_boot(get_absolute_time()) / 1000;
        if (uptime_s >= next_update_check_s) {
            next_update_check_s = uptime_s + UPDATE_CHECK_EVERY_S;
            fw_download_status_t st;
            fw_download_status(&st);
            if (feature_config.update_notice && fw_download_supported() && state == CHARGE_MODE_EXIT &&
                st.state != FW_DOWNLOAD_CHECKING && st.state != FW_DOWNLOAD_DOWNLOADING && st.state != FW_DOWNLOAD_DONE) {
                fw_download_check();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}


void system_monitor_start(void) {
    xTaskCreate(system_monitor_task, "System Monitor", 512, NULL, 1, NULL);
}


static const char * error_code_name(uint8_t code) {
    switch (code) {
        case ERROR_WATCHDOG: return "watchdog";
        case ERROR_ASSERT: return "assert";
        case ERROR_PANIC: return "panic";
        case ERROR_STACK_OVERFLOW: return "stack";
        case ERROR_OUT_OF_MEMORY: return "memory";
        case ERROR_HARD_FAULT: return "fault";
        default: return "none";
    }
}


// Keep the JSON valid whatever the text holds
static void json_text(char * dst, size_t size, const char * src) {
    size_t n = 0;
    for (; *src && n < size - 1; src += 1) {
        unsigned char c = (unsigned char) *src;
        dst[n++] = (c < 0x20 || c == '"' || c == '\\' || c > 0x7e) ? '?' : (char) c;
    }
    dst[n] = '\0';
}


bool http_rest_error_log(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // clr (bool): clear the log
    // Response: {"bc": starts, "lr": reason of the last start ("power", "restart" or an error),
    //            "up": uptime s, "wd": watchdog running, "heap": free heap bytes,
    //            "e": [[start, error, detail, uptime s, text], ...] the newest first}
    static char buf[160 + ERROR_LOG_LEN * 64];

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "clr") == 0 && string_to_boolean(values[idx])) {
            error_log.count = 0;
            error_log.next = 0;
            memset(error_log.entries, 0x0, sizeof(error_log.entries));
            error_log_save();
        }
    }

    const char * last_reason = last_restart_code != ERROR_NONE ? error_code_name(last_restart_code) :
                               (last_restart_watchdog ? "restart" : "power");
    size_t pos = snprintf(buf, sizeof(buf), "%s{\"bc\":%u,\"lr\":\"%s\",\"up\":%lu,\"wd\":%s,\"heap\":%u,\"e\":[",
                          http_json_header, error_log.boot_count, last_reason,
                          (unsigned long) (to_ms_since_boot(get_absolute_time()) / 1000),
                          boolean_to_string(watchdog_running), (unsigned) xPortGetFreeHeapSize());

    for (uint8_t i = 0; i < error_log.count && pos < sizeof(buf) - 72; i += 1) {
        uint8_t idx = (error_log.next + ERROR_LOG_LEN - 1 - i) % ERROR_LOG_LEN;
        const error_log_entry_t * e = &error_log.entries[idx];
        char text[sizeof(e->text)];
        char safe[sizeof(e->text)];
        memcpy(text, e->text, sizeof(text));
        text[sizeof(text) - 1] = '\0';
        json_text(safe, sizeof(safe), text);
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "%s[%u,\"%s\",%lu,%lu,\"%s\"]",
                        i ? "," : "", e->boot, error_code_name(e->code),
                        (unsigned long) e->detail, (unsigned long) e->uptime_s, safe);
    }
    snprintf(&buf[pos], sizeof(buf) - pos, "]}");

    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}
