#ifndef FEATURE_CONFIG_H_
#define FEATURE_CONFIG_H_

/*
 * Switches of the optional functions: charge guards (pan removed, bumps),
 * reduced motor standstill current, display dimming, result sound on the web
 * page, firmware update notice and the watchdog. Own EEPROM block, so the
 * older blocks keep their layout.
 */

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

#define EEPROM_FEATURE_CONFIG_REV           1       // 16 bit

typedef struct {
    uint16_t feature_config_rev;

    // Charging
    bool pan_guard;                 // Stop the motors when the pan is lifted during a charge
    bool bump_guard;                // Pause when the scale reading drops (bumped table), resume when stable
    uint8_t bump_drop_cgr;          // Drop that counts as a bump, 0.01 gr

    // Motors
    bool idle_current;              // Reduced current while a motor stands still
    uint8_t idle_current_pct;       // Standstill current in % of the run current

    // Display and LEDs
    bool dim_enable;                // Dim backlight and LEDs after a while without use
    uint16_t dim_after_s;
    uint8_t dim_level_pct;          // Brightness while dimmed

    // Web page
    bool sound;                     // Sound on the web page when a charge is done
    bool update_notice;             // The web page looks for new firmware

    // System
    bool watchdog;                  // Restart when the firmware hangs (motors stop)

    uint8_t reserved[16];
} eeprom_feature_config_t;

#ifdef __cplusplus
extern "C" {
#endif

extern eeprom_feature_config_t feature_config;

bool feature_config_init(void);
bool feature_config_save(void);

bool http_rest_feature_config(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // FEATURE_CONFIG_H_
