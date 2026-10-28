#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "feature_config.h"
#include "common.h"
#include "eeprom.h"
#include "motors.h"
#include "neopixel_led.h"


eeprom_feature_config_t feature_config;

static const eeprom_feature_config_t default_feature_config = {
    .feature_config_rev = 0,

    .pan_guard = true,
    .bump_guard = true,
    .bump_drop_cgr = 50,

    .idle_current = true,
    .idle_current_pct = 30,

    .dim_enable = true,
    .dim_after_s = 300,
    .dim_level_pct = 10,

    .sound = true,
    .update_notice = true,

    .watchdog = true,
};


bool feature_config_save(void) {
    return save_config(EEPROM_FEATURE_CONFIG_BASE_ADDR, &feature_config, sizeof(feature_config));
}


static void feature_config_limit(void) {
    if (feature_config.bump_drop_cgr < 10) {
        feature_config.bump_drop_cgr = 10;
    }
    if (feature_config.idle_current_pct < 10 || feature_config.idle_current_pct > 100) {
        feature_config.idle_current_pct = default_feature_config.idle_current_pct;
    }
    if (feature_config.dim_after_s < 10) {
        feature_config.dim_after_s = 10;
    }
    if (feature_config.dim_level_pct > 100) {
        feature_config.dim_level_pct = default_feature_config.dim_level_pct;
    }
}


bool feature_config_init(void) {
    bool is_ok = load_config(EEPROM_FEATURE_CONFIG_BASE_ADDR, &feature_config, &default_feature_config,
                             sizeof(feature_config), EEPROM_FEATURE_CONFIG_REV);
    if (!is_ok) {
        printf("Unable to read the feature configuration\n");
        feature_config = default_feature_config;
    }
    feature_config_limit();

    eeprom_register_handler(feature_config_save);
    return is_ok;
}


bool http_rest_feature_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // f0 (bool): pan_guard
    // f1 (bool): bump_guard
    // f2 (float): bump drop in gr (0.1 ... 2.5)
    // f3 (bool): idle_current
    // f4 (int): idle_current_pct (10 ... 100)
    // f5 (bool): dim_enable
    // f6 (int): dim_after_s
    // f7 (int): dim_level_pct (0 ... 100)
    // f8 (bool): sound
    // f9 (bool): update_notice
    // f10 (bool): watchdog (takes effect after a restart when switched off)
    // ee (bool): save to eeprom
    static char buf[256];
    bool save_to_eeprom = false;
    bool idle_changed = false;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "f0") == 0) {
            feature_config.pan_guard = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "f1") == 0) {
            feature_config.bump_guard = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "f2") == 0) {
            float drop = strtof(values[idx], NULL);
            if (drop >= 0.1f && drop <= 2.5f) {
                feature_config.bump_drop_cgr = (uint8_t) (drop * 100.0f + 0.5f);
            }
        }
        else if (strcmp(params[idx], "f3") == 0) {
            bool v = string_to_boolean(values[idx]);
            idle_changed |= v != feature_config.idle_current;
            feature_config.idle_current = v;
        }
        else if (strcmp(params[idx], "f4") == 0) {
            int v = atoi(values[idx]);
            if (v >= 10 && v <= 100) {
                idle_changed |= v != feature_config.idle_current_pct;
                feature_config.idle_current_pct = (uint8_t) v;
            }
        }
        else if (strcmp(params[idx], "f5") == 0) {
            feature_config.dim_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "f6") == 0) {
            int v = atoi(values[idx]);
            if (v >= 10 && v <= 3600) {
                feature_config.dim_after_s = (uint16_t) v;
            }
        }
        else if (strcmp(params[idx], "f7") == 0) {
            int v = atoi(values[idx]);
            if (v >= 0 && v <= 100) {
                feature_config.dim_level_pct = (uint8_t) v;
            }
        }
        else if (strcmp(params[idx], "f8") == 0) {
            feature_config.sound = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "f9") == 0) {
            feature_config.update_notice = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "f10") == 0) {
            feature_config.watchdog = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    if (idle_changed) {
        motor_apply_standstill_current();
    }
    // A changed dimming applies right away (and wakes the display)
    neopixel_led_activity();

    if (save_to_eeprom) {
        feature_config_save();
    }

    snprintf(buf, sizeof(buf),
             "%s{\"f0\":%s,\"f1\":%s,\"f2\":%.2f,\"f3\":%s,\"f4\":%u,\"f5\":%s,\"f6\":%u,\"f7\":%u,\"f8\":%s,\"f9\":%s,\"f10\":%s}",
             http_json_header,
             boolean_to_string(feature_config.pan_guard),
             boolean_to_string(feature_config.bump_guard),
             feature_config.bump_drop_cgr * 0.01f,
             boolean_to_string(feature_config.idle_current),
             feature_config.idle_current_pct,
             boolean_to_string(feature_config.dim_enable),
             feature_config.dim_after_s,
             feature_config.dim_level_pct,
             boolean_to_string(feature_config.sound),
             boolean_to_string(feature_config.update_notice),
             boolean_to_string(feature_config.watchdog));

    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}
