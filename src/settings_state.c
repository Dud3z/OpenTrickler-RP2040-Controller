#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "settings_state.h"
#include "common.h"


// Config endpoints and the settings page they belong to (web page names)
typedef struct {
    const char * path;
    const char * page;
} settings_endpoint_t;

static const settings_endpoint_t settings_endpoints[] = {
    {"/rest/charge_mode_config", "Charging"},
    {"/rest/profile_config", "Profiles"},
    {"/rest/ai_config", "AI Trickle"},
    {"/rest/scale_config", "Scale"},
    {"/rest/coarse_motor_config", "Motors"},
    {"/rest/fine_motor_config", "Motors"},
    {"/rest/servo_gate_config", "Servo Gate"},
    {"/rest/mini_12864_config", "Display & LEDs"},
    {"/rest/neopixel_led_config", "Display & LEDs"},
    {"/rest/feature_config", "Safety & options"},
    {"/rest/wireless_config", "Wireless"},
};

#define SETTINGS_ENDPOINT_CNT   (sizeof(settings_endpoints) / sizeof(settings_endpoints[0]))

static uint32_t unsaved_bits = 0;


void settings_state_note_request(const char * path, int num_params, char *params[], char *values[]) {
    int endpoint = -1;
    for (size_t i = 0; i < SETTINGS_ENDPOINT_CNT; i += 1) {
        if (strcmp(path, settings_endpoints[i].path) == 0) {
            endpoint = (int) i;
            break;
        }
    }
    if (endpoint < 0) {
        return;
    }

    bool changes = false;
    bool saved = false;
    for (int idx = 0; idx < num_params; idx += 1) {
        // pf/ro only pick the profile to read
        if (strcmp(params[idx], "ee") == 0) {
            saved = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "pf") != 0 && strcmp(params[idx], "ro") != 0) {
            changes = true;
        }
    }

    if (saved) {
        unsaved_bits &= ~(1u << endpoint);
    }
    else if (changes) {
        unsaved_bits |= 1u << endpoint;
    }
}


void settings_state_all_saved(void) {
    unsaved_bits = 0;
}


bool http_rest_unsaved(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Response: {"p": [settings pages with changes that are not saved to the EEPROM]}
    (void) num_params;
    (void) params;
    (void) values;
    static char buf[256];

    size_t pos = snprintf(buf, sizeof(buf), "%s{\"p\":[", http_json_header);
    bool comma = false;
    for (size_t i = 0; i < SETTINGS_ENDPOINT_CNT; i += 1) {
        if (!(unsaved_bits & (1u << i))) {
            continue;
        }
        // Two endpoints can share a page: name it once
        bool listed = false;
        for (size_t j = 0; j < i; j += 1) {
            if ((unsaved_bits & (1u << j)) && strcmp(settings_endpoints[j].page, settings_endpoints[i].page) == 0) {
                listed = true;
            }
        }
        if (!listed && pos < sizeof(buf) - 32) {
            pos += snprintf(&buf[pos], sizeof(buf) - pos, "%s\"%s\"", comma ? "," : "", settings_endpoints[i].page);
            comma = true;
        }
    }
    snprintf(&buf[pos], sizeof(buf) - pos, "]}");

    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
    return true;
}
