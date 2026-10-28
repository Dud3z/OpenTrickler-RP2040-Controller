#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "recipe.h"
#include "common.h"
#include "eeprom.h"
#include "profile.h"
#include "charge_mode.h"


extern charge_mode_config_t charge_mode_config;
extern uint8_t charge_weight_digits[];

static eeprom_recipe_data_t recipe_data;
static uint8_t applied_idx = RECIPE_NONE;

uint16_t recipe_menu_idx = 0;

static const eeprom_recipe_data_t default_recipe_data = {
    .recipe_data_rev = 0,
    .count = 0,
};


bool recipe_config_save(void) {
    return save_config(EEPROM_RECIPE_BASE_ADDR, &recipe_data, sizeof(recipe_data));
}


bool recipe_init(void) {
    bool is_ok = load_config(EEPROM_RECIPE_BASE_ADDR, &recipe_data, &default_recipe_data,
                             sizeof(recipe_data), EEPROM_RECIPE_DATA_REV);
    if (!is_ok) {
        printf("Unable to read recipes\n");
        recipe_data = default_recipe_data;
    }
    if (recipe_data.count > MAX_RECIPE_CNT) {
        recipe_data.count = 0;
    }
    for (int i = 0; i < recipe_data.count; i += 1) {
        recipe_t * r = &recipe_data.recipes[i];
        r->name[RECIPE_NAME_LEN - 1] = '\0';
        r->note[RECIPE_NOTE_LEN - 1] = '\0';
        if (r->profile_idx >= MAX_PROFILE_CNT) {
            r->profile_idx = 0;
        }
        if (isnan(r->target_gr) || r->target_gr < 0.0f) {
            r->target_gr = 0.0f;
        }
    }

    eeprom_register_handler(recipe_config_save);
    return is_ok;
}


uint8_t recipe_count(void) {
    return recipe_data.count;
}


const recipe_t * recipe_get(uint8_t idx) {
    return idx < recipe_data.count ? &recipe_data.recipes[idx] : NULL;
}


// The display builds the charge weight from its digits
static void set_charge_weight_digits(float target_gr) {
    float scale = charge_mode_config.eeprom_charge_mode_data.decimal_places == DP_3 ? 1000.0f : 100.0f;
    uint32_t value = (uint32_t) (target_gr * scale + 0.5f);
    for (int i = 0; i < 5; i += 1) {
        charge_weight_digits[i] = value % 10;
        value /= 10;
    }
}


bool recipe_apply(uint8_t idx) {
    const recipe_t * r = recipe_get(idx);
    if (r == NULL || charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        return false;
    }
    profile_select(r->profile_idx);
    charge_mode_config.target_charge_weight = r->target_gr;
    set_charge_weight_digits(r->target_gr);
    applied_idx = idx;
    return true;
}


uint8_t recipe_active_for(uint8_t profile_idx, float target_gr) {
    const recipe_t * r = recipe_get(applied_idx);
    if (r == NULL || r->profile_idx != profile_idx || fabsf(r->target_gr - target_gr) > 0.0005f) {
        return RECIPE_NONE;
    }
    return applied_idx;
}


const char * recipe_menu_name(void * data, uint16_t idx) {
    const recipe_t * r = recipe_get((uint8_t) idx);
    return r ? r->name : "(none, add on web)";
}


uint16_t recipe_menu_count(void) {
    return recipe_data.count ? recipe_data.count : 1;
}


void recipe_menu_apply(void) {
    recipe_apply((uint8_t) recipe_menu_idx);
}


static void rest_set_response(struct fs_file *file, const char * buf) {
    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
}


// Keep the JSON output valid: no quotes, backslashes or control characters
static void copy_text(char * dst, const char * src, size_t size) {
    size_t n = 0;
    for (; *src && n < size - 1; src += 1) {
        unsigned char c = (unsigned char) *src;
        if (c < 0x20 || c == '"' || c == '\\') {
            continue;
        }
        dst[n++] = (char) c;
    }
    dst[n] = '\0';
}


bool http_rest_recipes(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // i (int): recipe to change, count = add a new one
    // n (str): name, no (str): note, t (float): charge weight, p (int): profile
    // del (int): delete this recipe
    // up (int): move this recipe one up
    // apply (int): select the profile and set the charge weight of this recipe
    // ee (bool): save to eeprom
    // Response: {"a": applied recipe or -1, "ok": last action worked, "r": [[name, note, weight, profile], ...]}
    static char buf[64 + MAX_RECIPE_CNT * (RECIPE_NAME_LEN + RECIPE_NOTE_LEN + 32)];
    bool save_to_eeprom = false;
    bool ok = true;
    int edit = -1;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "i") == 0) {
            edit = atoi(values[idx]);
            if (edit < 0 || edit > recipe_data.count || edit >= MAX_RECIPE_CNT) {
                edit = -1;
                ok = false;
            }
            else if (edit == recipe_data.count) {
                // New recipe: start from the current profile and charge weight
                recipe_t * r = &recipe_data.recipes[edit];
                memset(r, 0x0, sizeof(recipe_t));
                snprintf(r->name, sizeof(r->name), "Recipe %d", edit + 1);
                r->profile_idx = (uint8_t) profile_get_selected_idx();
                r->target_gr = charge_mode_config.target_charge_weight;
                recipe_data.count += 1;
            }
        }
        else if (edit >= 0 && strcmp(params[idx], "n") == 0) {
            copy_text(recipe_data.recipes[edit].name, values[idx], RECIPE_NAME_LEN);
        }
        else if (edit >= 0 && strcmp(params[idx], "no") == 0) {
            copy_text(recipe_data.recipes[edit].note, values[idx], RECIPE_NOTE_LEN);
        }
        else if (edit >= 0 && strcmp(params[idx], "t") == 0) {
            float t = strtof(values[idx], NULL);
            if (t >= 0.0f && t < 1000.0f) {
                recipe_data.recipes[edit].target_gr = t;
            }
        }
        else if (edit >= 0 && strcmp(params[idx], "p") == 0) {
            int p = atoi(values[idx]);
            if (p >= 0 && p < MAX_PROFILE_CNT) {
                recipe_data.recipes[edit].profile_idx = (uint8_t) p;
            }
        }
        else if (strcmp(params[idx], "del") == 0) {
            int d = atoi(values[idx]);
            if (d >= 0 && d < recipe_data.count) {
                memmove(&recipe_data.recipes[d], &recipe_data.recipes[d + 1],
                        (recipe_data.count - d - 1) * sizeof(recipe_t));
                recipe_data.count -= 1;
                if (applied_idx == d) {
                    applied_idx = RECIPE_NONE;
                }
                else if (applied_idx != RECIPE_NONE && applied_idx > d) {
                    applied_idx -= 1;
                }
            }
            else {
                ok = false;
            }
        }
        else if (strcmp(params[idx], "up") == 0) {
            int u = atoi(values[idx]);
            if (u > 0 && u < recipe_data.count) {
                recipe_t tmp = recipe_data.recipes[u - 1];
                recipe_data.recipes[u - 1] = recipe_data.recipes[u];
                recipe_data.recipes[u] = tmp;
                if (applied_idx == u) {
                    applied_idx = u - 1;
                }
                else if (applied_idx == u - 1) {
                    applied_idx = u;
                }
            }
        }
        else if (strcmp(params[idx], "apply") == 0) {
            ok = recipe_apply((uint8_t) atoi(values[idx]));
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    if (save_to_eeprom) {
        recipe_config_save();
    }

    size_t pos = snprintf(buf, sizeof(buf), "%s{\"a\":%d,\"ok\":%s,\"r\":[", http_json_header,
                          applied_idx == RECIPE_NONE ? -1 : applied_idx, boolean_to_string(ok));
    for (int i = 0; i < recipe_data.count && pos < sizeof(buf); i += 1) {
        const recipe_t * r = &recipe_data.recipes[i];
        pos += snprintf(&buf[pos], sizeof(buf) - pos, "%s[\"%s\",\"%s\",%.3f,%u]",
                        i ? "," : "", r->name, r->note, r->target_gr, r->profile_idx);
    }
    if (pos < sizeof(buf)) {
        snprintf(&buf[pos], sizeof(buf) - pos, "]}");
    }

    rest_set_response(file, buf);
    return true;
}
