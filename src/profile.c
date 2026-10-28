#include <string.h>

#include "profile.h"
#include "eeprom.h"
#include "common.h"
#include "ai_trickle.h"
#include "charge_mode.h"


extern charge_mode_config_t charge_mode_config;


eeprom_profile_data_t profile_data;

extern void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength);

const eeprom_profile_data_t default_profile_data = {
    .profile_data_rev = 0,
    .profiles[0] = {
        .compatibility = 0,
        .name = "AR2208,gr",

        .coarse_kp = 0.025f,
        .coarse_ki = 0.0f,
        .coarse_kd = 0.3f,
        .coarse_min_flow_speed_rps = 0.1f,
        .coarse_max_flow_speed_rps = 5.0f,

        .fine_kp = 2.0f,
        .fine_ki = 0.0f,
        .fine_kd = 10.0f,
        .fine_min_flow_speed_rps = 0.08f,
        .fine_max_flow_speed_rps = 5.0f,
    },
    .profiles[1] = {
        .compatibility = 0,
        .name = "AR2209,gr",

        .coarse_kp = 0.05f,
        .coarse_ki = 0.0f,
        .coarse_kd = 0.3f,
        .coarse_min_flow_speed_rps = 0.1f,
        .coarse_max_flow_speed_rps = 8.0f,

        .fine_kp = 0.8f,
        .fine_ki = 0.0f,
        .fine_kd = 15.0f,
        .fine_min_flow_speed_rps = 0.08f,
        .fine_max_flow_speed_rps = 2.0f,
    },
    .profiles[2] = {
        .compatibility = 0,
        .name = "8208XBR,gr",

        .coarse_kp = 0.05f,
        .coarse_ki = 0.0f,
        .coarse_kd = 0.3f,
        .coarse_min_flow_speed_rps = 0.1f,
        .coarse_max_flow_speed_rps = 5.0f,

        .fine_kp = 2.0f,
        .fine_ki = 0.0f,
        .fine_kd = 12.0f,
        .fine_min_flow_speed_rps = 0.06f,
        .fine_max_flow_speed_rps = 5.0f,
    },
    .profiles[3] = {
        .compatibility = 0,
        .name = "Benchmark2,gr",

        .coarse_kp = 0.06f,
        .coarse_ki = 0.0f,
        .coarse_kd = 0.3f,
        .coarse_min_flow_speed_rps = 0.1f,
        .coarse_max_flow_speed_rps = 5.0f,

        .fine_kp = 0.8f,
        .fine_ki = 0.0f,
        .fine_kd = 15.0f,
        .fine_min_flow_speed_rps = 0.08f,
        .fine_max_flow_speed_rps = 5.0f,
    },
    .profiles[4] = {
        .compatibility = 0,
        .name = "Profile4",
    },
    .profiles[5] = {
        .compatibility = 0,
        .name = "Profile5",
    },
    .profiles[6] = {
        .compatibility = 0,
        .name = "Profile6",
    },
    .profiles[7] = {
        .compatibility = 0,
        .name = "Profile7",
    },
};


bool profile_data_save() {
    bool is_ok = save_config(EEPROM_PROFILE_DATA_BASE_ADDR, &profile_data, sizeof(profile_data));
    return is_ok;
}


bool profile_data_init() {
    bool is_ok = true;

    // Read profile index table
    memset(&profile_data, 0x0, sizeof(eeprom_profile_data_t));
    is_ok = load_config(EEPROM_PROFILE_DATA_BASE_ADDR, &profile_data, &default_profile_data, sizeof(profile_data), EEPROM_PROFILE_DATA_REV);

    if (!is_ok) {
        printf("Unable to read profile data\n");
        return false;
    }

    // Register to eeprom save all
    eeprom_register_handler(profile_data_save);

    return true;
}


uint16_t profile_get_selected_idx() {
    return profile_data.current_profile_idx;
}


profile_t * profile_get_selected() {
    return &profile_data.profiles[profile_get_selected_idx()];
}


profile_t * profile_select(uint8_t idx) {
    profile_data.current_profile_idx = idx;

    return profile_get_selected(idx);
}


void profile_update_checksum() {
    swuart_calcCRC((uint8_t *) profile_get_selected(), sizeof(profile_t));
}

bool http_rest_profile_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings:
    // pf (int): profile index
    // p0 (int): rev
    // p1 (int): compatibility
    // p2 (str): name
    // p3 (float): coarse_kp
    // p4 (float): coarse_ki
    // p5 (float): coarse_kd
    // p6 (float): coarse_min_flow_speed_rps
    // p7 (float): coarse_max_flow_speed_rps
    // p8 (float): fine_kp
    // p9 (float): fine_ki
    // p10 (float): fine_kd
    // p11 (float): fine_min_flow_speed_rps
    // p12 (float): fine_max_flow_speed_rps
    // ee (bool): save to eeprom
    // ro (bool): read or write the profile given by pf without selecting it
    //            (export / import / other pages); without it pf selects the profile
    static char buf[256];

    // Read the current loaded profile index
    uint8_t profile_idx = profile_get_selected_idx();
    bool keep_selection = false;

    // Overwrite the profile index (if applicable)
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "pf") == 0) {
            profile_idx = (uint16_t) atoi(values[idx]);
        }
        else if (strcmp(params[idx], "ro") == 0) {
            keep_selection = string_to_boolean(values[idx]);
        }
    }

    if (profile_idx >= MAX_PROFILE_CNT) {
        strcpy(buf, "{\"error\":\"InvalidProfileIndex\"}");
    }

    else {
        profile_t * current_profile = keep_selection ? &profile_data.profiles[profile_idx] : profile_select(profile_idx);
        bool save_to_eeprom = false;

        // Control
        for (int idx = 0; idx < num_params; idx += 1) {
            if (strcmp(params[idx], "p0") == 0) {
                current_profile->rev = strtol(values[idx], NULL, 10);
            }
            else if (strcmp(params[idx], "p1") == 0) {
                current_profile->compatibility = strtol(values[idx], NULL, 10);
            }
            else if (strcmp(params[idx], "p2") == 0) {
                strncpy(current_profile->name, values[idx], sizeof(current_profile->name));
            }
            else if (strcmp(params[idx], "p3") == 0) {
                current_profile->coarse_kp = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p4") == 0) {
                current_profile->coarse_ki = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p5") == 0) {
                current_profile->coarse_kd = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p6") == 0) {
                current_profile->coarse_min_flow_speed_rps = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p7") == 0) {
                current_profile->coarse_max_flow_speed_rps = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p8") == 0) {
                current_profile->fine_kp = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p9") == 0) {
                current_profile->fine_ki = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p10") == 0) {
                current_profile->fine_kd = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p11") == 0) {
                current_profile->fine_min_flow_speed_rps = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "p12") == 0) {
                current_profile->fine_max_flow_speed_rps = strtof(values[idx], NULL);
            }
            else if (strcmp(params[idx], "ee") == 0) {
                save_to_eeprom = string_to_boolean(values[idx]);
            }
        }

        // Perform action
        if (save_to_eeprom) {
            profile_data_save();
        }

        // Response
        snprintf(buf, sizeof(buf), 
                 "%s"
                 "{\"pf\":%d,\"p0\":%ld,\"p1\":%ld,\"p2\":\"%s\",\"p3\":%0.3f,\"p4\":%0.3f,\"p5\":%0.3f,\"p6\":%0.3f,\"p7\":%0.3f,\"p8\":%0.3f,\"p9\":%0.3f,\"p10\":%0.3f,\"p11\":%0.3f,\"p12\":%0.3f}",
                 http_json_header,
                 profile_idx, 
                 current_profile->rev,
                 current_profile->compatibility,
                 current_profile->name,
                 current_profile->coarse_kp,
                 current_profile->coarse_ki,
                 current_profile->coarse_kd,
                 current_profile->coarse_min_flow_speed_rps,
                 current_profile->coarse_max_flow_speed_rps,
                 current_profile->fine_kp,
                 current_profile->fine_ki,
                 current_profile->fine_kd,
                 current_profile->fine_min_flow_speed_rps,
                 current_profile->fine_max_flow_speed_rps);
    }

    size_t response_len = strlen(buf);
    file->data = buf;
    file->len = response_len;
    file->index = response_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


// A slot is free when it still holds an unused default profile
static bool profile_slot_is_free(uint8_t idx) {
    const profile_t * profile = &profile_data.profiles[idx];
    if (profile->name[0] == '\0') {
        return true;
    }
    char default_name[PROFILE_NAME_MAX_LEN];
    snprintf(default_name, sizeof(default_name), "Profile%d", idx);
    return strncmp(profile->name, default_name, PROFILE_NAME_MAX_LEN) == 0 &&
           profile->coarse_max_flow_speed_rps <= 0.0f && profile->fine_max_flow_speed_rps <= 0.0f;
}


bool http_rest_profile_new(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Create a profile from an existing one, e.g. before learning a new powder
    // Mappings
    // n (str): name of the new profile
    // from (int): profile to copy the settings from, default the selected one
    // slot (int): slot to use, default the first free one; an occupied slot is overwritten
    // ee (bool): save to eeprom
    // Response: {"pf": new index or -1, "err": ""}, the new profile is selected
    static char buf[160];
    const char * name = NULL;
    int from = profile_get_selected_idx();
    int slot = -1;
    bool save_to_eeprom = false;
    const char * error = "";

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "n") == 0) {
            name = values[idx];
        }
        else if (strcmp(params[idx], "from") == 0) {
            from = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "slot") == 0) {
            slot = atoi(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    if (slot < 0) {
        for (uint8_t idx = 0; idx < MAX_PROFILE_CNT; idx += 1) {
            if (profile_slot_is_free(idx)) {
                slot = idx;
                break;
            }
        }
    }

    if (charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
        error = "Leave charge mode first";
        slot = -1;
    }
    else if (name == NULL || name[0] == '\0') {
        error = "Name missing";
        slot = -1;
    }
    else if (from < 0 || from >= MAX_PROFILE_CNT) {
        error = "Invalid source profile";
        slot = -1;
    }
    else if (slot < 0 || slot >= MAX_PROFILE_CNT) {
        error = "No free profile slot";
        slot = -1;
    }
    else {
        if (slot != from) {
            profile_data.profiles[slot] = profile_data.profiles[from];
        }
        memset(profile_data.profiles[slot].name, 0x0, PROFILE_NAME_MAX_LEN);
        strncpy(profile_data.profiles[slot].name, name, PROFILE_NAME_MAX_LEN - 1);
        profile_select((uint8_t) slot);

        // A new powder starts with a fresh AI model
        ai_trickle_profile_reset((uint8_t) slot);

        if (save_to_eeprom) {
            profile_data_save();
            ai_trickle_config_save();
        }
    }

    snprintf(buf, sizeof(buf), "%s{\"pf\":%d,\"err\":\"%s\"}", http_json_header, slot, error);

    size_t response_len = strlen(buf);
    file->data = buf;
    file->len = response_len;
    file->index = response_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_profile_summary(struct fs_file *file, int num_params, char *params[], char *values[])
{
    // It does not take argument
    assert(MAX_PROFILE_CNT <= 8);  // Ensures 256 byte buffer us sufficient
    static char buf[256];

    // Response
    // s0 (dict): A dictionary of all profiles in {idx: name} format. 
    // s1 (int): The current loaded profile index
    memset(buf, 0x0, sizeof(buf));
    const char * item_template = "\"%d\":\"%s\",";

    // Create header
    snprintf(buf, sizeof(buf), 
             "%s{\"s0\":{",
             http_json_header);

    size_t char_idx = strlen(buf);

    // Write profile information
    for (uint8_t p_idx=0; p_idx < MAX_PROFILE_CNT; p_idx+=1) {
        snprintf(&buf[char_idx], sizeof(buf) - char_idx, 
                 item_template,
                 p_idx, &profile_data.profiles[p_idx].name);
        char_idx += strnlen((const char *) &buf[char_idx], sizeof(buf));
    }

    // Append close bracket (replace the last comma)
    buf[char_idx - 1] = '}';

    // Append s1
    snprintf(&buf[char_idx], sizeof(buf) - char_idx,
             ",\"s1\":%d}", 
             profile_data.current_profile_idx);

    size_t response_len = strlen(buf);
    file->data = buf;
    file->len = response_len;
    file->index = response_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}