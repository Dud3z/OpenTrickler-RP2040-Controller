#ifndef SETTINGS_STATE_H_
#define SETTINGS_STATE_H_

/*
 * Settings applied over REST without saving them to the EEPROM are lost on
 * the next restart. The REST dispatcher reports every config request here;
 * the web page shows which pages still need saving.
 */

#include <stdbool.h>
#include "http_rest.h"

#ifdef __cplusplus
extern "C" {
#endif

void settings_state_note_request(const char * path, int num_params, char *params[], char *values[]);
void settings_state_all_saved(void);

bool http_rest_unsaved(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // SETTINGS_STATE_H_
