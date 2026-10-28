#ifndef RECIPE_H_
#define RECIPE_H_

/*
 * Recipes (load data): a name, a note, the charge weight and the powder
 * profile. Applying a recipe selects the profile and sets the charge weight
 * in one step, from the web page or the display.
 */

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

#define EEPROM_RECIPE_DATA_REV              1       // 16 bit

#define MAX_RECIPE_CNT                      16
#define RECIPE_NAME_LEN                     20
#define RECIPE_NOTE_LEN                     28
#define RECIPE_NONE                         0xFF

typedef struct {
    char name[RECIPE_NAME_LEN];         // e.g. ".308 Match"
    char note[RECIPE_NOTE_LEN];         // e.g. "175 SMK, CCI BR2, Lapua"
    float target_gr;
    uint8_t profile_idx;
    uint8_t reserved[3];
} recipe_t;

typedef struct {
    uint16_t recipe_data_rev;
    uint8_t count;                      // Recipes are kept packed at the start
    uint8_t reserved;
    recipe_t recipes[MAX_RECIPE_CNT];
} eeprom_recipe_data_t;


#ifdef __cplusplus
extern "C" {
#endif

bool recipe_init(void);
bool recipe_config_save(void);

uint8_t recipe_count(void);
const recipe_t * recipe_get(uint8_t idx);

// Select the profile and set the charge weight. Refused while charging.
bool recipe_apply(uint8_t idx);

// The recipe a charge belongs to: the applied recipe while its profile and
// charge weight are still in use, otherwise RECIPE_NONE
uint8_t recipe_active_for(uint8_t profile_idx, float target_gr);

// Display menu: recipe list and the selected entry
extern uint16_t recipe_menu_idx;
const char * recipe_menu_name(void * data, uint16_t idx);
uint16_t recipe_menu_count(void);
void recipe_menu_apply(void);

bool http_rest_recipes(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // RECIPE_H_
