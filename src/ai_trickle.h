#ifndef AI_TRICKLE_H_
#define AI_TRICKLE_H_

/*
 * AI Trickle: adaptive, self learning charge controller
 *
 * Glue between the pure model in trickle_model.c and the controller: per
 * profile powder models in EEPROM, learning sessions, throw history and the
 * REST interface used by the AI web page.
 */

#include <stdint.h>
#include <stdbool.h>

#include "http_rest.h"
#include "profile.h"
#include "trickle_model.h"


#define EEPROM_AI_TRICKLE_DATA_REV          1       // 16 bit

#define AI_TRICKLE_HISTORY_LEN              32
#define AI_TRICKLE_DEFAULT_LEARN_THROWS     8
#define AI_TRICKLE_MAX_LEARN_THROWS         20

// Throw history flags
#define AI_THROW_FLAG_ADAPTIVE              (1 << 0)
#define AI_THROW_FLAG_LEARN                 (1 << 1)
#define AI_THROW_FLAG_PROBE                 (1 << 2)
#define AI_THROW_FLAG_FAULT                 (1 << 3)
#define AI_THROW_FLAG_MODEL_UPDATED         (1 << 4)
#define AI_THROW_FLAG_SERIES                (1 << 5)

#define AI_SERIES_MAX_THROWS                100
#define AI_LADDER_MAX_STEP_GR               2.0f


typedef enum {
    AI_PRESET_PRECISION = 0,
    AI_PRESET_BALANCED = 1,
    AI_PRESET_SPEED = 2,
    AI_PRESET_CUSTOM = 3,
} ai_trickle_preset_t;


typedef struct {
    uint16_t ai_trickle_data_rev;

    uint8_t enable;                 // Use the adaptive controller (0/1)
    uint8_t precision_level;        // 0 = precision ... 100 = speed
    uint8_t auto_learn;             // Keep learning during normal charging (0/1)
    uint8_t learn_throws;           // Throws per learning session
    uint8_t mode_preset;            // ai_trickle_preset_t, for the display menu
    uint8_t reserved;

    trickle_model_t models[MAX_PROFILE_CNT];

    // Profile name each model was learned with; a renamed profile starts fresh
    char model_names[MAX_PROFILE_CNT][PROFILE_NAME_MAX_LEN];
} eeprom_ai_trickle_data_t;


typedef struct {
    uint32_t seq;
    float target_gr;
    float final_gr;
    float landing_gr;
    float time_s;
    uint8_t profile_idx;
    uint8_t level;
    uint8_t topups;
    uint8_t flags;
} ai_trickle_history_entry_t;


#ifdef __cplusplus
extern "C" {
#endif

bool ai_trickle_init(void);
bool ai_trickle_config_save(void);

// Charge mode integration
bool ai_trickle_adaptive_active(void);
float ai_trickle_tolerance(void);
float ai_trickle_last_error(void);

void ai_trickle_classic_begin(void);
void ai_trickle_classic_sample(float t, float weight_gr, float coarse_rps, float fine_rps);
void ai_trickle_classic_coarse_stop(float t);
void ai_trickle_classic_end(float target_gr, float settled_gr, float charge_time_s);

void ai_trickle_adaptive_begin(float target_gr, const trickle_motor_limits_t * limits, float current_weight_gr);
trickle_phase_t ai_trickle_adaptive_step(float t, float weight_gr, float * coarse_rps, float * fine_rps,
                                         uint32_t * events);
void ai_trickle_adaptive_resume(float t, float current_weight_gr);
void ai_trickle_adaptive_end(float charge_time_s);
const char * ai_trickle_phase_name(trickle_phase_t phase);

// Learning session
void ai_trickle_learn_start(uint8_t throws);
void ai_trickle_learn_stop(void);
bool ai_trickle_learn_active(void);
bool ai_trickle_learn_just_finished(void);
uint8_t ai_trickle_learn_done(void);
uint8_t ai_trickle_learn_total(void);

// Series (n charges at one weight) and ladder (weight increases step by step)
bool ai_trickle_series_active(void);
void ai_trickle_series_stop(void);
bool ai_trickle_series_is_ladder(void);
bool ai_trickle_series_just_finished(void);
uint8_t ai_trickle_series_done(void);
uint8_t ai_trickle_series_total(void);

// Display menu actions, return the form to go back to
uint8_t ai_trickle_menu_apply_preset(void);
uint8_t ai_trickle_menu_reset_model(void);
const trickle_model_t * ai_trickle_selected_model(void);

// REST
bool http_ai_trickle_page(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_model(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_status(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_history(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_trace(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_learn(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ai_series(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // AI_TRICKLE_H_
