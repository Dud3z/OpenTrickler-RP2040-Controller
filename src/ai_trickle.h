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


#define EEPROM_AI_TRICKLE_DATA_REV          3       // 16 bit

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
#define AI_TAP_DEFAULT_DEGREES              20
#define AI_FINE_REVERSE_DEFAULT_REVS        0.1f
#define AI_FINE_REVERSE_OFF                 255


typedef enum {
    AI_PRESET_PRECISION = 0,
    AI_PRESET_BALANCED = 1,
    AI_PRESET_SPEED = 2,
    AI_PRESET_AUTO = 3,
    AI_PRESET_CUSTOM = 4,
} ai_trickle_preset_t;


// Tolerance choices of the display menu, in 0.01 grain
#define AI_TOLERANCE_CHOICES                {1, 2, 3, 4, 5, 6, 8, 10, 15, 20}
#define AI_TOLERANCE_CHOICE_COUNT           10


typedef struct {
    uint16_t ai_trickle_data_rev;

    uint8_t enable;                 // Use the adaptive controller (0/1)
    uint8_t precision_level;        // Manual speed: 0 = careful ... 100 = fastest
    uint8_t auto_learn;             // Keep learning during normal charging (0/1)
    uint8_t learn_throws;           // Throws per learning session
    uint8_t mode_preset;            // ai_trickle_preset_t, for the display menu
    uint8_t reserved;

    trickle_model_t models[MAX_PROFILE_CNT];

    // Profile name each model was learned with; a renamed profile starts fresh
    char model_names[MAX_PROFILE_CNT][PROFILE_NAME_MAX_LEN];

    // Since rev 2, a rev 1 block is migrated by ai_trickle_init()
    float tolerance_gr;             // Accepted error of an adaptive charge, independent of the speed
    uint8_t auto_speed;             // Tune the speed of every profile automatically (0/1)
    uint8_t tap_degrees;            // Angle of the fine trickler per tap, 0 = default (since rev 2, was reserved)
    uint8_t fine_reverse_centirev;  // Fine tube reverse after each fine stop in 0.01 rev, 0 = default, 255 = off
    uint8_t reserved2[1];
    trickle_speed_state_t speed[MAX_PROFILE_CNT];   // Automatic speed per profile

    // Since rev 3, a rev 2 block is migrated by ai_trickle_init()
    float kernel_gr[MAX_PROFILE_CNT];   // Estimated weight of a kernel (or typical clump) per profile, 0 = unknown
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
uint8_t ai_trickle_speed(void);
float ai_trickle_tap_degrees(void);
float ai_trickle_fine_reverse_revs(void);
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
void ai_trickle_menu_sync(void);
uint8_t ai_trickle_menu_reset_model(void);
void ai_trickle_profile_reset(uint8_t idx);
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
