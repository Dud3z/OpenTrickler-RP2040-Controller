#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <u8g2.h>
#include <math.h>

#include "app.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "profile.h"
#include "common.h"
#include "servo_gate.h"
#include "ai_trickle.h"
#include "feature_config.h"
#include "charge_guard.h"


uint8_t charge_weight_digits[] = {0, 0, 0, 0, 0};

charge_mode_config_t charge_mode_config;

// Scale related
extern scale_config_t scale_config;
extern servo_gate_t servo_gate;


const eeprom_charge_mode_data_t default_charge_mode_data = {
    .charge_mode_data_rev = 0,

    .coarse_stop_threshold = 5,
    .fine_stop_threshold = 0.03,

    .set_point_sd_margin = 0.02,
    .set_point_mean_margin = 0.02,
    .coarse_stop_gate_ratio = 0,   // NEW
    .coarse_stop_backoff_enable = true,
    .coarse_stop_backoff_turns = 0.25f,
    .coarse_stop_backoff_speed_rps = 1.0f,
    .decimal_places = DP_2,

    // Precharges
    .precharge_enable = false,
    .precharge_time_ms = 1000,
    .precharge_speed_rps = 2,

    // LED related
    .neopixel_normal_charge_colour = RGB_COLOUR_GREEN,        // green
    .neopixel_under_charge_colour = RGB_COLOUR_YELLOW,        // yellow
    .neopixel_over_charge_colour = RGB_COLOUR_RED,            // red
    .neopixel_not_ready_colour = RGB_COLOUR_BLUE,             // blue
};

// Configures
TaskHandle_t scale_measurement_render_task_handler = NULL;
static char title_string[30];
static char status_string[32];  // Line below the weight: phase while charging, result afterwards

static TickType_t charge_start_tick = 0;
static float last_charge_elapsed_seconds = 0.0f;
static bool last_charge_adaptive = false;

// Coarse trickler end-of-trickle backoff
#define COARSE_STOP_BACKOFF_MAX_MS 15000  // safety cap

static bool coarse_backoff_in_progress = false;
static TickType_t coarse_backoff_end_tick = 0;

// Tap: turn the fine trickler by a small angle to add a kernel by hand
#define CHARGE_MODE_TAP_SPEED_RPS       1.0f    // Slow enough for single kernels, fast enough to be precise in time
#define CHARGE_MODE_TAP_SETTLE_MS       1500    // Wait for the kernels to land before judging again
#define CHARGE_MODE_TAP_MAX_QUEUE       5
#define CHARGE_MODE_FINE_REVERSE_RPS    1.0f    // Speed of the fine tube reverse after a fine stop
static volatile uint8_t tap_requests = 0;
static volatile bool tap_allowed = false;

static void charge_mode_get_trickler_limits(trickle_motor_limits_t * limits);

static void charge_mode_tap_fine_trickler() {
    float degrees = ai_trickle_tap_degrees();

    // Stay within what the motor and the profile allow
    trickle_motor_limits_t limits;
    charge_mode_get_trickler_limits(&limits);
    float speed = fminf(fmaxf(CHARGE_MODE_TAP_SPEED_RPS, limits.fine_min_rps), limits.fine_max_rps);

    uint32_t duration_ms = (uint32_t) (degrees / 360.0f / speed * 1000.0f);
    if (duration_ms < 1) {
        duration_ms = 1;
    }

    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, speed);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
}

// Fine tube reverse right after a fine stop, timed exactly (the charge loop
// only runs at the scale rate, which made the reverse far too long)
static void charge_mode_reverse_fine_trickler(float revs, const trickle_motor_limits_t * limits) {
    float speed = fminf(fmaxf(CHARGE_MODE_FINE_REVERSE_RPS, limits->fine_min_rps), limits->fine_max_rps);
    if (revs <= 0.0f || speed <= 0.0f) {
        return;
    }

    uint32_t duration_ms = (uint32_t) (revs / speed * 1000.0f + 0.5f);
    if (duration_ms < 1) {
        duration_ms = 1;
    }

    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, -speed);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
}

static void coarse_trickler_backoff(float min_output_speed_rps) {
    if (!charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable) {
        return;
    }

    float backoff_turns = charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns;
    if (backoff_turns <= 0.0f) {
        return;
    }

    float configured_speed = charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps;
    float backoff_speed = configured_speed;
    if (backoff_speed <= 0.0f) {
        backoff_speed = min_output_speed_rps;
    }

    if (backoff_speed <= 0.0f) {
        return;
    }

    float backoff_time_s = backoff_turns / fabsf(backoff_speed);
    uint32_t backoff_time_ms = (uint32_t)(backoff_time_s * 1000.0f);

    if (backoff_time_ms == 0) {
        return;
    }

    if (backoff_time_ms > COARSE_STOP_BACKOFF_MAX_MS) {
        backoff_time_ms = COARSE_STOP_BACKOFF_MAX_MS;
    }

    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, -fabsf(backoff_speed));
    coarse_backoff_in_progress = true;
    coarse_backoff_end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(backoff_time_ms);
}


// Menu system
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;
extern neopixel_led_config_t neopixel_led_config;


// Definitions
typedef enum {
    CHARGE_MODE_EVENT_NO_EVENT = (1 << 0),
    CHARGE_MODE_EVENT_UNDER_CHARGE = (1 << 1),
    CHARGE_MODE_EVENT_OVER_CHARGE = (1 << 2),
    CHARGE_MODE_EVENT_PAN_REMOVED = (1 << 3),
} ChargeModeEventBit_t;


static void format_elapsed_time(char *buffer, size_t len, TickType_t start_tick) {
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ticks = now - start_tick;

    // Tick to milliseconds
    float elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(buffer, len, "%.2f s", elapsed_seconds);
}


void scale_measurement_render_task(void *p) {
    char current_weight_string[WEIGHT_STRING_LEN];
    char time_buffer[16];

    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        u8g2_ClearBuffer(display_handler);

        // Set font for title and timer
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);

        // Format the timer string based on current state
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
            format_elapsed_time(time_buffer, sizeof(time_buffer), charge_start_tick);
        } else if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_RETURN ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_ZERO) {
            snprintf(time_buffer, sizeof(time_buffer), "%.2f s", last_charge_elapsed_seconds);
        } else {
            snprintf(time_buffer, sizeof(time_buffer), "--.- s");
        }

        // Calculate x positions
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);
        uint8_t time_width = u8g2_GetStrWidth(display_handler, time_buffer);

        // Draw title on left, cut short so it never runs into the timer
        char title_buffer[sizeof(title_string)];
        strncpy(title_buffer, title_string, sizeof(title_buffer) - 1);
        title_buffer[sizeof(title_buffer) - 1] = '\0';
        int title_max_width = screen_width - time_width - 5 - 5 - 4;
        size_t title_len = strlen(title_buffer);
        while (title_len > 0 && u8g2_GetStrWidth(display_handler, title_buffer) > title_max_width) {
            title_buffer[--title_len] = '\0';
        }
        u8g2_DrawStr(display_handler, 5, 10, title_buffer);

        // Draw timer on right edge
        u8g2_DrawStr(display_handler, screen_width - time_width - 5, 10, time_buffer);  // 5 px padding from edge

        // Draw line under title
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        // Current weight (only show values > -1.0)
        memset(current_weight_string, 0x0, sizeof(current_weight_string));
        float scale_measurement = scale_get_current_measurement();
        if (scale_measurement > -1.0) {
            float_to_string(current_weight_string, scale_measurement, charge_mode_config.eeprom_charge_mode_data.decimal_places);
        } else {
            strcpy(current_weight_string, "---");
        }

        // Draw current weight value
        u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
        u8g2_DrawStr(display_handler, 26, 35, current_weight_string);

        // Draw status line (charge phase, or over / under charge of the last charge)
        u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
        u8g2_DrawStr(display_handler, 5, 49, status_string);

        // Draw profile name
        profile_t *current_profile = profile_get_selected();
        u8g2_DrawStr(display_handler, 5, 61, current_profile->name);

        u8g2_SendBuffer(display_handler);

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(20));
    }
}


void charge_mode_wait_for_zero() {
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );
    
    // Wait for 5 measurements and wait for stable
    FloatRingBuffer data_buffer(10);

    // Update current status
    snprintf(title_string, sizeof(title_string), "Waiting for Zero");

    // Stop condition: 10 stable measurements in 200ms apart (2 seconds minimum)
    while (true) {
        TickType_t last_measurement_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement (max delay 300 seconds   )
        float current_measurement;
        if (scale_block_wait_for_next_measurement(300, &current_measurement)){
            data_buffer.enqueue(current_measurement);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 10){
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                abs(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
                break;
            }
        }

        // Wait for minimum 300 ms (but can skip if previously wait already)
        vTaskDelayUntil(&last_measurement_tick, pdMS_TO_TICKS(300));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
}

static float charge_elapsed_s(TickType_t tick) {
    return (float) ((tick - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
}


static void charge_mode_get_trickler_limits(trickle_motor_limits_t * limits) {
    profile_t * current_profile = profile_get_selected();

    // Find the minimum of max speed from the motor and the profile
    limits->coarse_max_rps = fmin(get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                  current_profile->coarse_max_flow_speed_rps);
    limits->coarse_min_rps = fmax(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                  current_profile->coarse_min_flow_speed_rps);
    limits->fine_max_rps = fmin(get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR),
                                current_profile->fine_max_flow_speed_rps);
    limits->fine_min_rps = fmax(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                current_profile->fine_min_flow_speed_rps);

    // The motor ramps (motor settings: acceleration and gear ratio) decide
    // how fast a speed change really happens
    limits->coarse_accel_rps2 = get_motor_acceleration(SELECT_COARSE_TRICKLER_MOTOR);
    limits->fine_accel_rps2 = get_motor_acceleration(SELECT_FINE_TRICKLER_MOTOR);
}


// Common end of a charge: stop the timer, close the gate and precharge
static void charge_mode_finish_charge() {
    // Stop the timer 
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ticks = now - charge_start_tick;
    last_charge_elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    // Close the gate if the servo gate is present
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
    servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
    }

    // Precharge
    if (charge_mode_config.eeprom_charge_mode_data.precharge_enable &&
    servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        // Set a fixed delay between closing the gate and precharge to allow the gate to fully close
        vTaskDelay(pdMS_TO_TICKS(500));

        // Start the pre-charge
        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps);
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.precharge_time_ms));

        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(20));  // Wait for other tasks to complete  
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_REMOVAL;
}


static void charge_mode_set_charging_title(trickle_phase_t phase) {
    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string, charge_mode_config.target_charge_weight, charge_mode_config.eeprom_charge_mode_data.decimal_places);

    if (ai_trickle_learn_active()) {
        snprintf(title_string, sizeof(title_string), "Learn %d/%d %s",
                 ai_trickle_learn_done() + 1, ai_trickle_learn_total(), target_weight_string);
    }
    else if (ai_trickle_series_active()) {
        snprintf(title_string, sizeof(title_string), "%s %d/%d %s",
                 ai_trickle_series_is_ladder() ? "Ladder" : "Series",
                 ai_trickle_series_done() + 1, ai_trickle_series_total(), target_weight_string);
    }
    else if (phase == TRICKLE_PHASE_PROBE || phase == TRICKLE_PHASE_TOPUP) {
        snprintf(title_string, sizeof(title_string), "%s %s", ai_trickle_phase_name(phase), target_weight_string);
    }
    else {
        snprintf(title_string, sizeof(title_string), "AI: %s", target_weight_string);
    }

    snprintf(status_string, sizeof(status_string), "%s  Spd %u  Tol %.2f",
             ai_trickle_phase_name(phase), ai_trickle_speed(), ai_trickle_tolerance());
}


// Charge guards (charge_guard.c): pan lifted, table bumped
#define CHARGE_GUARD_STABLE_SAMPLES     5
#define CHARGE_GUARD_MAX_WAIT_MS        60000   // Give up waiting for a stable scale (then the controller copes)

static uint32_t guard_now_ms(void) {
    return (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static charge_guard_result_t guard_check(charge_guard_t * guard, float weight, bool coarse_running) {
    charge_guard_config_t config = {
        .pan_guard = feature_config.pan_guard,
        .bump_guard = feature_config.bump_guard,
        .bump_drop_gr = feature_config.bump_drop_cgr * 0.01f,
    };
    return charge_guard_check(guard, &config, weight, coarse_running, guard_now_ms());
}

static void charge_guard_stop_motors(void) {
    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    coarse_backoff_in_progress = false;
}

// Pan lifted during the charge: the charge is lost
static void charge_guard_pan_removed(void) {
    charge_guard_stop_motors();
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, false);
    }
    last_charge_elapsed_seconds = (float) ((xTaskGetTickCount() - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(title_string, sizeof(title_string), "Pan removed!");
    snprintf(status_string, sizeof(status_string), "Aborted, empty the pan");
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
        true
    );
    charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_PAN_REMOVED;

    // Wait for the empty pan, then zero as for every charge
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

/*
 * Bump: motors stopped, wait until the scale is stable. Returns the stable
 * weight, or NAN when the user left the charge mode (or the pan was lifted:
 * then the state is set already).
 */
static float charge_guard_wait_stable(void) {
    charge_guard_stop_motors();

    char saved_status[sizeof(status_string)];
    strncpy(saved_status, status_string, sizeof(saved_status));
    snprintf(status_string, sizeof(status_string), "Bump! Waiting for scale");

    FloatRingBuffer data_buffer(CHARGE_GUARD_STABLE_SAMPLES);
    float sd_limit = fmaxf(charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin, 0.02f);
    TickType_t start = xTaskGetTickCount();

    while (true) {
        ButtonEncoderEvent_t event = button_wait_for_input(false);
        if (event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return NAN;
        }

        float weight;
        if (!scale_block_wait_for_next_measurement(200, &weight)) {
            continue;
        }
        if (feature_config.pan_guard && weight < -CHARGE_GUARD_PAN_REMOVED_GR) {
            charge_guard_pan_removed();
            return NAN;
        }
        data_buffer.enqueue(weight);

        bool stable = data_buffer.getCounter() >= CHARGE_GUARD_STABLE_SAMPLES && data_buffer.getSd() < sd_limit;
        // Still restless after a minute: carry on, the controller copes with the rest
        bool timeout = xTaskGetTickCount() - start > pdMS_TO_TICKS(CHARGE_GUARD_MAX_WAIT_MS);
        if (stable || timeout) {
            strncpy(status_string, saved_status, sizeof(status_string));
            return data_buffer.getMean();
        }
    }
}


/*
 * Adaptive charge: the AI trickle controller predicts the powder in flight
 * and stops ahead of time, then tops up if needed. See trickle_model.h.
 */
static void charge_mode_wait_for_complete_adaptive() {
    charge_start_tick = xTaskGetTickCount();
    coarse_backoff_in_progress = false;

    // Set colour to under charge
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        true
    );

    // If the servo gate is used then it has to be opened
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }

    trickle_motor_limits_t limits;
    charge_mode_get_trickler_limits(&limits);

    ai_trickle_adaptive_begin(charge_mode_config.target_charge_weight, &limits, scale_get_current_measurement());

    trickle_phase_t last_phase = TRICKLE_PHASE_DONE;

    // Only send speed changes: every command makes the motor task ramp, and
    // its queue is short, so repeating unchanged speeds would only stall
    // this loop
    float sent_coarse_rps = 0.0f;
    float sent_fine_rps = 0.0f;
    // After a coarse stop the motor ramps down by itself (and may back off);
    // the controller's own ramp down must not restart it
    bool coarse_stop_hold = false;

    // Fine tube reverse after each fine stop (anti-dribble), chosen per throw
    float fine_reverse_revs = ai_trickle_fine_reverse_revs();

    charge_guard_t guard;
    charge_guard_reset(&guard, guard_now_ms());
    trickle_phase_t guard_phase = TRICKLE_PHASE_COARSE;

    while (true) {
        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            ai_trickle_adaptive_end(charge_elapsed_s(xTaskGetTickCount()));
            ai_trickle_learn_stop();
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            continue;
        }
        TickType_t current_sample_tick = xTaskGetTickCount();
        float t = charge_elapsed_s(current_sample_tick);

        // Pan lifted or table bumped: before the controller sees the reading
        bool coarse_running = guard_phase == TRICKLE_PHASE_COARSE || guard_phase == TRICKLE_PHASE_PROBE;
        charge_guard_result_t guard_result = guard_check(&guard, current_weight, coarse_running);
        if (guard_result == CHARGE_GUARD_PAN_REMOVED) {
            // Motors first, then the log
            charge_guard_pan_removed();
            ai_trickle_adaptive_end(t);
            return;
        }
        if (guard_result == CHARGE_GUARD_BUMP) {
            float stable_weight = charge_guard_wait_stable();
            if (isnan(stable_weight)) {
                ai_trickle_adaptive_end(charge_elapsed_s(xTaskGetTickCount()));
                if (charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                    ai_trickle_learn_stop();
                }
                return;
            }
            // Carry on from the stable weight, as after a refill
            sent_coarse_rps = 0.0f;
            sent_fine_rps = 0.0f;
            coarse_stop_hold = false;
            ai_trickle_adaptive_resume(charge_elapsed_s(xTaskGetTickCount()), stable_weight);
            charge_guard_reset(&guard, guard_now_ms() + CHARGE_GUARD_COARSE_GRACE_MS);
            last_phase = TRICKLE_PHASE_DONE;
            continue;
        }

        // Handle coarse trickler backoff stop
        if (coarse_backoff_in_progress && (current_sample_tick >= coarse_backoff_end_tick)) {
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            sent_coarse_rps = 0.0f;
            coarse_backoff_in_progress = false;
        }

        float coarse_rps;
        float fine_rps;
        uint32_t events;
        trickle_phase_t phase = ai_trickle_adaptive_step(t, current_weight, &coarse_rps, &fine_rps, &events);
        guard_phase = phase;

        if (events & TRICKLE_EVENT_COARSE_STOPPED) {
            // The stream still lands and its push on the pan fades
            charge_guard_coarse_stopped(&guard, guard_now_ms());
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            sent_coarse_rps = 0.0f;
            coarse_stop_hold = true;

            // When the coarse trickler stops, move the servo gate to a configured ratio
            if (servo_gate.eeprom_servo_gate_config.servo_gate_enable && phase != TRICKLE_PHASE_PROBE) {
                servo_gate_set_ratio(charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio, false);
            }
            coarse_trickler_backoff(limits.coarse_min_rps);
        }

        if ((events & TRICKLE_EVENT_FINE_STOPPED) && fine_reverse_revs > 0.0f) {
            charge_mode_reverse_fine_trickler(fine_reverse_revs, &limits);
            sent_fine_rps = 0.0f;
        }

        if (coarse_stop_hold && coarse_rps <= 0.0f) {
            coarse_stop_hold = false;
        }
        if (!coarse_backoff_in_progress && !coarse_stop_hold && coarse_rps != sent_coarse_rps) {
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, coarse_rps);
            sent_coarse_rps = coarse_rps;
        }
        if (fine_rps != sent_fine_rps) {
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, fine_rps);
            sent_fine_rps = fine_rps;
        }

        if (phase != last_phase) {
            charge_mode_set_charging_title(phase);
            last_phase = phase;
        }

        if (phase == TRICKLE_PHASE_FAULT_NO_FLOW || phase == TRICKLE_PHASE_FAULT_TIMEOUT) {
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            sent_coarse_rps = 0.0f;
            sent_fine_rps = 0.0f;
            coarse_stop_hold = false;
            coarse_backoff_in_progress = false;

            // Most likely the hopper ran empty: wait for the user
            snprintf(title_string, sizeof(title_string),
                     phase == TRICKLE_PHASE_FAULT_NO_FLOW ? "No flow!" : "Timeout!");
            snprintf(status_string, sizeof(status_string), "Refill, press to retry");
            neopixel_led_set_colour(
                neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
                charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
                true
            );

            while (true) {
                ButtonEncoderEvent_t event = button_wait_for_input(true);
                if (event == BUTTON_RST_PRESSED) {
                    ai_trickle_adaptive_end(charge_elapsed_s(xTaskGetTickCount()));
                    ai_trickle_learn_stop();
                    charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                    return;
                }
                if (event == BUTTON_ENCODER_PRESSED) {
                    break;
                }
            }

            neopixel_led_set_colour(
                neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour,
                charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour,
                true
            );
            ai_trickle_adaptive_resume(charge_elapsed_s(xTaskGetTickCount()), scale_get_current_measurement());
            charge_guard_reset(&guard, guard_now_ms() + CHARGE_GUARD_COARSE_GRACE_MS);
            last_phase = TRICKLE_PHASE_DONE;
            continue;
        }

        if (phase == TRICKLE_PHASE_DONE) {
            break;
        }
    }

    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

    // Learning (model fit) happens here, while the user removes the cup
    ai_trickle_adaptive_end(charge_elapsed_s(xTaskGetTickCount()));

    charge_mode_finish_charge();
}


// A profile with a max speed of 0 (e.g. an unused slot) or motors that failed
// to start cannot charge: say so instead of waiting with standing motors
static bool charge_mode_check_profile() {
    trickle_motor_limits_t limits;
    charge_mode_get_trickler_limits(&limits);
    motor_init_err_t motor_error = get_motor_init_error();
    if (motor_error == MOTOR_INIT_OK && limits.coarse_max_rps > 0.0f && limits.fine_max_rps > 0.0f) {
        return true;
    }

    if (motor_error != MOTOR_INIT_OK) {
        // The motor drivers did not answer at start up, the motors are off
        snprintf(title_string, sizeof(title_string), "Motor error");
        snprintf(status_string, sizeof(status_string), "%s, press to exit", get_motor_init_error_string(motor_error));
    }
    else {
        snprintf(title_string, sizeof(title_string), "Profile not set up");
        snprintf(status_string, sizeof(status_string), "Max speed is 0, press to exit");
    }
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
        true
    );
    ai_trickle_learn_stop();
    ai_trickle_series_stop();

    while (true) {
        ButtonEncoderEvent_t event = button_wait_for_input(true);
        if (event == BUTTON_RST_PRESSED || event == BUTTON_ENCODER_PRESSED) {
            break;
        }
    }
    charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
    return false;
}


void charge_mode_wait_for_complete() {
    if (!charge_mode_check_profile()) {
        return;
    }

    last_charge_adaptive = ai_trickle_adaptive_active();
    if (last_charge_adaptive) {
        charge_mode_wait_for_complete_adaptive();
        return;
    }

    charge_start_tick = xTaskGetTickCount();
    coarse_backoff_in_progress = false;

    // Set colour to under charge
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        true
    );

    // If the servo gate is used then it has to be opened
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }
    // Update current status
    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string, charge_mode_config.target_charge_weight, charge_mode_config.eeprom_charge_mode_data.decimal_places);

    snprintf(title_string, sizeof(title_string), 
             "Target: %s", 
             target_weight_string);
    if (ai_trickle_series_active()) {
        charge_mode_set_charging_title(TRICKLE_PHASE_COARSE);
    }
    // Classic PID charge: no AI phase to show
    status_string[0] = '\0';

    // Read trickling parameter from the current profile
    profile_t * current_profile = profile_get_selected();

    // Find the minimum of max speed from the motor and the profile
    float coarse_trickler_max_speed = fmin(get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_max_flow_speed_rps);
    float coarse_trickler_min_speed = fmax(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_min_flow_speed_rps);
    float fine_trickler_max_speed = fmin(get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_max_flow_speed_rps);
    float fine_trickler_min_speed = fmax(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_min_flow_speed_rps);

    // Define PID terms
    float coarse_trickler_integral = 0.0f;
    float fine_trickler_integral = 0.0f;
    float coarse_trickler_last_error = 0.0f;
    float fine_trickler_last_error = 0.0f;

    // Calculate target weight for coarse trickler
    // The coarse trickler is suppose to stop ahead of the target weight by an offset
    float coarse_trickler_target_charge_weight = fmaxf(0.0f, charge_mode_config.target_charge_weight - charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold);

    TickType_t last_sample_tick = xTaskGetTickCount();
    TickType_t current_sample_tick = last_sample_tick;
    bool should_coarse_trickler_move = true;

    // Record the charge so the AI model can learn from classic charges too
    ai_trickle_classic_begin();
    float recorded_coarse_rps = 0.0f;

    charge_guard_t guard;
    charge_guard_reset(&guard, guard_now_ms());

    while (true) {
        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Run the PID controlled loop to start charging
        // Perform the measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        current_sample_tick = xTaskGetTickCount();

        // Pan lifted or table bumped
        charge_guard_result_t guard_result = guard_check(&guard, current_weight, should_coarse_trickler_move);
        if (guard_result == CHARGE_GUARD_PAN_REMOVED) {
            charge_guard_pan_removed();
            return;
        }
        if (guard_result == CHARGE_GUARD_BUMP) {
            float stable_weight = charge_guard_wait_stable();
            if (isnan(stable_weight)) {
                return;
            }
            // Carry on: the PID terms continue from the stable weight
            current_weight = stable_weight;
            current_sample_tick = xTaskGetTickCount();
            last_sample_tick = current_sample_tick - pdMS_TO_TICKS(100);
            fine_trickler_last_error = charge_mode_config.target_charge_weight - current_weight;
            coarse_trickler_last_error = coarse_trickler_target_charge_weight - current_weight;
            charge_guard_reset(&guard, guard_now_ms() + CHARGE_GUARD_COARSE_GRACE_MS);
        }

        // Handle coarse trickler backoff stop
        if (coarse_backoff_in_progress && (current_sample_tick >= coarse_backoff_end_tick)) {
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            coarse_backoff_in_progress = false;
        }

        float coarse_trickler_error = coarse_trickler_target_charge_weight - current_weight;
        float fine_trickler_error = charge_mode_config.target_charge_weight - current_weight;

        // Fine & Coarse trickler stop condition
        if (fine_trickler_error < charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold) {
            // Stop all motors
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            ai_trickle_classic_sample(charge_elapsed_s(current_sample_tick), current_weight, 0.0f, 0.0f);

            // Fine tube reverse (anti-dribble)
            trickle_motor_limits_t limits;
            charge_mode_get_trickler_limits(&limits);
            charge_mode_reverse_fine_trickler(ai_trickle_classic_fine_reverse_revs(), &limits);

            break;
        }

        // Coarse trickler stop condition
        else if (coarse_trickler_error <= 0 &&
                 should_coarse_trickler_move) {

            should_coarse_trickler_move = false;
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            recorded_coarse_rps = 0.0f;
            charge_guard_coarse_stopped(&guard, guard_now_ms());
            ai_trickle_classic_coarse_stop(charge_elapsed_s(current_sample_tick));

            // NEW: When the coarse trickler stops, move the servo gate to a configured ratio
            // Ratio convention: 0.0 = open, 1.0 = close
            if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
                float r = charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio;

                servo_gate_set_ratio(r, false); // don't block the charge loop
            }

            // NEW: Reverse the coarse trickler for a short backoff (quarter turn).
            coarse_trickler_backoff(coarse_trickler_min_speed);
        }
    

        // Update fine trickler speed
        float elapse_time_ms = (current_sample_tick - last_sample_tick) / portTICK_RATE_MS;
        fine_trickler_integral += fine_trickler_error;
        float fine_trickler_derivative = (fine_trickler_error - fine_trickler_last_error) / elapse_time_ms;

        // Update fine trickler speed
        float new_p = current_profile->fine_kp * fine_trickler_error;
        float new_i = current_profile->fine_ki * fine_trickler_integral;
        float new_d = current_profile->fine_kd * fine_trickler_derivative;
        float new_speed = fmaxf(fine_trickler_min_speed, fminf(new_p + new_i + new_d, fine_trickler_max_speed));
        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, new_speed);
        float recorded_fine_rps = new_speed;

        // Update coarse trickler speed
        if (should_coarse_trickler_move && !coarse_backoff_in_progress) {
            coarse_trickler_integral += coarse_trickler_error;
            float coarse_trickler_derivative = (coarse_trickler_error - coarse_trickler_last_error) / elapse_time_ms;

            new_p = current_profile->coarse_kp * coarse_trickler_error;
            new_i = current_profile->coarse_ki * coarse_trickler_integral;
            new_d = current_profile->coarse_kd * coarse_trickler_derivative ;

            new_speed = fmaxf(coarse_trickler_min_speed, fminf(new_p + new_i + new_d, coarse_trickler_max_speed));

            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
            recorded_coarse_rps = new_speed;
        }

        ai_trickle_classic_sample(charge_elapsed_s(current_sample_tick), current_weight, recorded_coarse_rps, recorded_fine_rps);

        // Record state
        last_sample_tick = current_sample_tick;
        fine_trickler_last_error = fine_trickler_error;
        coarse_trickler_last_error = coarse_trickler_error;
    }

    charge_mode_finish_charge();
}

// Over / under / OK of the charge on the status line, LEDs and REST event
static void charge_mode_show_result(float current_measurement) {
    float error = charge_mode_config.target_charge_weight - current_measurement;

    float threshold = charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold;
    if (last_charge_adaptive) {
        // Green means within the AI tolerance
        threshold = ai_trickle_tolerance() + 0.001f;
    }

    // Result on its own line, the title is too short for it next to the timer
    const char * verdict = error <= -threshold ? "Over" : (error >= threshold ? "Under" : "OK");
    if (charge_mode_config.eeprom_charge_mode_data.decimal_places == DP_3) {
        snprintf(status_string, sizeof(status_string), "%s  %+.3f", verdict, -error);
    }
    else {
        snprintf(status_string, sizeof(status_string), "%s  %+.2f", verdict, -error);
    }

    // Update LED colour before moving to the next stage
    // Over charged
    if (error <= -threshold) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            true
        );

        // Set over charge
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_OVER_CHARGE;
    }
    // Under charged
    else if (error >= threshold) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            true
        );

        // Set under charge flag
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_UNDER_CHARGE;

    }
    // Normal
    else {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            true
        );

        // Clear over and under charge bit
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
    }

}


void charge_mode_wait_for_cup_removal() {
    // Update current status
    snprintf(title_string, sizeof(title_string), "Remove Cup");

    FloatRingBuffer data_buffer(5);

    // Post charge analysis (while waiting for removal of the cup)
    if (last_charge_adaptive) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for other tasks to complete
    }
    else {
        // Same wait, but keep recording so the AI model sees the powder settle
        TickType_t settle_start_tick = xTaskGetTickCount();
        while (xTaskGetTickCount() - settle_start_tick < pdMS_TO_TICKS(1000)) {
            float settle_weight;
            if (scale_block_wait_for_next_measurement(200, &settle_weight)) {
                ai_trickle_classic_sample(charge_elapsed_s(xTaskGetTickCount()), settle_weight, 0.0f, 0.0f);
            }
        }
    }

    // Take current measurement
    float current_measurement = scale_get_current_measurement();

    if (last_charge_adaptive) {
        if (ai_trickle_learn_just_finished()) {
            snprintf(title_string, sizeof(title_string), "Learned! Remove Cup");
        }
    }
    else {
        ai_trickle_classic_end(charge_mode_config.target_charge_weight, current_measurement, last_charge_elapsed_seconds);
    }

    // A finished series or ladder leaves charge mode once the cup is removed
    bool series_finished = ai_trickle_series_just_finished();
    if (series_finished) {
        snprintf(title_string, sizeof(title_string), "Done! Remove Cup");
    }

    charge_mode_show_result(current_measurement);

    // Taps requested from the web page are carried out while the cup is still
    // on the scale; the result is judged again once the kernels landed
    tap_requests = 0;
    tap_allowed = true;
    TickType_t tap_judge_tick = 0;
    bool tap_judge_pending = false;

    // Stop condition: 5 stable measurements in 300ms apart (1.5 seconds minimum)
    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            tap_allowed = false;
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        if (tap_requests > 0) {
            tap_requests -= 1;
            charge_mode_tap_fine_trickler();
            snprintf(status_string, sizeof(status_string), "Tap...");
            tap_judge_tick = xTaskGetTickCount() + pdMS_TO_TICKS(CHARGE_MODE_TAP_SETTLE_MS);
            tap_judge_pending = true;
            data_buffer.reset();
            continue;
        }
        if (tap_judge_pending && xTaskGetTickCount() >= tap_judge_tick) {
            tap_judge_pending = false;
            float tapped = scale_get_current_measurement();
            charge_mode_show_result(tapped);
            // The charge log keeps the weight after the taps
            ai_trickle_history_update_final(tapped);
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        data_buffer.enqueue(current_weight);

        // Generate stop condition
        if (data_buffer.getCounter() >= 5) {
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                data_buffer.getMean() + 10 < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin){
                break;
            }
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(300));
    }

    tap_allowed = false;

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    charge_mode_config.charge_mode_state = series_finished ? CHARGE_MODE_EXIT : CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

void charge_mode_wait_for_cup_return() { 
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );

    snprintf(title_string, sizeof(title_string), "Return Cup");


    FloatRingBuffer data_buffer(5);

    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (current_weight >= 0) {
            break;
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(20));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
}


uint8_t charge_mode_menu(bool charge_mode_skip_user_input) {
    // Create target weight, if the charge mode weight is built by charge_weight_digits
    if (!charge_mode_skip_user_input) {
        switch (charge_mode_config.eeprom_charge_mode_data.decimal_places) {
            case DP_2:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 100 + \
                                                charge_weight_digits[3] * 10 + \
                                                charge_weight_digits[2] * 1 + \
                                                charge_weight_digits[1] * 0.1 + \
                                                charge_weight_digits[0] * 0.01;
                break;
            case DP_3:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 10 + \
                                                charge_weight_digits[3] * 1 + \
                                                charge_weight_digits[2] * 0.1 + \
                                                charge_weight_digits[1] * 0.01 + \
                                                charge_weight_digits[0] * 0.001;
                break;
            default:
                charge_mode_config.target_charge_weight = 0;
                break;
        }
    }

    // If the display task is never created then we shall create one, otherwise we shall resume the task
    if (scale_measurement_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(scale_measurement_render_task, "Scale Measurement Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &scale_measurement_render_task_handler);
    }
    else {
        vTaskResume(scale_measurement_render_task_handler);
    }

    // Enable motor on entering the charge mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);
    
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

    bool quit = false;
    while (quit == false) {
        switch (charge_mode_config.charge_mode_state) {
            case CHARGE_MODE_WAIT_FOR_ZERO:
                charge_mode_wait_for_zero();
                break;
            case CHARGE_MODE_WAIT_FOR_COMPLETE:
                charge_mode_wait_for_complete();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL:
                charge_mode_wait_for_cup_removal();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_RETURN:
                charge_mode_wait_for_cup_return();
                break;
            case CHARGE_MODE_EXIT:
            default:
                quit = true;
                break;
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    // A learning session or series ends with the charge mode
    ai_trickle_learn_stop();
    ai_trickle_series_stop();

    // vTaskDelete(scale_measurement_render_handler);
    vTaskSuspend(scale_measurement_render_task_handler);

    // Diable motors on exiting the mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    return 1;  // return back to main menu
}


bool charge_mode_config_init(void) {
    bool is_ok = false;

    // Read charge mode config from EEPROM
    is_ok = load_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(charge_mode_config.eeprom_charge_mode_data), EEPROM_CHARGE_MODE_DATA_REV);
    if (!is_ok) {
        printf("Unable to read charge mode configuration\n");
        return is_ok;
    }

    // Register to eeprom save all
    eeprom_register_handler(charge_mode_config_save);

    return true;
}


bool charge_mode_config_save(void) {
    bool is_ok = save_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    return is_ok;
}



bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // c1 (str): neopixel_normal_charge_colour
    // c2 (str): neopixel_under_charge_colour
    // c3 (str): neopixel_over_charge_colour
    // c4 (str): neopixel_not_ready_colour

    // c5 (float): coarse_stop_threshold
    // c6 (float): fine_stop_threshold
    // c7 (float): set_point_sd_margin
    // c8 (float): set_point_mean_margin
    // c9 (int): decimal point enum
    // c10 (bool): precharge_enable
    // c11 (int): precharge_time_ms
    // c12 (float): precharge_speed_rps
    // c13 (float): coarse_stop_gate_ratio
    // c14 (bool): coarse_stop_backoff_enable
    // c15 (float): coarse_stop_backoff_turns
    // c16 (float): coarse_stop_backoff_speed_rps
    // c17 (str/float): fine tube reverse after each fine stop: "auto", "off" or revolutions (max 0.3)
    // ee (bool): save to eeprom

    static char charge_mode_json_buffer[448];
    bool save_to_eeprom = false;

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "c5") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c6") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c8") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c9") == 0) {
            charge_mode_config.eeprom_charge_mode_data.decimal_places = (decimal_places_t) atoi(values[idx]);
        }
        
        // Pre charge related settings
        else if (strcmp(params[idx], "c10") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c11") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_time_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c12") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c13") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c14") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c15") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c16") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c17") == 0) {
            // Stored with the AI data (learned per profile in Auto)
            ai_trickle_set_fine_reverse(values[idx]);
        }


        // LED related settings
        else if (strcmp(params[idx], "c1") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c2") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c3") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c4") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }
    
    // Perform action
    if (save_to_eeprom) {
        charge_mode_config_save();
        ai_trickle_config_save();
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"
             "{\"c1\":\"#%06lx\",\"c2\":\"#%06lx\",\"c3\":\"#%06lx\",\"c4\":\"#%06lx\","
             "\"c5\":%.3f,\"c6\":%.3f,\"c7\":%.3f,\"c8\":%.3f,\"c9\":%d,\"c10\":%s,\"c11\":%ld,\"c12\":%0.3f,\"c13\":%0.3f,\"c14\":%s,\"c15\":%0.3f,\"c16\":%0.3f,\"c17\":%s}",
             charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin,
             charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin,
             charge_mode_config.eeprom_charge_mode_data.decimal_places,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.precharge_enable),
             charge_mode_config.eeprom_charge_mode_data.precharge_time_ms,
             charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_enable),
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_turns,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_backoff_speed_rps,
             ai_trickle_fine_reverse_setting());

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // s0 (float): Charge weight set point (unitless)
    // s1 (float): Current weight (unitless)
    // s2 (charge_mode_state_t | int): Charge mode state
    // s3 (uint32_t): Charge mode event
    // s4 (string): Profile Name
    // s5 (string): Elapsed time in seconds, live during charging
    // s6 (bool): a tap is possible now (charge done, cup still on the scale)
    // tap (int): turn the fine trickler by the tap angle this many times

    static char charge_mode_json_buffer[176];  // Increased to fit s5 and s6
    char elapsed_time_buffer[16] = {0};

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "s0") == 0) {
            charge_mode_config.target_charge_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "s2") == 0) {
            charge_mode_state_t new_state = (charge_mode_state_t) atoi(values[idx]);

            // Exit
            if (new_state == CHARGE_MODE_EXIT && charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
                ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }
            // Enter
            else if (new_state == CHARGE_MODE_WAIT_FOR_ZERO && charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                // Set exit_status for the menu
                exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;

                // Then signal the menu to stop
                ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }

            charge_mode_config.charge_mode_state = new_state;
        }
        else if (strcmp(params[idx], "tap") == 0 && tap_allowed) {
            int taps = atoi(values[idx]);
            if (taps > 0 && tap_requests + taps <= CHARGE_MODE_TAP_MAX_QUEUE) {
                tap_requests += (uint8_t) taps;
            }
        }
    }

    // Handle the special case
    float current_measurement = scale_get_current_measurement();
    char weight_string[16];
    if (isnanf(current_measurement)) {
        sprintf(weight_string, "\"nan\"");
    }
    else if (isinff(current_measurement)) {
        sprintf(weight_string, "\"inf\"");
    }
    else {
        sprintf(weight_string, "%0.3f", current_measurement);
    }

    // Format elapsed time
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        TickType_t now = xTaskGetTickCount();
        float elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", elapsed_seconds);
    } else {
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", last_charge_elapsed_seconds);
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "%s"
             "{\"s0\":%0.3f,\"s1\":%s,\"s2\":%d,\"s3\":%lu,\"s4\":\"%s\",\"s5\":\"%s\",\"s6\":%s}",
             http_json_header,
             charge_mode_config.target_charge_weight,
             weight_string,
             (int) charge_mode_config.charge_mode_state,
             charge_mode_config.charge_mode_event,
             profile_get_selected()->name,
             elapsed_time_buffer,
             boolean_to_string(tap_allowed));

    // Clear events
    charge_mode_config.charge_mode_event = 0;

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
