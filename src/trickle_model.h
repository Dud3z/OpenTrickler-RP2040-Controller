#ifndef TRICKLE_MODEL_H_
#define TRICKLE_MODEL_H_

/*
 * Adaptive trickle model
 *
 * Pure C (no Pico SDK / FreeRTOS dependency) so the same code runs on the
 * controller and in the host side simulator under tests/.
 *
 * The model describes how a powder behaves in this particular trickler:
 *
 *   W(t) = coarse_gpr * Rc(t - latency) + fine_gpr * Rf(t - latency) + coarse_tail * T(t)
 *
 * where Rc/Rf are the cumulative revolutions of the coarse/fine trickler,
 * latency is the combined powder flight time and scale response time and
 * T(t) rises exponentially from 0 to 1 once the coarse trickler stopped (the
 * last kernels keep dribbling from the tube). With
 * this model the controller can predict how much powder is still "in flight"
 * and stop the motors before the scale shows the target weight.
 *
 * Every throw is recorded as a trace (weight + cumulative revolutions). After
 * the throw the trace is fitted against the model and the model parameters
 * are refined, so the trickler keeps learning the powder.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define TRICKLE_TRACE_MAX_SAMPLES       512
#define TRICKLE_CMD_HISTORY_LEN         128
#define TRICKLE_MAX_TOPUPS              4
#define TRICKLE_MAX_COARSE_STOPS        4

#define TRICKLE_LEVEL_PRECISION         0
#define TRICKLE_LEVEL_BALANCED          50
#define TRICKLE_LEVEL_SPEED             100

// Model flags
#define TRICKLE_MODEL_FLAG_VALID        (1 << 0)    // Model has been fitted at least once
#define TRICKLE_MODEL_FLAG_CONVERGED    (1 << 1)    // Learning session reached the accuracy goal
#define TRICKLE_MODEL_FLAG_DRIFT        (1 << 2)    // Recent throws disagree with the model


typedef struct {
    float coarse_gpr;           // Powder delivered per coarse trickler revolution (grain/rev)
    float fine_gpr;             // Powder delivered per fine trickler revolution (grain/rev)
    float latency_s;            // Delay between motor and scale reading (s)
    float coarse_tail_gr;       // Extra powder dropping after the coarse trickler stops (grain)
    float coarse_tail_sd_gr;    // Spread of the coarse tail (grain)
    float coarse_tail_tau_s;    // Time constant of the coarse tail arriving (s)
    float flow_cv;              // Relative error of the powder in flight (flow fluctuation)
    float stop_bias_gr;         // Learned correction of the stop point, positive stops earlier (grain)
    float landing_sd_gr;        // Spread of the weight right after the main stop, before top up (grain)
    float final_mean_gr;        // Mean final error against target (grain)
    float final_sd_gr;          // Spread of the final error against target (grain)
    float mean_time_s;          // Mean charge time (s)
    uint16_t learned_throws;    // Number of throws that contributed to the model
    uint16_t flags;
} trickle_model_t;


typedef struct {
    float tolerance_gr;         // Accepted final error (grain)
    float approach_time_s;      // Time constant of the final approach (s)
    float fine_reserve_gr;      // Powder left for the fine trickler after the coarse stop (grain)
    float coarse_margin_sigma;  // Safety margin on the coarse stop in standard deviations
    float aim_below_gr;         // Aim this much below target, top up covers the rest (grain)
    float aim_sigma;            // Additionally aim below by this many landing standard deviations
    float topup_threshold_gr;   // Top up when the settled weight is this much below target (grain)
    float settle_timeout_s;     // Maximum wait for the scale to settle (s)
} trickle_mode_params_t;


typedef struct {
    float coarse_min_rps;
    float coarse_max_rps;
    float fine_min_rps;
    float fine_max_rps;
} trickle_motor_limits_t;


typedef enum {
    TRICKLE_PHASE_COARSE = 0,
    TRICKLE_PHASE_FINE = 1,
    TRICKLE_PHASE_SETTLE = 2,
    TRICKLE_PHASE_TOPUP = 3,
    TRICKLE_PHASE_DONE = 4,
    TRICKLE_PHASE_FAULT_NO_FLOW = 5,
    TRICKLE_PHASE_FAULT_TIMEOUT = 6,
    TRICKLE_PHASE_PROBE = 7,
} trickle_phase_t;


// Events raised by trickle_ctrl_step(), cleared on every call
#define TRICKLE_EVENT_COARSE_STOPPED    (1 << 0)
#define TRICKLE_EVENT_MAIN_STOPPED      (1 << 1)
#define TRICKLE_EVENT_TOPUP_STARTED     (1 << 2)
#define TRICKLE_EVENT_PROBE_DONE        (1 << 3)    // Caller may fit the trace and update the model now


typedef struct {
    float t;
    float coarse_rps;
    float fine_rps;
} trickle_cmd_t;


typedef struct {
    // Configuration
    float target_gr;
    trickle_model_t model;
    trickle_mode_params_t mode;
    trickle_motor_limits_t limits;

    // Runtime
    trickle_phase_t phase;
    uint32_t events;
    float t_start;
    float t_phase;
    float t_coarse_stop;        // Final coarse stop of the main charge, negative while running
    float coarse_stops[TRICKLE_MAX_COARSE_STOPS];   // All coarse stops incl. probe pulses
    uint8_t coarse_stop_count;
    float coarse_goal_gr;
    float aim_below_gr;         // Effective aim offset for this throw

    // Probe pulses at the start of a learning throw
    uint8_t probe_step;
    bool probe_pulse_on;
    float probe_until;

    trickle_cmd_t cmd_history[TRICKLE_CMD_HISTORY_LEN];
    uint16_t cmd_head;
    uint16_t cmd_count;

    float coarse_rps;
    float fine_rps;
    float predicted_gr;         // Predicted final weight if the motors stopped now

    // Settle detection
    float settle_window[5];
    uint8_t settle_count;

    // Top up
    uint8_t topup_count;
    float topup_end_t;
    float topup_rps;

    // No flow detection
    float flow_check_t;
    float flow_check_weight_gr;
    float flow_check_expected_gr;

    // Results
    float landing_weight_gr;    // Settled weight after the main stop
    float final_weight_gr;      // Settled weight when done
    float last_weight_gr;
} trickle_ctrl_t;


typedef struct {
    float t;
    float weight_gr;
    float coarse_revs;          // Cumulative revolutions
    float fine_revs;
} trickle_sample_t;


typedef struct {
    trickle_sample_t samples[TRICKLE_TRACE_MAX_SAMPLES];
    uint16_t count;
    uint16_t stride;            // Record every n-th sample, doubles when the buffer is full
    uint16_t skipped;
    float t0;
    float last_t;
    float coarse_rps;
    float fine_rps;
    float coarse_revs;
    float fine_revs;
    float coarse_stops[TRICKLE_MAX_COARSE_STOPS];   // Relative to t0
    uint8_t coarse_stop_count;
} trickle_trace_t;


typedef struct {
    bool valid;
    float coarse_gpr;
    float fine_gpr;
    float latency_s;
    float coarse_tail_gr;
    float coarse_tail_tau_s;
    float rms_residual_gr;
    // Standard errors, large when the trace says little about a parameter
    float coarse_gpr_se;
    float fine_gpr_se;
    float coarse_tail_se_gr;
    float flow_cv;              // NAN when the trace was too short to tell
} trickle_fit_t;


typedef struct {
    float target_gr;
    float landing_weight_gr;    // NAN if unknown
    float final_weight_gr;
    float charge_time_s;
    bool adaptive;              // true if the adaptive controller produced the throw
    float aim_below_gr;         // Aim offset used for the throw, without the learned stop bias
} trickle_throw_result_t;


// Mode
void trickle_mode_params_from_level(uint8_t level, trickle_mode_params_t * params);

// Model
void trickle_model_reset(trickle_model_t * model);
bool trickle_model_is_ready(const trickle_model_t * model);
void trickle_model_sanitize(trickle_model_t * model);
float trickle_model_estimate_time(const trickle_model_t * model,
                                  const trickle_mode_params_t * mode,
                                  const trickle_motor_limits_t * limits,
                                  float target_gr);
float trickle_model_effective_aim_below(const trickle_model_t * model,
                                        const trickle_mode_params_t * mode);

// Controller
void trickle_ctrl_init(trickle_ctrl_t * ctrl, float target_gr,
                       const trickle_model_t * model,
                       const trickle_mode_params_t * mode,
                       const trickle_motor_limits_t * limits,
                       float t_now, float current_weight_gr);
trickle_phase_t trickle_ctrl_step(trickle_ctrl_t * ctrl, float t_now, float weight_gr,
                                  float * coarse_rps, float * fine_rps);
void trickle_ctrl_start_probe(trickle_ctrl_t * ctrl, float t_now);
void trickle_ctrl_update_model(trickle_ctrl_t * ctrl, const trickle_model_t * model);
float trickle_ctrl_inflight_gr(const trickle_ctrl_t * ctrl, float t_now);
bool trickle_phase_is_final(trickle_phase_t phase);
void trickle_ctrl_get_result(const trickle_ctrl_t * ctrl, float charge_time_s, trickle_throw_result_t * result);

// Trace
void trickle_trace_reset(trickle_trace_t * trace, float t0);
void trickle_trace_add(trickle_trace_t * trace, float t, float weight_gr, float coarse_rps, float fine_rps);
void trickle_trace_mark_coarse_stop(trickle_trace_t * trace, float t);

// Learning
bool trickle_fit_trace(const trickle_trace_t * trace, const trickle_model_t * prior, trickle_fit_t * fit);
void trickle_model_learn(trickle_model_t * model, const trickle_fit_t * fit,
                         const trickle_throw_result_t * result, float gain);

#ifdef __cplusplus
}
#endif

#endif  // TRICKLE_MODEL_H_
