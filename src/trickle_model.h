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

#define TRICKLE_DEFAULT_TOLERANCE_GR    0.02f
#define TRICKLE_SPEED_STEP_UP           10      // Speed increase after a clean streak
#define TRICKLE_SPEED_STEP_DOWN         25      // Speed decrease after an over charge
#define TRICKLE_SPEED_CLEAN_STREAK      12      // Clean charges needed before speeding up
#define TRICKLE_SPEED_AUTO_MIN          30      // Automatic tuning does not go slower than this

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
    float fine_surge_sigma;     // Flow surge the fine approach must survive without overshooting (sigma)
    float end_allowance_gr;     // Share of the tolerance a surge at the end of the approach may use (grain)
    float overshoot_sigma;      // Aim low enough that landing above target + tolerance is this unlikely (sigma)
    float topup_threshold_gr;   // Top up when the settled weight is this much below target (grain)
    float topup_pulse_s;        // Longest top up pulse, shorter is faster but less gentle (s)
    float topup_ratio;          // Share of the missing powder a top up asks for
    float topup_settle_extra_s; // Settle wait after a top up on top of the latency (s)
    uint8_t early_topup;        // Top up before the scale is fully settled when clearly short
    float settle_timeout_s;     // Maximum wait for the scale to settle (s)
    uint8_t settle_samples;     // Stable readings needed before judging the weight
} trickle_mode_params_t;


/*
 * Speed of one profile and the state of its automatic tuning. The speed goes
 * up step by step while charges stay clean and drops quickly after an over
 * charge, so it settles just below where over charges start to happen.
 */
typedef struct {
    uint8_t speed;              // 0 = careful ... 100 = fastest
    uint8_t clean_streak;       // Adaptive charges without over charge since the last change
    uint8_t best_speed;         // Speed with the shortest mean charge time seen so far
    uint8_t tolerance_centigr;  // Tolerance of this profile in 0.01 gr, 0 = the global default (firmware only)
    float over_rate;            // Exponentially weighted share of over charges
    float topup_rate;           // Exponentially weighted top ups per charge
    float mean_time_s;          // Exponentially weighted charge time at the current speed (s)
    float best_time_s;          // Mean charge time at best_speed (s), 0 if unknown
} trickle_speed_state_t;


typedef struct {
    float coarse_min_rps;
    float coarse_max_rps;
    float fine_min_rps;
    float fine_max_rps;
    float coarse_accel_rps2;    // Trickler acceleration of the motor ramp, 0 = instant
    float fine_accel_rps2;
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
#define TRICKLE_EVENT_FINE_STOPPED      (1 << 4)    // The fine trickler stopped for settling (main stop or end of a top up)


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

    float coarse_rps;           // Wanted speeds
    float fine_rps;
    float coarse_out;           // Speeds sent to the motors, limited by their acceleration
    float fine_out;
    float t_out;
    float predicted_gr;         // Predicted final weight if the motors stopped now

    // Settle detection
    float settle_window[8];
    uint8_t settle_count;

    // Top up
    uint8_t topup_count;
    float topup_end_t;
    float topup_rps;
    float topup_goal_gr;
    float topup_pause_end_t;
    bool topup_pulse_on;
    uint8_t topup_pulses;

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


/*
 * Strategy details of settling and topping up. Fixed in the firmware, the
 * host benchmark varies them to compare strategies.
 */
typedef struct {
    float settle_extra_s;       // Wait at least latency + this after a stop before judging the scale
    float settle_range_gr;      // Max-min of the settle window to count as stable
    float topup_ratio;          // Share of the missing powder a timed top up pulse asks for
    uint8_t topup_style;        // TRICKLE_TOPUP_*
    float topup_short_pulse_s;  // Pulse length of the pulse train
    float topup_pause_factor;   // Pause after each pulse, in latencies
    uint8_t early_topup;        // Start a top up before the scale is fully settled when clearly short
    float topup_settle_extra_s; // Settle wait after a top up (less powder in flight than after the main stop)
    float fine_prime_s;         // Start the fine trickler this long before the coarse stop
    float coarse_handover_ratio;// Stop the coarse trickler when it is this close to the fine flow
    float end_allowance_factor; // Share of the tolerance a surge at the end of the fine approach may use, at speed 100
} trickle_tuning_t;

#define TRICKLE_TOPUP_TIMED     0       // One pulse for a share of the missing powder, then settle
#define TRICKLE_TOPUP_TRAIN     1       // Short pulses, after each one check the prediction

extern trickle_tuning_t trickle_tuning;

// Kernel weight from the kernel by kernel noise of the fine flow in a trace,
// NAN if the trace has no usable fine-only stretch
float trickle_estimate_kernel_gr(const trickle_trace_t * trace, const trickle_model_t * model);

// Mode
void trickle_mode_params(uint8_t speed, float tolerance_gr, trickle_mode_params_t * params);
float trickle_default_tolerance(uint8_t speed);
void trickle_mode_params_from_level(uint8_t level, trickle_mode_params_t * params);

// Automatic speed
void trickle_speed_reset(trickle_speed_state_t * state, uint8_t speed);
void trickle_speed_sanitize(trickle_speed_state_t * state);
bool trickle_speed_update(trickle_speed_state_t * state, bool over_charge, uint8_t topups, float charge_time_s);

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
