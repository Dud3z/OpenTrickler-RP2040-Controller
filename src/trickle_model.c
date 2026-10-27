#include <math.h>
#include <string.h>

#include "trickle_model.h"


// Bounds for sanitizing learned values
#define COARSE_GPR_MIN              0.01f
#define COARSE_GPR_MAX              50.0f
#define FINE_GPR_MIN                0.001f
#define FINE_GPR_MAX                5.0f
#define LATENCY_MIN_S               0.05f
#define LATENCY_MAX_S               3.0f
#define TAIL_MIN_GR                 -2.0f
#define TAIL_MAX_GR                 5.0f
#define TAIL_TAU_MIN_S              0.02f
#define TAIL_TAU_MAX_S              2.0f
#define STOP_BIAS_LIMIT_GR          1.0f

// Controller constants
#define COARSE_APPROACH_TIME_S      0.6f        // Time constant when the coarse trickler slows down
#define FINE_PRIME_S                1.5f        // Start the fine trickler this long before the coarse stop
#define SETTLE_MIN_EXTRA_S          0.25f       // Wait at least latency + this before judging the scale
#define SETTLE_STABLE_RANGE_GR      0.03f       // Max-min of the settle window to be considered stable
#define SETTLE_WINDOW_LEN           4
#define TOPUP_DELIVERY_RATIO        0.7f        // Ask the top up pulse for 70% of the missing powder
#define TOPUP_MAX_PULSE_S           3.0f        // Top up pulses run as slow as possible within this time
#define FLOW_CHECK_WINDOW_S         4.0f
#define FLOW_CHECK_MIN_EXPECTED_GR  0.6f
#define CHARGE_TIMEOUT_S            240.0f
#define PROBE_ABORT_FRACTION        0.35f       // Stop probing once this share of the target is in the pan
#define PROBE_MIN_TARGET_FOR_COARSE 15.0f       // Smaller charges only probe the fine trickler

// Fit constants
#define FIT_LATENCY_STEP_S          0.05f
#define FIT_MIN_SAMPLES             20
#define FIT_MAX_POINTS              384         // Subsample longer traces, keeps the fit fast on the RP2040
#define FIT_LATENCY_MAX_S           2.0f
#define FIT_RIDGE_COARSE            1.0f
#define FIT_RIDGE_FINE              2.0f
#define FIT_RIDGE_TAIL              4.0f
#define FIT_FLOW_MIN_DELTA_GR       0.5f        // Ignore windows with less powder for the flow fluctuation
#define FLOW_CV_DEFAULT             0.25f
#define FLOW_CV_SAFETY              1.25f


static float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}


static float lerpf(float a, float b, float x) {
    return a + (b - a) * x;
}


// Geometric interpolation, used for parameters spanning a large range
static float gerpf(float a, float b, float x) {
    return a * powf(b / a, x);
}


static float positive(float v) {
    return v > 0.0f ? v : 0.0f;
}


// Fraction of the coarse tail that reached the scale, dt counted from the
// moment the first tail kernels become visible
static float tail_fraction(float dt, float tau) {
    if (dt <= 0.0f) {
        return 0.0f;
    }
    return 1.0f - expf(-dt / tau);
}


void trickle_mode_params_from_level(uint8_t level, trickle_mode_params_t * params) {
    // Level 0 favours precision, level 100 favours speed
    float x = clampf((float) level / 100.0f, 0.0f, 1.0f);

    params->tolerance_gr = gerpf(0.02f, 0.10f, x);
    params->approach_time_s = gerpf(1.6f, 0.3f, x);
    params->fine_reserve_gr = gerpf(2.0f, 0.5f, x);
    params->coarse_margin_sigma = lerpf(3.0f, 2.0f, x);
    params->aim_below_gr = params->tolerance_gr * 0.5f * (1.0f - x);
    params->aim_sigma = lerpf(1.0f, 0.0f, x);
    // The scale reading is quantized, so precision levels top up before the
    // reading leaves the tolerance band
    params->topup_threshold_gr = params->tolerance_gr * lerpf(0.5f, 1.0f, x);
    params->settle_timeout_s = lerpf(4.0f, 2.0f, x);
}


void trickle_model_reset(trickle_model_t * model) {
    memset(model, 0x0, sizeof(trickle_model_t));

    // Conservative defaults, only used as prior for the first fit
    model->coarse_gpr = 1.0f;
    model->fine_gpr = 0.1f;
    model->latency_s = 0.5f;
    model->coarse_tail_gr = 0.0f;
    model->coarse_tail_sd_gr = 0.3f;
    model->coarse_tail_tau_s = 0.3f;
    model->flow_cv = FLOW_CV_DEFAULT;
    model->stop_bias_gr = 0.0f;
    model->landing_sd_gr = 0.05f;
    model->final_mean_gr = 0.0f;
    model->final_sd_gr = 0.05f;
    model->mean_time_s = 0.0f;
}


bool trickle_model_is_ready(const trickle_model_t * model) {
    return (model->flags & TRICKLE_MODEL_FLAG_VALID) && model->learned_throws > 0;
}


static float sanitize_value(float v, float lo, float hi, float fallback) {
    if (isnan(v) || isinf(v)) {
        return fallback;
    }
    return clampf(v, lo, hi);
}


void trickle_model_sanitize(trickle_model_t * model) {
    model->coarse_gpr = sanitize_value(model->coarse_gpr, COARSE_GPR_MIN, COARSE_GPR_MAX, 1.0f);
    model->fine_gpr = sanitize_value(model->fine_gpr, FINE_GPR_MIN, FINE_GPR_MAX, 0.1f);
    model->latency_s = sanitize_value(model->latency_s, LATENCY_MIN_S, LATENCY_MAX_S, 0.5f);
    model->coarse_tail_gr = sanitize_value(model->coarse_tail_gr, TAIL_MIN_GR, TAIL_MAX_GR, 0.0f);
    model->coarse_tail_sd_gr = sanitize_value(model->coarse_tail_sd_gr, 0.01f, 5.0f, 0.3f);
    model->coarse_tail_tau_s = sanitize_value(model->coarse_tail_tau_s, TAIL_TAU_MIN_S, TAIL_TAU_MAX_S, 0.3f);
    model->flow_cv = sanitize_value(model->flow_cv, 0.02f, 1.0f, FLOW_CV_DEFAULT);
    model->stop_bias_gr = sanitize_value(model->stop_bias_gr, -STOP_BIAS_LIMIT_GR, STOP_BIAS_LIMIT_GR, 0.0f);
    model->landing_sd_gr = sanitize_value(model->landing_sd_gr, 0.0f, 5.0f, 0.05f);
    model->final_mean_gr = sanitize_value(model->final_mean_gr, -10.0f, 10.0f, 0.0f);
    model->final_sd_gr = sanitize_value(model->final_sd_gr, 0.0f, 10.0f, 0.05f);
    model->mean_time_s = sanitize_value(model->mean_time_s, 0.0f, 600.0f, 0.0f);
}


float trickle_model_effective_aim_below(const trickle_model_t * model,
                                        const trickle_mode_params_t * mode) {
    float aim = mode->aim_below_gr;

    // Once the landing spread is known, aim below by a few sigma so that an
    // over charge (which cannot be corrected) becomes unlikely. The top up
    // phase fills the remaining gap.
    if (model->learned_throws >= 3) {
        float sigma_aim = mode->aim_sigma * model->landing_sd_gr;
        if (sigma_aim > aim) {
            aim = sigma_aim;
        }
    }

    // Never aim so low that the fine trickler has nothing left to do
    return clampf(aim, 0.0f, mode->fine_reserve_gr * 0.5f);
}


float trickle_model_estimate_time(const trickle_model_t * model,
                                  const trickle_mode_params_t * mode,
                                  const trickle_motor_limits_t * limits,
                                  float target_gr) {
    float coarse_flow = model->coarse_gpr * limits->coarse_max_rps + model->fine_gpr * limits->fine_max_rps;
    float fine_max_flow = model->fine_gpr * limits->fine_max_rps;
    float fine_min_flow = model->fine_gpr * limits->fine_min_rps;

    float coarse_part = positive(target_gr - mode->fine_reserve_gr);
    float fine_part = target_gr - coarse_part;

    float t = 0.0f;
    if (coarse_flow > 0.0f) {
        t += coarse_part / coarse_flow;
    }

    // Numerically walk the fine approach: flow = remaining / T, clamped
    float remaining = fine_part;
    float dt = 0.05f;
    for (int i = 0; i < 4000 && remaining > 0.0f; i += 1) {
        float q = clampf(remaining / mode->approach_time_s, fine_min_flow, fine_max_flow);
        if (remaining <= q * model->latency_s) {
            break;
        }
        remaining -= q * dt;
        t += dt;
    }

    // Settle at the end
    t += model->latency_s + 0.6f;

    return t;
}


// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

/*
 * Probe pulses: short, separated bursts of each trickler. The sharp on/off
 * edges make latency, flow per revolution and coarse tail clearly visible,
 * independent of how well the existing profile is tuned.
 */
typedef struct {
    float coarse_fraction;      // Of the coarse max speed
    float fine_fraction;        // Of the fine max speed
    float on_s;
    float off_s;
} probe_pulse_t;

static const probe_pulse_t probe_schedule[] = {
    {0.0f, 1.0f, 1.5f, 1.5f},
    {0.4f, 0.0f, 0.4f, 1.8f},
    {0.4f, 0.0f, 0.4f, 1.8f},
    {0.0f, 1.0f, 1.0f, 1.5f},
};
#define PROBE_STEPS (sizeof(probe_schedule) / sizeof(probe_schedule[0]))

static void ctrl_record_command(trickle_ctrl_t * ctrl, float t, float coarse_rps, float fine_rps) {
    // Only store a new entry when the command changes
    if (ctrl->cmd_count > 0) {
        uint16_t last_idx = (ctrl->cmd_head + TRICKLE_CMD_HISTORY_LEN - 1) % TRICKLE_CMD_HISTORY_LEN;
        trickle_cmd_t * last = &ctrl->cmd_history[last_idx];
        if (last->coarse_rps == coarse_rps && last->fine_rps == fine_rps) {
            return;
        }
    }

    trickle_cmd_t * cmd = &ctrl->cmd_history[ctrl->cmd_head];
    cmd->t = t;
    cmd->coarse_rps = coarse_rps;
    cmd->fine_rps = fine_rps;

    ctrl->cmd_head = (ctrl->cmd_head + 1) % TRICKLE_CMD_HISTORY_LEN;
    if (ctrl->cmd_count < TRICKLE_CMD_HISTORY_LEN) {
        ctrl->cmd_count += 1;
    }
}


// Powder dispensed in [t_from, t_to] according to the command history
static float ctrl_dispensed_gr(const trickle_ctrl_t * ctrl, float t_from, float t_to) {
    float mass = 0.0f;
    float seg_end = t_to;

    for (uint16_t n = 0; n < ctrl->cmd_count; n += 1) {
        uint16_t idx = (ctrl->cmd_head + TRICKLE_CMD_HISTORY_LEN - 1 - n) % TRICKLE_CMD_HISTORY_LEN;
        const trickle_cmd_t * cmd = &ctrl->cmd_history[idx];

        float seg_start = cmd->t;
        float a = seg_start > t_from ? seg_start : t_from;
        float b = seg_end < t_to ? seg_end : t_to;

        if (b > a) {
            mass += (ctrl->model.coarse_gpr * positive(cmd->coarse_rps) +
                     ctrl->model.fine_gpr * positive(cmd->fine_rps)) * (b - a);
        }

        if (seg_start <= t_from) {
            break;
        }
        seg_end = seg_start;
    }

    return mass;
}


float trickle_ctrl_inflight_gr(const trickle_ctrl_t * ctrl, float t_now) {
    return ctrl_dispensed_gr(ctrl, t_now - ctrl->model.latency_s, t_now);
}


bool trickle_phase_is_final(trickle_phase_t phase) {
    return phase == TRICKLE_PHASE_DONE ||
           phase == TRICKLE_PHASE_FAULT_NO_FLOW ||
           phase == TRICKLE_PHASE_FAULT_TIMEOUT;
}


static void ctrl_compute_goals(trickle_ctrl_t * ctrl) {
    ctrl->aim_below_gr = trickle_model_effective_aim_below(&ctrl->model, &ctrl->mode);

    // The coarse trickler stops early enough that its tail plus a safety
    // margin can never cross the fine trickler's reserve.
    float coarse_margin = ctrl->mode.coarse_margin_sigma * ctrl->model.coarse_tail_sd_gr;
    ctrl->coarse_goal_gr = ctrl->target_gr - ctrl->aim_below_gr - ctrl->model.stop_bias_gr -
                           ctrl->mode.fine_reserve_gr - coarse_margin;
}


static void ctrl_mark_coarse_stop(trickle_ctrl_t * ctrl, float t_now) {
    if (ctrl->coarse_stop_count < TRICKLE_MAX_COARSE_STOPS) {
        ctrl->coarse_stops[ctrl->coarse_stop_count++] = t_now;
    }
    ctrl->events |= TRICKLE_EVENT_COARSE_STOPPED;
}


void trickle_ctrl_update_model(trickle_ctrl_t * ctrl, const trickle_model_t * model) {
    ctrl->model = *model;
    trickle_model_sanitize(&ctrl->model);
    ctrl_compute_goals(ctrl);
}


void trickle_ctrl_start_probe(trickle_ctrl_t * ctrl, float t_now) {
    ctrl->phase = TRICKLE_PHASE_PROBE;
    ctrl->t_phase = t_now;
    ctrl->probe_step = 0;
    ctrl->probe_pulse_on = false;
    ctrl->probe_until = t_now;
}


void trickle_ctrl_get_result(const trickle_ctrl_t * ctrl, float charge_time_s, trickle_throw_result_t * result) {
    result->target_gr = ctrl->target_gr;
    result->landing_weight_gr = ctrl->landing_weight_gr;
    result->final_weight_gr = ctrl->final_weight_gr;
    result->charge_time_s = charge_time_s;
    result->adaptive = true;
    result->aim_below_gr = ctrl->aim_below_gr;
}


void trickle_ctrl_init(trickle_ctrl_t * ctrl, float target_gr,
                       const trickle_model_t * model,
                       const trickle_mode_params_t * mode,
                       const trickle_motor_limits_t * limits,
                       float t_now, float current_weight_gr) {
    memset(ctrl, 0x0, sizeof(trickle_ctrl_t));

    ctrl->target_gr = target_gr;
    ctrl->model = *model;
    trickle_model_sanitize(&ctrl->model);
    ctrl->mode = *mode;
    ctrl->limits = *limits;

    ctrl_compute_goals(ctrl);

    ctrl->t_start = t_now;
    ctrl->t_phase = t_now;
    ctrl->t_coarse_stop = -1.0f;
    ctrl->landing_weight_gr = NAN;
    ctrl->final_weight_gr = NAN;
    ctrl->last_weight_gr = current_weight_gr;

    ctrl->flow_check_t = t_now;
    ctrl->flow_check_weight_gr = current_weight_gr;

    if (current_weight_gr >= ctrl->coarse_goal_gr) {
        // Already close to the target (e.g. resumed charge), skip the coarse phase
        ctrl->phase = TRICKLE_PHASE_FINE;
    }
    else {
        ctrl->phase = TRICKLE_PHASE_COARSE;
    }
}


static void ctrl_enter_settle(trickle_ctrl_t * ctrl, float t_now) {
    ctrl->phase = TRICKLE_PHASE_SETTLE;
    ctrl->t_phase = t_now;
    ctrl->settle_count = 0;
}


static void ctrl_stop_main(trickle_ctrl_t * ctrl, float t_now) {
    bool coarse_was_running = ctrl->coarse_rps > 0.0f;
    ctrl->coarse_rps = 0.0f;
    ctrl->fine_rps = 0.0f;
    if (ctrl->t_coarse_stop < 0.0f) {
        ctrl->t_coarse_stop = t_now;
        if (coarse_was_running) {
            ctrl_mark_coarse_stop(ctrl, t_now);
        }
    }
    ctrl->events |= TRICKLE_EVENT_MAIN_STOPPED;
    ctrl_enter_settle(ctrl, t_now);
}


static float ctrl_aim_weight(const trickle_ctrl_t * ctrl) {
    return ctrl->target_gr - ctrl->aim_below_gr - ctrl->model.stop_bias_gr;
}


/*
 * The powder in flight can only be predicted to within flow_cv. With an
 * exponential approach of time constant T in prediction space, the powder in
 * flight is remaining * (exp(latency / T) - 1). Requiring that a z-sigma flow
 * surge of the in-flight powder still stays below the remaining distance
 * gives T >= latency / ln(1 + 1 / (z * cv)).
 */
static float ctrl_min_approach_time(const trickle_ctrl_t * ctrl) {
    // Flow surges come with heavy tails (clumps), so be generous on both the
    // number of sigmas and the fluctuation itself.
    float z = ctrl->mode.coarse_margin_sigma + 1.0f;
    float zcv = z * FLOW_CV_SAFETY * ctrl->model.flow_cv;
    return ctrl->model.latency_s / logf(1.0f + 1.0f / zcv);
}


static void ctrl_check_flow(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    if (t_now - ctrl->flow_check_t < FLOW_CHECK_WINDOW_S) {
        return;
    }

    // Compare what the model expects to have landed in the window with what
    // the scale actually saw. Shifted by the latency so in-flight powder does
    // not count.
    float lat = ctrl->model.latency_s;
    float expected = ctrl_dispensed_gr(ctrl, ctrl->flow_check_t - lat, t_now - lat);
    float measured = weight_gr - ctrl->flow_check_weight_gr;

    float min_expected = FLOW_CHECK_MIN_EXPECTED_GR;
    if (min_expected < 10.0f * ctrl->mode.tolerance_gr) {
        min_expected = 10.0f * ctrl->mode.tolerance_gr;
    }

    if (expected > min_expected && measured < 0.1f * expected) {
        ctrl->phase = TRICKLE_PHASE_FAULT_NO_FLOW;
        ctrl->coarse_rps = 0.0f;
        ctrl->fine_rps = 0.0f;
    }

    ctrl->flow_check_t = t_now;
    ctrl->flow_check_weight_gr = weight_gr;
}


static void ctrl_step_main(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    float inflight = trickle_ctrl_inflight_gr(ctrl, t_now);

    // Coarse tail that has not reached the scale yet
    float tail_pending = 0.0f;
    for (uint8_t i = 0; i < ctrl->coarse_stop_count; i += 1) {
        float arrived = tail_fraction(t_now - ctrl->coarse_stops[i] - ctrl->model.latency_s,
                                      ctrl->model.coarse_tail_tau_s);
        tail_pending += ctrl->model.coarse_tail_gr * (1.0f - arrived);
    }

    float predicted = weight_gr + inflight + tail_pending;
    ctrl->predicted_gr = predicted;

    float aim = ctrl_aim_weight(ctrl);

    // Backstop independent from the model: the scale already shows the aim
    if (weight_gr >= aim) {
        ctrl_stop_main(ctrl, t_now);
        return;
    }

    if (ctrl->phase == TRICKLE_PHASE_COARSE) {
        float predicted_with_tail = predicted + positive(ctrl->model.coarse_tail_gr);

        if (predicted_with_tail >= ctrl->coarse_goal_gr) {
            bool coarse_was_running = ctrl->coarse_rps > 0.0f;
            ctrl->coarse_rps = 0.0f;
            ctrl->t_coarse_stop = t_now;
            if (coarse_was_running) {
                ctrl_mark_coarse_stop(ctrl, t_now);
            }
            ctrl->phase = TRICKLE_PHASE_FINE;
            ctrl->t_phase = t_now;
        }
        else {
            float remaining = ctrl->coarse_goal_gr - predicted_with_tail;
            float approach_time = ctrl_min_approach_time(ctrl);
            if (approach_time < COARSE_APPROACH_TIME_S) {
                approach_time = COARSE_APPROACH_TIME_S;
            }
            float rps = remaining / approach_time / ctrl->model.coarse_gpr;
            ctrl->coarse_rps = clampf(rps, ctrl->limits.coarse_min_rps, ctrl->limits.coarse_max_rps);

            // The fine trickler only joins shortly before the coarse stop so
            // that it is primed. Running it the whole time would make its flow
            // indistinguishable from the coarse flow for the model fit.
            float coarse_flow = ctrl->model.coarse_gpr * ctrl->coarse_rps;
            float time_to_stop = coarse_flow > 0.0f ? remaining / coarse_flow : 0.0f;
            ctrl->fine_rps = time_to_stop < FINE_PRIME_S ? ctrl->limits.fine_max_rps : 0.0f;
            return;
        }
    }

    // Fine phase: exponential approach in prediction space. Since the
    // in-flight powder is part of the prediction, the dead time of the
    // system does not cause an overshoot.
    float remaining = aim - predicted;
    if (remaining <= 0.0f) {
        ctrl_stop_main(ctrl, t_now);
        return;
    }

    float approach_time = ctrl->mode.approach_time_s;
    float min_approach_time = ctrl_min_approach_time(ctrl);
    if (approach_time < min_approach_time) {
        approach_time = min_approach_time;
    }
    float flow = remaining / approach_time;
    float rps = flow / ctrl->model.fine_gpr;
    ctrl->fine_rps = clampf(rps, ctrl->limits.fine_min_rps, ctrl->limits.fine_max_rps);
}


static void ctrl_finish_probe(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    if (ctrl->coarse_rps > 0.0f) {
        ctrl_mark_coarse_stop(ctrl, t_now);
    }
    ctrl->coarse_rps = 0.0f;
    ctrl->fine_rps = 0.0f;
    ctrl->events |= TRICKLE_EVENT_PROBE_DONE;
    ctrl->phase = weight_gr >= ctrl->coarse_goal_gr ? TRICKLE_PHASE_FINE : TRICKLE_PHASE_COARSE;
    ctrl->t_phase = t_now;
    ctrl->flow_check_t = t_now;
    ctrl->flow_check_weight_gr = weight_gr;
}


static void ctrl_step_probe(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    if (weight_gr >= PROBE_ABORT_FRACTION * ctrl->target_gr) {
        ctrl_finish_probe(ctrl, t_now, weight_gr);
        return;
    }

    if (t_now < ctrl->probe_until) {
        return;
    }

    if (ctrl->probe_pulse_on) {
        // End of a pulse, pause so the powder can land
        if (ctrl->coarse_rps > 0.0f) {
            ctrl_mark_coarse_stop(ctrl, t_now);
        }
        ctrl->coarse_rps = 0.0f;
        ctrl->fine_rps = 0.0f;
        ctrl->probe_pulse_on = false;
        ctrl->probe_until = t_now + probe_schedule[ctrl->probe_step].off_s;
        ctrl->probe_step += 1;
        return;
    }

    // Skip coarse pulses for small charges, they could overshoot on their own
    while (ctrl->probe_step < PROBE_STEPS &&
           probe_schedule[ctrl->probe_step].coarse_fraction > 0.0f &&
           ctrl->target_gr < PROBE_MIN_TARGET_FOR_COARSE) {
        ctrl->probe_step += 1;
    }

    if (ctrl->probe_step >= PROBE_STEPS) {
        ctrl_finish_probe(ctrl, t_now, weight_gr);
        return;
    }

    const probe_pulse_t * pulse = &probe_schedule[ctrl->probe_step];
    ctrl->coarse_rps = pulse->coarse_fraction > 0.0f ?
        clampf(pulse->coarse_fraction * ctrl->limits.coarse_max_rps, ctrl->limits.coarse_min_rps, ctrl->limits.coarse_max_rps) : 0.0f;
    ctrl->fine_rps = pulse->fine_fraction > 0.0f ?
        clampf(pulse->fine_fraction * ctrl->limits.fine_max_rps, ctrl->limits.fine_min_rps, ctrl->limits.fine_max_rps) : 0.0f;
    ctrl->probe_pulse_on = true;
    ctrl->probe_until = t_now + pulse->on_s;
}


static void ctrl_step_settle(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    ctrl->coarse_rps = 0.0f;
    ctrl->fine_rps = 0.0f;

    float elapsed = t_now - ctrl->t_phase;
    if (elapsed < ctrl->model.latency_s + SETTLE_MIN_EXTRA_S) {
        return;
    }

    // Shift the settle window
    if (ctrl->settle_count < SETTLE_WINDOW_LEN) {
        ctrl->settle_window[ctrl->settle_count] = weight_gr;
        ctrl->settle_count += 1;
    }
    else {
        for (int i = 1; i < SETTLE_WINDOW_LEN; i += 1) {
            ctrl->settle_window[i - 1] = ctrl->settle_window[i];
        }
        ctrl->settle_window[SETTLE_WINDOW_LEN - 1] = weight_gr;
    }

    if (ctrl->settle_count < SETTLE_WINDOW_LEN) {
        return;
    }

    float w_min = ctrl->settle_window[0];
    float w_max = ctrl->settle_window[0];
    float w_sum = 0.0f;
    for (int i = 0; i < SETTLE_WINDOW_LEN; i += 1) {
        float w = ctrl->settle_window[i];
        w_min = w < w_min ? w : w_min;
        w_max = w > w_max ? w : w_max;
        w_sum += w;
    }

    bool stable = (w_max - w_min) <= SETTLE_STABLE_RANGE_GR;
    bool timeout = elapsed >= ctrl->model.latency_s + ctrl->mode.settle_timeout_s;
    if (!stable && !timeout) {
        return;
    }

    // Use the latest reading when stable (the scale's own filter already
    // averages), the mean otherwise.
    float settled = stable ? weight_gr : w_sum / SETTLE_WINDOW_LEN;

    if (isnan(ctrl->landing_weight_gr)) {
        ctrl->landing_weight_gr = settled;
    }

    float missing = ctrl->target_gr - settled;
    if (missing > ctrl->mode.topup_threshold_gr && ctrl->topup_count < TRICKLE_MAX_TOPUPS) {
        // Aim the top up at the middle of the lower half of the tolerance band
        float goal = ctrl->target_gr - 0.25f * ctrl->mode.tolerance_gr;
        float amount = goal - settled;
        if (amount < 0.5f * ctrl->mode.tolerance_gr) {
            amount = 0.5f * ctrl->mode.tolerance_gr;
        }
        if (amount > 0.0f) {
            // Slow pulses: less powder in flight, smaller chance that a
            // clump arrives after the pulse ended
            float revs = TOPUP_DELIVERY_RATIO * amount / ctrl->model.fine_gpr;
            float rps = clampf(revs / TOPUP_MAX_PULSE_S, ctrl->limits.fine_min_rps,
                               ctrl->limits.fine_max_rps * 0.5f);

            ctrl->topup_rps = rps;
            ctrl->topup_end_t = t_now + revs / rps;
            ctrl->topup_count += 1;
            ctrl->phase = TRICKLE_PHASE_TOPUP;
            ctrl->t_phase = t_now;
            ctrl->events |= TRICKLE_EVENT_TOPUP_STARTED;
            return;
        }
    }

    ctrl->final_weight_gr = settled;
    ctrl->phase = TRICKLE_PHASE_DONE;
}


static void ctrl_step_topup(trickle_ctrl_t * ctrl, float t_now, float weight_gr) {
    ctrl->coarse_rps = 0.0f;

    // Stop the pulse when it is complete or the target is reached early
    if (t_now >= ctrl->topup_end_t || weight_gr >= ctrl->target_gr) {
        ctrl->fine_rps = 0.0f;
        ctrl_enter_settle(ctrl, t_now);
        return;
    }

    ctrl->fine_rps = ctrl->topup_rps;
}


trickle_phase_t trickle_ctrl_step(trickle_ctrl_t * ctrl, float t_now, float weight_gr,
                                  float * coarse_rps, float * fine_rps) {
    ctrl->events = 0;

    // Ignore invalid readings, keep the previous command
    if (isnan(weight_gr) || isinf(weight_gr)) {
        *coarse_rps = ctrl->coarse_rps;
        *fine_rps = ctrl->fine_rps;
        return ctrl->phase;
    }
    ctrl->last_weight_gr = weight_gr;

    switch (ctrl->phase) {
        case TRICKLE_PHASE_COARSE:
        case TRICKLE_PHASE_FINE:
            ctrl_step_main(ctrl, t_now, weight_gr);
            if (ctrl->phase == TRICKLE_PHASE_COARSE || ctrl->phase == TRICKLE_PHASE_FINE) {
                ctrl_check_flow(ctrl, t_now, weight_gr);
            }
            break;
        case TRICKLE_PHASE_SETTLE:
            ctrl_step_settle(ctrl, t_now, weight_gr);
            break;
        case TRICKLE_PHASE_TOPUP:
            ctrl_step_topup(ctrl, t_now, weight_gr);
            break;
        case TRICKLE_PHASE_PROBE:
            ctrl_step_probe(ctrl, t_now, weight_gr);
            break;
        default:
            ctrl->coarse_rps = 0.0f;
            ctrl->fine_rps = 0.0f;
            break;
    }

    if (!trickle_phase_is_final(ctrl->phase) && t_now - ctrl->t_start > CHARGE_TIMEOUT_S) {
        ctrl->phase = TRICKLE_PHASE_FAULT_TIMEOUT;
        ctrl->coarse_rps = 0.0f;
        ctrl->fine_rps = 0.0f;
    }

    ctrl_record_command(ctrl, t_now, ctrl->coarse_rps, ctrl->fine_rps);

    *coarse_rps = ctrl->coarse_rps;
    *fine_rps = ctrl->fine_rps;

    return ctrl->phase;
}


// ---------------------------------------------------------------------------
// Trace
// ---------------------------------------------------------------------------

void trickle_trace_reset(trickle_trace_t * trace, float t0) {
    trace->count = 0;
    trace->stride = 1;
    trace->skipped = 0;
    trace->t0 = t0;
    trace->last_t = t0;
    trace->coarse_rps = 0.0f;
    trace->fine_rps = 0.0f;
    trace->coarse_revs = 0.0f;
    trace->fine_revs = 0.0f;
    trace->coarse_stop_count = 0;
}


static void trace_decimate(trickle_trace_t * trace) {
    uint16_t dst = 0;
    for (uint16_t src = 0; src < trace->count; src += 2) {
        trace->samples[dst++] = trace->samples[src];
    }
    trace->count = dst;
    trace->stride *= 2;
}


void trickle_trace_add(trickle_trace_t * trace, float t, float weight_gr, float coarse_rps, float fine_rps) {
    // Integrate revolutions with the speed commanded since the last sample.
    // Reverse rotation (coarse backoff) does not take powder out of the pan.
    float dt = t - trace->last_t;
    if (dt > 0.0f) {
        trace->coarse_revs += positive(trace->coarse_rps) * dt;
        trace->fine_revs += positive(trace->fine_rps) * dt;
        trace->last_t = t;
    }
    trace->coarse_rps = coarse_rps;
    trace->fine_rps = fine_rps;

    if (isnan(weight_gr) || isinf(weight_gr)) {
        return;
    }

    trace->skipped += 1;
    if (trace->skipped < trace->stride) {
        return;
    }
    trace->skipped = 0;

    if (trace->count >= TRICKLE_TRACE_MAX_SAMPLES) {
        trace_decimate(trace);
    }

    trickle_sample_t * s = &trace->samples[trace->count++];
    s->t = t - trace->t0;
    s->weight_gr = weight_gr;
    s->coarse_revs = trace->coarse_revs;
    s->fine_revs = trace->fine_revs;
}


void trickle_trace_mark_coarse_stop(trickle_trace_t * trace, float t) {
    if (trace->coarse_stop_count < TRICKLE_MAX_COARSE_STOPS) {
        trace->coarse_stops[trace->coarse_stop_count++] = t - trace->t0;
    }
}


// ---------------------------------------------------------------------------
// Learning
// ---------------------------------------------------------------------------

// Solve a symmetric 3x3 system with Gaussian elimination and partial pivoting
static bool solve3(double a[3][3], double b[3], double x[3]) {
    double m[3][4];
    for (int r = 0; r < 3; r += 1) {
        for (int c = 0; c < 3; c += 1) {
            m[r][c] = a[r][c];
        }
        m[r][3] = b[r];
    }

    for (int col = 0; col < 3; col += 1) {
        int pivot = col;
        for (int r = col + 1; r < 3; r += 1) {
            if (fabs(m[r][col]) > fabs(m[pivot][col])) {
                pivot = r;
            }
        }
        if (fabs(m[pivot][col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (int c = 0; c < 4; c += 1) {
                double tmp = m[col][c];
                m[col][c] = m[pivot][c];
                m[pivot][c] = tmp;
            }
        }
        for (int r = col + 1; r < 3; r += 1) {
            double f = m[r][col] / m[col][col];
            for (int c = col; c < 4; c += 1) {
                m[r][c] -= f * m[col][c];
            }
        }
    }

    for (int r = 2; r >= 0; r -= 1) {
        double s = m[r][3];
        for (int c = r + 1; c < 3; c += 1) {
            s -= m[r][c] * x[c];
        }
        x[r] = s / m[r][r];
    }

    return true;
}


// Cumulative revolutions at time t by linear interpolation. The cursor makes
// consecutive look ups with increasing t linear in total.
static void trace_revs_at(const trickle_trace_t * trace, float t, uint16_t * cursor,
                          float * coarse_revs, float * fine_revs) {
    const trickle_sample_t * s = trace->samples;

    if (t <= s[0].t) {
        // Before the first sample the motors had not delivered anything yet
        float r = t <= 0.0f ? 0.0f : t / (s[0].t > 0.0f ? s[0].t : 1.0f);
        *coarse_revs = s[0].coarse_revs * clampf(r, 0.0f, 1.0f);
        *fine_revs = s[0].fine_revs * clampf(r, 0.0f, 1.0f);
        return;
    }

    while (*cursor + 1 < trace->count && s[*cursor + 1].t < t) {
        *cursor += 1;
    }

    if (*cursor + 1 >= trace->count) {
        *coarse_revs = s[trace->count - 1].coarse_revs;
        *fine_revs = s[trace->count - 1].fine_revs;
        return;
    }

    const trickle_sample_t * a = &s[*cursor];
    const trickle_sample_t * b = &s[*cursor + 1];
    float span = b->t - a->t;
    float f = span > 0.0f ? (t - a->t) / span : 0.0f;

    *coarse_revs = a->coarse_revs + (b->coarse_revs - a->coarse_revs) * f;
    *fine_revs = a->fine_revs + (b->fine_revs - a->fine_revs) * f;
}


// Regression features at the delayed time: coarse revs, fine revs, coarse tail
static void trace_features(const trickle_trace_t * trace, float t_delayed, float tail_tau_s,
                           uint16_t * cursor, float x[3]) {
    trace_revs_at(trace, t_delayed, cursor, &x[0], &x[1]);
    x[2] = 0.0f;
    for (uint8_t i = 0; i < trace->coarse_stop_count; i += 1) {
        x[2] += tail_fraction(t_delayed - trace->coarse_stops[i], tail_tau_s);
    }
}


typedef struct {
    float latency_s;
    float tail_tau_s;
    float theta[3];
    float inv_diag[3];      // Diagonal of the inverse normal matrix
    uint16_t points;        // Samples used
    float sse;
    bool ok;
} fit_candidate_t;


static fit_candidate_t fit_at_latency(const trickle_trace_t * trace, const trickle_model_t * prior,
                                      float latency_s, float tail_tau_s) {
    fit_candidate_t cand = {.latency_s = latency_s, .tail_tau_s = tail_tau_s, .ok = false};

    // Normal equations are accumulated in double precision: the coarse and
    // fine revolutions are strongly correlated and single precision loses
    // too many digits.
    double a[3][3] = {{0}};
    double b[3] = {0};
    uint16_t cursor = 0;
    uint16_t step = trace->count / FIT_MAX_POINTS + 1;
    uint16_t used = 0;

    for (uint16_t i = 0; i < trace->count; i += step) {
        const trickle_sample_t * s = &trace->samples[i];
        float x[3];

        trace_features(trace, s->t - latency_s, tail_tau_s, &cursor, x);

        for (int r = 0; r < 3; r += 1) {
            for (int c = 0; c < 3; c += 1) {
                a[r][c] += x[r] * x[c];
            }
            b[r] += x[r] * s->weight_gr;
        }
        used += 1;
    }

    // Ridge regression towards the prior keeps unidentifiable parameters
    // (e.g. no fine-only section in the trace) at their previous value.
    float ridge[3] = {FIT_RIDGE_COARSE, FIT_RIDGE_FINE, FIT_RIDGE_TAIL};
    float prior_theta[3] = {prior->coarse_gpr, prior->fine_gpr, prior->coarse_tail_gr};
    double ar[3][3];
    double br[3];
    double theta[3];
    for (int r = 0; r < 3; r += 1) {
        for (int c = 0; c < 3; c += 1) {
            ar[r][c] = a[r][c];
        }
        ar[r][r] += ridge[r];
        br[r] = b[r] + ridge[r] * prior_theta[r];
    }

    if (!solve3(ar, br, theta)) {
        return cand;
    }
    for (int r = 0; r < 3; r += 1) {
        cand.theta[r] = (float) theta[r];
    }

    // Diagonal of the inverse, used for the parameter standard errors
    for (int r = 0; r < 3; r += 1) {
        double unit[3] = {0.0, 0.0, 0.0};
        double column[3];
        unit[r] = 1.0;
        if (!solve3(ar, unit, column)) {
            return cand;
        }
        cand.inv_diag[r] = (float) column[r];
    }

    // Residuals are summed directly: the expanded normal equation form
    // cancels catastrophically in single precision.
    float sse = 0.0f;
    cursor = 0;
    for (uint16_t i = 0; i < trace->count; i += step) {
        const trickle_sample_t * s = &trace->samples[i];
        float x[3];

        trace_features(trace, s->t - latency_s, tail_tau_s, &cursor, x);
        float residual = s->weight_gr - (cand.theta[0] * x[0] + cand.theta[1] * x[1] + cand.theta[2] * x[2]);
        sse += residual * residual;
    }
    cand.sse = sse;
    cand.points = used;
    cand.ok = true;

    return cand;
}


/*
 * Relative error of the model over windows of one latency: exactly the
 * uncertainty of the in-flight prediction the controller has to live with.
 */
static float fit_flow_cv(const trickle_trace_t * trace, const fit_candidate_t * best) {
    float window = best->latency_s;
    float sum2 = 0.0f;
    int n = 0;
    uint16_t cursor_a = 0;
    uint16_t cursor_b = 0;
    uint16_t j = 0;

    for (uint16_t i = 0; i < trace->count; i += 1) {
        const trickle_sample_t * a = &trace->samples[i];

        while (j < trace->count && trace->samples[j].t < a->t + window) {
            j += 1;
        }
        if (j >= trace->count) {
            break;
        }
        const trickle_sample_t * b = &trace->samples[j];

        float xa[3];
        float xb[3];
        trace_features(trace, a->t - best->latency_s, best->tail_tau_s, &cursor_a, xa);
        trace_features(trace, b->t - best->latency_s, best->tail_tau_s, &cursor_b, xb);

        float predicted = 0.0f;
        for (int k = 0; k < 3; k += 1) {
            predicted += best->theta[k] * (xb[k] - xa[k]);
        }
        if (predicted < FIT_FLOW_MIN_DELTA_GR) {
            continue;
        }

        float relative = ((b->weight_gr - a->weight_gr) - predicted) / predicted;
        sum2 += relative * relative;
        n += 1;
    }

    if (n < 5) {
        return NAN;
    }
    return sqrtf(sum2 / n);
}


// Grid search on the latency with a parabolic refinement around the minimum
static fit_candidate_t fit_search_latency(const trickle_trace_t * trace, const trickle_model_t * prior,
                                          float tail_tau_s, float max_latency_s) {
    int n_steps = (int) ((FIT_LATENCY_MAX_S - LATENCY_MIN_S) / FIT_LATENCY_STEP_S) + 1;
    fit_candidate_t best = {.ok = false};
    float sse_prev = NAN;
    float sse_next = NAN;
    float sse_left = NAN;

    for (int i = 0; i < n_steps; i += 1) {
        float latency = LATENCY_MIN_S + i * FIT_LATENCY_STEP_S;
        if (latency > max_latency_s) {
            break;
        }

        fit_candidate_t cand = fit_at_latency(trace, prior, latency, tail_tau_s);
        if (!cand.ok) {
            sse_left = NAN;
            continue;
        }

        if (!best.ok || cand.sse < best.sse) {
            best = cand;
            sse_prev = sse_left;
            sse_next = NAN;
        }
        else if (isnan(sse_next) && fabsf(latency - best.latency_s - FIT_LATENCY_STEP_S) < 1e-4f) {
            sse_next = cand.sse;
        }
        sse_left = cand.sse;
    }

    if (best.ok && !isnan(sse_prev) && !isnan(sse_next)) {
        float denom = sse_prev - 2.0f * best.sse + sse_next;
        if (denom > 1e-9f) {
            float offset = clampf(0.5f * (sse_prev - sse_next) / denom, -0.5f, 0.5f);
            fit_candidate_t refined = fit_at_latency(trace, prior, best.latency_s + offset * FIT_LATENCY_STEP_S,
                                                     tail_tau_s);
            if (refined.ok && refined.sse <= best.sse) {
                best = refined;
            }
        }
    }

    return best;
}


bool trickle_fit_trace(const trickle_trace_t * trace, const trickle_model_t * prior, trickle_fit_t * fit) {
    static const float tail_tau_candidates[] = {0.05f, 0.15f, 0.3f, 0.5f, 0.8f, 1.2f};

    memset(fit, 0x0, sizeof(trickle_fit_t));

    if (trace->count < FIT_MIN_SAMPLES) {
        return false;
    }

    const trickle_sample_t * last = &trace->samples[trace->count - 1];
    if (last->coarse_revs + last->fine_revs <= 0.0f) {
        return false;
    }

    float max_latency = last->t * 0.5f;
    float tail_tau = clampf(prior->coarse_tail_tau_s, TAIL_TAU_MIN_S, TAIL_TAU_MAX_S);

    // 1. Latency with the prior tail time constant
    fit_candidate_t best = fit_search_latency(trace, prior, tail_tau, max_latency);
    if (!best.ok) {
        return false;
    }

    // 2. Tail time constant at that latency, then 3. latency again
    if (trace->coarse_stop_count > 0) {
        float best_tau = tail_tau;
        for (size_t i = 0; i < sizeof(tail_tau_candidates) / sizeof(tail_tau_candidates[0]); i += 1) {
            fit_candidate_t cand = fit_at_latency(trace, prior, best.latency_s, tail_tau_candidates[i]);
            if (cand.ok && cand.sse < best.sse) {
                best = cand;
                best_tau = tail_tau_candidates[i];
            }
        }

        if (best_tau != tail_tau) {
            fit_candidate_t again = fit_search_latency(trace, prior, best_tau, max_latency);
            if (again.ok && again.sse <= best.sse) {
                best = again;
            }
        }
    }

    fit->coarse_gpr = best.theta[0];
    fit->fine_gpr = best.theta[1];
    fit->coarse_tail_gr = best.theta[2];
    fit->coarse_tail_tau_s = best.tail_tau_s;
    fit->latency_s = best.latency_s;
    fit->rms_residual_gr = sqrtf(best.sse / (best.points > 0 ? best.points : 1));
    fit->coarse_gpr_se = fit->rms_residual_gr * sqrtf(positive(best.inv_diag[0]));
    fit->fine_gpr_se = fit->rms_residual_gr * sqrtf(positive(best.inv_diag[1]));
    fit->coarse_tail_se_gr = fit->rms_residual_gr * sqrtf(positive(best.inv_diag[2]));

    fit->flow_cv = fit_flow_cv(trace, &best);

    fit->valid = fit->coarse_gpr >= COARSE_GPR_MIN && fit->coarse_gpr <= COARSE_GPR_MAX &&
                 fit->fine_gpr >= FINE_GPR_MIN && fit->fine_gpr <= FINE_GPR_MAX &&
                 fit->coarse_tail_gr >= TAIL_MIN_GR && fit->coarse_tail_gr <= TAIL_MAX_GR;

    return fit->valid;
}


/*
 * Weight of a fitted parameter: close to 1 when its standard error is small
 * compared to the precision we want, close to 0 when the throw said little
 * about it (e.g. the fine trickler barely ran on its own).
 */
static float fit_weight(float standard_error, float wanted_precision) {
    if (wanted_precision <= 0.0f) {
        return 1.0f;
    }
    float ratio = standard_error / wanted_precision;
    return 1.0f / (1.0f + ratio * ratio);
}


// Exponentially weighted standard deviation update
static float ew_sd(float sd, float deviation, float gain) {
    float var = (1.0f - gain) * sd * sd + gain * deviation * deviation;
    return sqrtf(var);
}


void trickle_model_learn(trickle_model_t * model, const trickle_fit_t * fit,
                         const trickle_throw_result_t * result, float gain) {
    // Learn fast at the beginning, then settle to the configured gain
    float g = gain;
    float first_throws_gain = 1.0f / (float) (model->learned_throws + 1);
    if (first_throws_gain > g) {
        g = first_throws_gain;
    }
    g = clampf(g, 0.0f, 1.0f);

    if (fit != NULL && fit->valid) {
        bool had_model = (model->flags & TRICKLE_MODEL_FLAG_VALID) != 0;
        float tail_deviation = fit->coarse_tail_gr - model->coarse_tail_gr;

        // Drift: the powder behaves clearly different from what we learned
        if (had_model && model->learned_throws >= 5) {
            float coarse_ratio = fit->coarse_gpr / model->coarse_gpr;
            float fine_ratio = fit->fine_gpr / model->fine_gpr;
            if (fabsf(coarse_ratio - 1.0f) > 0.3f || fabsf(fine_ratio - 1.0f) > 0.4f) {
                model->flags |= TRICKLE_MODEL_FLAG_DRIFT;
            }
            else {
                model->flags &= ~TRICKLE_MODEL_FLAG_DRIFT;
            }
        }

        if (!had_model) {
            model->coarse_gpr = fit->coarse_gpr;
            model->fine_gpr = fit->fine_gpr;
            model->latency_s = fit->latency_s;
            model->coarse_tail_gr = fit->coarse_tail_gr;
            model->coarse_tail_tau_s = fit->coarse_tail_tau_s;
            if (!isnan(fit->flow_cv)) {
                model->flow_cv = fit->flow_cv;
            }
        }
        else {
            float g_coarse = g * fit_weight(fit->coarse_gpr_se, 0.02f * model->coarse_gpr);
            float g_fine = g * fit_weight(fit->fine_gpr_se, 0.03f * model->fine_gpr);
            float g_tail = g * fit_weight(fit->coarse_tail_se_gr, 0.05f);

            model->coarse_gpr = lerpf(model->coarse_gpr, fit->coarse_gpr, g_coarse);
            model->fine_gpr = lerpf(model->fine_gpr, fit->fine_gpr, g_fine);
            model->latency_s = lerpf(model->latency_s, fit->latency_s, g);
            model->coarse_tail_gr = lerpf(model->coarse_tail_gr, fit->coarse_tail_gr, g_tail);
            model->coarse_tail_tau_s = lerpf(model->coarse_tail_tau_s, fit->coarse_tail_tau_s, g_tail);
            model->coarse_tail_sd_gr = ew_sd(model->coarse_tail_sd_gr, tail_deviation, g);
            if (!isnan(fit->flow_cv)) {
                model->flow_cv = lerpf(model->flow_cv, fit->flow_cv, g);
            }
            if (model->coarse_tail_sd_gr < 0.02f) {
                model->coarse_tail_sd_gr = 0.02f;
            }
        }

        model->flags |= TRICKLE_MODEL_FLAG_VALID;
    }

    if (result != NULL && result->adaptive) {
        if (!isnan(result->landing_weight_gr)) {
            // Where did we land compared to where we aimed?
            float aim = result->target_gr - result->aim_below_gr;
            float landing_error = result->landing_weight_gr - aim;

            model->stop_bias_gr += 0.7f * g * landing_error;
            model->landing_sd_gr = ew_sd(model->landing_sd_gr, landing_error, g);
        }

        if (!isnan(result->final_weight_gr)) {
            float final_error = result->final_weight_gr - result->target_gr;
            model->final_mean_gr = lerpf(model->final_mean_gr, final_error, g);
            model->final_sd_gr = ew_sd(model->final_sd_gr, final_error - model->final_mean_gr, g);
        }

        if (result->charge_time_s > 0.0f) {
            if (model->mean_time_s <= 0.0f) {
                model->mean_time_s = result->charge_time_s;
            }
            else {
                model->mean_time_s = lerpf(model->mean_time_s, result->charge_time_s, g);
            }
        }
    }

    if (model->learned_throws < 0xFFFF) {
        model->learned_throws += 1;
    }

    trickle_model_sanitize(model);
}
