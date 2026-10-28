/*
 * Host side benchmark for the adaptive trickle controller.
 *
 * Runs the classic PID charge loop (same logic as charge_mode.cpp) and the
 * adaptive controller against the physical powder simulator and prints
 * accuracy and speed statistics for each powder type and precision level.
 *
 * Build and run:
 *
 *     make -C tests/sim run
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "powder_sim.h"
#include "trickle_model.h"


#define SIM_DT_S            0.002f
#define TARGET_GR           42.0f
#define LEARN_THROWS        8
#define EVAL_THROWS         150
#define LEARN_GAIN          0.3f
#define CONTINUOUS_GAIN     0.1f

#ifndef BENCH_ACCEL
#define BENCH_ACCEL         50.0f       // Motor ramp, same as powder_sim accel_rps2
#endif


typedef struct {
    // Mirrors profile_t / eeprom_charge_mode_data_t defaults used by the firmware
    float coarse_kp, coarse_ki, coarse_kd;
    float coarse_min_rps, coarse_max_rps;
    float fine_kp, fine_ki, fine_kd;
    float fine_min_rps, fine_max_rps;
    float coarse_stop_threshold;
    float fine_stop_threshold;
    float backoff_turns;
    float backoff_rps;
} classic_profile_t;


typedef struct {
    float final_gr;         // Settled scale reading
    float true_gr;          // True powder in the pan
    float time_s;
    int topups;
    bool fault;
    float phase_s[TRICKLE_PHASE_PROBE + 1];  // Time spent in each controller phase
} throw_outcome_t;


typedef struct {
    int n;
    double sum, sum2;
    int within_002, within_004, within_010;
    int over_tol;
    int under_tol;
    int gross_over;
    int reading_in_tol;
    double time_sum;
    int faults;
} stats_t;


static const classic_profile_t classic_profile = {
    // "AR2209,gr" default profile
    .coarse_kp = 0.05f, .coarse_ki = 0.0f, .coarse_kd = 0.3f,
    .coarse_min_rps = 0.1f, .coarse_max_rps = 8.0f,
    .fine_kp = 0.8f, .fine_ki = 0.0f, .fine_kd = 15.0f,
    .fine_min_rps = 0.08f, .fine_max_rps = 2.0f,
    .coarse_stop_threshold = 5.0f,
    .fine_stop_threshold = 0.03f,
    .backoff_turns = 0.25f,
    .backoff_rps = 1.0f,
};


static bool bench_debug = false;
static bool bench_throws = false;
static bool bench_trace_now = false;
static int bench_trace_throw = -1;
static const char * bench_config = NULL;


static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}


// Run the sim until the scale reports a new value
static float next_reading(powder_sim_t * sim) {
    float reading;
    while (!powder_sim_step(sim, SIM_DT_S, &reading)) {
    }
    return reading;
}




static void prepare_pan(powder_sim_t * sim) {
    powder_sim_set_motors(sim, 0.0f, 0.0f);
    powder_sim_empty_pan(sim);
    powder_sim_tare(sim);
    for (int i = 0; i < 5; i += 1) {
        next_reading(sim);
    }
}


/*
 * Classic charge loop, same structure as charge_mode_wait_for_complete().
 * Optionally records a trace so that the adaptive model can learn from it.
 */
static throw_outcome_t run_classic(powder_sim_t * sim, const classic_profile_t * p, float target,
                                   trickle_trace_t * trace) {
    throw_outcome_t out = {0};
    prepare_pan(sim);

    float t_start = sim->t;
    if (trace) {
        trickle_trace_reset(trace, t_start);
    }

    float coarse_integral = 0.0f, fine_integral = 0.0f;
    float coarse_last_error = 0.0f, fine_last_error = 0.0f;
    float coarse_target = fmaxf(0.0f, target - p->coarse_stop_threshold);
    bool coarse_move = true;
    bool backoff = false;
    float backoff_end = 0.0f;
    float last_sample_t = sim->t;
    float coarse_cmd = 0.0f, fine_cmd = 0.0f;

    while (true) {
        float w = next_reading(sim);
        float now = sim->t;

        if (backoff && now >= backoff_end) {
            coarse_cmd = 0.0f;
            backoff = false;
        }

        float coarse_error = coarse_target - w;
        float fine_error = target - w;

        if (fine_error < p->fine_stop_threshold) {
            coarse_cmd = 0.0f;
            fine_cmd = 0.0f;
            powder_sim_set_motors(sim, 0.0f, 0.0f);
            if (trace) {
                trickle_trace_add(trace, now, w, 0.0f, 0.0f);
            }
            break;
        }
        else if (coarse_error <= 0 && coarse_move) {
            coarse_move = false;
            coarse_cmd = 0.0f;
            if (trace) {
                trickle_trace_mark_coarse_stop(trace, now);
            }
            if (p->backoff_turns > 0.0f) {
                coarse_cmd = -p->backoff_rps;
                backoff = true;
                backoff_end = now + p->backoff_turns / p->backoff_rps;
            }
        }

        // Elapsed time in ms like the firmware tick based implementation
        float elapse_ms = (now - last_sample_t) * 1000.0f;
        if (elapse_ms < 1.0f) {
            elapse_ms = 1.0f;
        }

        fine_integral += fine_error;
        float fine_derivative = (fine_error - fine_last_error) / elapse_ms;
        float speed = p->fine_kp * fine_error + p->fine_ki * fine_integral + p->fine_kd * fine_derivative;
        fine_cmd = clampf(speed, p->fine_min_rps, p->fine_max_rps);

        if (coarse_move && !backoff) {
            coarse_integral += coarse_error;
            float coarse_derivative = (coarse_error - coarse_last_error) / elapse_ms;
            speed = p->coarse_kp * coarse_error + p->coarse_ki * coarse_integral + p->coarse_kd * coarse_derivative;
            coarse_cmd = clampf(speed, p->coarse_min_rps, p->coarse_max_rps);
        }

        powder_sim_set_motors(sim, coarse_cmd, fine_cmd);
        if (trace) {
            trickle_trace_add(trace, now, w, coarse_cmd, fine_cmd);
        }

        last_sample_t = now;
        fine_last_error = fine_error;
        coarse_last_error = coarse_error;

        if (now - t_start > 300.0f) {
            out.fault = true;
            break;
        }
    }

    out.time_s = sim->t - t_start;

    // Let the scale settle, record the tail for the fit
    for (int i = 0; i < 30; i += 1) {
        float w = next_reading(sim);
        if (trace) {
            trickle_trace_add(trace, sim->t, w, 0.0f, 0.0f);
        }
    }
    out.final_gr = sim->last_reading_gr;
    out.true_gr = powder_sim_true_weight(sim);

    return out;
}


static trickle_motor_limits_t limits_from_profile(const classic_profile_t * p) {
    trickle_motor_limits_t limits = {
        .coarse_min_rps = p->coarse_min_rps,
        .coarse_max_rps = p->coarse_max_rps,
        .fine_min_rps = p->fine_min_rps,
        .fine_max_rps = p->fine_max_rps,
        .coarse_accel_rps2 = BENCH_ACCEL,
        .fine_accel_rps2 = BENCH_ACCEL,
    };
    return limits;
}


static throw_outcome_t run_adaptive(powder_sim_t * sim, const classic_profile_t * p, float target,
                                    trickle_model_t * model, const trickle_mode_params_t * mode_in,
                                    trickle_trace_t * trace, trickle_throw_result_t * result, bool probe) {
    throw_outcome_t out = {0};
    prepare_pan(sim);

    trickle_mode_params_t mode = *mode_in;
    trickle_motor_limits_t limits = limits_from_profile(p);

    static trickle_ctrl_t ctrl;
    float t_start = sim->t;
    float scale_period = sim->scale.sample_period_s;
    trickle_ctrl_init(&ctrl, target, model, &mode, &limits, t_start, 0.0f);
    trickle_trace_reset(trace, t_start);
    if (probe) {
        trickle_ctrl_start_probe(&ctrl, t_start);
    }

    bool backoff = false;
    float backoff_end = 0.0f;
    bool coarse_stop_hold = false;   // Same as charge_mode.cpp

    while (true) {
        float w = next_reading(sim);
        float now = sim->t;
        float coarse_rps, fine_rps;

        trickle_phase_t prev_phase = ctrl.phase;
        trickle_phase_t phase = trickle_ctrl_step(&ctrl, now, w, &coarse_rps, &fine_rps);
        if (prev_phase <= TRICKLE_PHASE_PROBE) {
            out.phase_s[prev_phase] += scale_period;
        }

        if (ctrl.events & TRICKLE_EVENT_COARSE_STOPPED) {
            coarse_stop_hold = true;
            trickle_trace_mark_coarse_stop(trace, now);
            if (p->backoff_turns > 0.0f) {
                backoff = true;
                backoff_end = now + p->backoff_turns / p->backoff_rps;
            }
        }
        if (ctrl.events & TRICKLE_EVENT_TOPUP_STARTED) {
            out.topups += 1;
        }
        if (ctrl.events & TRICKLE_EVENT_PROBE_DONE) {
            // Learn from the probe pulses before charging the rest
            trickle_fit_t probe_fit;
            if (trickle_fit_trace(trace, model, &probe_fit)) {
                trickle_model_learn(model, &probe_fit, NULL, 1.0f);
                trickle_ctrl_update_model(&ctrl, model);
            }
        }
        if (coarse_stop_hold && coarse_rps <= 0.0f) {
            coarse_stop_hold = false;
        }
        if (coarse_stop_hold) {
            coarse_rps = 0.0f;
        }
        if (backoff) {
            if (now >= backoff_end) {
                backoff = false;
            }
            else {
                coarse_rps = -p->backoff_rps;
            }
        }

        powder_sim_set_motors(sim, coarse_rps, fine_rps);
        trickle_trace_add(trace, now, w, coarse_rps, fine_rps);
        if (bench_debug && bench_trace_now) {
            printf("    t %6.2f ph %d w %7.3f pred %7.3f c %5.2f f %5.2f\n", now - t_start, phase, w,
                   ctrl.predicted_gr, coarse_rps, fine_rps);
        }

        if (trickle_phase_is_final(phase)) {
            out.fault = phase != TRICKLE_PHASE_DONE;
            break;
        }
    }

    out.time_s = sim->t - t_start;
    out.final_gr = isnan(ctrl.final_weight_gr) ? sim->last_reading_gr : ctrl.final_weight_gr;

    // Keep recording a little longer so the fit sees the settled weight
    for (int i = 0; i < 10; i += 1) {
        float w = next_reading(sim);
        trickle_trace_add(trace, sim->t, w, 0.0f, 0.0f);
    }
    out.true_gr = powder_sim_true_weight(sim);

    trickle_ctrl_get_result(&ctrl, out.time_s, result);
    result->final_weight_gr = out.final_gr;

    return out;
}


static void stats_add(stats_t * s, const throw_outcome_t * o, float target, float tol) {
    float e = o->true_gr - target;
    s->n += 1;
    s->sum += e;
    s->sum2 += e * e;
    s->within_002 += fabsf(e) <= 0.02f + 1e-4f;
    s->within_004 += fabsf(e) <= 0.04f + 1e-4f;
    s->within_010 += fabsf(e) <= 0.10f + 1e-4f;
    s->over_tol += e > tol + 1e-4f;
    s->under_tol += e < -tol - 1e-4f;
    s->gross_over += e > 0.1f;
    s->reading_in_tol += fabsf(o->final_gr - target) <= tol + 1e-3f;
    s->time_sum += o->time_s;
    s->faults += o->fault;
}


static void stats_print(const char * label, const stats_t * s) {
    double mean = s->sum / s->n;
    double sd = sqrt(fmax(0.0, s->sum2 / s->n - mean * mean));
    printf("  %-38s mean %+6.3f  sd %5.3f  |e|<=.02 %5.1f%%  <=.04 %5.1f%%  <=.10 %5.1f%%  over-tol %5.1f%%  time %5.1fs%s\n",
           label, mean, sd,
           100.0 * s->within_002 / s->n, 100.0 * s->within_004 / s->n, 100.0 * s->within_010 / s->n,
           100.0 * s->over_tol / s->n, s->time_sum / s->n,
           s->faults ? "  FAULTS!" : "");
}


static void print_model(const trickle_model_t * m, const powder_sim_powder_t * powder) {
    printf("  learned: coarse %.3f gr/rev (true %.3f), fine %.4f gr/rev (true %.4f), latency %.2fs, "
           "tail %.2f+-%.2f gr, flow cv %.2f, bias %+.3f, landing sd %.3f\n",
           m->coarse_gpr, powder->coarse_gpr, m->fine_gpr, powder->fine_gpr, m->latency_s,
           m->coarse_tail_gr, m->coarse_tail_sd_gr, m->flow_cv, m->stop_bias_gr, m->landing_sd_gr);
}


typedef struct {
    stats_t stats;
    int topups;
    double phase_s[TRICKLE_PHASE_PROBE + 1];
    float predicted_s;
    trickle_model_t model;
} config_result_t;


// Learning session followed by an evaluation with continuous learning
static void run_config(powder_sim_t * sim, trickle_trace_t * trace, const powder_sim_powder_t * powder,
                       const powder_sim_scale_t * scale, uint32_t seed, const classic_profile_t * p,
                       const trickle_mode_params_t * mode, float target, int eval_throws,
                       config_result_t * out) {
    memset(out, 0, sizeof(*out));
    powder_sim_init(sim, powder, scale, seed);
    trickle_model_reset(&out->model);

    // The first throw starts with probe pulses
    trickle_fit_t fit;
    for (int i = 0; i < LEARN_THROWS; i += 1) {
        trickle_throw_result_t result;
        run_adaptive(sim, p, target, &out->model, mode, trace, &result, i == 0);
        trickle_fit_trace(trace, &out->model, &fit);
        trickle_model_learn(&out->model, &fit, &result, LEARN_GAIN);
    }

    for (int i = 0; i < eval_throws; i += 1) {
        trickle_throw_result_t result;
        bench_trace_now = bench_trace_throw < 0 || bench_trace_throw == i;
        throw_outcome_t o = run_adaptive(sim, p, target, &out->model, mode, trace, &result, false);
        out->topups += o.topups;
        if (bench_throws) {
            printf("    throw %3d landing %+6.3f final %+6.3f true %+6.3f top ups %d time %5.1fs  lsd %.3f aim %.3f bias %+.3f cv %.2f lat %.2f\n",
                   i, result.landing_weight_gr - target, o.final_gr - target, o.true_gr - target, o.topups, o.time_s,
                   out->model.landing_sd_gr, result.aim_below_gr, out->model.stop_bias_gr, out->model.flow_cv,
                   out->model.latency_s);
        }
        for (int ph = 0; ph <= TRICKLE_PHASE_PROBE; ph += 1) {
            out->phase_s[ph] += o.phase_s[ph];
        }
        stats_add(&out->stats, &o, target, mode->tolerance_gr);
        trickle_fit_trace(trace, &out->model, &fit);
        trickle_model_learn(&out->model, &fit, &result, CONTINUOUS_GAIN);
    }

    trickle_motor_limits_t limits = limits_from_profile(p);
    out->predicted_s = trickle_model_estimate_time(&out->model, mode, &limits, target);
}


// Like run_config, but the speed is tuned automatically after every charge
static void run_config_auto(powder_sim_t * sim, trickle_trace_t * trace, const powder_sim_powder_t * powder,
                            const powder_sim_scale_t * scale, uint32_t seed, const classic_profile_t * p,
                            float tolerance, float target, int warmup_throws, int eval_throws,
                            config_result_t * out, trickle_speed_state_t * speed) {
    memset(out, 0, sizeof(*out));
    powder_sim_init(sim, powder, scale, seed);
    trickle_model_reset(&out->model);
    trickle_speed_reset(speed, TRICKLE_LEVEL_BALANCED);

    trickle_fit_t fit;
    trickle_mode_params_t mode;
    for (int i = 0; i < LEARN_THROWS + warmup_throws + eval_throws; i += 1) {
        bool learning = i < LEARN_THROWS;
        bool eval = i >= LEARN_THROWS + warmup_throws;
        uint8_t speed_used = speed->speed;
        trickle_mode_params(speed_used, tolerance, &mode);

        trickle_throw_result_t result;
        throw_outcome_t o = run_adaptive(sim, p, target, &out->model, &mode, trace, &result, i == 0);
        trickle_fit_trace(trace, &out->model, &fit);
        trickle_model_learn(&out->model, &fit, &result, learning ? LEARN_GAIN : CONTINUOUS_GAIN);
        if (learning) {
            continue;
        }

        // The controller only knows the scale reading
        bool over = o.final_gr > target + tolerance + 1e-3f;
        trickle_speed_update(speed, over, (uint8_t) o.topups, o.time_s);

        if (eval) {
            out->topups += o.topups;
            for (int ph = 0; ph <= TRICKLE_PHASE_PROBE; ph += 1) {
                out->phase_s[ph] += o.phase_s[ph];
            }
            stats_add(&out->stats, &o, target, tolerance);
        }
        if (bench_throws) {
            printf("    throw %3d speed %3u final %+6.3f true %+6.3f top ups %d time %5.1fs\n", i, (unsigned) speed_used,
                   o.final_gr - target, o.true_gr - target, o.topups, o.time_s);
        }
    }
}


/*
 * Speed study: how fast can a charge be thrown while keeping a tight tolerance,
 * depending on the speed setting and the motor speed limits of the profile.
 */
static int speed_study(int eval_throws) {
    static powder_sim_t sim;
    static trickle_trace_t trace;
    powder_sim_scale_t scale;
    powder_sim_default_scale(&scale);

    const float target = 44.0f;
    const float tolerances[] = {0.02f, 0.04f};
    const uint8_t speeds[] = {0, 50, 100};

    classic_profile_t fast = classic_profile;
    fast.coarse_max_rps = 15.0f;
    fast.fine_max_rps = 5.0f;
    const classic_profile_t * profiles[] = {&classic_profile, &fast};
    const char * profile_names[] = {"limits 8/2 rps", "limits 15/5 rps"};

    int failures = 0;
    printf("Speed study, target %.1f gr, %d throws per configuration\n", target, eval_throws);

    const powder_sim_preset_t presets[] = {POWDER_SIM_BALL, POWDER_SIM_SHORT_EXTRUDED};
    for (unsigned pr = 0; pr < sizeof(presets) / sizeof(presets[0]); pr += 1) {
        const powder_sim_powder_t * powder = powder_sim_get_preset(presets[pr]);
        printf("\n%s\n", powder->name);
        for (int pi = 0; pi < 2; pi += 1) {
            for (int ti = 0; ti < 2; ti += 1) {
                for (int si = 0; si < 4; si += 1) {
                    if (bench_config) {
                        char key[32];
                        snprintf(key, sizeof(key), "%u,%d,%d,%d", pr, pi, ti, si);
                        if (strcmp(key, bench_config) != 0) {
                            continue;
                        }
                    }
                    config_result_t r;
                    trickle_speed_state_t auto_speed;
                    if (si < 3) {
                        trickle_mode_params_t mode;
                        trickle_mode_params(speeds[si], tolerances[ti], &mode);
                        run_config(&sim, &trace, powder, &scale, 5000 + pr * 100 + pi * 10 + si, profiles[pi],
                                   &mode, target, eval_throws, &r);
                    }
                    else {
                        run_config_auto(&sim, &trace, powder, &scale, 5000 + pr * 100 + pi * 10 + si, profiles[pi],
                                        tolerances[ti], target, 100, eval_throws, &r, &auto_speed);
                    }
                    const stats_t * st = &r.stats;
                    double mean = st->sum / st->n;
                    char speed_label[16];
                    if (si < 3) {
                        snprintf(speed_label, sizeof(speed_label), "%3u", speeds[si]);
                    }
                    else {
                        snprintf(speed_label, sizeof(speed_label), "A%2u", auto_speed.speed);
                    }
                    printf("  %-15s tol %.2f speed %s  mean %+6.3f sd %5.3f  in tol %5.1f%% (scale %5.1f%%)  over %4.1f%% (>0.1 %d)  "
                           "time %5.1fs  top ups %.2f  (coarse %.1f fine %.1f settle %.1f top up %.1f)\n",
                           profile_names[pi], tolerances[ti], speed_label, mean,
                           sqrt(fmax(0.0, st->sum2 / st->n - mean * mean)),
                           100.0 * (st->n - st->over_tol - st->under_tol) / st->n, 100.0 * st->reading_in_tol / st->n,
                           100.0 * st->over_tol / st->n, st->gross_over,
                           st->time_sum / st->n, (double) r.topups / eval_throws,
                           r.phase_s[TRICKLE_PHASE_COARSE] / eval_throws, r.phase_s[TRICKLE_PHASE_FINE] / eval_throws,
                           r.phase_s[TRICKLE_PHASE_SETTLE] / eval_throws, r.phase_s[TRICKLE_PHASE_TOPUP] / eval_throws);
                    if (bench_throws) {
                        print_model(&r.model, powder);
                        printf("  tail tau %.2fs, coarse tail sd %.3f\n", r.model.coarse_tail_tau_s, r.model.coarse_tail_sd_gr);
                    }
                    failures += r.stats.faults != 0;
                }
            }
        }
    }
    return failures ? 1 : 0;
}


/*
 * Strategy sweep: the same charges with different settle / top up
 * strategies (trickle_tuning_t), to pick the defaults.
 */
typedef struct {
    const char * name;
    trickle_tuning_t tuning;
} tuning_variant_t;


static int tuning_sweep(int eval_throws) {
    static powder_sim_t sim;
    static trickle_trace_t trace;
    powder_sim_scale_t scale;
    powder_sim_default_scale(&scale);

    const trickle_tuning_t base = trickle_tuning;
    tuning_variant_t variants[24];
    int n = 0;
    trickle_tuning_t t;
    // The defaults won the first sweep; the old strategy for comparison
    trickle_tuning_t best = base;
    t = base; t.early_topup = 0; t.topup_settle_extra_s = 0.25f; t.topup_ratio = 0.7f;
    variants[n++] = (tuning_variant_t) {"previous", t};
    variants[n++] = (tuning_variant_t) {"current", best};
    t = base; t.topup_style = TRICKLE_TOPUP_TRAIN;
    variants[n++] = (tuning_variant_t) {"train 0.1s/0.5lat", t};
    t = base; t.topup_ratio = 0.7f;
    variants[n++] = (tuning_variant_t) {"ratio 0.7", t};
    t = best; t.fine_prime_s = 0.8f;
    variants[n++] = (tuning_variant_t) {"+ prime 0.8s", t};
    t = best; t.fine_prime_s = 2.5f;
    variants[n++] = (tuning_variant_t) {"+ prime 2.5s", t};
    t = best; t.coarse_handover_ratio = 0.6f;
    variants[n++] = (tuning_variant_t) {"+ handover 0.6", t};
    t = best; t.coarse_handover_ratio = 1.6f;
    variants[n++] = (tuning_variant_t) {"+ handover 1.6", t};
    t = best; t.end_allowance_factor = 0.25f;
    variants[n++] = (tuning_variant_t) {"+ end allowance 0.25", t};
    t = best; t.end_allowance_factor = 0.8f;
    variants[n++] = (tuning_variant_t) {"+ end allowance 0.8", t};
    t = best; t.settle_extra_s = 0.15f;
    variants[n++] = (tuning_variant_t) {"+ settle 0.15", t};

    classic_profile_t fast = classic_profile;
    fast.coarse_max_rps = 15.0f;
    fast.fine_max_rps = 5.0f;
    const float targets[] = {23.0f, 44.0f};
    const uint8_t speeds[] = {50, 100};
    const powder_sim_preset_t presets[] = {POWDER_SIM_BALL, POWDER_SIM_SHORT_EXTRUDED, POWDER_SIM_FLAKE};

    printf("Strategy sweep, limits 15/5 rps, tolerance 0.02, %d throws per cell\n", eval_throws);
    printf("  %-22s", "variant");
    for (unsigned pr = 0; pr < 3; pr += 1) {
        printf(" | %-26s", powder_sim_get_preset(presets[pr])->name);
    }
    printf(" | all\n");

    for (int v = 0; v < n; v += 1) {
        trickle_tuning = variants[v].tuning;
        printf("  %-22s", variants[v].name);
        double all_time = 0.0, all_in = 0.0, all_over = 0.0;
        int cells = 0;
        for (unsigned pr = 0; pr < 3; pr += 1) {
            double time = 0.0, in = 0.0, over = 0.0;
            int k = 0;
            for (unsigned ti = 0; ti < 2; ti += 1) {
                for (unsigned si = 0; si < 2; si += 1) {
                    trickle_mode_params_t mode;
                    trickle_mode_params(speeds[si], 0.02f, &mode);
                    config_result_t r;
                    run_config(&sim, &trace, powder_sim_get_preset(presets[pr]), &scale,
                               7000 + pr * 100 + ti * 10 + si, &fast, &mode, targets[ti], eval_throws, &r);
                    time += r.stats.time_sum / r.stats.n;
                    in += 100.0 * r.stats.reading_in_tol / r.stats.n;
                    over += 100.0 * r.stats.over_tol / r.stats.n;
                    k += 1;
                }
            }
            printf(" | %5.1fs in %5.1f%% ov %4.1f%% =%5.1f", time / k, in / k, over / k, time / k / (in / k / 100.0));
            all_time += time / k; all_in += in / k; all_over += over / k; cells += 1;
        }
        printf(" | %5.1fs %5.1f%% %4.1f%% =%5.1f s per good charge\n", all_time / cells, all_in / cells, all_over / cells,
               all_time / cells / (all_in / cells / 100.0));
    }
    trickle_tuning = base;
    return 0;
}


int main(int argc, char ** argv) {
    int eval_throws = EVAL_THROWS;
    if (argc > 1 && strcmp(argv[1], "sweep") == 0) {
        return tuning_sweep(argc > 2 ? atoi(argv[2]) : 60);
    }
    if (argc > 1 && strcmp(argv[1], "study") == 0) {
        bench_debug = getenv("BENCH_DEBUG") != NULL;
        bench_throws = getenv("BENCH_THROWS") != NULL;
        bench_trace_throw = getenv("BENCH_TRACE_THROW") ? atoi(getenv("BENCH_TRACE_THROW")) : -1;
        bench_config = getenv("BENCH_CONFIG");
        return speed_study(argc > 2 ? atoi(argv[2]) : 60);
    }
    if (argc > 1) {
        eval_throws = atoi(argv[1]);
    }

    static powder_sim_t sim;
    static trickle_trace_t trace;
    powder_sim_scale_t scale;
    powder_sim_default_scale(&scale);

    const uint8_t levels[] = {TRICKLE_LEVEL_PRECISION, TRICKLE_LEVEL_BALANCED, TRICKLE_LEVEL_SPEED};
    const char * level_names[] = {"adaptive precision", "adaptive balanced", "adaptive speed"};
    int failures = 0;

    printf("Target %.2f gr, %d throws per configuration\n", TARGET_GR, eval_throws);

    for (int preset = 0; preset < POWDER_SIM_PRESET_COUNT; preset += 1) {
        const powder_sim_powder_t * powder = powder_sim_get_preset((powder_sim_preset_t) preset);
        printf("\n%s (kernel %.3f gr)\n", powder->name, powder->kernel_gr);

        // Baseline: classic PID
        stats_t classic = {0};
        powder_sim_init(&sim, powder, &scale, 1000 + preset);
        for (int i = 0; i < eval_throws; i += 1) {
            throw_outcome_t o = run_classic(&sim, &classic_profile, TARGET_GR, NULL);
            stats_add(&classic, &o, TARGET_GR, 0.04f);
        }
        stats_print("classic PID", &classic);

        for (int l = 0; l < 3; l += 1) {
            trickle_mode_params_t mode;
            trickle_mode_params_from_level(levels[l], &mode);
            config_result_t r;
            run_config(&sim, &trace, powder, &scale, 2000 + preset * 10 + l, &classic_profile, &mode,
                       TARGET_GR, eval_throws, &r);

            stats_print(level_names[l], &r.stats);
            printf("  %-22s tolerance %.3f gr, %.2f top ups per throw, predicted time %.1fs\n", "",
                   mode.tolerance_gr, (double) r.topups / eval_throws, r.predicted_s);
            print_model(&r.model, powder);

            if (r.stats.faults) {
                failures += 1;
            }
        }
    }

    return failures ? 1 : 0;
}
