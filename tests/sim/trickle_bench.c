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
} throw_outcome_t;


typedef struct {
    int n;
    double sum, sum2;
    int within_002, within_004, within_010;
    int over_tol;
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
    };
    return limits;
}


static throw_outcome_t run_adaptive(powder_sim_t * sim, const classic_profile_t * p, float target,
                                    trickle_model_t * model, uint8_t level, trickle_trace_t * trace,
                                    trickle_throw_result_t * result, bool probe) {
    throw_outcome_t out = {0};
    prepare_pan(sim);

    trickle_mode_params_t mode;
    trickle_mode_params_from_level(level, &mode);
    trickle_motor_limits_t limits = limits_from_profile(p);

    static trickle_ctrl_t ctrl;
    float t_start = sim->t;
    trickle_ctrl_init(&ctrl, target, model, &mode, &limits, t_start, 0.0f);
    trickle_trace_reset(trace, t_start);
    if (probe) {
        trickle_ctrl_start_probe(&ctrl, t_start);
    }

    bool backoff = false;
    float backoff_end = 0.0f;

    while (true) {
        float w = next_reading(sim);
        float now = sim->t;
        float coarse_rps, fine_rps;

        trickle_phase_t phase = trickle_ctrl_step(&ctrl, now, w, &coarse_rps, &fine_rps);

        if (ctrl.events & TRICKLE_EVENT_COARSE_STOPPED) {
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
    s->time_sum += o->time_s;
    s->faults += o->fault;
}


static void stats_print(const char * label, const stats_t * s) {
    double mean = s->sum / s->n;
    double sd = sqrt(fmax(0.0, s->sum2 / s->n - mean * mean));
    printf("  %-22s mean %+6.3f  sd %5.3f  |e|<=.02 %5.1f%%  <=.04 %5.1f%%  <=.10 %5.1f%%  over-tol %5.1f%%  time %5.1fs%s\n",
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


int main(int argc, char ** argv) {
    int eval_throws = EVAL_THROWS;
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

            powder_sim_init(&sim, powder, &scale, 2000 + preset * 10 + l);
            trickle_model_t model;
            trickle_model_reset(&model);

            // Learning session: the first throw starts with probe pulses
            trickle_fit_t fit;
            for (int i = 0; i < LEARN_THROWS; i += 1) {
                trickle_throw_result_t result;
                run_adaptive(&sim, &classic_profile, TARGET_GR, &model, levels[l], &trace, &result, i == 0);
                trickle_fit_trace(&trace, &model, &fit);
                trickle_model_learn(&model, &fit, &result, LEARN_GAIN);
            }

            // Evaluation with continuous learning
            stats_t adaptive = {0};
            int topups = 0;
            for (int i = 0; i < eval_throws; i += 1) {
                trickle_throw_result_t result;
                throw_outcome_t o = run_adaptive(&sim, &classic_profile, TARGET_GR, &model, levels[l], &trace, &result, false);
                topups += o.topups;
                stats_add(&adaptive, &o, TARGET_GR, mode.tolerance_gr);
                trickle_fit_trace(&trace, &model, &fit);
                trickle_model_learn(&model, &fit, &result, CONTINUOUS_GAIN);
            }

            char label[64];
            snprintf(label, sizeof(label), "%s", level_names[l]);
            stats_print(label, &adaptive);
            printf("  %-22s tolerance %.3f gr, %.2f top ups per throw, predicted time %.1fs\n", "",
                   mode.tolerance_gr, (double) topups / eval_throws,
                   trickle_model_estimate_time(&model, &mode, &(trickle_motor_limits_t){
                       classic_profile.coarse_min_rps, classic_profile.coarse_max_rps,
                       classic_profile.fine_min_rps, classic_profile.fine_max_rps}, TARGET_GR));
            print_model(&model, powder);

            if (adaptive.faults) {
                failures += 1;
            }
        }
    }

    return failures ? 1 : 0;
}
