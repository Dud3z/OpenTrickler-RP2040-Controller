/*
 * Runs a simulated learning session followed by normal charges and writes
 * the result in the JSON format of the /rest/ai_* endpoints. Used by
 * tests/ai_web_mock.py to serve realistic data to the AI web page.
 *
 *     ./trickle_export > export.json
 */

#include <math.h>
#include <stdio.h>

#include "powder_sim.h"
#include "trickle_model.h"


#define TARGET_GR   42.0f
#define THROWS      30
#define DT_S        0.002f


static float next_reading(powder_sim_t * sim) {
    float reading;
    while (!powder_sim_step(sim, DT_S, &reading)) {
    }
    return reading;
}


int main(void) {
    static powder_sim_t sim;
    static trickle_trace_t trace;
    static trickle_ctrl_t ctrl;
    powder_sim_scale_t scale;
    powder_sim_default_scale(&scale);
    powder_sim_init(&sim, powder_sim_get_preset(POWDER_SIM_SHORT_EXTRUDED), &scale, 4242);

    trickle_model_t model;
    trickle_model_reset(&model);
    trickle_mode_params_t mode;
    trickle_mode_params_from_level(TRICKLE_LEVEL_PRECISION, &mode);
    trickle_motor_limits_t limits = {0.1f, 8.0f, 0.08f, 2.0f};

    printf("{\"history\":[");
    for (int i = 0; i < THROWS; i += 1) {
        powder_sim_set_motors(&sim, 0, 0);
        powder_sim_empty_pan(&sim);
        powder_sim_tare(&sim);
        for (int k = 0; k < 5; k += 1) {
            next_reading(&sim);
        }

        float t0 = sim.t;
        trickle_ctrl_init(&ctrl, TARGET_GR, &model, &mode, &limits, 0.0f, 0.0f);
        trickle_trace_reset(&trace, 0.0f);
        bool probe = i == 0;
        if (probe) {
            trickle_ctrl_start_probe(&ctrl, 0.0f);
        }

        trickle_phase_t phase = ctrl.phase;
        while (!trickle_phase_is_final(phase)) {
            float w = next_reading(&sim);
            float t = sim.t - t0;
            float c, f;
            phase = trickle_ctrl_step(&ctrl, t, w, &c, &f);
            if (ctrl.events & TRICKLE_EVENT_COARSE_STOPPED) {
                trickle_trace_mark_coarse_stop(&trace, t);
            }
            powder_sim_set_motors(&sim, c, f);
            trickle_trace_add(&trace, t, w, c, f);
            if (ctrl.events & TRICKLE_EVENT_PROBE_DONE) {
                trickle_fit_t fit;
                if (trickle_fit_trace(&trace, &model, &fit)) {
                    trickle_model_learn(&model, &fit, NULL, 1.0f);
                    trickle_ctrl_update_model(&ctrl, &model);
                }
            }
        }
        float time_s = sim.t - t0;

        trickle_fit_t fit;
        trickle_throw_result_t result;
        trickle_ctrl_get_result(&ctrl, time_s, &result);
        bool fit_ok = trickle_fit_trace(&trace, &model, &fit);
        trickle_model_learn(&model, fit_ok ? &fit : NULL, &result, i < 8 ? 0.3f : 0.1f);

        int flags = 1 | (i < 8 ? 2 : 0) | (probe ? 4 : 0) | 16;
        printf("%s[%d,0,%.3f,%.3f,%.3f,%.2f,%d,%d,0]", i ? "," : "", i + 1, TARGET_GR,
               ctrl.final_weight_gr, ctrl.landing_weight_gr, time_s, ctrl.topup_count, flags);
    }

    printf("],\"model\":{\"pf\":0,\"pn\":\"Varget,gr\",\"kc\":%.4f,\"kf\":%.4f,\"lat\":%.3f,\"tl\":%.3f,\"tsd\":%.3f,"
           "\"tau\":%.3f,\"cv\":%.3f,\"bias\":%.3f,\"lsd\":%.3f,\"fm\":%.3f,\"fsd\":%.3f,\"mt\":%.1f,\"n\":%u,\"fl\":%u},",
           model.coarse_gpr, model.fine_gpr, model.latency_s, model.coarse_tail_gr, model.coarse_tail_sd_gr,
           model.coarse_tail_tau_s, model.flow_cv, model.stop_bias_gr, model.landing_sd_gr, model.final_mean_gr,
           model.final_sd_gr, model.mean_time_s, model.learned_throws, model.flags | TRICKLE_MODEL_FLAG_CONVERGED);

    printf("\"trace\":{\"n\":%u,\"cs\":[", trace.count);
    for (int i = 0; i < trace.coarse_stop_count; i += 1) {
        printf("%s%.2f", i ? "," : "", trace.coarse_stops[i]);
    }
    printf("],\"p\":[");
    int step = trace.count / 150 + 1;
    for (int i = 0; i < trace.count; i += step) {
        const trickle_sample_t * s = &trace.samples[i];
        printf("%s[%.2f,%.3f,%.2f,%.2f]", i ? "," : "", s->t, s->weight_gr, s->coarse_revs, s->fine_revs);
    }
    printf("]}}\n");

    return 0;
}
