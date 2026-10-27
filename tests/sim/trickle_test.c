/*
 * Unit tests for trickle_model.c
 *
 *     make -C tests/sim run
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "trickle_model.h"
#include "sha256.h"


static int failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { \
        failures += 1; \
        printf("FAIL %s:%d: ", __FILE__, __LINE__); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } \
} while (0)


/*
 * Ideal plant without noise: powder lands exactly latency seconds after the
 * motor delivered it, the coarse trickler drops a fixed tail on stop.
 */
typedef struct {
    float coarse_gpr, fine_gpr, latency, tail;
    float t;
    float hist_t[4096];
    float hist_mass[4096];
    int n;
    float delivered;
    float coarse_rps, fine_rps;
    bool coarse_was_running;
    bool dead;      // No powder at all (empty hopper)
} plant_t;


static void plant_step(plant_t * p, float dt) {
    p->t += dt;
    if (!p->dead) {
        p->delivered += (p->coarse_gpr * fmaxf(0, p->coarse_rps) + p->fine_gpr * fmaxf(0, p->fine_rps)) * dt;
        bool running = p->coarse_rps > 0.0f;
        if (p->coarse_was_running && !running) {
            p->delivered += p->tail;
        }
        p->coarse_was_running = running;
    }
    p->hist_t[p->n % 4096] = p->t;
    p->hist_mass[p->n % 4096] = p->delivered;
    p->n += 1;
}


static float plant_reading(const plant_t * p) {
    float t_seen = p->t - p->latency;
    for (int i = p->n - 1; i >= 0 && i > p->n - 4096; i -= 1) {
        if (p->hist_t[i % 4096] <= t_seen) {
            // Interpolate to the next entry so the latency is exact
            if (i + 1 < p->n) {
                float t0 = p->hist_t[i % 4096];
                float t1 = p->hist_t[(i + 1) % 4096];
                float m0 = p->hist_mass[i % 4096];
                float m1 = p->hist_mass[(i + 1) % 4096];
                float f = t1 > t0 ? (t_seen - t0) / (t1 - t0) : 0.0f;
                return m0 + (m1 - m0) * f;
            }
            return p->hist_mass[i % 4096];
        }
    }
    return 0.0f;
}


static void test_mode_params(void) {
    trickle_mode_params_t precise, fast;
    trickle_mode_params_from_level(TRICKLE_LEVEL_PRECISION, &precise);
    trickle_mode_params_from_level(TRICKLE_LEVEL_SPEED, &fast);

    CHECK(fabsf(precise.tolerance_gr - 0.02f) < 1e-4f, "precision tolerance %f", precise.tolerance_gr);
    CHECK(fabsf(fast.tolerance_gr - 0.10f) < 1e-4f, "speed tolerance %f", fast.tolerance_gr);
    CHECK(precise.approach_time_s > fast.approach_time_s, "approach time not monotonic");
    CHECK(precise.fine_reserve_gr > fast.fine_reserve_gr, "fine reserve not monotonic");
    CHECK(fast.aim_below_gr == 0.0f, "speed mode should aim at the target");

    // Speed and tolerance are independent
    trickle_mode_params_t fast_tight;
    trickle_mode_params(100, 0.02f, &fast_tight);
    CHECK(fabsf(fast_tight.tolerance_gr - 0.02f) < 1e-4f, "tolerance not kept at speed 100: %f", fast_tight.tolerance_gr);
    CHECK(fast_tight.approach_time_s == fast.approach_time_s, "speed params depend on the tolerance");
    CHECK(fast_tight.topup_threshold_gr > 0.02f && fast_tight.topup_threshold_gr < 0.025f,
          "fast mode should only top up outside the band: %f", fast_tight.topup_threshold_gr);
    CHECK(precise.topup_threshold_gr < 0.015f, "precision mode tops up in the lower half: %f", precise.topup_threshold_gr);
}


static void test_auto_speed(void) {
    trickle_speed_state_t state;
    trickle_speed_reset(&state, 50);

    // Clean charges with improving time: speeds up
    for (int i = 0; i < TRICKLE_SPEED_CLEAN_STREAK; i += 1) {
        trickle_speed_update(&state, false, 0, 10.0f);
    }
    CHECK(state.speed == 50 + TRICKLE_SPEED_STEP_UP, "no speed up after a clean streak: %u", state.speed);

    // A single over charge only restarts the streak
    trickle_speed_update(&state, true, 0, 9.0f);
    CHECK(state.speed == 50 + TRICKLE_SPEED_STEP_UP, "single over charge slowed down: %u", state.speed);
    CHECK(state.clean_streak == 0, "streak not restarted");

    // A second one soon after steps down
    trickle_speed_update(&state, false, 0, 9.0f);
    trickle_speed_update(&state, true, 0, 9.0f);
    CHECK(state.speed == 50 + TRICKLE_SPEED_STEP_UP - TRICKLE_SPEED_STEP_DOWN, "no step down: %u", state.speed);

    // Never below the floor
    for (int i = 0; i < 20; i += 1) {
        trickle_speed_update(&state, true, 0, 9.0f);
    }
    CHECK(state.speed == TRICKLE_SPEED_AUTO_MIN, "below the floor: %u", state.speed);

    // Going faster made it slower: back to the best speed
    trickle_speed_reset(&state, 60);
    for (int i = 0; i < TRICKLE_SPEED_CLEAN_STREAK; i += 1) {
        trickle_speed_update(&state, false, 0, 10.0f);
    }
    CHECK(state.speed == 70, "speed %u", state.speed);
    for (int i = 0; i < TRICKLE_SPEED_CLEAN_STREAK; i += 1) {
        trickle_speed_update(&state, false, 3, 14.0f);
    }
    CHECK(state.speed == 60, "did not return to the best speed: %u", state.speed);

    // Never above 100
    trickle_speed_reset(&state, 100);
    for (int i = 0; i < 3 * TRICKLE_SPEED_CLEAN_STREAK; i += 1) {
        trickle_speed_update(&state, false, 0, 5.0f);
    }
    CHECK(state.speed == 100, "above 100: %u", state.speed);
}


static void build_trace(trickle_trace_t * trace, const plant_t * truth, float coarse_stop_t) {
    plant_t p = *truth;
    p.t = 0.0f;
    p.n = 0;
    p.delivered = 0.0f;
    trickle_trace_reset(trace, 0.0f);

    for (int i = 0; i < 300; i += 1) {
        float t = i * 0.1f;
        float coarse = t < coarse_stop_t ? 4.0f : 0.0f;
        float fine = t < 20.0f ? (t < coarse_stop_t ? 2.0f : 0.5f + 0.3f * sinf(t)) : 0.0f;
        if (t >= coarse_stop_t && t < coarse_stop_t + 0.1f) {
            trickle_trace_mark_coarse_stop(trace, t);
        }
        p.coarse_rps = coarse;
        p.fine_rps = fine;
        trickle_trace_add(trace, p.t, plant_reading(&p), coarse, fine);
        for (int k = 0; k < 10; k += 1) {
            plant_step(&p, 0.01f);
        }
    }
}


static void test_fit_recovers_parameters(void) {
    plant_t truth = {.coarse_gpr = 1.2f, .fine_gpr = 0.09f, .latency = 0.55f, .tail = 0.3f};
    static trickle_trace_t trace;
    build_trace(&trace, &truth, 6.0f);

    trickle_model_t prior;
    trickle_model_reset(&prior);
    trickle_fit_t fit;
    bool ok = trickle_fit_trace(&trace, &prior, &fit);

    CHECK(ok, "fit failed");
    CHECK(fabsf(fit.coarse_gpr - 1.2f) < 0.03f, "coarse gpr %f", fit.coarse_gpr);
    CHECK(fabsf(fit.fine_gpr - 0.09f) < 0.005f, "fine gpr %f", fit.fine_gpr);
    CHECK(fabsf(fit.latency_s - 0.55f) < 0.06f, "latency %f", fit.latency_s);
    CHECK(fabsf(fit.coarse_tail_gr - 0.3f) < 0.15f, "tail %f", fit.coarse_tail_gr);
}


static void test_trace_decimation(void) {
    static trickle_trace_t trace;
    trickle_trace_reset(&trace, 0.0f);

    // 3x the buffer length at 1 rps: revs must stay exact despite decimation
    int n = TRICKLE_TRACE_MAX_SAMPLES * 3;
    for (int i = 0; i <= n; i += 1) {
        trickle_trace_add(&trace, i * 0.1f, 0.0f, 1.0f, 0.5f);
    }

    CHECK(trace.count <= TRICKLE_TRACE_MAX_SAMPLES, "count %d", trace.count);
    CHECK(trace.stride >= 4, "stride %d", trace.stride);
    const trickle_sample_t * last = &trace.samples[trace.count - 1];
    CHECK(fabsf(last->coarse_revs - last->t) < 1e-2f, "coarse revs %f at %f", last->coarse_revs, last->t);
    CHECK(fabsf(last->fine_revs - 0.5f * last->t) < 1e-2f, "fine revs %f", last->fine_revs);
}


static float run_controller(plant_t * p, const trickle_model_t * model, uint8_t level, float target,
                            trickle_ctrl_t * ctrl, trickle_phase_t * final_phase) {
    trickle_mode_params_t mode;
    trickle_mode_params_from_level(level, &mode);
    trickle_motor_limits_t limits = {.coarse_min_rps = 0.1f, .coarse_max_rps = 6.0f,
                                     .fine_min_rps = 0.05f, .fine_max_rps = 3.0f};

    trickle_ctrl_init(ctrl, target, model, &mode, &limits, p->t, 0.0f);

    trickle_phase_t phase = ctrl->phase;
    for (int i = 0; i < 3000 && !trickle_phase_is_final(phase); i += 1) {
        float coarse, fine;
        phase = trickle_ctrl_step(ctrl, p->t, plant_reading(p), &coarse, &fine);
        p->coarse_rps = coarse;
        p->fine_rps = fine;
        for (int k = 0; k < 10; k += 1) {
            plant_step(p, 0.01f);
        }
    }
    *final_phase = phase;

    // Let everything land
    for (int k = 0; k < 300; k += 1) {
        plant_step(p, 0.01f);
    }
    return p->delivered;
}


static void test_controller_hits_target_with_exact_model(void) {
    static plant_t p;
    memset(&p, 0, sizeof(p));
    p.coarse_gpr = 1.0f;
    p.fine_gpr = 0.1f;
    p.latency = 0.6f;
    p.tail = 0.2f;

    trickle_model_t model;
    trickle_model_reset(&model);
    model.coarse_gpr = 1.0f;
    model.fine_gpr = 0.1f;
    model.latency_s = 0.6f;
    model.coarse_tail_gr = 0.2f;
    model.coarse_tail_sd_gr = 0.02f;
    model.flags = TRICKLE_MODEL_FLAG_VALID;
    model.learned_throws = 10;
    model.landing_sd_gr = 0.0f;

    static trickle_ctrl_t ctrl;
    trickle_phase_t phase;
    float final = run_controller(&p, &model, TRICKLE_LEVEL_BALANCED, 40.0f, &ctrl, &phase);

    CHECK(phase == TRICKLE_PHASE_DONE, "phase %d", phase);
    CHECK(fabsf(final - 40.0f) < 0.05f, "final %f", final);
    CHECK(final <= 40.0f + 0.04f, "overshoot %f", final);
}


static void test_controller_tops_up_when_short(void) {
    static plant_t p;
    memset(&p, 0, sizeof(p));
    p.coarse_gpr = 1.0f;
    p.fine_gpr = 0.1f;
    p.latency = 0.6f;
    p.tail = 0.0f;

    // Model claims more powder per revolution than reality: the main stop
    // lands short, top ups must fix it
    trickle_model_t model;
    trickle_model_reset(&model);
    model.coarse_gpr = 1.0f;
    model.fine_gpr = 0.1f;
    model.latency_s = 0.6f;
    model.stop_bias_gr = 0.3f;
    model.coarse_tail_sd_gr = 0.02f;
    model.flags = TRICKLE_MODEL_FLAG_VALID;
    model.learned_throws = 10;
    model.landing_sd_gr = 0.0f;

    static trickle_ctrl_t ctrl;
    trickle_phase_t phase;
    float final = run_controller(&p, &model, TRICKLE_LEVEL_PRECISION, 30.0f, &ctrl, &phase);

    CHECK(phase == TRICKLE_PHASE_DONE, "phase %d", phase);
    CHECK(ctrl.topup_count > 0, "no top up");
    CHECK(fabsf(final - 30.0f) <= 0.02f + 1e-3f, "final %f after %d top ups", final, ctrl.topup_count);
    CHECK(!isnan(ctrl.landing_weight_gr) && ctrl.landing_weight_gr < 29.9f, "landing %f", ctrl.landing_weight_gr);
}


static void test_controller_detects_no_flow(void) {
    static plant_t p;
    memset(&p, 0, sizeof(p));
    p.coarse_gpr = 1.0f;
    p.fine_gpr = 0.1f;
    p.latency = 0.5f;
    p.dead = true;

    trickle_model_t model;
    trickle_model_reset(&model);
    model.flags = TRICKLE_MODEL_FLAG_VALID;
    model.learned_throws = 1;

    static trickle_ctrl_t ctrl;
    trickle_phase_t phase;
    run_controller(&p, &model, TRICKLE_LEVEL_BALANCED, 40.0f, &ctrl, &phase);

    CHECK(phase == TRICKLE_PHASE_FAULT_NO_FLOW, "phase %d", phase);
    CHECK(p.t < 10.0f, "fault detected late at %f", p.t);
}


static void test_learning_converges(void) {
    // Start from a wrong model and learn from repeated ideal throws
    plant_t truth = {.coarse_gpr = 0.8f, .fine_gpr = 0.07f, .latency = 0.45f, .tail = 0.15f};

    trickle_model_t model;
    trickle_model_reset(&model);

    for (int i = 0; i < 6; i += 1) {
        static plant_t p;
        p = truth;
        p.t = 0;
        p.n = 0;
        p.delivered = 0;

        static trickle_trace_t trace;
        static trickle_ctrl_t ctrl;
        trickle_mode_params_t mode;
        trickle_mode_params_from_level(TRICKLE_LEVEL_BALANCED, &mode);
        trickle_motor_limits_t limits = {0.1f, 6.0f, 0.05f, 3.0f, 0.0f, 0.0f};
        trickle_ctrl_init(&ctrl, 35.0f, &model, &mode, &limits, 0.0f, 0.0f);
        trickle_trace_reset(&trace, 0.0f);

        trickle_phase_t phase = ctrl.phase;
        for (int k = 0; k < 3000 && !trickle_phase_is_final(phase); k += 1) {
            float coarse, fine;
            float w = plant_reading(&p);
            phase = trickle_ctrl_step(&ctrl, p.t, w, &coarse, &fine);
            if (ctrl.events & TRICKLE_EVENT_COARSE_STOPPED) {
                trickle_trace_mark_coarse_stop(&trace, p.t);
            }
            trickle_trace_add(&trace, p.t, w, coarse, fine);
            p.coarse_rps = coarse;
            p.fine_rps = fine;
            for (int j = 0; j < 10; j += 1) {
                plant_step(&p, 0.01f);
            }
        }

        trickle_fit_t fit;
        trickle_fit_trace(&trace, &model, &fit);
        trickle_throw_result_t result;
        trickle_ctrl_get_result(&ctrl, p.t, &result);
        trickle_model_learn(&model, &fit, &result, 0.3f);
    }

    CHECK(fabsf(model.coarse_gpr - 0.8f) < 0.05f, "coarse %f", model.coarse_gpr);
    CHECK(fabsf(model.fine_gpr - 0.07f) < 0.007f, "fine %f", model.fine_gpr);
    CHECK(fabsf(model.latency_s - 0.45f) < 0.1f, "latency %f", model.latency_s);
    CHECK(model.learned_throws == 6, "throws %d", model.learned_throws);
}


static void test_sanitize(void) {
    trickle_model_t model;
    trickle_model_reset(&model);
    model.coarse_gpr = NAN;
    model.fine_gpr = -3.0f;
    model.latency_s = 100.0f;
    trickle_model_sanitize(&model);

    CHECK(model.coarse_gpr == 1.0f, "coarse %f", model.coarse_gpr);
    CHECK(model.fine_gpr > 0.0f, "fine %f", model.fine_gpr);
    CHECK(model.latency_s <= 3.0f, "latency %f", model.latency_s);
}


static void test_sha256(void) {
    // FIPS 180-2 test vectors
    struct { const char * msg; const char * hex; } vectors[] = {
        {"", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
        {"abc", "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"},
        {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
         "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"},
    };
    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i += 1) {
        sha256_ctx_t ctx;
        uint8_t digest[32];
        char hex[65];
        sha256_init(&ctx);
        // Feed byte by byte to exercise the block handling
        for (const char * c = vectors[i].msg; *c; c += 1) {
            sha256_update(&ctx, c, 1);
        }
        sha256_final(&ctx, digest);
        sha256_to_hex(digest, hex);
        CHECK(strcmp(hex, vectors[i].hex) == 0, "sha256(%s) = %s", vectors[i].msg, hex);
    }
}


int main(void) {
    test_mode_params();
    test_auto_speed();
    test_fit_recovers_parameters();
    test_trace_decimation();
    test_controller_hits_target_with_exact_model();
    test_controller_tops_up_when_short();
    test_controller_detects_no_flow();
    test_learning_converges();
    test_sanitize();
    test_sha256();

    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("All trickle model tests passed\n");
    return 0;
}
