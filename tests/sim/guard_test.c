/*
 * Tests of the charge guards (src/charge_guard.c): no false alarms during
 * normal charges (noise, coarse stream push), bumps and a lifted pan are found.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "charge_guard.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures += 1; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const charge_guard_config_t config = {.pan_guard = true, .bump_guard = true, .bump_drop_gr = 0.5f};

static float noise(void) {
    return ((float) rand() / RAND_MAX - 0.5f) * 0.08f;     // +-0.04 gr
}

// A normal charge at 10 Hz: coarse at flow gr/s up to stop_gr (with the push
// of the stream on the pan), fine to target. Returns the number of alarms.
static int normal_charge(float flow, float push_gr, float target, int * bumps) {
    charge_guard_t g;
    charge_guard_reset(&g, 0);
    int alarms = 0;
    float w = 0.0f;
    uint32_t t = 0;
    float stop_gr = target - 1.5f;
    bool coarse = true;
    float push = 0.0f;
    for (int i = 0; i < 600; i += 1, t += 100) {
        if (coarse) {
            w += flow * 0.1f;
            push = fminf(push + push_gr * 0.5f, push_gr);  // builds up in two samples
            if (w >= stop_gr) {
                coarse = false;
                charge_guard_coarse_stopped(&g, t);
            }
        }
        else {
            push = fmaxf(push - push_gr * 0.5f, 0.0f);     // fades in two samples
            if (w < target) {
                w += 0.05f;     // fine flow 0.5 gr/s
            }
        }
        charge_guard_result_t r = charge_guard_check(&g, &config, w + push + noise(), coarse, t);
        if (r != CHARGE_GUARD_OK) {
            alarms += 1;
            if (r == CHARGE_GUARD_BUMP) {
                *bumps += 1;
            }
        }
    }
    return alarms;
}


int main(void) {
    srand(1);

    // Normal charges, slow to very fast coarse flow with the stream push
    for (int k = 0; k < 50; k += 1) {
        int bumps = 0;
        float flow = 5.0f + k * 0.5f;              // 5 ... 30 gr/s
        float push = flow * 0.1f;                  // ~1 gr at 10 gr/s (impulse of the stream)
        int alarms = normal_charge(flow, push, 42.0f, &bumps);
        CHECK(alarms == 0, "false alarm in a normal charge (flow %.1f gr/s, push %.1f gr): %d", flow, push, alarms);
    }

    // Bump in the fine phase: reading dips by 1.5 gr for two samples
    {
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        float w = 40.0f;
        int found = 0;
        for (int i = 0; i < 40; i += 1) {
            float r = w + noise() - ((i == 20 || i == 21) ? 1.5f : 0.0f);
            if (charge_guard_check(&g, &config, r, false, 10000 + i * 100) == CHARGE_GUARD_BUMP) {
                found += 1;
            }
            w += 0.02f;
        }
        CHECK(found >= 1, "bump in the fine phase not found");
    }

    // Small dip (0.3 gr) in the fine phase: below the 0.5 gr setting
    {
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        int found = 0;
        for (int i = 0; i < 40; i += 1) {
            float r = 40.0f + noise() - (i == 20 ? 0.3f : 0.0f);
            found += charge_guard_check(&g, &config, r, false, 10000 + i * 100) != CHARGE_GUARD_OK;
        }
        CHECK(found == 0, "0.3 gr dip counted as a bump");
    }

    // Bump while the coarse stream runs: 3 gr dip ignored, 7 gr dip found
    {
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        int small = 0;
        int big = 0;
        float w = 0.0f;
        for (int i = 0; i < 60; i += 1) {
            w += 1.3f;
            float dip = i == 20 ? 3.0f : (i == 40 ? 7.0f : 0.0f);
            charge_guard_result_t r = charge_guard_check(&g, &config, w - dip, true, i * 100);
            if (r == CHARGE_GUARD_BUMP) {
                if (i == 20) small += 1;
                if (i == 40) big += 1;
            }
        }
        CHECK(small == 0, "3 gr dip during coarse counted as a bump");
        CHECK(big == 1, "7 gr dip during coarse not found");
    }

    // Pan lifted: reading falls to minus the pan weight
    {
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        charge_guard_check(&g, &config, 20.0f, true, 0);
        CHECK(charge_guard_check(&g, &config, -150.0f, true, 100) == CHARGE_GUARD_PAN_REMOVED, "lifted pan not found");
        CHECK(charge_guard_check(&g, &config, -0.03f, true, 200) != CHARGE_GUARD_PAN_REMOVED, "noise around zero counted as a lifted pan");
    }

    // Switched off: nothing is reported
    {
        charge_guard_config_t off = {.pan_guard = false, .bump_guard = false, .bump_drop_gr = 0.5f};
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        charge_guard_check(&g, &off, 40.0f, false, 0);
        charge_guard_check(&g, &off, 40.0f, false, 100);
        CHECK(charge_guard_check(&g, &off, 30.0f, false, 200) == CHARGE_GUARD_OK, "bump reported while off");
        CHECK(charge_guard_check(&g, &off, -150.0f, false, 300) == CHARGE_GUARD_OK, "lifted pan reported while off");
    }

    // Tick counter wrap during the grace time
    {
        charge_guard_t g;
        charge_guard_reset(&g, 0);
        uint32_t t = 0xFFFFFF00u;
        charge_guard_coarse_stopped(&g, t);
        charge_guard_check(&g, &config, 40.0f, false, t);
        charge_guard_check(&g, &config, 40.0f, false, t + 100);
        CHECK(charge_guard_check(&g, &config, 39.0f, false, t + 300) == CHARGE_GUARD_OK, "grace time lost at the tick wrap");
    }

    printf(failures ? "%d charge guard test(s) failed\n" : "All charge guard tests passed\n", failures);
    return failures ? 1 : 0;
}
