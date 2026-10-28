#include <math.h>
#include <string.h>

#include "powder_sim.h"


#define DELAY_LINE_STEP_S   0.01f
#define TAIL_RELEASE_S      0.4f


static const powder_sim_powder_t powder_presets[POWDER_SIM_PRESET_COUNT] = {
    [POWDER_SIM_BALL] = {
        .name = "Ball powder",
        .kernel_gr = 0.004f,
        .coarse_gpr = 1.6f,
        .fine_gpr = 0.25f,
        .flow_cv = 0.12f,
        .flow_corr_s = 0.5f,
        .clump_per_rev = 0.0f,
        .clump_kernels = 0.0f,
        .fall_time_s = 0.30f,
        .fall_jitter_s = 0.05f,
        .coarse_tail_kernels = 25.0f,
        .fine_tail_kernels = 4.0f,
    },
    [POWDER_SIM_SHORT_EXTRUDED] = {
        .name = "Short extruded",
        .kernel_gr = 0.016f,
        .coarse_gpr = 1.1f,
        .fine_gpr = 0.2f,
        .flow_cv = 0.25f,
        .flow_corr_s = 0.4f,
        .clump_per_rev = 0.15f,
        .clump_kernels = 6.0f,
        .fall_time_s = 0.38f,
        .fall_jitter_s = 0.08f,
        .coarse_tail_kernels = 20.0f,
        .fine_tail_kernels = 3.0f,
    },
    [POWDER_SIM_LONG_STICK] = {
        .name = "Long stick",
        .kernel_gr = 0.03f,
        .coarse_gpr = 0.9f,
        .fine_gpr = 0.15f,
        .flow_cv = 0.4f,
        .flow_corr_s = 0.3f,
        .clump_per_rev = 0.35f,
        .clump_kernels = 8.0f,
        .fall_time_s = 0.45f,
        .fall_jitter_s = 0.12f,
        .coarse_tail_kernels = 18.0f,
        .fine_tail_kernels = 2.0f,
    },
    [POWDER_SIM_FLAKE] = {
        .name = "Flake",
        .kernel_gr = 0.008f,
        .coarse_gpr = 1.3f,
        .fine_gpr = 0.2f,
        .flow_cv = 0.3f,
        .flow_corr_s = 0.3f,
        .clump_per_rev = 0.1f,
        .clump_kernels = 10.0f,
        .fall_time_s = 0.35f,
        .fall_jitter_s = 0.1f,
        .coarse_tail_kernels = 30.0f,
        .fine_tail_kernels = 3.0f,
    },
};


const powder_sim_powder_t * powder_sim_get_preset(powder_sim_preset_t preset) {
    if (preset >= POWDER_SIM_PRESET_COUNT) {
        preset = POWDER_SIM_SHORT_EXTRUDED;
    }
    return &powder_presets[preset];
}


void powder_sim_default_scale(powder_sim_scale_t * scale) {
    // Roughly an A&D FX-120i in grain mode with fast response settings
    scale->dead_time_s = 0.15f;
    scale->filter_tau_s = 0.25f;
    scale->noise_sd_gr = 0.006f;
    scale->resolution_gr = 0.02f;
    scale->sample_period_s = 0.1f;
}


// xorshift32
static uint32_t rng_next(powder_sim_t * sim) {
    uint32_t x = sim->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    sim->rng = x;
    return x;
}


float powder_sim_randu(powder_sim_t * sim) {
    return (float) (rng_next(sim) >> 8) / 16777216.0f;
}


float powder_sim_randn(powder_sim_t * sim) {
    // Box-Muller
    float u1 = powder_sim_randu(sim);
    float u2 = powder_sim_randu(sim);
    if (u1 < 1e-7f) {
        u1 = 1e-7f;
    }
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}


void powder_sim_init(powder_sim_t * sim, const powder_sim_powder_t * powder,
                     const powder_sim_scale_t * scale, uint32_t seed) {
    memset(sim, 0x0, sizeof(powder_sim_t));
    sim->powder = *powder;
    sim->scale = *scale;
    sim->rng = seed ? seed : 0x12345678u;
    sim->accel_rps2 = 50.0f;
    sim->delay_step_s = DELAY_LINE_STEP_S;
}


void powder_sim_set_motors(powder_sim_t * sim, float coarse_rps, float fine_rps) {
    sim->coarse_cmd_rps = coarse_rps;
    sim->fine_cmd_rps = fine_rps;
}


static float ramp(float current, float target, float max_delta) {
    if (target > current + max_delta) {
        return current + max_delta;
    }
    if (target < current - max_delta) {
        return current - max_delta;
    }
    return target;
}


static void queue_push(powder_sim_t * sim, float arrival_t, float mass_gr) {
    if (mass_gr <= 0.0f) {
        return;
    }
    if (sim->queue_count >= POWDER_SIM_QUEUE_LEN) {
        // Merge into the last packet rather than losing powder
        sim->queue[sim->queue_count - 1].mass_gr += mass_gr;
        return;
    }
    sim->queue[sim->queue_count].t = arrival_t;
    sim->queue[sim->queue_count].mass_gr = mass_gr;
    sim->queue_count += 1;
}


static float fall_time(powder_sim_t * sim) {
    float t = sim->powder.fall_time_s + sim->powder.fall_jitter_s * powder_sim_randn(sim);
    return t < 0.05f ? 0.05f : t;
}


// Number of kernels for an expected (possibly fractional) count, keeps the mean
static int kernel_count(powder_sim_t * sim, float expected) {
    if (expected <= 0.0f) {
        return 0;
    }
    return (int) floorf(expected + powder_sim_randu(sim));
}


static void emit_flow(powder_sim_t * sim, float rps, float gpr, float clump_scale, float dt, float flow_mod) {
    if (rps <= 0.0f) {
        return;
    }

    float expected_kernels = gpr * rps * dt * flow_mod / sim->powder.kernel_gr;
    int n = kernel_count(sim, expected_kernels);
    if (n > 0) {
        queue_push(sim, sim->t + fall_time(sim), n * sim->powder.kernel_gr);
    }

    // Occasional clump of kernels sticking together
    float p_clump = sim->powder.clump_per_rev * clump_scale * rps * dt;
    if (p_clump > 0.0f && powder_sim_randu(sim) < p_clump) {
        float u = powder_sim_randu(sim);
        if (u < 1e-6f) {
            u = 1e-6f;
        }
        int clump = 1 + (int) (-logf(u) * sim->powder.clump_kernels);
        queue_push(sim, sim->t + fall_time(sim), clump * sim->powder.kernel_gr);
    }
}


bool powder_sim_step(powder_sim_t * sim, float dt, float * reading_gr) {
    sim->t += dt;

    // Motors follow the command with limited acceleration
    sim->coarse_rps = ramp(sim->coarse_rps, sim->coarse_cmd_rps, sim->accel_rps2 * dt);
    sim->fine_rps = ramp(sim->fine_rps, sim->fine_cmd_rps, sim->accel_rps2 * dt);

    // Slowly drifting flow fluctuation (Ornstein-Uhlenbeck process)
    float corr = sim->powder.flow_corr_s > 0.01f ? sim->powder.flow_corr_s : 0.01f;
    sim->flow_noise += -sim->flow_noise * dt / corr + sqrtf(2.0f * dt / corr) * powder_sim_randn(sim);
    float flow_mod = 1.0f + sim->powder.flow_cv * sim->flow_noise;
    if (flow_mod < 0.0f) {
        flow_mod = 0.0f;
    }

    emit_flow(sim, sim->coarse_rps, sim->powder.coarse_gpr, 1.0f, dt, flow_mod);
    emit_flow(sim, sim->fine_rps, sim->powder.fine_gpr, 0.3f, dt, flow_mod);

    // The coarse trickler keeps dropping a few kernels after it stops
    bool coarse_running = sim->coarse_rps > 0.05f;
    if (sim->coarse_was_running && !coarse_running) {
        float n = sim->powder.coarse_tail_kernels * (0.4f + 1.2f * powder_sim_randu(sim));
        sim->pending_tail_kernels = n;
        sim->tail_release_t = sim->t;
    }
    sim->coarse_was_running = coarse_running;

    // A few kernels sitting on the fine tube lip fall after it stops;
    // turning the fine tube backwards pulls them back in
    bool fine_running = sim->fine_rps > 0.05f;
    if (sim->fine_was_running && !fine_running) {
        sim->pending_fine_tail_kernels = sim->powder.fine_tail_kernels * (0.4f + 1.2f * powder_sim_randu(sim));
        sim->fine_tail_release_t = sim->t;
    }
    sim->fine_was_running = fine_running;
    if (sim->pending_fine_tail_kernels > 0.0f) {
        if (sim->fine_rps < 0.0f) {
            float pulled = 10.0f * (-sim->fine_rps) * dt;
            sim->pending_fine_tail_kernels *= (pulled < 1.0f) ? (1.0f - pulled) : 0.0f;
        }
        float release = sim->pending_fine_tail_kernels * dt / TAIL_RELEASE_S;
        if (sim->t - sim->fine_tail_release_t > TAIL_RELEASE_S * 3.0f) {
            release = sim->pending_fine_tail_kernels;
        }
        int n = kernel_count(sim, release);
        if (n > 0) {
            queue_push(sim, sim->t + fall_time(sim), n * sim->powder.kernel_gr);
        }
        sim->pending_fine_tail_kernels -= release;
        if (sim->pending_fine_tail_kernels < 0.01f) {
            sim->pending_fine_tail_kernels = 0.0f;
        }
    }

    if (sim->pending_tail_kernels > 0.0f) {
        // Reversing the coarse trickler pulls the tail back into the tube
        if (sim->coarse_rps < 0.0f) {
            float pulled = 6.0f * (-sim->coarse_rps) * dt;
            sim->pending_tail_kernels *= (pulled < 1.0f) ? (1.0f - pulled) : 0.0f;
        }

        float release = sim->pending_tail_kernels * dt / TAIL_RELEASE_S;
        if (sim->t - sim->tail_release_t > TAIL_RELEASE_S * 3.0f) {
            release = sim->pending_tail_kernels;
        }
        int n = kernel_count(sim, release);
        if (n > 0) {
            queue_push(sim, sim->t + fall_time(sim), n * sim->powder.kernel_gr);
        }
        sim->pending_tail_kernels -= release;
        if (sim->pending_tail_kernels < 0.01f) {
            sim->pending_tail_kernels = 0.0f;
        }
    }

    // Land everything that finished falling
    for (uint16_t i = 0; i < sim->queue_count;) {
        if (sim->queue[i].t <= sim->t) {
            sim->pan_gr += sim->queue[i].mass_gr;
            sim->queue[i] = sim->queue[sim->queue_count - 1];
            sim->queue_count -= 1;
        }
        else {
            i += 1;
        }
    }

    // Scale dead time via a delay line sampled every DELAY_LINE_STEP_S
    sim->delay_acc_s += dt;
    while (sim->delay_acc_s >= sim->delay_step_s) {
        sim->delay_acc_s -= sim->delay_step_s;
        sim->delay_line[sim->delay_head] = sim->pan_gr;
        sim->delay_head = (sim->delay_head + 1) % 32;
    }
    int delay_steps = (int) (sim->scale.dead_time_s / sim->delay_step_s + 0.5f);
    if (delay_steps > 31) {
        delay_steps = 31;
    }
    if (delay_steps < 1) {
        delay_steps = 1;
    }
    float delayed = sim->delay_line[(sim->delay_head + 32 - delay_steps) % 32];

    // Low pass filter of the scale
    float alpha = 1.0f - expf(-dt / (sim->scale.filter_tau_s > 0.001f ? sim->scale.filter_tau_s : 0.001f));
    sim->filtered_gr += (delayed - sim->filtered_gr) * alpha;

    if (sim->t < sim->next_sample_t) {
        return false;
    }
    sim->next_sample_t = sim->t + sim->scale.sample_period_s;

    float value = sim->filtered_gr + sim->scale.noise_sd_gr * powder_sim_randn(sim) - sim->tare_gr;
    if (sim->scale.resolution_gr > 0.0f) {
        value = roundf(value / sim->scale.resolution_gr) * sim->scale.resolution_gr;
    }
    sim->last_reading_gr = value;
    *reading_gr = value;

    return true;
}


void powder_sim_empty_pan(powder_sim_t * sim) {
    sim->pan_gr = 0.0f;
    sim->filtered_gr = 0.0f;
    sim->queue_count = 0;
    sim->pending_tail_kernels = 0.0f;
    sim->pending_fine_tail_kernels = 0.0f;
    for (int i = 0; i < 32; i += 1) {
        sim->delay_line[i] = 0.0f;
    }
}


void powder_sim_tare(powder_sim_t * sim) {
    sim->tare_gr = sim->filtered_gr;
}


float powder_sim_true_weight(const powder_sim_t * sim) {
    return sim->pan_gr - sim->tare_gr;
}
