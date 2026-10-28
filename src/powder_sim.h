#ifndef POWDER_SIM_H_
#define POWDER_SIM_H_

/*
 * Physical powder + scale simulator
 *
 * Pure C so it can drive both the simulated scale driver on the controller
 * and the host side tests. It models what makes real trickling hard:
 *
 *  - powder comes in discrete kernels, not as a smooth flow
 *  - the flow fluctuates (slow random drift) and stick powders form clumps
 *  - kernels need time to fall from the tube into the pan
 *  - the coarse trickler keeps dropping a few kernels after it stops
 *  - the scale has a dead time, a low pass filter, noise and a resolution
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define POWDER_SIM_QUEUE_LEN    512


typedef enum {
    POWDER_SIM_BALL = 0,
    POWDER_SIM_SHORT_EXTRUDED = 1,
    POWDER_SIM_LONG_STICK = 2,
    POWDER_SIM_FLAKE = 3,
    POWDER_SIM_PRESET_COUNT,
} powder_sim_preset_t;


typedef struct {
    const char * name;
    float kernel_gr;            // Mass of a single kernel
    float coarse_gpr;           // Mean powder per coarse revolution
    float fine_gpr;             // Mean powder per fine revolution
    float flow_cv;              // Relative flow fluctuation
    float flow_corr_s;          // Correlation time of the fluctuation
    float clump_per_rev;        // Clump probability per coarse revolution
    float clump_kernels;        // Mean kernels per clump
    float fall_time_s;          // Mean flight time tube -> pan
    float fall_jitter_s;        // Spread of the flight time
    float coarse_tail_kernels;  // Mean kernels dropping after the coarse stop
    float fine_tail_kernels;    // Mean kernels dribbling from the fine tube lip after it stops
    float fine_reverse_eject;   // Kernels thrown out per revolution of fine reverse beyond the first 0.02 rev
} powder_sim_powder_t;


typedef struct {
    float dead_time_s;
    float filter_tau_s;
    float noise_sd_gr;
    float resolution_gr;
    float sample_period_s;
} powder_sim_scale_t;


typedef struct {
    float t;
    float mass_gr;
} powder_sim_packet_t;


typedef struct {
    powder_sim_powder_t powder;
    powder_sim_scale_t scale;

    uint32_t rng;
    float t;

    // Motors
    float coarse_cmd_rps;
    float fine_cmd_rps;
    float coarse_rps;
    float fine_rps;
    float accel_rps2;
    bool coarse_was_running;
    float coarse_reverse_revs;  // Revolutions reversed since the last stop

    // Flow state
    float flow_noise;
    float coarse_kernel_acc;
    float fine_kernel_acc;
    float coarse_clump_acc;

    // Powder in flight
    powder_sim_packet_t queue[POWDER_SIM_QUEUE_LEN];
    uint16_t queue_count;
    float pending_tail_kernels;
    bool fine_was_running;
    float pending_fine_tail_kernels;
    float fine_tail_release_t;
    float fine_reversed_revs;   // Fine reverse since it last ran forward
    float tail_release_t;

    // Pan and scale
    float pan_gr;
    float filtered_gr;
    float delay_line[32];
    uint8_t delay_head;
    float delay_step_s;
    float delay_acc_s;
    float next_sample_t;
    float tare_gr;
    float last_reading_gr;
} powder_sim_t;


const powder_sim_powder_t * powder_sim_get_preset(powder_sim_preset_t preset);
void powder_sim_default_scale(powder_sim_scale_t * scale);
void powder_sim_init(powder_sim_t * sim, const powder_sim_powder_t * powder,
                     const powder_sim_scale_t * scale, uint32_t seed);
void powder_sim_set_motors(powder_sim_t * sim, float coarse_rps, float fine_rps);

/*
 * Advance the simulation by dt seconds. Returns true when the scale produced
 * a new reading during this step, written to *reading_gr.
 */
bool powder_sim_step(powder_sim_t * sim, float dt, float * reading_gr);

void powder_sim_empty_pan(powder_sim_t * sim);
void powder_sim_tare(powder_sim_t * sim);
float powder_sim_true_weight(const powder_sim_t * sim);
float powder_sim_randn(powder_sim_t * sim);
float powder_sim_randu(powder_sim_t * sim);

#ifdef __cplusplus
}
#endif

#endif  // POWDER_SIM_H_
