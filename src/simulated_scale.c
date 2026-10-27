#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "powder_sim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <math.h>

/*
 * Simulated scale backed by the physical powder simulator (powder_sim.c):
 * kernels, flow fluctuation, clumps, flight time, coarse tail and a scale
 * with dead time, filter, noise and 0.02 gr resolution. Good enough to try
 * the AI trickle learning without powder.
 */

extern scale_config_t scale_config;
extern motor_config_t coarse_trickler_motor_config;
extern motor_config_t fine_trickler_motor_config;
extern charge_mode_config_t charge_mode_config;

#define SIM_STEP_MS                 10
#define SIM_CUP_REMOVAL_DELAY_MS    2500    // "User" removes the cup after this
#define SIM_CUP_AWAY_MS             2000
#define SIM_HOLDER_WEIGHT_GR        150.0f

static powder_sim_t sim;
static volatile bool zero_requested = false;


void simulated_scale_force_zero(void) {
    zero_requested = true;
}


static void publish(float weight) {
    scale_config.current_scale_measurement = weight;
    if (scale_config.scale_measurement_ready) {
        xSemaphoreGive(scale_config.scale_measurement_ready);
    }
}


void simulated_scale_read_loop_task(void *p) {
    powder_sim_scale_t scale;
    powder_sim_default_scale(&scale);
    powder_sim_init(&sim, powder_sim_get_preset(POWDER_SIM_SHORT_EXTRUDED), &scale, 0xC0FFEEu);

    uint32_t cup_removal_wait_ms = 0;
    uint32_t cup_away_ms = 0;

    while (true) {
        if (scale_config.persistent_config.scale_driver != SCALE_DRIVER_SIMULATED) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (zero_requested) {
            zero_requested = false;
            powder_sim_tare(&sim);
        }

        // Motors as commanded by the controller
        powder_sim_set_motors(&sim, coarse_trickler_motor_config.prev_velocity,
                              fine_trickler_motor_config.prev_velocity);

        float reading;
        bool new_reading = powder_sim_step(&sim, SIM_STEP_MS / 1000.0f, &reading);

        // Play the user: remove the cup some time after the charge, empty it
        // and put it back
        if (cup_away_ms > 0) {
            cup_away_ms = cup_away_ms > SIM_STEP_MS ? cup_away_ms - SIM_STEP_MS : 0;
            if (cup_away_ms == 0) {
                powder_sim_empty_pan(&sim);
                powder_sim_tare(&sim);
            }
            if (new_reading) {
                publish(-SIM_HOLDER_WEIGHT_GR);
            }
        }
        else {
            if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL) {
                cup_removal_wait_ms += SIM_STEP_MS;
                if (cup_removal_wait_ms >= SIM_CUP_REMOVAL_DELAY_MS) {
                    cup_removal_wait_ms = 0;
                    cup_away_ms = SIM_CUP_AWAY_MS;
                }
            }
            else {
                cup_removal_wait_ms = 0;
            }

            if (new_reading) {
                publish(reading);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SIM_STEP_MS));
    }
}

scale_handle_t simulated_scale_handle = {
    .read_loop_task = simulated_scale_read_loop_task,
    .force_zero = simulated_scale_force_zero,
};
