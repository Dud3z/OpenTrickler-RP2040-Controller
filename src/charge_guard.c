#include <math.h>

#include "charge_guard.h"


void charge_guard_reset(charge_guard_t * guard, uint32_t quiet_until_ms) {
    guard->count = 0;
    guard->next = 0;
    guard->quiet_until_ms = quiet_until_ms;
}


void charge_guard_coarse_stopped(charge_guard_t * guard, uint32_t now_ms) {
    guard->quiet_until_ms = now_ms + CHARGE_GUARD_COARSE_GRACE_MS;
}


charge_guard_result_t charge_guard_check(charge_guard_t * guard, const charge_guard_config_t * config,
                                         float weight, bool coarse_running, uint32_t now_ms) {
    if (isnan(weight) || isinf(weight)) {
        return CHARGE_GUARD_OK;
    }
    if (config->pan_guard && weight < -CHARGE_GUARD_PAN_REMOVED_GR) {
        return CHARGE_GUARD_PAN_REMOVED;
    }

    charge_guard_result_t result = CHARGE_GUARD_OK;
    if (config->bump_guard && guard->count >= 2) {
        float recent_max = guard->recent[0];
        for (uint8_t i = 1; i < guard->count; i += 1) {
            recent_max = fmaxf(recent_max, guard->recent[i]);
        }
        // Wrap safe: (int32_t) of the difference
        bool quiet = coarse_running || (int32_t) (guard->quiet_until_ms - now_ms) > 0;
        float drop_limit = quiet ? CHARGE_GUARD_COARSE_DROP_GR : config->bump_drop_gr;
        if (recent_max - weight > drop_limit) {
            result = CHARGE_GUARD_BUMP;
        }
    }

    guard->recent[guard->next] = weight;
    guard->next = (guard->next + 1) % CHARGE_GUARD_WINDOW;
    if (guard->count < CHARGE_GUARD_WINDOW) {
        guard->count += 1;
    }
    return result;
}
