#ifndef CHARGE_GUARD_H_
#define CHARGE_GUARD_H_

/*
 * Charge guards (Settings - Charging - Safety), pure logic so it can be
 * tested on the host (tests/sim/guard_test.c):
 *  - pan removed: the reading falls far below zero
 *  - bump: the reading drops while powder can only be added
 * Powder never makes the reading fall, but the coarse stream pushes on the
 * pan: when it stops, the reading falls by the push (about 1 gr at high flow).
 * So while the coarse trickler runs, and shortly after it stopped, only big
 * drops count.
 */

#include <stdint.h>
#include <stdbool.h>

#define CHARGE_GUARD_PAN_REMOVED_GR     2.0f    // Below zero by this much: pan lifted
#define CHARGE_GUARD_COARSE_DROP_GR     5.0f    // Drop that counts as a bump while the coarse stream lands
#define CHARGE_GUARD_COARSE_GRACE_MS    2000    // After a coarse stop the stream still lands / the push fades
#define CHARGE_GUARD_WINDOW             4       // Recent samples the drop is measured against

typedef enum {
    CHARGE_GUARD_OK = 0,
    CHARGE_GUARD_PAN_REMOVED,
    CHARGE_GUARD_BUMP,
} charge_guard_result_t;

typedef struct {
    float recent[CHARGE_GUARD_WINDOW];
    uint8_t count;
    uint8_t next;
    uint32_t quiet_until_ms;    // Only big drops count until then
} charge_guard_t;

typedef struct {
    bool pan_guard;
    bool bump_guard;
    float bump_drop_gr;
} charge_guard_config_t;

#ifdef __cplusplus
extern "C" {
#endif

void charge_guard_reset(charge_guard_t * guard, uint32_t quiet_until_ms);

// After a coarse stop: the stream still lands and its push fades
void charge_guard_coarse_stopped(charge_guard_t * guard, uint32_t now_ms);

charge_guard_result_t charge_guard_check(charge_guard_t * guard, const charge_guard_config_t * config,
                                         float weight, bool coarse_running, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif  // CHARGE_GUARD_H_
