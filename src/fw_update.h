#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

/*
 * Firmware update over WiFi (RP2350 / Pico 2 W)
 *
 * The flash holds a partition table with two application partitions (A/B).
 * A new image is uploaded with HTTP POST /upload/firmware into the partition
 * that is not running, verified with SHA-256 and then started with a
 * "flash update" reboot. The image is built "try before you buy": unless the
 * new firmware confirms itself with fw_update_confirm_boot() after a
 * successful start, the bootrom falls back to the previous firmware.
 *
 * Settings and learned models live in the external EEPROM and survive.
 */

#include <stdbool.h>
#include "http_rest.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mark a freshly updated firmware as good. Call once the system is up.
void fw_update_confirm_boot(void);

// REST
bool http_rest_fw_info(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_fw_apply(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // FW_UPDATE_H_
