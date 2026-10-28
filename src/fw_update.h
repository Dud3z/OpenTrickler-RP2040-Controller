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
#include <stdint.h>
#include "http_rest.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mark a freshly updated firmware as good. Call once the system is up.
void fw_update_confirm_boot(void);

// Download from GitHub by the trickler itself (Pico 2 W, display menu)
typedef enum {
    FW_DOWNLOAD_IDLE = 0,
    FW_DOWNLOAD_CHECKING,       // Reading the manifest
    FW_DOWNLOAD_CHECKED,        // remote_build is known
    FW_DOWNLOAD_DOWNLOADING,    // Writing the image into the update partition
    FW_DOWNLOAD_DONE,           // Image verified, ready to restart
    FW_DOWNLOAD_ERROR,
} fw_download_state_t;

typedef struct {
    fw_download_state_t state;
    const char * error;
    int remote_build;
    int running_build;
    uint32_t received;
    uint32_t size;
    bool verified;
} fw_download_status_t;

bool fw_download_supported(void);
bool fw_download_check(void);
bool fw_download_install(void);
void fw_download_abandon(const char * reason);
void fw_download_status(fw_download_status_t * status);
bool fw_update_available(void);
bool fw_update_apply(void);

// Display menu (Settings - Firmware Update), returns the form to go back to
uint8_t fw_update_display_menu(void);

// REST
bool http_rest_fw_info(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_fw_apply(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // FW_UPDATE_H_
