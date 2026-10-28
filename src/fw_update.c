#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico.h"
#include "lwip/apps/httpd.h"
#include "lwip/pbuf.h"

#include "fw_update.h"
#include "charge_mode.h"
#include "common.h"
#include "sha256.h"
#include "system_monitor.h"
#include "version.h"

#if PICO_RP2350
#include "boot/picobin.h"
#include "boot/uf2.h"
#include "hardware/flash.h"
#include "pico/bootrom.h"
#include "boot/picoboot_constants.h"
#include "pico/flash.h"
#include <FreeRTOS.h>
#include <task.h>
#define FW_UPDATE_SUPPORTED 1
#else
#define FW_UPDATE_SUPPORTED 0
#define FLASH_SECTOR_SIZE 4096
#endif

#ifndef APP_BUILD_NUMBER
#define APP_BUILD_NUMBER 0
#endif

#ifdef PICO_BOARD
#define BOARD_NAME PICO_BOARD
#else
#define BOARD_NAME "unknown"
#endif

#define FW_UPLOAD_URI       "/upload/firmware"
#define FW_RESPONSE_URI     "/rest/fw_info"
#define FW_REBOOT_DELAY_MS  1500


typedef enum {
    FW_STATE_IDLE = 0,
    FW_STATE_RECEIVING = 1,
    FW_STATE_VERIFIED = 2,
    FW_STATE_ERROR = 3,
    FW_STATE_REBOOTING = 4,
} fw_state_t;


extern charge_mode_config_t charge_mode_config;

static fw_state_t fw_state = FW_STATE_IDLE;
static const char * fw_error = "";
static void * fw_connection = NULL;
static uint32_t fw_expected_size = 0;
static uint32_t fw_received = 0;
static uint32_t fw_written = 0;
static char fw_expected_sha[65];
static char fw_actual_sha[65];
static sha256_ctx_t fw_sha;
static bool fw_boot_confirmed = false;

static uint32_t target_offset = 0;      // Physical flash offset of the partition to write
static uint32_t target_size = 0;

static uint32_t sector_fill = 0;


#if FW_UPDATE_SUPPORTED

static uint8_t sector_buffer[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

static uint8_t rom_workarea[4096] __attribute__((aligned(4)));


static bool partition_location(uint32_t location, uint32_t * offset, uint32_t * size) {
    uint32_t first = (location & PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_BITS) >> PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_LSB;
    uint32_t last = (location & PICOBIN_PARTITION_LOCATION_LAST_SECTOR_BITS) >> PICOBIN_PARTITION_LOCATION_LAST_SECTOR_LSB;
    if (last < first) {
        return false;
    }
    *offset = first * FLASH_SECTOR_SIZE;
    *size = (last + 1 - first) * FLASH_SECTOR_SIZE;
    return true;
}


static int running_partition(void) {
    boot_info_t boot_info;
    if (!rom_get_boot_info(&boot_info)) {
        return -1;
    }
    return boot_info.partition;
}


/*
 * Ask the bootrom where a UF2 for this chip would go: with an A/B pair this
 * is the partition we are not running from.
 */
static bool find_target_partition(uint32_t * offset, uint32_t * size) {
    resident_partition_t target;
    int rc = rom_get_uf2_target_partition(rom_workarea, sizeof(rom_workarea), RP2350_ARM_S_FAMILY_ID, &target);
    if (rc < 0) {
        return false;
    }
    if (!partition_location(target.permissions_and_location, offset, size)) {
        return false;
    }

    // Never overwrite the partition we are running from
    int running = running_partition();
    if (running < 0) {
        // No partition table: running from the start of flash
        return false;
    }
    uint32_t info[3];
    int words = rom_get_partition_table_info(info, 3, PT_INFO_SINGLE_PARTITION | PT_INFO_PARTITION_LOCATION_AND_FLAGS |
                                                      ((uint32_t) running << 24));
    if (words >= 2) {
        uint32_t running_offset;
        uint32_t running_size;
        if (partition_location(info[1], &running_offset, &running_size) && running_offset == *offset) {
            return false;
        }
    }

    return true;
}


typedef struct {
    uint32_t offset;
    const uint8_t * data;
} sector_write_t;


static void flash_write_sector_unsafe(void * param) {
    const sector_write_t * w = (const sector_write_t *) param;
    flash_range_erase(w->offset, FLASH_SECTOR_SIZE);
    flash_range_program(w->offset, w->data, FLASH_SECTOR_SIZE);
}


static bool write_sector(uint32_t offset, const uint8_t * data) {
    sector_write_t w = {.offset = offset, .data = data};

    // flash_safe_execute parks the other core; the calling task must not
    // migrate to that core meanwhile, so pin it for the duration
    UBaseType_t affinity = vTaskCoreAffinityGet(NULL);
    vTaskCoreAffinitySet(NULL, 1u << get_core_num());
    int rc = flash_safe_execute(flash_write_sector_unsafe, &w, 2000);
    vTaskCoreAffinitySet(NULL, affinity);

    return rc == PICO_OK;
}


void fw_update_confirm_boot(void) {
#if PICO_CRT0_IMAGE_TYPE_TBYB
    // A "flash update" boot of a try-before-you-buy image has to be confirmed,
    // otherwise the bootrom returns to the previous firmware on the next reset
    if (rom_get_last_boot_type() == BOOT_TYPE_FLASH_UPDATE) {
        int rc = rom_explicit_buy(rom_workarea, sizeof(rom_workarea));
        fw_boot_confirmed = rc == BOOTROM_OK;
        printf("Firmware update confirmed: %d\n", rc);
    }
#endif
}

#else

static uint8_t sector_buffer[1];

static bool find_target_partition(uint32_t * offset, uint32_t * size) {
    return false;
}


static bool write_sector(uint32_t offset, const uint8_t * data) {
    return false;
}


void fw_update_confirm_boot(void) {
}

#endif  // FW_UPDATE_SUPPORTED


static void fw_fail(const char * reason) {
    fw_state = FW_STATE_ERROR;
    fw_error = reason;
    fw_connection = NULL;
    printf("Firmware upload failed: %s\n", reason);
}


static bool flush_sector(void) {
    if (sector_fill == 0) {
        return true;
    }
    // Pad the last sector with erased flash
    memset(&sector_buffer[sector_fill], 0xff, FLASH_SECTOR_SIZE - sector_fill);
    if (!write_sector(target_offset + fw_written, sector_buffer)) {
        return false;
    }
    fw_written += FLASH_SECTOR_SIZE;
    sector_fill = 0;
    return true;
}


// Value of a query parameter, copied into buf
static bool query_value(const char * uri, const char * key, char * buf, size_t len) {
    const char * query = strchr(uri, '?');
    size_t key_len = strlen(key);

    while (query != NULL) {
        query += 1;
        if (strncmp(query, key, key_len) == 0 && query[key_len] == '=') {
            const char * value = query + key_len + 1;
            size_t n = strcspn(value, "& ");
            if (n >= len) {
                n = len - 1;
            }
            memcpy(buf, value, n);
            buf[n] = '\0';
            return true;
        }
        query = strchr(query, '&');
    }
    return false;
}


// lwIP httpd POST callbacks ------------------------------------------------

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd) {
    strncpy(response_uri, FW_RESPONSE_URI, response_uri_len);

    if (strncmp(uri, FW_UPLOAD_URI, strlen(FW_UPLOAD_URI)) != 0) {
        return ERR_VAL;
    }

    if (!FW_UPDATE_SUPPORTED) {
        fw_fail("Not supported on this board");
        return ERR_VAL;
    }
    if (fw_state == FW_STATE_RECEIVING || fw_state == FW_STATE_REBOOTING) {
        // Do not touch the state of the running upload
        return ERR_VAL;
    }
    if (charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
        fw_fail("Leave charge mode first");
        return ERR_VAL;
    }

    char size_buf[16];
    if (!query_value(uri, "size", size_buf, sizeof(size_buf)) ||
        !query_value(uri, "sha", fw_expected_sha, sizeof(fw_expected_sha)) ||
        strlen(fw_expected_sha) != 64) {
        fw_fail("Missing size or sha");
        return ERR_VAL;
    }
    fw_expected_size = strtoul(size_buf, NULL, 10);
    for (char * c = fw_expected_sha; *c; c += 1) {
        if (*c >= 'A' && *c <= 'F') {
            *c = (char) (*c - 'A' + 'a');
        }
    }

    if (content_len <= 0 || (uint32_t) content_len != fw_expected_size) {
        fw_fail("Size mismatch");
        return ERR_VAL;
    }
    if (!find_target_partition(&target_offset, &target_size)) {
        fw_fail("No A/B partition table, install it once via USB");
        return ERR_VAL;
    }
    if (fw_expected_size > target_size) {
        fw_fail("Image larger than the partition");
        return ERR_VAL;
    }

    fw_state = FW_STATE_RECEIVING;
    fw_error = "";
    fw_connection = connection;
    fw_received = 0;
    fw_written = 0;
    sector_fill = 0;
    fw_actual_sha[0] = '\0';
    sha256_init(&fw_sha);
    *post_auto_wnd = 1;

    printf("Firmware upload: %lu bytes to flash offset 0x%08lx\n", (unsigned long) fw_expected_size, (unsigned long) target_offset);
    return ERR_OK;
}


err_t httpd_post_receive_data(void *connection, struct pbuf *p) {
    err_t result = ERR_OK;

    if (connection != fw_connection || fw_state != FW_STATE_RECEIVING) {
        pbuf_free(p);
        return ERR_VAL;
    }

    for (struct pbuf * q = p; q != NULL; q = q->next) {
        const uint8_t * data = (const uint8_t *) q->payload;
        uint32_t len = q->len;

        if (fw_received + len > fw_expected_size) {
            fw_fail("More data than announced");
            result = ERR_VAL;
            break;
        }
        sha256_update(&fw_sha, data, len);
        fw_received += len;

        while (len > 0) {
            uint32_t take = FLASH_SECTOR_SIZE - sector_fill;
            if (take > len) {
                take = len;
            }
            memcpy(&sector_buffer[sector_fill], data, take);
            sector_fill += take;
            data += take;
            len -= take;

            if (sector_fill == FLASH_SECTOR_SIZE) {
                if (!write_sector(target_offset + fw_written, sector_buffer)) {
                    fw_fail("Flash write failed");
                    pbuf_free(p);
                    return ERR_VAL;
                }
                fw_written += FLASH_SECTOR_SIZE;
                sector_fill = 0;
            }
        }
    }

    pbuf_free(p);
    return result;
}


void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) {
    strncpy(response_uri, FW_RESPONSE_URI, response_uri_len);

    if (connection != fw_connection || fw_state != FW_STATE_RECEIVING) {
        return;
    }
    fw_connection = NULL;

    if (fw_received != fw_expected_size) {
        fw_fail("Upload incomplete");
        return;
    }
    if (!flush_sector()) {
        fw_fail("Flash write failed");
        return;
    }

    uint8_t digest[32];
    sha256_final(&fw_sha, digest);
    sha256_to_hex(digest, fw_actual_sha);

    if (strcmp(fw_actual_sha, fw_expected_sha) != 0) {
        fw_fail("Checksum mismatch");
        return;
    }

    fw_state = FW_STATE_VERIFIED;
    printf("Firmware upload verified\n");
}


// REST ---------------------------------------------------------------------

static const char * state_name(fw_state_t state) {
    switch (state) {
        case FW_STATE_IDLE:
            return "idle";
        case FW_STATE_RECEIVING:
            return "receiving";
        case FW_STATE_VERIFIED:
            return "verified";
        case FW_STATE_ERROR:
            return "error";
        case FW_STATE_REBOOTING:
            return "rebooting";
        default:
            return "?";
    }
}


bool http_rest_fw_info(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Read only
    // board (str), ver (str), vcs (str), build (int): running firmware
    // sup (bool): updates supported on this board
    // part (int): partition the firmware runs from, -1 without partition table
    // ab (bool): an A/B partition table is present and an update target exists
    // max (int): maximum image size
    // st (str), err (str), rx (int), size (int), sha (str): upload state
    // conf (bool): this boot was an update that has been confirmed
    static char buf[512];

    int part = -1;
    bool ab = false;
    uint32_t size = 0;

#if FW_UPDATE_SUPPORTED
    uint32_t offset = 0;
    part = running_partition();
    if (fw_state != FW_STATE_RECEIVING) {
        ab = find_target_partition(&offset, &size);
    }
    else {
        ab = true;
        size = target_size;
    }
#endif

    snprintf(buf, sizeof(buf),
             "%s"
             "{\"board\":\"%s\",\"ver\":\"%s\",\"vcs\":\"%s\",\"build\":%d,\"sup\":%s,\"part\":%d,\"ab\":%s,"
             "\"max\":%lu,\"st\":\"%s\",\"err\":\"%s\",\"rx\":%lu,\"size\":%lu,\"sha\":\"%s\",\"conf\":%s}",
             http_json_header,
             BOARD_NAME,
             version_string,
             vcs_hash,
             APP_BUILD_NUMBER,
             boolean_to_string(FW_UPDATE_SUPPORTED),
             part,
             boolean_to_string(ab),
             (unsigned long) size,
             state_name(fw_state),
             fw_error,
             (unsigned long) fw_received,
             (unsigned long) fw_expected_size,
             fw_actual_sha,
             boolean_to_string(fw_boot_confirmed));

    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_fw_apply(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // go (bool): reboot into the verified firmware
    // reset (bool): discard an upload that failed or was not applied
    bool go = false;
    bool reset = false;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "go") == 0) {
            go = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "reset") == 0) {
            reset = string_to_boolean(values[idx]);
        }
    }

    if (reset && fw_state != FW_STATE_RECEIVING && fw_state != FW_STATE_REBOOTING) {
        fw_state = FW_STATE_IDLE;
        fw_error = "";
    }

#if FW_UPDATE_SUPPORTED
    if (go && fw_state == FW_STATE_VERIFIED) {
        if (charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
            fw_fail("Leave charge mode first");
        }
        else {
            fw_state = FW_STATE_REBOOTING;
            // The bootrom tries the new partition first; the delay lets this response go out
            system_monitor_planned_reboot();
            rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_FLASH_UPDATE, FW_REBOOT_DELAY_MS, XIP_BASE + target_offset, 0);
        }
    }
#else
    (void) go;
#endif

    return http_rest_fw_info(file, num_params, params, values);
}
