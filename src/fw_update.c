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

#if FW_DOWNLOAD
#include "lwip/apps/http_client.h"
#include "lwip/altcp_tls.h"
#include "lwip/netif.h"
#include "mbedtls/ssl.h"
#include "pico/cyw43_arch.h"
#include "fw_ca_bundle.h"
#endif

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

#ifndef FW_DOWNLOAD
#define FW_DOWNLOAD 0
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


// Writing an image (web upload and download) ------------------------------

// Checks and preparation for an image of the given size and SHA-256 (hex)
static bool fw_begin(uint32_t size, const char * sha) {
    if (!FW_UPDATE_SUPPORTED) {
        fw_fail("Not supported on this board");
        return false;
    }
    if (charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
        fw_fail("Leave charge mode first");
        return false;
    }
    if (strlen(sha) != 64) {
        fw_fail("Missing size or sha");
        return false;
    }
    strncpy(fw_expected_sha, sha, sizeof(fw_expected_sha) - 1);
    fw_expected_sha[sizeof(fw_expected_sha) - 1] = '\0';
    for (char * c = fw_expected_sha; *c; c += 1) {
        if (*c >= 'A' && *c <= 'F') {
            *c = (char) (*c - 'A' + 'a');
        }
    }
    fw_expected_size = size;
    if (size == 0) {
        fw_fail("Size mismatch");
        return false;
    }
    if (!find_target_partition(&target_offset, &target_size)) {
        fw_fail("No A/B partition table, install it once via USB");
        return false;
    }
    if (fw_expected_size > target_size) {
        fw_fail("Image larger than the partition");
        return false;
    }

    fw_state = FW_STATE_RECEIVING;
    fw_error = "";
    fw_received = 0;
    fw_written = 0;
    sector_fill = 0;
    fw_actual_sha[0] = '\0';
    sha256_init(&fw_sha);

    printf("Firmware: %lu bytes to flash offset 0x%08lx\n", (unsigned long) fw_expected_size, (unsigned long) target_offset);
    return true;
}


static bool fw_feed(const uint8_t * data, uint32_t len) {
    if (fw_state != FW_STATE_RECEIVING) {
        return false;
    }
    if (fw_received + len > fw_expected_size) {
        fw_fail("More data than announced");
        return false;
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
                return false;
            }
            fw_written += FLASH_SECTOR_SIZE;
            sector_fill = 0;
        }
    }
    return true;
}


static void fw_end(void) {
    if (fw_state != FW_STATE_RECEIVING) {
        return;
    }
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
    printf("Firmware verified\n");
}


// lwIP httpd POST callbacks ------------------------------------------------

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd) {
    strncpy(response_uri, FW_RESPONSE_URI, response_uri_len);

    if (strncmp(uri, FW_UPLOAD_URI, strlen(FW_UPLOAD_URI)) != 0) {
        return ERR_VAL;
    }

    if (fw_state == FW_STATE_RECEIVING || fw_state == FW_STATE_REBOOTING) {
        // Do not touch the state of the running upload or download
        return ERR_VAL;
    }

    char size_buf[16];
    char sha_buf[72];
    if (!query_value(uri, "size", size_buf, sizeof(size_buf)) || !query_value(uri, "sha", sha_buf, sizeof(sha_buf))) {
        fw_fail("Missing size or sha");
        return ERR_VAL;
    }
    uint32_t size = strtoul(size_buf, NULL, 10);
    if (content_len <= 0 || (uint32_t) content_len != size) {
        fw_fail("Size mismatch");
        return ERR_VAL;
    }
    if (!fw_begin(size, sha_buf)) {
        return ERR_VAL;
    }
    fw_connection = connection;
    *post_auto_wnd = 1;
    return ERR_OK;
}


err_t httpd_post_receive_data(void *connection, struct pbuf *p) {
    err_t result = ERR_OK;

    if (connection != fw_connection || fw_state != FW_STATE_RECEIVING) {
        pbuf_free(p);
        return ERR_VAL;
    }

    for (struct pbuf * q = p; q != NULL; q = q->next) {
        if (!fw_feed((const uint8_t *) q->payload, q->len)) {
            fw_connection = NULL;
            result = ERR_VAL;
            break;
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
    fw_end();
}


// Download from GitHub by the trickler itself (Pico 2 W) -------------------
//
// raw.githubusercontent.com/<repo>/firmware/manifest.json tells the newest
// build with the size and SHA-256 of pico2_w.bin. The image is streamed into
// the update partition exactly like a web upload and checked the same way.
// TLS: the server certificate has to chain to one of the roots in
// fw_ca_bundle.h, otherwise nothing is downloaded.

#ifndef FW_UPDATE_REPO
#define FW_UPDATE_REPO      "Dud3z/OpenTrickler-RP2040-Controller"
#endif
#define FW_HOST             "raw.githubusercontent.com"
#define FW_IMAGE_NAME       "pico2_w.bin"
#define FW_MANIFEST_MAX     6144

static volatile fw_download_state_t dl_state = FW_DOWNLOAD_IDLE;
static const char * dl_error = "";
static int dl_remote_build = -1;
static uint32_t dl_remote_size = 0;

#if FW_DOWNLOAD

static char dl_remote_sha[65];

typedef enum {
    DL_JOB_MANIFEST,
    DL_JOB_IMAGE,
} dl_job_t;

static dl_job_t dl_job;
static uint32_t dl_generation = 0;      // Callbacks of an abandoned request are ignored
static char dl_manifest[FW_MANIFEST_MAX + 1];
static uint32_t dl_manifest_len = 0;
static bool dl_bad = false;             // Data of this request is ignored (wrong status, write failed)
static struct altcp_tls_config * dl_tls_config = NULL;
static struct altcp_tls_config * dl_tls_config_old = NULL;
static httpc_connection_t dl_settings;
static altcp_allocator_t dl_allocator;
static char dl_uri[160];


static void dl_fail(const char * reason) {
    dl_error = reason;
    dl_state = FW_DOWNLOAD_ERROR;
    printf("Firmware download failed: %s\n", reason);
}


// TLS connection with the server name set (SNI and the name check of the certificate)
static struct altcp_pcb * dl_tls_alloc(void * arg, u8_t ip_type) {
    struct altcp_pcb * pcb = altcp_tls_alloc((struct altcp_tls_config *) arg, ip_type);
    if (pcb != NULL) {
        mbedtls_ssl_set_hostname((mbedtls_ssl_context *) altcp_tls_context(pcb), FW_HOST);
    }
    return pcb;
}


// Value after "key": in the manifest (after start), as text
static const char * json_find(const char * text, const char * key) {
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char * at = strstr(text, pattern);
    if (at == NULL) {
        return NULL;
    }
    at = strchr(at + strlen(pattern), ':');
    if (at == NULL) {
        return NULL;
    }
    at += 1;
    while (*at == ' ' || *at == '\t' || *at == '\n' || *at == '\r') {
        at += 1;
    }
    return at;
}


static bool dl_parse_manifest(void) {
    dl_manifest[dl_manifest_len] = '\0';
    const char * build = json_find(dl_manifest, "build");
    const char * image = strstr(dl_manifest, "\"" FW_IMAGE_NAME "\"");
    if (build == NULL || image == NULL) {
        return false;
    }
    const char * size = json_find(image, "size");
    const char * sha = json_find(image, "sha256");
    if (size == NULL || sha == NULL || *sha != '"') {
        return false;
    }
    sha += 1;
    if (strspn(sha, "0123456789abcdefABCDEF") != 64 || sha[64] != '"') {
        return false;
    }
    dl_remote_build = (int) strtol(build, NULL, 10);
    dl_remote_size = strtoul(size, NULL, 10);
    memcpy(dl_remote_sha, sha, 64);
    dl_remote_sha[64] = '\0';
    return dl_remote_build > 0 && dl_remote_size > 0;
}


static err_t dl_headers(httpc_state_t * connection, void * arg, struct pbuf * hdr, u16_t hdr_len, u32_t content_len) {
    (void) connection;
    (void) hdr_len;
    if ((uint32_t) (uintptr_t) arg != dl_generation) {
        return ERR_OK;
    }
    // Status line: "HTTP/1.1 200 OK"
    char line[16];
    u16_t n = pbuf_copy_partial(hdr, line, sizeof(line) - 1, 0);
    line[n] = '\0';
    char * space = strchr(line, ' ');
    // A failure closes the connection (the result callback follows)
    if (space == NULL || strncmp(space + 1, "200", 3) != 0) {
        dl_bad = true;
        dl_fail(dl_job == DL_JOB_MANIFEST ? "No firmware published (manifest missing)" : "Image not found on the server");
        return ERR_VAL;
    }
    if (dl_job == DL_JOB_IMAGE) {
        if (content_len != dl_remote_size) {
            dl_bad = true;
            dl_fail("Image size does not match the manifest");
            return ERR_VAL;
        }
        if (!fw_begin(dl_remote_size, dl_remote_sha)) {
            dl_bad = true;
            dl_fail(fw_error);
            return ERR_VAL;
        }
    }
    return ERR_OK;
}


static err_t dl_recv(void * arg, struct altcp_pcb * conn, struct pbuf * p, err_t err) {
    (void) err;
    if (p == NULL) {
        return ERR_OK;
    }
    bool current = (uint32_t) (uintptr_t) arg == dl_generation;
    if (current && !dl_bad) {
        for (struct pbuf * q = p; q != NULL; q = q->next) {
            if (dl_job == DL_JOB_MANIFEST) {
                uint32_t take = q->len;
                if (dl_manifest_len + take > FW_MANIFEST_MAX) {
                    dl_bad = true;
                    dl_fail("Manifest too large");
                    break;
                }
                memcpy(&dl_manifest[dl_manifest_len], q->payload, take);
                dl_manifest_len += take;
            }
            else if (!fw_feed((const uint8_t *) q->payload, q->len)) {
                dl_bad = true;
                dl_fail(fw_error);
                break;
            }
        }
    }
    altcp_recved(conn, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}


static void dl_result(void * arg, httpc_result_t result, u32_t rx_content_len, u32_t srv_res, err_t err) {
    (void) rx_content_len;
    (void) srv_res;
    (void) err;
    if ((uint32_t) (uintptr_t) arg != dl_generation) {
        return;
    }
    if (dl_bad) {
        // Reported already; a started image write is discarded
        if (fw_state == FW_STATE_RECEIVING) {
            fw_fail(dl_error);
        }
        return;
    }
    if (result != HTTPC_RESULT_OK) {
        if (fw_state == FW_STATE_RECEIVING) {
            fw_fail("Download interrupted");
        }
        dl_fail(result == HTTPC_RESULT_ERR_HOSTNAME ? "No internet (DNS failed)" :
                result == HTTPC_RESULT_ERR_CONNECT ? "Connection or certificate check failed" :
                result == HTTPC_RESULT_ERR_TIMEOUT ? "Timeout" : "Download interrupted");
        return;
    }

    if (dl_job == DL_JOB_MANIFEST) {
        if (!dl_parse_manifest()) {
            dl_fail("Manifest not readable");
            return;
        }
        dl_state = FW_DOWNLOAD_CHECKED;
        printf("Firmware on GitHub: build %d (running %d)\n", dl_remote_build, APP_BUILD_NUMBER);
    }
    else {
        fw_end();
        if (fw_state == FW_STATE_VERIFIED) {
            dl_state = FW_DOWNLOAD_DONE;
        }
        else {
            dl_fail(fw_error);
        }
    }
}


static bool dl_start(dl_job_t job, const char * file) {
    if (dl_state == FW_DOWNLOAD_CHECKING || dl_state == FW_DOWNLOAD_DOWNLOADING) {
        return false;
    }
    cyw43_arch_lwip_begin();

    if (netif_default == NULL || !netif_is_link_up(netif_default) || ip4_addr_isany_val(*netif_ip4_addr(netif_default))) {
        cyw43_arch_lwip_end();
        dl_fail("WiFi not connected");
        return false;
    }

    // The config of the last request is freed now, its connection is gone
    if (dl_tls_config_old != NULL) {
        altcp_tls_free_config(dl_tls_config_old);
        dl_tls_config_old = NULL;
    }
    if (dl_tls_config == NULL) {
        dl_tls_config = altcp_tls_create_config_client((const u8_t *) fw_ca_bundle, sizeof(fw_ca_bundle));
    }
    if (dl_tls_config == NULL) {
        cyw43_arch_lwip_end();
        dl_fail("TLS setup failed (memory)");
        return false;
    }

    dl_generation += 1;
    dl_job = job;
    dl_bad = false;
    dl_manifest_len = 0;
    dl_error = "";

    memset(&dl_settings, 0x0, sizeof(dl_settings));
    dl_allocator.alloc = dl_tls_alloc;
    dl_allocator.arg = dl_tls_config;
    dl_settings.altcp_allocator = &dl_allocator;
    dl_settings.headers_done_fn = dl_headers;
    dl_settings.result_fn = dl_result;

    snprintf(dl_uri, sizeof(dl_uri), "/%s/firmware/%s", FW_UPDATE_REPO, file);
    dl_state = job == DL_JOB_MANIFEST ? FW_DOWNLOAD_CHECKING : FW_DOWNLOAD_DOWNLOADING;

    httpc_state_t * connection = NULL;
    err_t err = httpc_get_file_dns(FW_HOST, 443, dl_uri, &dl_settings, dl_recv,
                                   (void *) (uintptr_t) dl_generation, &connection);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        dl_fail("Could not start the download");
        return false;
    }
    return true;
}

#endif  // FW_DOWNLOAD


bool fw_download_supported(void) {
    return FW_DOWNLOAD && FW_UPDATE_SUPPORTED;
}


bool fw_download_check(void) {
#if FW_DOWNLOAD
    return dl_start(DL_JOB_MANIFEST, "manifest.json");
#else
    dl_error = "Updates over WiFi need the Pico 2 W";
    dl_state = FW_DOWNLOAD_ERROR;
    return false;
#endif
}


bool fw_download_install(void) {
#if FW_DOWNLOAD
    if (dl_state != FW_DOWNLOAD_CHECKED || dl_remote_build <= 0) {
        return false;
    }
    if (fw_state == FW_STATE_RECEIVING || fw_state == FW_STATE_REBOOTING) {
        return false;
    }
    return dl_start(DL_JOB_IMAGE, FW_IMAGE_NAME);
#else
    return false;
#endif
}


// A request that hangs (no answer at all) is given up: its callbacks are ignored
void fw_download_abandon(const char * reason) {
#if FW_DOWNLOAD
    dl_generation += 1;
    if (dl_tls_config != NULL && dl_tls_config_old == NULL) {
        // Freed with the next request, the stale connection may still use it
        dl_tls_config_old = dl_tls_config;
        dl_tls_config = NULL;
    }
#endif
    if (fw_state == FW_STATE_RECEIVING && fw_connection == NULL) {
        fw_fail(reason);
    }
    dl_error = reason;
    dl_state = FW_DOWNLOAD_ERROR;
}


void fw_download_status(fw_download_status_t * status) {
    status->state = dl_state;
    status->error = dl_error;
    status->remote_build = dl_remote_build;
    status->running_build = APP_BUILD_NUMBER;
    status->received = fw_received;
    status->size = dl_state == FW_DOWNLOAD_DOWNLOADING ? dl_remote_size : fw_expected_size;
    status->verified = fw_state == FW_STATE_VERIFIED;
}


bool fw_update_available(void) {
    return dl_remote_build > APP_BUILD_NUMBER;
}


// Restart into the verified image (the bootrom tries it, the new firmware confirms itself)
bool fw_update_apply(void) {
#if FW_UPDATE_SUPPORTED
    if (fw_state != FW_STATE_VERIFIED) {
        return false;
    }
    if (charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
        fw_fail("Leave charge mode first");
        return false;
    }
    fw_state = FW_STATE_REBOOTING;
    // The delay lets a REST response go out
    system_monitor_planned_reboot();
    rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_FLASH_UPDATE, FW_REBOOT_DELAY_MS, XIP_BASE + target_offset, 0);
    return true;
#else
    return false;
#endif
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
    // rb (int): newest build the trickler found on GitHub itself, -1 unknown
    static char buf[576];

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
             "\"max\":%lu,\"st\":\"%s\",\"err\":\"%s\",\"rx\":%lu,\"size\":%lu,\"sha\":\"%s\",\"conf\":%s,\"rb\":%d}",
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
             boolean_to_string(fw_boot_confirmed),
             dl_remote_build);

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

    if (go) {
        fw_update_apply();
    }

    return http_rest_fw_info(file, num_params, params, values);
}
