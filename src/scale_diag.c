#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

#include "scale_diag.h"
#include "charge_mode.h"
#include "cleanup_mode.h"
#include "common.h"
#include "scale.h"


#define DIAG_MIN_SECONDS        5
#define DIAG_MAX_SECONDS        120
#define DIAG_DEFAULT_SECONDS    20


extern charge_mode_config_t charge_mode_config;

typedef enum {
    DIAG_IDLE = 0,
    DIAG_RUNNING = 1,
    DIAG_DONE = 2,
    DIAG_ABORTED = 3,
} diag_state_t;

typedef struct {
    diag_state_t state;
    uint32_t duration_s;
    float elapsed_s;

    // Results
    uint32_t samples;
    uint32_t changes;           // Readings that differ from the previous one
    float rate_hz;
    float mean_gr;
    float noise_sd_gr;          // After removing a linear drift
    float drift_gr_per_min;
    float min_gr;
    float max_gr;
    float max_jump_gr;          // Largest change between two readings
    float resolution_gr;        // Smallest non zero change seen
} diag_result_t;

static diag_result_t diag;
static TaskHandle_t diag_task_handle = NULL;


static bool scale_is_free(void) {
    // The diagnostics consume the measurements, only run when nothing else does
    return charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT;
}


static void scale_diag_task(void * p) {
    double sum_t = 0.0, sum_w = 0.0, sum_tt = 0.0, sum_tw = 0.0, sum_ww = 0.0;
    float previous = NAN;
    TickType_t start = xTaskGetTickCount();
    TickType_t first_tick = 0;
    TickType_t last_tick = 0;

    diag.samples = 0;
    diag.changes = 0;
    diag.min_gr = INFINITY;
    diag.max_gr = -INFINITY;
    diag.max_jump_gr = 0.0f;
    diag.resolution_gr = INFINITY;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        diag.elapsed_s = (float) ((now - start) * portTICK_PERIOD_MS) / 1000.0f;

        if (diag.elapsed_s >= diag.duration_s) {
            diag.state = DIAG_DONE;
            break;
        }
        if (!scale_is_free()) {
            diag.state = DIAG_ABORTED;
            break;
        }

        float w;
        if (!scale_block_wait_for_next_measurement(500, &w) || isnan(w) || isinf(w)) {
            continue;
        }
        TickType_t tick = xTaskGetTickCount();
        if (diag.samples == 0) {
            first_tick = tick;
        }
        last_tick = tick;

        double t = (double) ((tick - first_tick) * portTICK_PERIOD_MS) / 1000.0;
        sum_t += t;
        sum_w += w;
        sum_tt += t * t;
        sum_tw += t * w;
        sum_ww += (double) w * w;
        diag.samples += 1;

        diag.min_gr = fminf(diag.min_gr, w);
        diag.max_gr = fmaxf(diag.max_gr, w);
        if (!isnan(previous)) {
            float jump = fabsf(w - previous);
            if (jump > 0.0f) {
                diag.changes += 1;
                diag.resolution_gr = fminf(diag.resolution_gr, jump);
            }
            diag.max_jump_gr = fmaxf(diag.max_jump_gr, jump);
        }
        previous = w;
    }

    uint32_t n = diag.samples;
    if (n >= 3) {
        double span_s = (double) ((last_tick - first_tick) * portTICK_PERIOD_MS) / 1000.0;
        diag.rate_hz = span_s > 0.0 ? (float) ((n - 1) / span_s) : 0.0f;

        // Linear fit w = a + b t, noise is the residual standard deviation
        double denom = n * sum_tt - sum_t * sum_t;
        double b = denom != 0.0 ? (n * sum_tw - sum_t * sum_w) / denom : 0.0;
        double a = (sum_w - b * sum_t) / n;
        double sse = sum_ww - 2.0 * a * sum_w - 2.0 * b * sum_tw + n * a * a + 2.0 * a * b * sum_t + b * b * sum_tt;
        diag.mean_gr = (float) (sum_w / n);
        diag.noise_sd_gr = (float) sqrt(fmax(0.0, sse / (n - 2)));
        diag.drift_gr_per_min = (float) (b * 60.0);
    }

    diag_task_handle = NULL;
    vTaskDelete(NULL);
}


static int json_float(char * buf, size_t len, float v, int decimals) {
    if (isnan(v) || isinf(v)) {
        return snprintf(buf, len, "null");
    }
    return snprintf(buf, len, "%.*f", decimals, v);
}


bool http_rest_scale_diag(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // go (bool): start a measurement (only outside charge mode)
    // s (int): duration in seconds
    // Response: st (0 idle, 1 running, 2 done, 3 aborted), el (s), dur (s), n, ch,
    //           hz, mean, sd, drift (gr/min), min, max, jump, res (gr)
    static char buf[384];
    bool go = false;
    int seconds = DIAG_DEFAULT_SECONDS;
    const char * error = "";

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "go") == 0) {
            go = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "s") == 0) {
            seconds = atoi(values[idx]);
        }
    }

    if (go) {
        if (diag_task_handle != NULL) {
            error = "Already running";
        }
        else if (!scale_is_free()) {
            error = "Leave charge mode first";
        }
        else {
            if (seconds < DIAG_MIN_SECONDS) {
                seconds = DIAG_MIN_SECONDS;
            }
            if (seconds > DIAG_MAX_SECONDS) {
                seconds = DIAG_MAX_SECONDS;
            }
            memset(&diag, 0x0, sizeof(diag));
            diag.duration_s = (uint32_t) seconds;
            diag.state = DIAG_RUNNING;
            if (xTaskCreate(scale_diag_task, "Scale Diag", 512, NULL, 2, &diag_task_handle) != pdPASS) {
                diag.state = DIAG_IDLE;
                error = "Unable to start";
            }
        }
    }

    char hz[16], mean[16], sd[16], drift[16], min[16], max[16], jump[16], res[16];
    bool has = diag.state == DIAG_DONE || diag.state == DIAG_ABORTED;
    json_float(hz, sizeof(hz), has ? diag.rate_hz : NAN, 2);
    json_float(mean, sizeof(mean), has ? diag.mean_gr : NAN, 3);
    json_float(sd, sizeof(sd), has ? diag.noise_sd_gr : NAN, 4);
    json_float(drift, sizeof(drift), has ? diag.drift_gr_per_min : NAN, 4);
    json_float(min, sizeof(min), has ? diag.min_gr : NAN, 3);
    json_float(max, sizeof(max), has ? diag.max_gr : NAN, 3);
    json_float(jump, sizeof(jump), has ? diag.max_jump_gr : NAN, 3);
    json_float(res, sizeof(res), has ? diag.resolution_gr : NAN, 3);

    snprintf(buf, sizeof(buf),
             "%s{\"st\":%d,\"el\":%.1f,\"dur\":%lu,\"n\":%lu,\"ch\":%lu,\"hz\":%s,\"mean\":%s,\"sd\":%s,"
             "\"drift\":%s,\"min\":%s,\"max\":%s,\"jump\":%s,\"res\":%s,\"err\":\"%s\"}",
             http_json_header, (int) diag.state, diag.elapsed_s, (unsigned long) diag.duration_s,
             (unsigned long) diag.samples, (unsigned long) diag.changes, hz, mean, sd, drift, min, max, jump, res, error);

    size_t len = strlen(buf);
    file->data = buf;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
