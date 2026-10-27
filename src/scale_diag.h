#ifndef SCALE_DIAG_H_
#define SCALE_DIAG_H_

/*
 * Scale diagnostics: watches the idle scale for a while and reports update
 * rate, noise, drift and jumps. Draft, vibration and a cold scale are the
 * most common reasons for inconsistent charges.
 */

#include <stdbool.h>
#include "http_rest.h"

#ifdef __cplusplus
extern "C" {
#endif

bool http_rest_scale_diag(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // SCALE_DIAG_H_
