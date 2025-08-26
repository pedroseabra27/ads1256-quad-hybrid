#ifndef ADS1256_SYSTEM_H
#define ADS1256_SYSTEM_H

#include "ads1256_error.h"
#include "ads1256_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ads1256_system_s ads1256_system_t;

// Allocate & initialize internal structures (no hardware yet)
int ads1256_system_create(const ads1256_system_cfg_t *cfg, ads1256_system_t **out);

// Release all resources
int ads1256_system_destroy(ads1256_system_t *sys);

// Start continuous acquisition (synthetic for now)
int ads1256_system_start(ads1256_system_t *sys);

// Stop acquisition
int ads1256_system_stop(ads1256_system_t *sys);

// Read synthetic samples (float volts) into provided buffer; returns samples per channel
int ads1256_system_read_voltages(ads1256_system_t *sys, float *out, int max_samples);

#ifdef __cplusplus
}
#endif

#endif // ADS1256_SYSTEM_H
