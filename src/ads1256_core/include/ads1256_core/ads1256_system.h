#ifndef ADS1256_SYSTEM_H
#define ADS1256_SYSTEM_H

#include "ads1256_error.h"
#include "ads1256_types.h"
#include "ads1256_frame.h"
#include "ads1256_metrics.h"

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

// Read one frame (all channels for each initialized device) using channel sweep (blocking);
// out_volts size must be >= device_count*8; returns total channels filled or error.
int ads1256_system_read_frame(ads1256_system_t *sys, float *out_volts);

// Raw variant: output buffer int32_t sized >= device_count*8 (24-bit sign-extended codes)
int ads1256_system_read_frame_raw(ads1256_system_t *sys, int32_t *out_raw);

// Start background acquisition thread for single-device prototype (frames into ring)
int ads1256_system_start_thread(ads1256_system_t *sys);
int ads1256_system_stop_thread(ads1256_system_t *sys);
int ads1256_system_pop_frame(ads1256_system_t *sys, ads1256_frame_t *out_frame); // returns 1 frame, 0 empty, <0 error
// Zero-copy variant: returns internal frame pointer (valid until that slot reused after ring wraps)
int ads1256_system_pop_frame_ref(ads1256_system_t *sys, const ads1256_frame_t **out_frame);

// Simple metrics: dropped frames (ring overwrites). Returns 0 on success.
int ads1256_system_get_dropped(ads1256_system_t *sys, unsigned long long *out_dropped);

// Copy current metrics snapshot
int ads1256_system_get_metrics(ads1256_system_t *sys, ads1256_metrics_t *out);

// Experimental: non-blocking check or blocking wait for DRDY (returns 1 ready, 0 timeout, <0 error)
// Placeholder until libgpiod integration implemented.
int ads1256_system_wait_drdy(ads1256_system_t *sys, int device_index, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // ADS1256_SYSTEM_H
