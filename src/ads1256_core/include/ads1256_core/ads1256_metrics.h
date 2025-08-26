#ifndef ADS1256_METRICS_H
#define ADS1256_METRICS_H

#include <stdint.h>

typedef struct {
    uint64_t frames_produced;
    uint64_t dropped_frames;   // from ring
    uint64_t last_frame_timestamp_ns;
    double   avg_frame_period_ns; // exponential moving average
    int      device_count;
    int      channels_total;
} ads1256_metrics_t;

#endif // ADS1256_METRICS_H
