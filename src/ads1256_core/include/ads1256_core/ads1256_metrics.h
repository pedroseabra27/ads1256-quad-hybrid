#ifndef ADS1256_METRICS_H
#define ADS1256_METRICS_H

#include <stdint.h>

typedef struct {
    uint64_t frames_produced;
    uint64_t dropped_frames;   // from ring
    uint64_t last_frame_timestamp_ns;
    double   avg_frame_period_ns; // exponential moving average
    uint64_t last_frame_acq_ns;      // duration to acquire last frame (channel sweep)
    double   avg_frame_acq_ns;                   // EMA of acquisition duration
    uint64_t drdy_waits;            // count of DRDY waits attempted
    uint64_t drdy_timeouts;         // count of DRDY timeouts
    uint64_t last_drdy_wait_ns;     // duration of last DRDY wait (ns)
    double   avg_drdy_wait_ns;      // EMA of DRDY wait time
    int      device_count;
    int      channels_total;
} ads1256_metrics_t;

#endif // ADS1256_METRICS_H
