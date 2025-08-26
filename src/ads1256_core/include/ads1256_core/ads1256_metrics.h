#ifndef ADS1256_METRICS_H
#define ADS1256_METRICS_H

#include <stdint.h>

typedef struct {
    uint64_t frames_produced;
    uint64_t dropped_frames;   // from ring
    uint64_t last_frame_timestamp_ns;
    double   avg_frame_period_ns; // exponential moving average
    double   rms_frame_jitter_ns; // running RMS of (period - avg)^2
    double   max_frame_jitter_ns; // max abs(period-avg) observed (since start)
    uint64_t last_frame_acq_ns;      // duration to acquire last frame (channel sweep)
    double   avg_frame_acq_ns;                   // EMA of acquisition duration
    uint64_t drdy_waits;            // count of DRDY waits attempted
    uint64_t drdy_timeouts;         // count of DRDY timeouts
    uint64_t last_drdy_wait_ns;     // duration of last DRDY wait (ns)
    double   avg_drdy_wait_ns;      // EMA of DRDY wait time
    int      device_count;
    int      channels_total;
    // Jitter histogram buckets (ns): <1k, <2k, <5k, <10k, <20k, <50k, <100k, >=100k
    uint64_t jitter_hist[8];
} ads1256_metrics_t;

#endif // ADS1256_METRICS_H
