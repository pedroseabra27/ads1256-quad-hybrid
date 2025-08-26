#ifndef ADS1256_FRAME_H
#define ADS1256_FRAME_H

#include <stdint.h>

typedef struct {
    uint64_t timestamp_ns;   // monotonic timestamp of first sample
    uint64_t seq;            // incremental sequence
    int device_count;        // number of devices represented
    int channels_per_device; // usually 8
    // For now we store converted volts (future: separate raw buffer)
    float volts[32];         // up to 4*8 channels
} ads1256_frame_t;

#endif // ADS1256_FRAME_H
