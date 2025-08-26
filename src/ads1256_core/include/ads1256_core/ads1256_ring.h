#ifndef ADS1256_RING_H
#define ADS1256_RING_H

#include <stdint.h>
#include <stddef.h>
#include "ads1256_frame.h"

typedef struct {
    ads1256_frame_t *frames;
    size_t capacity; // number of frames
    volatile size_t write_idx;
    volatile size_t read_idx;
    volatile uint64_t dropped;
} ads1256_ring_t;

int ads1256_ring_init(ads1256_ring_t *r, size_t capacity);
int ads1256_ring_free(ads1256_ring_t *r);
int ads1256_ring_push(ads1256_ring_t *r, const ads1256_frame_t *frame); // returns 0 ok, -1 dropped (overwrote)
int ads1256_ring_pop(ads1256_ring_t *r, ads1256_frame_t *out); // returns 1 frame, 0 empty

#endif // ADS1256_RING_H
