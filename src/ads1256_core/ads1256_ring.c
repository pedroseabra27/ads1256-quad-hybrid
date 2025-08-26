#include <stdlib.h>
#include <string.h>
#include "include/ads1256_core/ads1256_ring.h"

int ads1256_ring_init(ads1256_ring_t *r, size_t capacity) {
    if(!r || capacity == 0) return -1;
    r->frames = (ads1256_frame_t*)calloc(capacity, sizeof(ads1256_frame_t));
    if(!r->frames) return -1;
    r->capacity = capacity;
    r->write_idx = 0;
    r->read_idx = 0;
    r->dropped = 0;
    return 0;
}

int ads1256_ring_free(ads1256_ring_t *r) {
    if(!r) return -1;
    free(r->frames);
    r->frames = NULL;
    r->capacity = 0;
    return 0;
}

int ads1256_ring_push(ads1256_ring_t *r, const ads1256_frame_t *frame) {
    if(!r || !frame) return -1;
    size_t next = (r->write_idx + 1) % r->capacity;
    if(next == r->read_idx) {
        // overflow: drop oldest (advance read)
        r->read_idx = (r->read_idx + 1) % r->capacity;
        r->dropped++;
    }
    r->frames[r->write_idx] = *frame;
    r->write_idx = next;
    return 0;
}

int ads1256_ring_pop(ads1256_ring_t *r, ads1256_frame_t *out) {
    if(!r || !out) return -1;
    if(r->read_idx == r->write_idx) return 0; // empty
    *out = r->frames[r->read_idx];
    r->read_idx = (r->read_idx + 1) % r->capacity;
    return 1;
}
