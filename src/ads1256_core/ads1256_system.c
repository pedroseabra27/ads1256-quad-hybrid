#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "include/ads1256_core/ads1256_system.h"

struct ads1256_system_s {
    ads1256_system_cfg_t cfg;
    int running;
    unsigned long synthetic_counter;
};

int ads1256_system_create(const ads1256_system_cfg_t *cfg, ads1256_system_t **out) {
    if(!cfg || !out) return ADS1256_ERR_INVALID_ARG;
    ads1256_system_t *s = (ads1256_system_t*)calloc(1, sizeof(*s));
    if(!s) return ADS1256_ERR_STATE;
    memcpy(&s->cfg, cfg, sizeof(*cfg));
    s->running = 0;
    s->synthetic_counter = 0;
    *out = s;
    return ADS1256_OK;
}

int ads1256_system_destroy(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    free(sys);
    return ADS1256_OK;
}

int ads1256_system_start(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    sys->running = 1;
    return ADS1256_OK;
}

int ads1256_system_stop(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    sys->running = 0;
    return ADS1256_OK;
}

int ads1256_system_read_voltages(ads1256_system_t *sys, float *out, int max_samples) {
    if(!sys || !out || max_samples <= 0) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    int channels_total = sys->cfg.device_count * 8; // assume all enabled for now
    // produce ramp + mild sine dither
    for(int i=0;i<max_samples;i++) {
        float base = (float)((sys->synthetic_counter + i) % 1000) / 1000.0f; // 0..1
        for(int ch=0; ch<channels_total; ch++) {
            float val = (base * 2.0f - 1.0f) + 0.01f * sinf((float)ch + (float)(sys->synthetic_counter+i)/50.0f);
            out[i*channels_total + ch] = val; // volts placeholder (-1..1 V range approx)
        }
    }
    sys->synthetic_counter += (unsigned long)max_samples;
    return max_samples;
}
