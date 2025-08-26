#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "include/ads1256_core/ads1256_system.h"
#include "include/ads1256_core/ads1256_spi.h"
#include "include/ads1256_core/ads1256_regs.h"
#include "include/ads1256_core/ads1256_ring.h"
#include "include/ads1256_core/ads1256_frame.h"
#ifdef __linux__
#include <pthread.h>
#endif

struct ads1256_system_s {
    ads1256_system_cfg_t cfg;
    int running;
    unsigned long synthetic_counter;
    ads1256_spi_dev_t spi_devs[4]; // future multi devices
    int hw_initialized;
    ads1256_ring_t ring;
    int ring_ready;
    int thread_running;
    pthread_t thread;
    uint64_t seq_counter;
    ads1256_metrics_t metrics;
};

static int write_reg(ads1256_spi_dev_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[3];
    buf[0] = ADS1256_CMD_WREG | reg;
    buf[1] = 0x00; // one register
    buf[2] = value;
    return ads1256_spi_write(dev, buf, 3);
}

static int write_regs(ads1256_spi_dev_t *dev, uint8_t start_reg, const uint8_t *values, uint8_t count) {
    if(count == 0) return ADS1256_OK;
    uint8_t hdr[2];
    hdr[0] = ADS1256_CMD_WREG | start_reg;
    hdr[1] = (uint8_t)(count - 1);
    // allocate temp buffer stack (small)
    uint8_t buf[2 + 16];
    if(count > 16) return ADS1256_ERR_INVALID_ARG;
    memcpy(buf, hdr, 2);
    memcpy(buf + 2, values, count);
    return ads1256_spi_write(dev, buf, 2 + count);
}

// Minimal settling delay between ADS1256 command phases / MUX changes.
// Currently a fixed 40us; will be replaced by DRDY-based wait later.
static int wait_short() {
    struct timespec ts = {0, 40000}; // 40us
    nanosleep(&ts, NULL);
    return 0;
}

static uint8_t map_drate(int sample_rate) {
    if(sample_rate >= 30000) return ADS1256_DRATE_30000;
    if(sample_rate >= 15000) return ADS1256_DRATE_15000;
    if(sample_rate >= 7500) return ADS1256_DRATE_7500;
    if(sample_rate >= 3750) return ADS1256_DRATE_3750;
    return ADS1256_DRATE_3750; // fallback
}

static uint8_t map_gain_bits(int gain) {
    switch(gain) {
        case 1: return 0x00; // gain bits 000
        case 2: return 0x01;
        case 4: return 0x02;
        case 8: return 0x03;
        case 16: return 0x04;
        case 32: return 0x05;
        case 64: return 0x06;
        default: return 0x00;
    }
}

// Perform single conversion for current MUX & gain: SYNC->WAKEUP->RDATA->read 3 bytes
static int single_conversion_read(ads1256_spi_dev_t *dev, int32_t *out_raw) {
    uint8_t cmd;
    cmd = ADS1256_CMD_SYNC; ads1256_spi_write(dev, &cmd, 1);
    cmd = ADS1256_CMD_WAKEUP; ads1256_spi_write(dev, &cmd, 1);
    // small settle time
    wait_short();
    cmd = ADS1256_CMD_RDATA; ads1256_spi_write(dev, &cmd, 1);
    // Wait t6 (datasheet) ~50 * 1/CLKIN; reuse wait_short minimal for now
    wait_short();
    return read_sample24(dev, out_raw);
}

static int read_sample24(ads1256_spi_dev_t *dev, int32_t *out_raw) {
    uint8_t dummy[3] = {0xFF,0xFF,0xFF};
    uint8_t rx[3] = {0};
    int r = ads1256_spi_transfer(dev, dummy, rx, 3);
    if(r != ADS1256_OK) return r;
    int32_t raw = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | (int32_t)rx[2];
    if(raw & 0x800000) raw |= 0xFF000000;
    *out_raw = raw;
    return ADS1256_OK;
}

int ads1256_system_create(const ads1256_system_cfg_t *cfg, ads1256_system_t **out) {
    if(!cfg || !out) return ADS1256_ERR_INVALID_ARG;
    ads1256_system_t *s = (ads1256_system_t*)calloc(1, sizeof(*s));
    if(!s) return ADS1256_ERR_STATE;
    memcpy(&s->cfg, cfg, sizeof(*cfg));
    s->running = 0;
    s->synthetic_counter = 0;
    s->hw_initialized = 0;
    s->ring_ready = 0;
    s->thread_running = 0;
    s->seq_counter = 0;
    memset(&s->metrics, 0, sizeof(s->metrics));
    s->metrics.device_count = s->cfg.device_count;
    s->metrics.channels_total = s->cfg.device_count * 8;
    *out = s;
    return ADS1256_OK;
}

int ads1256_system_destroy(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(sys->thread_running) {
        // user should have stopped, but force
        sys->thread_running = 0;
        #ifdef __linux__
        pthread_join(sys->thread, NULL);
        #endif
    }
    if(sys->ring_ready) {
        ads1256_ring_free(&sys->ring);
    }
    free(sys);
    return ADS1256_OK;
}

int ads1256_system_start(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    // attempt simple hardware init for first device if present
    if(sys->cfg.device_count > 0 && !sys->hw_initialized) {
        for(int d=0; d<sys->cfg.device_count; d++) {
            ads1256_device_cfg_t *dcfg = &sys->cfg.devices[d];
            if(ads1256_spi_open(&sys->spi_devs[d], dcfg->spi_bus, dcfg->chip_select, 1000000) != ADS1256_OK) {
                continue; // leave unopened; future: mark status
            }
            uint8_t cmd = ADS1256_CMD_RESET; ads1256_spi_write(&sys->spi_devs[d], &cmd, 1);
            struct timespec ts = {0, 10000000}; nanosleep(&ts, NULL);
            cmd = ADS1256_CMD_SDATAC; ads1256_spi_write(&sys->spi_devs[d], &cmd, 1);
            uint8_t regs[5];
            regs[0] = 0x00;
            regs[1] = 0x08; // CH0 vs COM init
            uint8_t gain_bits = map_gain_bits(1);
            regs[2] = (0x00 << 3) | gain_bits;
            regs[3] = map_drate(dcfg->sample_rate);
            regs[4] = 0x00;
            write_regs(&sys->spi_devs[d], ADS1256_REG_STATUS, regs, 5);
            cmd = ADS1256_CMD_SELFCAL; ads1256_spi_write(&sys->spi_devs[d], &cmd, 1);
            nanosleep(&ts, NULL);
        }
        sys->hw_initialized = 1; // mark overall (fine-grain status not tracked yet)
    }
    sys->running = 1;
    return ADS1256_OK;
}

int ads1256_system_read_frame_raw(ads1256_system_t *sys, int32_t *out_raw) {
    if(!sys || !out_raw) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    if(!sys->hw_initialized) return ADS1256_ERR_STATE;
    int dev_count = sys->cfg.device_count;
    for(int d=0; d<dev_count; d++) {
        ads1256_spi_dev_t *dev = &sys->spi_devs[d];
        if(dev->fd < 0) {
            // fill zeros for unopened device
            for(int ch=0; ch<8; ch++) out_raw[d*8 + ch] = 0;
            continue;
        }
        for(int ch=0; ch<8; ch++) {
            int gain = 1;
            if(sys->cfg.devices[d].channel_count > ch) {
                gain = sys->cfg.devices[d].channels[ch].gain;
                if(gain <=0) gain = 1;
            }
            uint8_t adcon = map_gain_bits(gain);
            write_reg(dev, ADS1256_REG_ADCON, adcon);
            uint8_t mux = (ch << 4) | 0x08;
            write_reg(dev, ADS1256_REG_MUX, mux);
            wait_short();
            int32_t raw;
            int r = single_conversion_read(dev, &raw);
            if(r != ADS1256_OK) return r;
            out_raw[d*8 + ch] = raw;
        }
    }
    return dev_count * 8;
}

int ads1256_system_read_frame(ads1256_system_t *sys, float *out_volts) {
    if(!sys || !out_volts) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    int channels_total = sys->cfg.device_count * 8;
    if(sys->hw_initialized) {
        int32_t *raw = (int32_t*)malloc(sizeof(int32_t)*channels_total);
        if(!raw) return ADS1256_ERR_STATE;
        int r = ads1256_system_read_frame_raw(sys, raw);
        if(r < 0) { free(raw); return r; }
        float vscale = (sys->cfg.vref_mv ? sys->cfg.vref_mv : 2500) / 1000.0f / 8388607.0f;
        for(int i=0;i<channels_total;i++) {
            out_volts[i] = raw[i] * vscale;
        }
        free(raw);
        return r;
    } else {
        // synthetic single frame
        for(int ch=0; ch<channels_total; ch++) {
            out_volts[ch] = 0.0f;
        }
        return channels_total;
    }
}

int ads1256_system_stop(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    sys->running = 0;
    return ADS1256_OK;
}

int ads1256_system_read_voltages(ads1256_system_t *sys, float *out, int max_samples) {
    if(!sys || !out || max_samples <= 0) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    int channels_total = sys->cfg.device_count * 8;
    if(sys->hw_initialized) {
        // Minimal blocking read path for first device only (fills all channels with same sample placeholder)
        for(int i=0;i<max_samples;i++) {
            uint8_t dummy_tx[3] = {0xFF,0xFF,0xFF};
            uint8_t rx[3] = {0};
            // Read single 24-bit sample (device running in RDATAC gives channel sequence, we just capture one) – simplified
            if(ads1256_spi_transfer(&sys->spi_devs[0], dummy_tx, rx, 3) != ADS1256_OK) {
                return ADS1256_ERR_SPI_IO;
            }
            int32_t raw = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | (int32_t)rx[2];
            if(raw & 0x800000) raw |= 0xFF000000; // sign extend
            float volts = (float)raw / 8388607.0f * (sys->cfg.vref_mv ? sys->cfg.vref_mv : 2500) / 1000.0f;
            for(int ch=0; ch<channels_total; ch++) {
                out[i*channels_total + ch] = volts; // placeholder replicate
            }
        }
        return max_samples;
    } else {
        // synthetic fallback
        for(int i=0;i<max_samples;i++) {
            float base = (float)((sys->synthetic_counter + i) % 1000) / 1000.0f; // 0..1
            for(int ch=0; ch<channels_total; ch++) {
                float val = (base * 2.0f - 1.0f) + 0.01f * sinf((float)ch + (float)(sys->synthetic_counter+i)/50.0f);
                out[i*channels_total + ch] = val;
            }
        }
        sys->synthetic_counter += (unsigned long)max_samples;
        return max_samples;
    }
}

static uint64_t monotonic_ns() {
#ifdef __linux__
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
#else
    return 0;
#endif
}

static void *acq_thread_func(void *arg) {
    ads1256_system_t *sys = (ads1256_system_t*)arg;
    while(sys->thread_running) {
        ads1256_frame_t frame; memset(&frame,0,sizeof(frame));
        frame.timestamp_ns = monotonic_ns();
        frame.seq = sys->seq_counter++;
        frame.device_count = sys->cfg.device_count;
        frame.channels_per_device = 8;
        // Read one frame (single device prototype) volts into temp array
        int total = sys->cfg.device_count * 8;
        if(total > 32) total = 32;
        float volts[32];
        int r = ads1256_system_read_frame(sys, volts);
        if(r > 0) {
            for(int i=0;i<total;i++) frame.volts[i] = volts[i];
            ads1256_ring_push(&sys->ring, &frame);
            // Metrics update
            sys->metrics.frames_produced++;
            uint64_t now = frame.timestamp_ns;
            if(sys->metrics.last_frame_timestamp_ns != 0) {
                double period = (double)(now - sys->metrics.last_frame_timestamp_ns);
                if(sys->metrics.avg_frame_period_ns == 0) sys->metrics.avg_frame_period_ns = period;
                else sys->metrics.avg_frame_period_ns = 0.9 * sys->metrics.avg_frame_period_ns + 0.1 * period;
            }
            sys->metrics.last_frame_timestamp_ns = frame.timestamp_ns;
            sys->metrics.dropped_frames = sys->ring.dropped;
        }
        // Pacing: approximate frame period = (channels_per_device * 1e9) / sample_rate
        // Using first device's sample_rate as reference (assumes uniform configuration).
        int sr = 0;
        if(sys->cfg.device_count > 0) sr = sys->cfg.devices[0].sample_rate;
        if(sr <= 0) sr = 7500; // fallback
        double frame_period_ns = (8.0 * 1e9) / (double)sr; // sequential scanning cost
        uint64_t now_ns = monotonic_ns();
        uint64_t elapsed = now_ns - frame.timestamp_ns;
        if(elapsed < (uint64_t)frame_period_ns) {
            uint64_t remain = (uint64_t)frame_period_ns - elapsed;
            // sleep remaining time (coarse): cap to 2ms to avoid long oversleeps
            if(remain > 2000000ull) remain = 2000000ull;
            struct timespec ts;
            ts.tv_sec = 0;
            ts.tv_nsec = (long)remain;
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

int ads1256_system_start_thread(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(!sys->ring_ready) {
        if(ads1256_ring_init(&sys->ring, 256) != 0) return ADS1256_ERR_STATE;
        sys->ring_ready = 1;
    }
    if(sys->thread_running) return ADS1256_ERR_ALREADY_INITIALIZED;
    sys->thread_running = 1;
#ifdef __linux__
    if(pthread_create(&sys->thread, NULL, acq_thread_func, sys) != 0) {
        sys->thread_running = 0; return ADS1256_ERR_STATE;
    }
#else
    sys->thread_running = 0; return ADS1256_ERR_STATE;
#endif
    return ADS1256_OK;
}

int ads1256_system_stop_thread(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(!sys->thread_running) return ADS1256_OK;
    sys->thread_running = 0;
#ifdef __linux__
    pthread_join(sys->thread, NULL);
#endif
    return ADS1256_OK;
}

int ads1256_system_pop_frame(ads1256_system_t *sys, ads1256_frame_t *out_frame) {
    if(!sys || !out_frame) return ADS1256_ERR_INVALID_ARG;
    if(!sys->ring_ready) return 0;
    int r = ads1256_ring_pop(&sys->ring, out_frame);
    return r;
}

int ads1256_system_get_dropped(ads1256_system_t *sys, unsigned long long *out_dropped) {
    if(!sys || !out_dropped) return ADS1256_ERR_INVALID_ARG;
    if(!sys->ring_ready) { *out_dropped = 0; return ADS1256_OK; }
    *out_dropped = sys->ring.dropped;
    return ADS1256_OK;
}

int ads1256_system_get_metrics(ads1256_system_t *sys, ads1256_metrics_t *out) {
    if(!sys || !out) return ADS1256_ERR_INVALID_ARG;
    *out = sys->metrics;
    return ADS1256_OK;
}

int ads1256_system_wait_drdy(ads1256_system_t *sys, int device_index, int timeout_ms) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(device_index < 0 || device_index >= sys->cfg.device_count) return ADS1256_ERR_INVALID_ARG;
    // Placeholder: no GPIO integration yet. We approximate wait by sleeping a short fraction of expected sample period.
    int sr = sys->cfg.devices[device_index].sample_rate;
    if(sr <= 0) sr = 7500;
    double sample_period_ns = 1e9 / (double)sr;
    // Sleep min(sample_period/4, 500us) while decrementing timeout.
    int waited_ms = 0;
    int step_us = (int)(sample_period_ns / 4.0 / 1000.0);
    if(step_us <= 0) step_us = 50; // 50us
    if(step_us > 500000) step_us = 500000;
    while(waited_ms < timeout_ms) {
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = step_us * 1000;
        nanosleep(&ts, NULL);
        waited_ms += (step_us / 1000);
        // Without GPIO can't detect readiness; assume ready after first interval.
        return 1;
    }
    return 0; // timeout
}
