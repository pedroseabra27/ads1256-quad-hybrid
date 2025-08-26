#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "include/ads1256_core/ads1256_system.h"
#include "include/ads1256_core/ads1256_spi.h"
#include "include/ads1256_core/ads1256_regs.h"
#include "include/ads1256_core/ads1256_ring.h"
#include "include/ads1256_core/ads1256_frame.h"

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

static int wait_short() {
    struct timespec ts = {0, 500000}; // 0.5ms
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
    if(sys->cfg.device_count > 0) {
        ads1256_device_cfg_t *d0 = &sys->cfg.devices[0];
        if(!sys->hw_initialized) {
            if(ads1256_spi_open(&sys->spi_devs[0], d0->spi_bus, d0->chip_select, 1000000) == ADS1256_OK) {
                // Basic init sequence (blocking, minimal error handling)
                uint8_t cmd;
                cmd = ADS1256_CMD_RESET; ads1256_spi_write(&sys->spi_devs[0], &cmd, 1);
                struct timespec ts = {0, 10000000}; // 10ms
                nanosleep(&ts, NULL);
                cmd = ADS1256_CMD_SDATAC; ads1256_spi_write(&sys->spi_devs[0], &cmd, 1);
                // Compose initial register block (STATUS, MUX, ADCON, DRATE, IO)
                uint8_t regs[5];
                regs[0] = 0x00; // STATUS: ORDER=0, ACAL=0, BUFEN=0
                regs[1] = 0x08; // MUX: AINP=0 AINN=COM initial
                uint8_t gain_bits = map_gain_bits(1);
                regs[2] = (0x00 << 3) | gain_bits; // ADCON: clock out off, gain set
                regs[3] = map_drate(d0->sample_rate);
                regs[4] = 0x00; // IO
                write_regs(&sys->spi_devs[0], ADS1256_REG_STATUS, regs, 5);
                // Self calib (system offset/gain could be done; basic only)
                cmd = ADS1256_CMD_SELFCAL; ads1256_spi_write(&sys->spi_devs[0], &cmd, 1);
                nanosleep(&ts, NULL);
                // Stay in SDATAC mode for manual per-channel conversions (avoid RDATAC during MUX switching)
                sys->hw_initialized = 1;
            }
        }
    }
    sys->running = 1;
    return ADS1256_OK;
}

int ads1256_system_read_frame_raw(ads1256_system_t *sys, int32_t *out_raw) {
    if(!sys || !out_raw) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    if(!sys->hw_initialized) return ADS1256_ERR_STATE;
    // Sweep 8 channels via MUX (single device prototype)
    ads1256_spi_dev_t *dev = &sys->spi_devs[0];
    // Use config from device cfg channels if available
    for(int ch=0; ch<8; ch++) {
        // Gain selection per channel (if configured)
        int gain = 1;
        if(sys->cfg.devices[0].channel_count > ch) {
            gain = sys->cfg.devices[0].channels[ch].gain;
            if(gain <=0) gain = 1;
        }
        uint8_t adcon = map_gain_bits(gain);
        write_reg(dev, ADS1256_REG_ADCON, adcon);
        // MUX: positive channel ch, negative COM (0x08 lower nibble)
        uint8_t mux = (ch << 4) | 0x08;
        write_reg(dev, ADS1256_REG_MUX, mux);
        wait_short();
        int32_t raw;
        int r = single_conversion_read(dev, &raw);
        if(r != ADS1256_OK) return r;
        out_raw[ch] = raw;
    }
    return 8;
}

int ads1256_system_read_frame(ads1256_system_t *sys, float *out_volts) {
    if(!sys || !out_volts) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    int channels_total = sys->cfg.device_count * 8;
    if(sys->hw_initialized) {
        int32_t raw[8];
        int r = ads1256_system_read_frame_raw(sys, raw);
        if(r < 0) return r;
        float vscale = (sys->cfg.vref_mv ? sys->cfg.vref_mv : 2500) / 1000.0f / 8388607.0f;
        for(int ch=0; ch<8; ch++) {
            float volts = raw[ch] * vscale;
            for(int d=0; d<sys->cfg.device_count; d++) { // replicate to all devices for now until multi-dev implemented
                int idx = d*8 + ch;
                out_volts[idx] = volts;
            }
        }
        return channels_total;
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
        float volts[32];
        int r = ads1256_system_read_frame(sys, volts);
        if(r > 0) {
            int total = sys->cfg.device_count * 8;
            if(total > 32) total = 32;
            for(int i=0;i<total;i++) frame.volts[i] = volts[i];
            ads1256_ring_push(&sys->ring, &frame);
        }
        // Simple pacing: remove once real DRDY event-driven logic added
        struct timespec ts = {0, 1000000}; // 1ms
        nanosleep(&ts, NULL);
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
