#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <math.h>
#include "include/ads1256_core/ads1256_log.h"
#include <time.h>

#include "include/ads1256_core/ads1256_system.h"
#include "include/ads1256_core/ads1256_spi.h"
#include "include/ads1256_core/ads1256_regs.h"
#include "include/ads1256_core/ads1256_ring.h"
#include "include/ads1256_core/ads1256_frame.h"
#include "include/ads1256_core/ads1256_gpio.h"
#ifdef __linux__
#include <pthread.h>
#endif

struct ads1256_system_s {
    ads1256_system_cfg_t cfg;
    int running;
    unsigned long synthetic_counter;
    ads1256_spi_dev_t spi_devs[4]; // future multi devices
    int drdy_handles[4];
    int hw_initialized;
    ads1256_ring_t ring;
    int ring_ready;
    int thread_running;
    pthread_t thread;
    uint64_t seq_counter;
    ads1256_metrics_t metrics;
    int32_t *raw_buf;
    size_t raw_buf_len;
    int use_rdatac;
    // Bus workers (up to 2 SPI buses)
    int bus_worker_enabled[2];
    pthread_t bus_threads[2];
    int bus_thread_running[2];
    // Synchronization
    pthread_mutex_t bus_mutex[2];
    pthread_cond_t  bus_cond[2];
    pthread_cond_t  bus_done_cond[2];
    int bus_request[2]; // incremented to signal work
    int bus_done[2];    // increments when work finished
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
static int single_conversion_read(ads1256_system_t *sys, int device_index, ads1256_spi_dev_t *dev, int32_t *out_raw) {
    uint8_t cmd;
    cmd = ADS1256_CMD_SYNC; ads1256_spi_write(dev, &cmd, 1);
    cmd = ADS1256_CMD_WAKEUP; ads1256_spi_write(dev, &cmd, 1);
    // small settle time
    if(device_index >=0 && device_index < sys->cfg.device_count && sys->drdy_handles[device_index] >= 0) {
        ads1256_system_wait_drdy(sys, device_index, 5); // short timeout
    } else {
        wait_short();
    }
    cmd = ADS1256_CMD_RDATA; ads1256_spi_write(dev, &cmd, 1);
    // Wait t6 (datasheet) ~50 * 1/CLKIN; reuse wait_short minimal for now
    if(device_index >=0 && device_index < sys->cfg.device_count && sys->drdy_handles[device_index] >= 0) {
        ads1256_system_wait_drdy(sys, device_index, 5);
    } else {
        wait_short();
    }
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
    for(int i=0;i<4;i++) s->drdy_handles[i] = -1;
    s->metrics.device_count = s->cfg.device_count;
    s->metrics.channels_total = s->cfg.device_count * 8;
    s->raw_buf = NULL;
    s->raw_buf_len = 0;
    s->use_rdatac = 1;
    for(int b=0;b<2;b++) {
        s->bus_worker_enabled[b] = 0;
        s->bus_thread_running[b] = 0;
        s->bus_request[b] = 0;
        s->bus_done[b] = 0;
        pthread_mutex_init(&s->bus_mutex[b], NULL);
        pthread_cond_init(&s->bus_cond[b], NULL);
        pthread_cond_init(&s->bus_done_cond[b], NULL);
    }
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
    if(sys->raw_buf) free(sys->raw_buf);
    for(int b=0;b<2;b++) {
        pthread_mutex_destroy(&sys->bus_mutex[b]);
        pthread_cond_destroy(&sys->bus_cond[b]);
        pthread_cond_destroy(&sys->bus_done_cond[b]);
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
            if(dcfg->drdy_chip >= 0 && dcfg->drdy_line >= 0) {
                int h = ads1256_gpio_open_drdy(dcfg->drdy_chip, dcfg->drdy_line);
                sys->drdy_handles[d] = h;
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
            if(sys->use_rdatac) {
                cmd = ADS1256_CMD_RDATAC; ads1256_spi_write(&sys->spi_devs[d], &cmd, 1);
            }
        }
        sys->hw_initialized = 1; // mark overall (fine-grain status not tracked yet)
    }
    size_t need = (size_t)sys->cfg.device_count * 8;
    if(need > 0 && (sys->raw_buf == NULL || sys->raw_buf_len < need)) {
        if(sys->raw_buf) free(sys->raw_buf);
        sys->raw_buf = (int32_t*)malloc(sizeof(int32_t)*need);
        if(!sys->raw_buf) return ADS1256_ERR_STATE;
        sys->raw_buf_len = need;
    }
    sys->running = 1;
    ADS1256_LOGI("system start (devices=%d)", sys->cfg.device_count);
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
            if(sys->use_rdatac) {
                // settle, then discard first, then read real sample
                if(sys->drdy_handles[d] >= 0) ads1256_system_wait_drdy(sys, d, 10); else wait_short();
                int32_t discard;
                read_sample24(dev, &discard);
                if(sys->drdy_handles[d] >= 0) ads1256_system_wait_drdy(sys, d, 10); else wait_short();
                int32_t raw;
                if(read_sample24(dev, &raw) != ADS1256_OK) return ADS1256_ERR_SPI_IO;
                out_raw[d*8 + ch] = raw;
            } else {
                wait_short();
                int32_t raw;
                int r = single_conversion_read(sys, d, dev, &raw);
                if(r != ADS1256_OK) return r;
                out_raw[d*8 + ch] = raw;
            }
        }
    }
    return dev_count * 8;
}

int ads1256_system_read_frame(ads1256_system_t *sys, float *out_volts) {
    if(!sys || !out_volts) return ADS1256_ERR_INVALID_ARG;
    if(!sys->running) return ADS1256_ERR_STATE;
    int channels_total = sys->cfg.device_count * 8;
    if(sys->hw_initialized) {
        if(sys->raw_buf_len < (size_t)channels_total) return ADS1256_ERR_STATE;
        int32_t *raw = sys->raw_buf;
        int r = ads1256_system_read_frame_raw(sys, raw);
        if(r < 0) { return r; }
        float vref = (sys->cfg.vref_mv ? sys->cfg.vref_mv : 2500) / 1000.0f; // volts
        for(int d=0; d<sys->cfg.device_count; d++) {
            for(int ch=0; ch<8; ch++) {
                int idx = d*8 + ch;
                int gain = 1;
                if(sys->cfg.devices[d].channel_count > ch) {
                    int g = sys->cfg.devices[d].channels[ch].gain;
                    if(g > 0) gain = g;
                }
                float scale = vref / (8388607.0f * (float)gain);
                out_volts[idx] = raw[idx] * scale;
            }
        }
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
    ADS1256_LOGI("system stop");
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
    ADS1256_LOGI("acquisition thread started");
    while(sys->thread_running) {
        ads1256_frame_t frame; memset(&frame,0,sizeof(frame));
        uint64_t t_start = monotonic_ns();
        frame.timestamp_ns = t_start;
        frame.seq = sys->seq_counter++;
        frame.device_count = sys->cfg.device_count;
        frame.channels_per_device = 8;
        int total_channels = sys->cfg.device_count * 8;
        if(total_channels > 32) total_channels = 32;
        int r = 0;
        if(sys->cfg.device_count > 0 && sys->hw_initialized) {
            // Parallel path if bus workers enabled
            int parallel = (sys->bus_worker_enabled[0] || sys->bus_worker_enabled[1]);
            if(parallel) {
                if(sys->raw_buf_len < (size_t)(sys->cfg.device_count * 8)) {
                    // raw buffer missing allocation unexpectedly
                    parallel = 0;
                }
            }
            if(parallel) {
                bus_request_work(sys); // signal each bus
                bus_wait_all(sys);     // wait completion
                // Convert raw buffer to volts
                float vref = (sys->cfg.vref_mv ? sys->cfg.vref_mv : 2500) / 1000.0f;
                for(int d=0; d<sys->cfg.device_count; d++) {
                    for(int ch=0; ch<8; ch++) {
                        int idx = d*8 + ch;
                        int gain = 1;
                        if(sys->cfg.devices[d].channel_count > ch) {
                            int g = sys->cfg.devices[d].channels[ch].gain;
                            if(g > 0) gain = g;
                        }
                        float scale = vref / (8388607.0f * (float)gain);
                        float val = sys->raw_buf[idx] * scale;
                        if(idx < 32) frame.volts[idx] = val;
                    }
                }
                r = sys->cfg.device_count * 8;
            } else {
                float volts[32];
                r = ads1256_system_read_frame(sys, volts);
                if(r > 0) {
                    for(int i=0;i<total_channels;i++) frame.volts[i] = volts[i];
                }
            }
        }
        if(r > 0) {
            ads1256_ring_push(&sys->ring, &frame);
            // Metrics update
            sys->metrics.frames_produced++;
            uint64_t t_end = monotonic_ns();
            uint64_t acq_ns = t_end - t_start;
            sys->metrics.last_frame_acq_ns = acq_ns;
            if(sys->metrics.avg_frame_acq_ns == 0) sys->metrics.avg_frame_acq_ns = (double)acq_ns;
            else sys->metrics.avg_frame_acq_ns = 0.9 * sys->metrics.avg_frame_acq_ns + 0.1 * (double)acq_ns;
            uint64_t now = frame.timestamp_ns;
            if(sys->metrics.last_frame_timestamp_ns != 0) {
                double period = (double)(now - sys->metrics.last_frame_timestamp_ns);
                if(sys->metrics.avg_frame_period_ns == 0) {
                    sys->metrics.avg_frame_period_ns = period;
                    sys->metrics.rms_frame_jitter_ns = 0.0;
                    sys->metrics.max_frame_jitter_ns = 0.0;
                } else {
                    double prev_avg = sys->metrics.avg_frame_period_ns;
                    double new_avg = 0.9 * prev_avg + 0.1 * period;
                    double jitter = period - new_avg; // centered on updated avg (approx)
                    // Update RMS jitter with exponential window similar to avg
                    double prev_rms2 = sys->metrics.rms_frame_jitter_ns * sys->metrics.rms_frame_jitter_ns;
                    double new_rms2 = 0.95 * prev_rms2 + 0.05 * (jitter * jitter);
                    double new_rms = 0.0;
                    if(new_rms2 > 0) {
                        // simple sqrt; could approximate to avoid math overhead
                        new_rms = sqrt(new_rms2);
                    }
                    sys->metrics.avg_frame_period_ns = new_avg;
                    sys->metrics.rms_frame_jitter_ns = new_rms;
                    double abs_jitter = jitter < 0 ? -jitter : jitter;
                    if(abs_jitter > sys->metrics.max_frame_jitter_ns) sys->metrics.max_frame_jitter_ns = abs_jitter;
                    // Histogram bucket (ns)
                    uint64_t aj = (uint64_t)(abs_jitter < 0 ? 0 : abs_jitter);
                    int b = 7;
                    if(aj < 1000) b = 0;
                    else if(aj < 2000) b = 1;
                    else if(aj < 5000) b = 2;
                    else if(aj < 10000) b = 3;
                    else if(aj < 20000) b = 4;
                    else if(aj < 50000) b = 5;
                    else if(aj < 100000) b = 6;
                    sys->metrics.jitter_hist[b]++;
                }
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
    ADS1256_LOGI("acquisition thread exiting");
    return NULL;
}

typedef struct {
    ads1256_system_t *sys;
    int bus;
} bus_ctx_t;

static void *bus_worker(void *arg) {
    bus_ctx_t *ctx = (bus_ctx_t*)arg;
    ads1256_system_t *sys = ctx->sys;
    int bus = ctx->bus;
    free(ctx);
    while(sys->bus_thread_running[bus]) {
        pthread_mutex_lock(&sys->bus_mutex[bus]);
        int initial_req = sys->bus_request[bus];
        while(sys->bus_thread_running[bus] && initial_req == sys->bus_request[bus]) {
            pthread_cond_wait(&sys->bus_cond[bus], &sys->bus_mutex[bus]);
        }
        pthread_mutex_unlock(&sys->bus_mutex[bus]);
        if(!sys->bus_thread_running[bus]) break;
        // Process devices on this bus: perform raw reads into shared raw_buf
        if(sys->hw_initialized && sys->raw_buf) {
            for(int d=0; d<sys->cfg.device_count; d++) {
                ads1256_device_cfg_t *dcfg = &sys->cfg.devices[d];
                if(dcfg->spi_bus != bus) continue;
                ads1256_spi_dev_t *dev = &sys->spi_devs[d];
                if(dev->fd < 0) {
                    for(int ch=0; ch<8; ch++) sys->raw_buf[d*8 + ch] = 0;
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
                    if(sys->use_rdatac) {
                        if(sys->drdy_handles[d] >= 0) ads1256_system_wait_drdy(sys, d, 10); else wait_short();
                        int32_t discard; read_sample24(dev, &discard);
                        if(sys->drdy_handles[d] >= 0) ads1256_system_wait_drdy(sys, d, 10); else wait_short();
                        int32_t raw; read_sample24(dev, &raw);
                        sys->raw_buf[d*8 + ch] = raw;
                    } else {
                        wait_short();
                        int32_t raw;
                        if(single_conversion_read(sys, d, dev, &raw) == ADS1256_OK) {
                            sys->raw_buf[d*8 + ch] = raw;
                        } else {
                            sys->raw_buf[d*8 + ch] = 0;
                        }
                    }
                }
            }
        }
        pthread_mutex_lock(&sys->bus_mutex[bus]);
        sys->bus_done[bus]++;
        pthread_cond_broadcast(&sys->bus_done_cond[bus]);
        pthread_mutex_unlock(&sys->bus_mutex[bus]);
    }
    return NULL;
}

static void bus_request_work(ads1256_system_t *sys) {
    for(int bus=0; bus<2; bus++) {
        if(!sys->bus_worker_enabled[bus]) continue;
        pthread_mutex_lock(&sys->bus_mutex[bus]);
        sys->bus_request[bus]++;
        pthread_cond_signal(&sys->bus_cond[bus]);
        pthread_mutex_unlock(&sys->bus_mutex[bus]);
    }
}

static void bus_wait_all(ads1256_system_t *sys) {
    for(int bus=0; bus<2; bus++) {
        if(!sys->bus_worker_enabled[bus]) continue;
        pthread_mutex_lock(&sys->bus_mutex[bus]);
        int target = sys->bus_request[bus];
        while(sys->bus_done[bus] < target) {
            pthread_cond_wait(&sys->bus_done_cond[bus], &sys->bus_mutex[bus]);
        }
        pthread_mutex_unlock(&sys->bus_mutex[bus]);
    }
}

int ads1256_system_start_thread(ads1256_system_t *sys) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(!sys->ring_ready) {
        if(ads1256_ring_init(&sys->ring, 256) != 0) return ADS1256_ERR_STATE;
        sys->ring_ready = 1;
    for(int i=0;i<4;i++) if(sys->drdy_handles[i] >= 0) ads1256_gpio_close(sys->drdy_handles[i]);
    }
    if(sys->thread_running) return ADS1256_ERR_ALREADY_INITIALIZED;
    sys->thread_running = 1;
#ifdef __linux__
    // Determine which buses are used
    int used_bus[2] = {0,0};
    for(int d=0; d<sys->cfg.device_count; d++) {
        int b = sys->cfg.devices[d].spi_bus;
        if(b>=0 && b<2) used_bus[b] = 1;
    }
    for(int b=0;b<2;b++) {
        if(used_bus[b]) {
            sys->bus_worker_enabled[b] = 1;
            sys->bus_thread_running[b] = 1;
            bus_ctx_t *ctx = (bus_ctx_t*)malloc(sizeof(bus_ctx_t));
            ctx->sys = sys; ctx->bus = b;
            if(pthread_create(&sys->bus_threads[b], NULL, bus_worker, ctx) != 0) {
                sys->bus_thread_running[b] = 0;
                sys->bus_worker_enabled[b] = 0;
            }
        }
    }
#endif
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
    // stop bus workers
    for(int b=0;b<2;b++) {
        if(sys->bus_thread_running[b]) {
            pthread_mutex_lock(&sys->bus_mutex[b]);
            sys->bus_thread_running[b] = 0;
            pthread_cond_broadcast(&sys->bus_cond[b]);
            pthread_mutex_unlock(&sys->bus_mutex[b]);
            pthread_join(sys->bus_threads[b], NULL);
        }
    }
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

int ads1256_system_pop_frame_ref(ads1256_system_t *sys, const ads1256_frame_t **out_frame) {
    if(!sys || !out_frame) return ADS1256_ERR_INVALID_ARG;
    if(!sys->ring_ready) return 0;
    ads1256_ring_t *r = &sys->ring;
    if(r->read_idx == r->write_idx) return 0; // empty
    *out_frame = &r->frames[r->read_idx];
    r->read_idx = (r->read_idx + 1) % r->capacity;
    return 1;
}

int ads1256_system_get_metrics(ads1256_system_t *sys, ads1256_metrics_t *out) {
    if(!sys || !out) return ADS1256_ERR_INVALID_ARG;
    *out = sys->metrics;
    return ADS1256_OK;
}

int ads1256_system_wait_drdy(ads1256_system_t *sys, int device_index, int timeout_ms) {
    if(!sys) return ADS1256_ERR_INVALID_ARG;
    if(device_index < 0 || device_index >= sys->cfg.device_count) return ADS1256_ERR_INVALID_ARG;
    sys->metrics.drdy_waits++;
    uint64_t start_ns = monotonic_ns();
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
        sys->metrics.last_drdy_wait_ns = monotonic_ns() - start_ns;
        if(sys->metrics.avg_drdy_wait_ns == 0)
            sys->metrics.avg_drdy_wait_ns = (double)sys->metrics.last_drdy_wait_ns;
        else
            sys->metrics.avg_drdy_wait_ns = 0.9 * sys->metrics.avg_drdy_wait_ns + 0.1 * (double)sys->metrics.last_drdy_wait_ns;
        return 1;
    }
    sys->metrics.drdy_timeouts++;
    sys->metrics.last_drdy_wait_ns = monotonic_ns() - start_ns;
    return 0; // timeout
}
