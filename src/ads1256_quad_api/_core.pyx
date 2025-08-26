# cython: language_level=3, boundscheck=False, wraparound=False

from cpython.mem cimport PyMem_Malloc, PyMem_Free
import numpy as np
cimport numpy as np

cimport ads1256_quad_api._core as ccore

cdef class CoreSystem:
    cdef ccore.ads1256_system_t *sys
    cdef bint started
    cdef bint threaded

    def __cinit__(self):
        self.sys = NULL
        self.started = False
        self.threaded = False

    def create_default(self, int vref_mv=2500):
        cdef ccore.ads1256_system_cfg_t cfg
        cfg.sync_mode = 1
        cfg.use_dma = 0
        cfg.real_time = 0
        cfg.rt_priority = 80
        cfg.device_count = 4
        cfg.vref_mv = vref_mv
        for d in range(4):
            cfg.devices[d].device_id = d
            cfg.devices[d].spi_bus = d // 2
            cfg.devices[d].chip_select = d % 2
            cfg.devices[d].sample_rate = 30000
            cfg.devices[d].buffer_size = 65536
            cfg.devices[d].auto_calibration = 1
            cfg.devices[d].channel_count = 8
            for ch in range(8):
                cfg.devices[d].channels[ch].enabled = 1
                cfg.devices[d].channels[ch].gain = 1
                cfg.devices[d].channels[ch].input_mode = 0
                cfg.devices[d].channels[ch].label[0] = 0
        cdef ccore.ads1256_system_t *tmp
        if ccore.ads1256_system_create(&cfg, &tmp) != 0:
            raise RuntimeError("Failed to create system")
        self.sys = tmp
        return self

    def start(self):
        if self.sys is NULL: raise RuntimeError("Not created")
        if ccore.ads1256_system_start(self.sys) != 0:
            raise RuntimeError("Failed to start system")
        self.started = True

    def start_thread(self):
        if not self.started: self.start()
        if ccore.ads1256_system_start_thread(self.sys) != 0:
            raise RuntimeError("Failed to start acquisition thread")
        self.threaded = True

    def stop_thread(self):
        if self.threaded:
            ccore.ads1256_system_stop_thread(self.sys)
            self.threaded = False

    def stop(self):
        if self.started:
            ccore.ads1256_system_stop(self.sys)
            self.started = False

    def __dealloc__(self):
        if self.sys is not NULL:
            if self.threaded:
                ccore.ads1256_system_stop_thread(self.sys)
            if self.started:
                ccore.ads1256_system_stop(self.sys)
            ccore.ads1256_system_destroy(self.sys)
            self.sys = NULL

    def pop_frame(self):
        cdef ccore.ads1256_frame_t frame
        r = ccore.ads1256_system_pop_frame(self.sys, &frame)
        if r == 1:
            total_channels = frame.device_count * frame.channels_per_device
            if total_channels > 32:
                total_channels = 32
            arr = np.empty(total_channels, dtype=np.float32)
            for i in range(total_channels):
                arr[i] = frame.volts[i]
            return {
                "timestamp_ns": frame.timestamp_ns,
                "seq": frame.seq,
                "device_count": frame.device_count,
                "channels_per_device": frame.channels_per_device,
                "volts": arr,
            }
        elif r == 0:
            return None
        else:
            raise RuntimeError("pop_frame error")

    def read_frame_blocking(self):
        cdef int dev_channels = 4 * 8
        cdef float[::1] buf = np.empty(dev_channels, dtype=np.float32)
        r = ccore.ads1256_system_read_frame(self.sys, &buf[0])
        if r < 0:
            raise RuntimeError("read_frame failed")
        return np.asarray(buf)[:r].copy()

    def get_dropped(self):
        cdef unsigned long long d
        if ccore.ads1256_system_get_dropped(self.sys, &d) != 0:
            raise RuntimeError("metric error")
        return d

    def get_metrics(self):
        """Return a dict with current acquisition metrics plus derived throughput."""
        if self.sys == NULL:
            return None
        cdef ads1256_metrics_t* m = ads1256_system_get_metrics(self.sys)
        if m == NULL:
            return None
        d = {
            "frames_produced": m.frames_produced,
            "dropped_frames": m.dropped_frames,
            "avg_frame_period_ns": m.avg_frame_period_ns,
        }
        if m.avg_frame_period_ns > 0:
            d["throughput_fps"] = 1e9 / m.avg_frame_period_ns
        else:
            d["throughput_fps"] = 0.0
        return d
