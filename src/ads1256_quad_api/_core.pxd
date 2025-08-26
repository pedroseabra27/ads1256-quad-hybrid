cdef extern from "ads1256_core/ads1256_system.h":
    ctypedef struct ads1256_system_s: pass
    ctypedef ads1256_system_s ads1256_system_t
    cdef int ads1256_system_create(const ads1256_system_cfg_t *cfg, ads1256_system_t **out)
    cdef int ads1256_system_destroy(ads1256_system_t *sys)
    cdef int ads1256_system_start(ads1256_system_t *sys)
    cdef int ads1256_system_stop(ads1256_system_t *sys)
    cdef int ads1256_system_start_thread(ads1256_system_t *sys)
    cdef int ads1256_system_stop_thread(ads1256_system_t *sys)
    cdef int ads1256_system_pop_frame(ads1256_system_t *sys, ads1256_frame_t *out_frame)
    cdef int ads1256_system_get_dropped(ads1256_system_t *sys, unsigned long long *out_dropped)
    cdef int ads1256_system_read_frame(ads1256_system_t *sys, float *out_volts)
    cdef int ads1256_system_read_frame_raw(ads1256_system_t *sys, int *out_raw)

cdef extern from "ads1256_core/ads1256_types.h":
    ctypedef struct ads1256_channel_cfg_t:
        int enabled
        int gain
        int input_mode
        char label[24]
    ctypedef struct ads1256_device_cfg_t:
        int device_id
        int spi_bus
        int chip_select
        int sample_rate
        size_t buffer_size
        int auto_calibration
        int channel_count
        ads1256_channel_cfg_t channels[8]
    ctypedef struct ads1256_system_cfg_t:
        int sync_mode
        int use_dma
        int real_time
        int rt_priority
        int device_count
        ads1256_device_cfg_t devices[4]
        int vref_mv

cdef extern from "ads1256_core/ads1256_frame.h":
    ctypedef struct ads1256_frame_t:
        unsigned long long timestamp_ns
        unsigned long long seq
        int device_count
        int channels_per_device
        float volts[32]
