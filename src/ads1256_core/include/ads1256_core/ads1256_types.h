#ifndef ADS1256_TYPES_H
#define ADS1256_TYPES_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int enabled;
    int gain;           // 1,2,4,8,16,32,64
    int input_mode;     // 0=single,1=diff
    char label[24];
} ads1256_channel_cfg_t;

typedef struct {
    int device_id;      // 0..3
    int spi_bus;        // 0 or 1
    int chip_select;    // 0 or 1
    int sample_rate;    // SPS
    int drdy_chip;      // gpiochip index (Linux) or -1 if unused
    int drdy_line;      // line offset within chip or -1 if unused
    size_t buffer_size; // bytes allocated for ring
    int auto_calibration;
    int channel_count;  // actual channels configured
    ads1256_channel_cfg_t channels[8];
} ads1256_device_cfg_t;

typedef struct {
    int sync_mode;      // 0=software,1=hardware
    int use_dma;
    int real_time;
    int rt_priority;
    int device_count;
    ads1256_device_cfg_t devices[4];
    int vref_mv;        // reference voltage in millivolts (all devices for now)
} ads1256_system_cfg_t;

#endif // ADS1256_TYPES_H
