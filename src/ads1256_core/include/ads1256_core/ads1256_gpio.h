// Lightweight GPIO (DRDY) abstraction with optional libgpiod backend.
// Build with -DADS1256_HAVE_GPIOD and link libgpiod to enable real edge waits.

#ifndef ADS1256_GPIO_H
#define ADS1256_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

// Open a DRDY line; returns handle >=0 or <0 on error.
int ads1256_gpio_open_drdy(int chip_index, int line_offset);

// Wait for falling edge / ready; timeout_ms <=0 for non-blocking poll.
// Returns 1 ready, 0 timeout, <0 error.
int ads1256_gpio_wait_drdy(int handle, int timeout_ms);

// Close line.
void ads1256_gpio_close(int handle);

#ifdef __cplusplus
}
#endif

#endif // ADS1256_GPIO_H