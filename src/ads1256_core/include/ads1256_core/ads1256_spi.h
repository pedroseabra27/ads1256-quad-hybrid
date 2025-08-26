#ifndef ADS1256_SPI_H
#define ADS1256_SPI_H

#include <stdint.h>

typedef struct {
    int fd;
    int bus;
    int cs;
    int speed_hz;
    int mode;  // SPI mode bits
} ads1256_spi_dev_t;

int ads1256_spi_open(ads1256_spi_dev_t *dev, int bus, int cs, int speed_hz);
int ads1256_spi_close(ads1256_spi_dev_t *dev);
int ads1256_spi_transfer(ads1256_spi_dev_t *dev, const uint8_t *tx, uint8_t *rx, int len);
int ads1256_spi_write(ads1256_spi_dev_t *dev, const uint8_t *tx, int len);

#endif // ADS1256_SPI_H
