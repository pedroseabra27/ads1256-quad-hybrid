#include "include/ads1256_core/ads1256_spi.h"
#include "include/ads1256_core/ads1256_error.h"

#ifdef __linux__
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

int ads1256_spi_open(ads1256_spi_dev_t *dev, int bus, int cs, int speed_hz) {
    if(!dev) return ADS1256_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->cs = cs;
    dev->speed_hz = speed_hz;
    dev->mode = SPI_MODE_1; // ADS1256 typically mode 1
    char path[64];
    snprintf(path, sizeof(path), "/dev/spidev%d.%d", bus, cs);
    dev->fd = open(path, O_RDWR);
    if(dev->fd < 0) return ADS1256_ERR_SPI_IO;
    uint8_t bits = 8;
    if(ioctl(dev->fd, SPI_IOC_WR_MODE, &dev->mode) < 0) return ADS1256_ERR_SPI_IO;
    if(ioctl(dev->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return ADS1256_ERR_SPI_IO;
    if(ioctl(dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->speed_hz) < 0) return ADS1256_ERR_SPI_IO;
    return ADS1256_OK;
}

int ads1256_spi_close(ads1256_spi_dev_t *dev) {
    if(!dev) return ADS1256_ERR_INVALID_ARG;
    if(dev->fd >= 0) close(dev->fd);
    dev->fd = -1;
    return ADS1256_OK;
}

int ads1256_spi_transfer(ads1256_spi_dev_t *dev, const uint8_t *tx, uint8_t *rx, int len) {
    if(!dev || dev->fd < 0 || len <=0) return ADS1256_ERR_INVALID_ARG;
    struct spi_ioc_transfer tr = {0};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = dev->speed_hz;
    tr.bits_per_word = 8;
    tr.delay_usecs = 0;
    int ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &tr);
    if(ret < 1) return ADS1256_ERR_SPI_IO;
    return ADS1256_OK;
}

int ads1256_spi_write(ads1256_spi_dev_t *dev, const uint8_t *tx, int len) {
    return ads1256_spi_transfer(dev, tx, NULL, len);
}

#else

int ads1256_spi_open(ads1256_spi_dev_t *dev, int bus, int cs, int speed_hz) {(void)dev;(void)bus;(void)cs;(void)speed_hz; return ADS1256_ERR_STATE;}
int ads1256_spi_close(ads1256_spi_dev_t *dev) {(void)dev; return ADS1256_ERR_STATE;}
int ads1256_spi_transfer(ads1256_spi_dev_t *dev, const uint8_t *tx, uint8_t *rx, int len){(void)dev;(void)tx;(void)rx;(void)len;return ADS1256_ERR_STATE;}
int ads1256_spi_write(ads1256_spi_dev_t *dev, const uint8_t *tx, int len){(void)dev;(void)tx;(void)len;return ADS1256_ERR_STATE;}

#endif
