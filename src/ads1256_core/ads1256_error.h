#ifndef ADS1256_ERROR_H
#define ADS1256_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

// Basic error codes (expand later)
#define ADS1256_OK                       0
#define ADS1256_ERR_NOT_INITIALIZED     -1
#define ADS1256_ERR_ALREADY_INITIALIZED -2
#define ADS1256_ERR_INVALID_ARG         -3
#define ADS1256_ERR_SPI_IO              -10
#define ADS1256_ERR_STATE               -20

const char *ads1256_core_strerror(int code);

#ifdef __cplusplus
}
#endif

#endif // ADS1256_ERROR_H
