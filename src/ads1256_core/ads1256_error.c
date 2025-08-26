#include "include/ads1256_core/ads1256_error.h"

const char *ads1256_core_strerror(int code) {
    switch(code) {
        case ADS1256_OK: return "OK";
        case ADS1256_ERR_NOT_INITIALIZED: return "Not initialized";
        case ADS1256_ERR_ALREADY_INITIALIZED: return "Already initialized";
        case ADS1256_ERR_INVALID_ARG: return "Invalid argument";
        case ADS1256_ERR_SPI_IO: return "SPI I/O error";
        case ADS1256_ERR_STATE: return "Invalid state";
        default: return "Unknown ads1256 error";
    }
}
