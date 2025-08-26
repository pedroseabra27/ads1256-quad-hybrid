#ifndef ADS1256_LOG_H
#define ADS1256_LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Log levels
#define ADS1256_LOG_DEBUG 10
#define ADS1256_LOG_INFO  20
#define ADS1256_LOG_WARN  30
#define ADS1256_LOG_ERROR 40

typedef void (*ads1256_log_sink_t)(int level, const char *message, void *user);

void ads1256_log_set_sink(ads1256_log_sink_t sink, void *user);
void ads1256_log_set_level(int level);
int  ads1256_log_get_level(void);

void ads1256_logf(int level, const char *fmt, ...);

// Convenience macros (compile-time guard by level)
#ifndef ADS1256_DISABLE_LOGGING
#define ADS1256_LOGD(fmt, ...) do { if(ads1256_log_get_level() <= ADS1256_LOG_DEBUG) ads1256_logf(ADS1256_LOG_DEBUG, fmt, ##__VA_ARGS__); } while(0)
#define ADS1256_LOGI(fmt, ...) do { if(ads1256_log_get_level() <= ADS1256_LOG_INFO)  ads1256_logf(ADS1256_LOG_INFO,  fmt, ##__VA_ARGS__); } while(0)
#define ADS1256_LOGW(fmt, ...) do { if(ads1256_log_get_level() <= ADS1256_LOG_WARN)  ads1256_logf(ADS1256_LOG_WARN,  fmt, ##__VA_ARGS__); } while(0)
#define ADS1256_LOGE(fmt, ...) do { if(ads1256_log_get_level() <= ADS1256_LOG_ERROR) ads1256_logf(ADS1256_LOG_ERROR, fmt, ##__VA_ARGS__); } while(0)
#else
#define ADS1256_LOGD(fmt, ...)
#define ADS1256_LOGI(fmt, ...)
#define ADS1256_LOGW(fmt, ...)
#define ADS1256_LOGE(fmt, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif // ADS1256_LOG_H