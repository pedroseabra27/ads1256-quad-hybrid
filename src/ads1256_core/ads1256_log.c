#include "include/ads1256_core/ads1256_log.h"
#include <time.h>
#include <string.h>

static ads1256_log_sink_t g_sink = NULL;
static void *g_sink_user = NULL;
static int g_level = ADS1256_LOG_INFO;

void ads1256_log_set_sink(ads1256_log_sink_t sink, void *user) {
    g_sink = sink; g_sink_user = user;
}

void ads1256_log_set_level(int level) { g_level = level; }
int ads1256_log_get_level(void) { return g_level; }

static void default_sink(int level, const char *message, void *user) {
    (void)user;
    const char *lvl = "INFO";
    if(level >= ADS1256_LOG_ERROR) lvl = "ERROR";
    else if(level >= ADS1256_LOG_WARN) lvl = "WARN";
    else if(level >= ADS1256_LOG_INFO) lvl = "INFO";
    else lvl = "DEBUG";
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    fprintf(stderr, "[%ld.%03ld] %s %s\n", (long)ts.tv_sec, ts.tv_nsec/1000000, lvl, message);
}

void ads1256_logf(int level, const char *fmt, ...) {
    if(level < g_level) return;
    char buf[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if(g_sink) g_sink(level, buf, g_sink_user); else default_sink(level, buf, NULL);
}
