#include "include/ads1256_core/ads1256_gpio.h"
#include <time.h>
#ifdef ADS1256_HAVE_GPIOD
#include <gpiod.h>
#endif

typedef struct {
    int in_use;
#ifdef ADS1256_HAVE_GPIOD
    struct gpiod_line *line;
    struct gpiod_chip *chip;
#endif
} drdy_slot_t;

#define MAX_DRDY 8
static drdy_slot_t slots[MAX_DRDY];

static int alloc_slot() {
    for(int i=0;i<MAX_DRDY;i++) if(!slots[i].in_use) { slots[i].in_use=1; return i; }
    return -1;
}

int ads1256_gpio_open_drdy(int chip_index, int line_offset) {
    if(chip_index < 0 || line_offset < 0) return -1;
    int idx = alloc_slot();
    if(idx < 0) return -2;
#ifdef ADS1256_HAVE_GPIOD
    char chip_name[32];
    snprintf(chip_name,sizeof(chip_name),"/dev/gpiochip%d", chip_index);
    struct gpiod_chip *chip = gpiod_chip_open(chip_name);
    if(!chip) { slots[idx].in_use=0; return -3; }
    struct gpiod_line *line = gpiod_chip_get_line(chip, line_offset);
    if(!line) { gpiod_chip_close(chip); slots[idx].in_use=0; return -4; }
    if(gpiod_line_request_falling_edge_events(line, "ads1256") != 0) {
        gpiod_chip_close(chip); slots[idx].in_use=0; return -5; }
    slots[idx].chip = chip; slots[idx].line = line;
#endif
    return idx;
}

int ads1256_gpio_wait_drdy(int handle, int timeout_ms) {
    if(handle < 0 || handle >= MAX_DRDY || !slots[handle].in_use) return -1;
#ifdef ADS1256_HAVE_GPIOD
    struct timespec ts;
    if(timeout_ms > 0) { ts.tv_sec = timeout_ms/1000; ts.tv_nsec = (timeout_ms%1000)*1000000; }
    int r = gpiod_line_event_wait(slots[handle].line, timeout_ms>0? &ts: NULL);
    if(r < 0) return -2; // error
    if(r == 0) return 0; // timeout
    struct gpiod_line_event ev;
    if(gpiod_line_event_read(slots[handle].line, &ev) != 0) return -3;
    return 1;
#else
    // Fallback: sleep a tiny time and assume ready
    struct timespec tiny = {0, 20000}; // 20us
    nanosleep(&tiny, NULL);
    return 1;
#endif
}

void ads1256_gpio_close(int handle) {
    if(handle < 0 || handle >= MAX_DRDY || !slots[handle].in_use) return;
#ifdef ADS1256_HAVE_GPIOD
    if(slots[handle].line) gpiod_line_release(slots[handle].line);
    if(slots[handle].chip) gpiod_chip_close(slots[handle].chip);
#endif
    slots[handle].in_use = 0;
}
