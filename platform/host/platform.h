#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_TIMERS 30
typedef void (*timer_callback_t)(void);

typedef struct {
    uint64_t periodic_time_ns;      // how many virtual ticks between calls
    uint64_t last_time_ns; // last tick we fired
    timer_callback_t cb;
} vtimer_t;


typedef struct {
    vtimer_t timers[HOST_TIMERS];
    unsigned int timer_index;
    int64_t  min_timestep_ns;
} vtimer_manager_t;

extern vtimer_manager_t timer_manager;
extern volatile uint64_t virtual_tick;
extern float vtime;

unsigned int register_timer(vtimer_manager_t* mgr, timer_callback_t cb, uint64_t timestep_ns);
bool configure_timer_timestep(vtimer_manager_t *mgr, unsigned int idx, uint64_t timestep_ns);

void platform_init();
void gui_loop();

#ifdef __cplusplus
}
#endif



