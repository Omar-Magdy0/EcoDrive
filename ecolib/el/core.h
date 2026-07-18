#ifndef ELCORE
#define ELCORE

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


//============================================================================================



//===========================================================================
// Utils
//===========================================================================
typedef struct{
    uint32_t tick_start;
    float tick_period;
}el_swttimer_t;

static inline uint32_t el_swttimer_elapsed_ticks(el_swttimer_t *t, uint32_t tick_current)
{
    return tick_current - t->tick_start;
}

static inline float el_swttimer_elapsed_time(el_swttimer_t *t, uint32_t tick_current)
{
    return el_swttimer_elapsed_ticks(t, tick_current)*t->tick_period;
}

static inline uint8_t el_swttimer_timout(el_swttimer_t *t, uint32_t tick_curret, uint32_t ticks_to_pass)
{
    return (el_swttimer_elapsed_ticks(t, tick_curret) >= ticks_to_pass);
}

static inline void el_swttimer_reset(el_swttimer_t *t, uint32_t start_tick)
{
    t->tick_start = start_tick;
} 



#ifdef __cplusplus
}
#endif

#endif 
