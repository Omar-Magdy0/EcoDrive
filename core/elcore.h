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

#define pTASK(task_name, ticker, task, divisor)do{\
    static uint16_t counter_##task_name = 0;\
    if(++counter_##task_name >= divisor){\
        counter_##task_name = 0;\
        task;\
    }\
}\
while(0);




//===========================================================================
// Utils
//===========================================================================
typedef struct{
    uint32_t tick_start;
    float tick_period;
}elcore_swttimer_t;

static inline uint32_t elcore_swttimer_elapsed_ticks(elcore_swttimer_t *t, uint32_t tick_current)
{
    return tick_current - t->tick_start;
}

static inline float elcore_swttimer_elapsed_time(elcore_swttimer_t *t, uint32_t tick_current)
{
    return elcore_swttimer_elapsed_ticks(t, tick_current)*t->tick_period;
}

static inline uint8_t elcore_swttimer_timout(elcore_swttimer_t *t, uint32_t tick_curret, uint32_t ticks_to_pass)
{
    return (elcore_swttimer_elapsed_ticks(t, tick_curret) >= ticks_to_pass);
}

static inline void elcore_swttimer_reset(elcore_swttimer_t *t, uint32_t start_tick)
{
    t->tick_start = start_tick;
} 



#ifdef __cplusplus
}
#endif

#endif 
