#pragma once

#include <stdint.h>



#ifdef __cplusplus
extern "C"{
#endif

typedef struct{



}eldriver_core_t;

void eldriver_core_init(eldriver_core_t *h);


static inline uint32_t eldriver_core_prof_tick()
{
    return 0;
};

static inline uint32_t eldriver_core_prof_tock(uint32_t start)
{
    (void)start;
    return 0;
};

#ifdef __cplusplus
}
#endif
