#include "eldriver_core.h"
#include "stm32f4xx.h"


static inline void dwt_init(void)
{
    // Enable TRC (trace)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset cycle counter
    DWT->CYCCNT = 0;

    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void eldriver_core_init(eldriver_core_t *h)
{
    dwt_init();
}
