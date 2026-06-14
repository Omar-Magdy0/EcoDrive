#pragma once
#include "PmsmControl.h"


static inline constexpr uint32_t PWM_FREQ_DEFAULT = 25000; /** Default PWM frequency (Hz). */
static inline constexpr float STUP_BEMFZC_ERROR_MARGIN = 0.05f; /** Max relative BEMF ZC error. */
static inline constexpr uint8_t STUP_BEMFZC_GOOD_EST_COUNT = 10; /** Consecutive good ZC estimates. */

#define DEFAULT_POLE_PAIRS 6
#ifdef ELDRIVER_HALL1_ENABLED
#define HALL_ENABLED
#else
#define BEMFZC_ENABLED
#endif
