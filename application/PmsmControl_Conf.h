#pragma once
#include "PmsmControl.h"


static inline constexpr uint32_t PWM_FREQ_DEFAULT = 25000; /** Default PWM frequency (Hz). */
static inline constexpr float STUP_BEMFZC_ERROR_MARGIN = 0.05f; /** Max relative BEMF ZC error. */
static inline constexpr uint8_t STUP_BEMFZC_GOOD_EST_COUNT = 10; /** Consecutive good ZC estimates. */
constexpr float SCOMM_ALIGN_VOLTAGE = 1;
constexpr float SCOMM_CS_ERROR_MARGIN = 0.2;
constexpr float SCOMM_ALIGN_DURATION_MS = 1000;
constexpr float SCOMM_HFI_FREQ = 1000;
constexpr float SCOMM_HFI_VOLTAGE = 3.5;
constexpr uint32_t SCOMM_ID_OVERSAMPLE = 20000;
constexpr float SCOMM_HFI_SETTLE_TIME_MS = 100;
constexpr float SCOMM_HFI_DEMOD_ALPHA = 0.005;
constexpr uint16_t SCOMM_HFI_IIR_SETTLE = (4.6 / SCOMM_HFI_DEMOD_ALPHA);

#define DEFAULT_POLE_PAIRS 6
#ifdef ELDRIVER_HALL1_ENABLED
#define HALL_ENABLED
#else
#define BEMFZC_ENABLED
#endif
