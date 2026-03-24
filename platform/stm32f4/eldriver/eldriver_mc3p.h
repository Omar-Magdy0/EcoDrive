/**
 * @file    eldriver_mc3p.h
 * @author  Carol Nasser
 * @brief   Header file for 3-Phase Motor Controller (MC3P) Driver.
 * @details This driver manages PWM generation, sector switching (Trap/SVM), 
 * and ADC synchronization for motor control in the EcoDrive system.
 */

#ifndef MCADCPWM3P_H
#define MCADCPWM3P_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of synchronized ADC channels for MC3P. */
#define MC3P_SYNC_CHANNELS 7
#include <stdbool.h>
typedef enum {
    ELDRIVER_MC3P_SECTOR_FLOAT = 0, /**< Floating state (all MOSFETs OFF) */
    ELDRIVER_MC3P_SECTOR_TRAP1,     /**< Trapezoidal Sector 1 */
    ELDRIVER_MC3P_SECTOR_TRAP2,     /**< Trapezoidal Sector 2 */
    ELDRIVER_MC3P_SECTOR_TRAP3,     /**< Trapezoidal Sector 3 */
    ELDRIVER_MC3P_SECTOR_TRAP4,     /**< Trapezoidal Sector 4 */
    ELDRIVER_MC3P_SECTOR_TRAP5,     /**< Trapezoidal Sector 5 */
    ELDRIVER_MC3P_SECTOR_TRAP6,     /**< Trapezoidal Sector 6 */
    ELDRIVER_MC3P_SECTOR_SVM1,      /**< SVM Sector 1 */
    ELDRIVER_MC3P_SECTOR_SVM2,      /**< SVM Sector 2 */
    ELDRIVER_MC3P_SECTOR_SVM3,      /**< SVM Sector 3 */
    ELDRIVER_MC3P_SECTOR_SVM4,      /**< SVM Sector 4 */
    ELDRIVER_MC3P_SECTOR_SVM5,      /**< SVM Sector 5 */
    ELDRIVER_MC3P_SECTOR_SVM6       /**< SVM Sector 6 */
} eldriver_mc3p_sector_t;

/**
 * @brief Enumeration for motor control modulation modes.
 */
typedef enum {
    ELDRIVER_MC3P_MODE_NONE = 0,
    ELDRIVER_MC3P_MODE_TRAP,
    ELDRIVER_MC3P_MODE_SVM,
    ELDRIVER_MC3P_MODE_CALIB
} eldriver_mc3p_mode_t;

/** @brief Check if sector is within Space Vector Modulation range. */
#define IS_SVM_SECTOR(sector) (sector >= ELDRIVER_MC3P_SECTOR_SVM1 && sector <= ELDRIVER_MC3P_SECTOR_SVM6)
/** @brief Check if sector is within Trapezoidal range. */
#define IS_TRAP_SECTOR(sector) (sector >= ELDRIVER_MC3P_SECTOR_TRAP1 && sector <= ELDRIVER_MC3P_SECTOR_TRAP6)

/**
 * @brief Enumeration for the state of each motor phase.
 */
typedef enum {
    ELDRIVER_MC3P_PHASE_FLOAT = 0,  /**< Phase is floating */
    ELDRIVER_MC3P_PHASE_H_PWM,      /**< High-side PWM active */
    ELDRIVER_MC3P_PHASE_L_PWM,      /**< Low-side PWM active */
    ELDRIVER_MC3P_PHASE_COMP,       /**< Complementary PWM active */
    ELDRIVER_MC3P_PHASE_H_ON,       /**< High-side strictly ON */
    ELDRIVER_MC3P_PHASE_L_ON        /**< Low-side strictly ON */
} eldriver_mc3p_phase_state_t;  

/**
 * @brief Mapping for synchronized ADC measurements.
 */
typedef enum{
    ELDRIVER_MC3P_VSBUS = 0,        /**< DC Bus Voltage */
    ELDRIVER_MC3P_VSU,              /**< Phase U Voltage */
    ELDRIVER_MC3P_VSV,              /**< Phase V Voltage */
    ELDRIVER_MC3P_VSW,              /**< Phase W Voltage */
    ELDRIVER_MC3P_CSU,              /**< Phase U Current */
    ELDRIVER_MC3P_CSV,              /**< Phase V Current */
    ELDRIVER_MC3P_CSW,              /**< Phase W Current */
    ELDRIVER_MC3P_CSBUS = ELDRIVER_MC3P_CSU, /**< DC Bus Current */
}eldriver_mc3p_sync;

/**
 * @brief Configuration structure for MC3P PWM and duty limits.
 */
typedef struct{
    uint32_t pwm_Hz;                /**< PWM Frequency in Hertz */
    uint32_t deadtime_nS;           /**< Deadtime in Nanoseconds */
    float duty_max;                 /**< Maximum allowable duty cycle */
    float duty_min;                 /**< Minimum allowable duty cycle */
}eldriver_mc3p_config_t;

/**
 * @brief Main handle structure for the MC3P driver instance.
 */
typedef struct{
    eldriver_mc3p_config_t config;
    int16_t adc_to_uV;
    float adc_ref_V;
    float internal_ref_V;
    volatile eldriver_mc3p_mode_t mode;
    volatile eldriver_mc3p_sector_t sector_last;
    uint32_t timer_max_q15;
    uint16_t duty_max_q15;
    uint16_t duty_min_q15;
    uint16_t dutyu_q15;
    uint16_t dutyv_q15;
    uint16_t dutyw_q15;
    int32_t sync_scale_q31[MC3P_SYNC_CHANNELS][2];
    uint8_t sync_rank_scale[4];
    bool offset_calibration;
    volatile uint8_t offset_calibration_remaining_samples;
    volatile uint16_t offset_calibration_sum[4];
}eldriver_mc3p_t;

/** @brief Data structure for SVM-specific measurements. */
typedef struct
{   
    uint32_t vbus_q31;
    uint32_t cu_q31;
    uint32_t cv_q31;
    uint32_t cw_q31;
} eldriver_mc3p_svm_data_t;

/** @brief Data structure for Trapezoidal-specific measurements. */
typedef struct{
    uint32_t vbus_q31;
    uint32_t vbemf_q31;
    uint32_t cbus_q31;
} eldriver_mc3p_trap_data_t;

/* Conversion Macros */
#define ELDRIVER_MC3P_VS_TO_FLOAT(vs)((float)(((int64_t)(vs)* ELDRIVER_MC3P_VS_SCALE)  >> 31 ))
#define ELDRIVER_MC3P_CS_TO_FLOAT(cs)((float)(((int64_t)(cs)* ELDRIVER_MC3P_CS_SCALE)  >> 31 ))
#define ELDRIVER_MC3P_FLOAT_TO_VS(f)((int32_t)(((float)(f)/ELDRIVER_MC3P_VS_SCALE) * INT32_MAX ))
#define ELDRIVER_MC3P_FLOAT_TO_CS(f)((int32_t)(((float)(f)/ELDRIVER_MC3P_CS_SCALE) * INT32_MAX ))


//TODO  FINISH ADC IMPLEMENTATION FOR 1)TRAP & 2)SVM
void mc3p_irq_bind(eldriver_mc3p_t *h);
void eldriver_mc3p_init(eldriver_mc3p_t *h);
void eldriver_mc3p_setGain(eldriver_mc3p_t *h, eldriver_mc3p_sync s, float gain);
void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h);

/** @brief Returns number of active background channels. */
uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h);

/** @brief Reads background scan data into buffer. */
uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float* scanData);

/** @brief Check if background ADC data is ready. */
uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h);

/** @brief Reads synchronized ADC data. */
void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void* scanData);
float eldriver_mc3p_adc_read_single(eldriver_mc3p_t *h, uint32_t channel);

/** @brief Set specific phase states (U, V, W). */
void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w);

/** @brief Manually set PWM duty cycle for each phase in Q15. */
void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, uint16_t duty_u_q15, uint16_t duty_v_q15, uint16_t duty_w_q15);

/** @brief Set all phases to floating state. */
void eldriver_mc3p_write_float(eldriver_mc3p_t *h);

/** @brief Set sector and duty for Trapezoidal control. */
void eldriver_mc3p_write_trap(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15);

/** @brief Set Alpha/Beta components for SVM control. */
void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15);

/** @brief Weak callback for system ticker. */
__attribute__((weak)) void eldriver_xmc3p_tickerCallback(void);

/** @brief Weak callback after synchronized ADC scan completion. */
__attribute__((weak)) void eldriver_mc3p_sync_postScanCallback(void);

#ifdef __cplusplus
}
#endif

#endif