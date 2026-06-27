/**
 * @file eldriver_mc3p.h
 * @author Omar Magdy
 * @brief STM32F4 MC3P 3phase motor-control driver API.
 *
 * @details
 * This module configures TIM1, ADC1, and DMA2 to provide:
 * - PWM generation for 3-phase bridge control (trap or SVM)
 * - Injected ADC sampling synchronized to PWM for phase/current sensing
 * - Background ADC scanning via DMA for slow-monitoring channels
 *
 * Timing is driven by TIM1 center-aligned PWM. ADC injected conversions are
 * triggered from TIM1 TRGO (CH4 reference), while regular conversions use
 * DMA in circular double-buffer mode.
 */

#ifndef MCADCPWM3P_H
#define MCADCPWM3P_H

#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Number of synchronized ADC channels for MC3P. */
#define MC3P_SYNC_CHANNELS 7
#include <stdbool.h>
    typedef enum
    {
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
    typedef enum
    {
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
    typedef enum
    {
        ELDRIVER_MC3P_PHASE_FLOAT = 0, /**< Phase is floating */
        ELDRIVER_MC3P_PHASE_H_PWM,     /**< High-side PWM active */
        ELDRIVER_MC3P_PHASE_L_PWM,     /**< Low-side PWM active */
        ELDRIVER_MC3P_PHASE_COMP,      /**< Complementary PWM active */
        ELDRIVER_MC3P_PHASE_H_ON,      /**< High-side strictly ON */
        ELDRIVER_MC3P_PHASE_L_ON       /**< Low-side strictly ON */
    } eldriver_mc3p_phase_state_t;

    /**
     * @brief Mapping for synchronized ADC measurements.
     */
    typedef enum
    {
        ELDRIVER_MC3P_VSBUS = 0,                 /**< DC Bus Voltage */
        ELDRIVER_MC3P_VSU,                       /**< Phase U Voltage */
        ELDRIVER_MC3P_VSV,                       /**< Phase V Voltage */
        ELDRIVER_MC3P_VSW,                       /**< Phase W Voltage */
        ELDRIVER_MC3P_CSU,                       /**< Phase U Current */
        ELDRIVER_MC3P_CSV,                       /**< Phase V Current */
        ELDRIVER_MC3P_CSW,                       /**< Phase W Current */
        ELDRIVER_MC3P_CSBUS = ELDRIVER_MC3P_CSU, /**< DC Bus Current */
    } eldriver_mc3p_sync;

    /**
     * @brief Configuration structure for MC3P PWM and duty limits.
     */
    typedef struct
    {
        uint32_t pwm_Hz;      /**< PWM Frequency in Hertz */
        uint32_t deadtime_nS; /**< Deadtime in Nanoseconds */
        float duty_max;       /**< Maximum allowable duty cycle */
        float duty_min;       /**< Minimum allowable duty cycle */
    } eldriver_mc3p_config_t;

    /**
     * @brief Main handle structure for the MC3P driver instance.
     */
    typedef struct
    {
        eldriver_mc3p_config_t config;
        int16_t adc_to_uV;
        float adc_ref_V;
        float internal_ref_V;
        volatile eldriver_mc3p_mode_t mode;
        volatile eldriver_mc3p_sector_t sector_last;
        uint32_t timer_max_q15;
        uint16_t duty_max_q15;
        uint16_t duty_min_q15;
        int16_t dutyu_q15;
        int16_t dutyv_q15;
        int16_t dutyw_q15;
        int32_t sync_scale_q31[MC3P_SYNC_CHANNELS][2];
        uint8_t sync_rank_scale[4];
        bool offset_calibration;
        volatile uint8_t offset_calibration_remaining_samples;
        volatile uint16_t offset_calibration_sum[4];
        uint16_t dtc_comp_q15;
        uint8_t dtc_state;
    } eldriver_mc3p_t;

    /** @brief Data structure for SVM-specific measurements. */
    typedef struct
    {
        int32_t vbus_q31;
        int32_t cu_q31;
        int32_t cv_q31;
        int32_t cw_q31;
    } eldriver_mc3p_svm_data_t;

    /** @brief Data structure for Trapezoidal-specific measurements. */
    typedef struct
    {
        int32_t vbus_q31;
        int32_t vbemf_q31;
        int32_t cbus_q31;
    } eldriver_mc3p_trap_data_t;

/* Conversion Macros */
#define ELDRIVER_MC3P_VS_TO_FLOAT(vs) ((float)(((int64_t)(vs) * ELDRIVER_MC3P_VS_SCALE) >> 31))
#define ELDRIVER_MC3P_CS_TO_FLOAT(cs) ((float)(((int64_t)(cs) * ELDRIVER_MC3P_CS_SCALE) >> 31))
#define ELDRIVER_MC3P_FLOAT_TO_VS(f) ((int32_t)(((float)(f) / ELDRIVER_MC3P_VS_SCALE) * INT32_MAX))
#define ELDRIVER_MC3P_FLOAT_TO_CS(f) ((int32_t)(((float)(f) / ELDRIVER_MC3P_CS_SCALE) * INT32_MAX))
#define ELDRIVER_MC3P_SYNC_CHANNELS_NUM()(MC3P_SYNC_CHANNELS)

    // TODO  FINISH ADC IMPLEMENTATION FOR 1)TRAP & 2)SVM
    void mc3p_irq_bind(eldriver_mc3p_t *h);

    /**
     * @brief Initialize the MC3P driver (GPIO, TIM1, ADC, DMA, IRQ).
     *
     * @details
     * Computes duty scaling, binds IRQ handlers, and optionally performs
     * offset calibration if enabled.
     *
     * @param h Driver handle.
     */
    void eldriver_mc3p_init(eldriver_mc3p_t *h);
    void eldriver_mc3p_reconfigure_pwm(eldriver_mc3p_t *h);
    /**
     * @brief Update scaling gain for a synced signal (voltage or current).
     *
     * @param h Driver handle.
     * @param s Signal selector.
     * @param gain Gain applied before Q31 scaling.
     */
    void eldriver_mc3p_set_gain(eldriver_mc3p_t *h, eldriver_mc3p_sync s, float gain);

    void eldriver_mc3p_set_sync_scale(eldriver_mc3p_t *h, const float scales[MC3P_SYNC_CHANNELS][2]);
    /**
     * @brief Start a regular-group ADC conversion for background scanning.
     *
     * @param h Driver handle.
     */
    void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h);

    /**
     * @brief Get number of configured background channels.
     *
     * @param h Driver handle.
     * @return Channel count.
     */
    uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h);

    /**
     * @brief Copy background scan results (DMA) into a float buffer.
     *
     * @param h Driver handle.
     * @param scanData Output buffer sized to ELDRIVER_MC3P_BG_CHANNELS.
     * @return Number of channels copied.
     */
    uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float *scanData);

    /**
     * @brief Check if a new background DMA buffer is ready.
     *
     * @param h Driver handle.
     * @return 1 if ready, 0 otherwise.
     */
    uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h);

    /**
     * @brief Read injected ADC data for the active mode into the provided struct.
     *
     * @details
     * The `scanData` pointer must point to the correct struct type for the active
     * mode (trap or SVM). Conversion data is scaled to Q31 using precomputed
     * scale/offset in `h->sync_scale_q31`.
     *
     * @param h Driver handle.
     * @param scanData Output struct pointer (mode-specific).
     */
    void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void *scanData);

    /**
     * @brief Read a single ADC channel and convert to volts.
     *
     * @param h Driver handle (adc_ref_V must be calibrated).
     * @param channel ADC channel ID (LL_ADC_CHANNEL_x).
     * @return Voltage in volts.
     */
    float eldriver_mc3p_adc_read_single(eldriver_mc3p_t *h, uint32_t channel);

    /**
     * @brief Set per-phase drive state (float, low, high, complementary PWM).
     *
     * @param h Driver handle.
     * @param state_u Phase-U state.
     * @param state_v Phase-V state.
     * @param state_w Phase-W state.
     */
    void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w);

    /**
     * @brief Update PWM duty for each phase (Q15).
     *
     * @details
     * Duty is saturated to [duty_min_q15, duty_max_q15] and scaled to TIM1 ARR.
     *
     * @param h Driver handle.
     * @param duty_u_q15 Phase-U duty in Q15.
     * @param duty_v_q15 Phase-V duty in Q15.
     * @param duty_w_q15 Phase-W duty in Q15.
     */
    void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, int16_t duty_u_q15, int16_t duty_v_q15, int16_t duty_w_q15);

    /**
     * @brief Set all phases to float (high-Z) with zero duty.
     *
     * @param h Driver handle.
     */
    void eldriver_mc3p_write_float(eldriver_mc3p_t *h);

    /**
     * @brief Apply trap-commutation sector and duty.
     *
     * @details
     * Updates phase states and ADC injected channel map when the sector changes.
     *
     * @param h Driver handle.
     * @param sector Trap sector (1..6).
     * @param duty_q15 Duty in Q15.
     */
    void eldriver_mc3p_write_trap(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15);
    
    /**
     * @brief Apply SVM duty from alpha/beta inputs (Q15).
     *
     * @details
     * Computes phase duties, performs zero-sequence injection, then updates ADC
     * synchronization and PWM outputs.
     *
     * @param h Driver handle.
     * @param alpha_q15 Alpha component in Q15.
     * @param beta_q15 Beta component in Q15.
     */
    void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15);

    /** @brief Weak callback for system ticker. */
    __attribute__((weak)) void eldriver_xmc3p_tickerCallback(void);

    /** @brief Weak callback after synchronized ADC scan completion. */
    __attribute__((weak)) void eldriver_mc3p_sync_postScanCallback(void);
#ifdef __cplusplus
}
#endif

#endif
