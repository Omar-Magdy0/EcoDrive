/**
 * @file PmsmMode_OpenFocIF.cpp
 * @brief Implementation of the Open-Loop V/F (Voltage/Frequency) control mode.
 * @details Drives the motor by applying a synthetic rotating voltage vector without relying on actual rotor position feedback.
 *          This is often used for simple loads or as an initial alignment/forced-commutation step.
 */
#include "PmsmControl.h"

/**
 * @brief Empty stub for the Reset stage in Open-Loop V/F mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ResetHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Alignment stage in Open-Loop V/F mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void AlignHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Ramp stage in Open-Loop V/F mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void RampHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Closed stage in Open-Loop V/F mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ClosedHandler(PmsmControl *cp) {}

/**
 * @brief Executes the PWM loop specifically for Open-Loop V/F (or Open FOC) control.
 * @details Evaluates the startup state machine and calculates the Space Vector Modulation (SVM) 
 *          duty cycles based on a synthetic electrical angle and target dq-axis voltages.
 */
void PmsmControl::OpenFocIF_pwmLoop()
{
    /// Route execution based on the current startup stage
    switch (stup.stage_current)
    {
    case PmsmControl::StupStage::Reset:
        ResetHandler(this);
        break;
    case PmsmControl::StupStage::Align:
        AlignHandler(this);
        break;
    case PmsmControl::StupStage::Ramp:
        RampHandler(this);
        break;
    case PmsmControl::StupStage::Closed:
        ClosedHandler(this);
        break;
    default:
        break;
    }

    /// Define target D-axis and Q-axis voltages (currently 0 as default/placeholder)
    float vd = 0;
    float vq = 0;
    float sin_val, cos_val, valpha, vbeta;
    
    /// Calculate sine and cosine for the current synthetic electrical angle (theta)
    arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
    
    /// Inverse Park Transform: Convert rotating dq-frame voltages into stationary alpha-beta frame voltages
    arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
    
    /// Normalize the alpha-axis voltage against the DC bus voltage and convert to fixed-point Q15
    int16_t valpha_q15 = (valpha / elec.vbus) * (1 << 15);
    /// Normalize the beta-axis voltage against the DC bus voltage and convert to fixed-point Q15
    int16_t vbeta_q15 = (vbeta / elec.vbus) * (1 << 15);
    
    /// Send the calculated alpha-beta voltages to the underlying Space Vector Modulation (SVM) hardware driver
    eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
}