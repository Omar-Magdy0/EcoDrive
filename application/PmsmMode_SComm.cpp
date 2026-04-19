/**
 * @file PmsmMode_SComm.cpp
 * @brief Implementation of the Self-Commissioning (Parameter Identification) control mode.
 * @details Executes automated electrical tests to identify motor parameters without spinning the rotor.
 *          This includes DC alignment for rotor positioning, DC voltage injection for Stator Resistance (Rs) measurement,
 *          and High-Frequency Injection (HFI) on the D and Q axes for Inductance (Ld and Lq) identification.
 */
#include "PmsmControl.h"
#include <arm_math.h>

constexpr float SCOMM_ALIGN_VOLTAGE = 2;
constexpr float SCOMM_CS_ERROR_MARGIN = 0.2;
constexpr float SCOMM_ALIGN_DURATION_MS = 100;
constexpr float SCOMM_HFI_FREQ = 1000;
constexpr float SCOMM_HFI_VOLTAGE = 3.5;
constexpr uint8_t SCOMM_ID_OVERSAMPLE = 20;
constexpr float SCOMM_HFI_SETTLE_TIME_MS = 50;
constexpr float SCOMM_HFI_DEMOD_ALPHA = 0.005;
constexpr uint16_t SCOMM_HFI_IIR_SETTLE = (4.6 / SCOMM_HFI_DEMOD_ALPHA);

/**
 * @brief Initializes the Self-Commissioning state machine.
 * @details Resets the identification stages back to the initial condition (`RESET`),
 *          ensuring that any previous commissioning data or states are cleared before starting a new test.
 */
void PmsmControl::SelfCommission_init()
{
    SComm.IDstage = SComm.IDStage::RESET;
    SComm.IDstage_last = SComm.IDStage::RESET;
}

/**
 * @brief High-frequency PWM loop for Self-Commissioning parameter identification.
 * @details Executed at the PWM switching frequency. It sequences through the following stages:
 *          - **DAXIS_ALIGN**: Applies a fixed DC voltage to align the rotor to the D-axis (theta = 0).
 *          - **RS_ID**: Measures the steady-state DC current to estimate Stator Resistance (Rs).
 *          - **LD_ID**: Injects a high-frequency (HF) sinusoidal voltage on the D-axis and processes the current response.
 *          - **LQ_ID**: Injects an HF sinusoidal voltage on the Q-axis and processes the current response.
 *          - **ELEC_POSTPROCESS**: Safely disables outputs and signals the slow loop to calculate the final values.
 */
void PmsmControl::SelfCommission_pwmLoop()
{
    switch (SComm.IDstage)
    {
    case SComm.IDStage::RESET:
    {
        SComm.IDstage = SComm.IDStage::DAXIS_ALIGN;
    }
    case SComm.IDStage::DAXIS_ALIGN:
    {
        /// **Stage Entry Point:** Initialize D-Axis alignment variables
        if (SComm.IDstage != SComm.IDstage_last)
        {
            /// Reset the stage timer
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::DAXIS_ALIGN;
            elec.theta = 0;
            float sin, cos, valpha, vbeta;
            float vd = SCOMM_ALIGN_VOLTAGE;
            float vq = 0;
            arm_sin_cos_f32(elec.theta, &sin, &cos);
            arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin, cos);
            int16_t valpha_q15 = valpha / elec.vbus * (1 << 15);
            int16_t vbeta_q15 = vbeta / elec.vbus * (1 << 15);
            /// Apply the synthesized voltage vector via Space Vector Modulation (SVM)
            eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        }

        /// **Stage Exit Point:** Check if the alignment duration has passed
        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_ALIGN_DURATION_MS)))
        {
            /// Proceed to Stator Resistance (Rs) identification stage
            SComm.IDstage = SComm.IDStage::RS_ID;
        }
    }
    break;
    case SComm.IDStage::RS_ID:
    { 
        /// **Stage Entry Point:** Initialize Resistance Identification
        if (SComm.IDstage != SComm.IDstage_last)
        {
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::RS_ID;
            SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
        }
        
        /// After the alignment period, the phase currents should have reached a steady DC state.
        /// Measure the phase currents to estimate the DC resistance (Since theta = 0, valpha = Vd and Ia = Id).
        float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
        float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
        float Ic = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cw_q31);

        /// Sanity check on the current sensors: according to Kirchhoff's Current Law, the sum of phase currents (Ia + Ib + Ic) should be approx zero.
        float nsum = Ia + Ib + Ic;
        if (((nsum >= SCOMM_CS_ERROR_MARGIN) || (nsum <= -SCOMM_CS_ERROR_MARGIN)) && Ia != 0 && Ib != 0 && Ic != 0)
        {
            /// @todo Implement fault handling logic: sensors are providing inconsistent readings.
        }

        float Id, Iq, Ialpha, Ibeta, sin, cos;
        arm_sin_cos_f32(elec.theta, &sin, &cos);
        arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
        arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin, cos);
        
        /// Accumulate estimated resistance samples (Ohm's law: R = Vd / Id)
        model.Rs += (SCOMM_ALIGN_VOLTAGE) / (Id);

        SComm.remaining_id_samples--;

        /// **Stage Exit Point:** Check if all oversampling points are collected
        if (SComm.remaining_id_samples == 0)
        {
            /// Calculate the average resistance from the accumulated samples
            SComm.rs_dc = model.Rs / SCOMM_ID_OVERSAMPLE;
            /// Proceed to D-Axis Inductance (Ld) identification stage
            SComm.IDstage = SComm.IDStage::LD_ID;
        }
    }
    break;
    case SComm.IDStage::LD_ID:
    { 
        /// **Stage Entry Point:** Initialize Ld Identification
        if (SComm.IDstage != SComm.IDstage_last)
        {
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::LD_ID;
            SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
            SComm.id_cos_prod_2 = 0;
            SComm.id_sin_prod_2 = 0;
            SComm.hfi_angle = 0;
            SComm.iir_filtered_cnt = 0;
        }
        
        /// Inject a High-Frequency (HF) sinusoidal voltage signal purely on the D-axis (theta = 0)
        elec.theta = 0;
        float sin_val, cos_val, valpha, vbeta;
        float hfi_sin, hfi_cos;
        arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
        float vd = SCOMM_HFI_VOLTAGE * hfi_sin;
        float vq = 0;
        arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
        arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
        int16_t valpha_q15 = valpha / elec.vbus * (1 << 15);
        int16_t vbeta_q15 = vbeta / elec.vbus * (1 << 15);
        eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        
        /// Advance the internal high-frequency angle
        SComm.hfi_angle += 2 * PI * (ticks_to_ms(1) / 1000) * SCOMM_HFI_FREQ;

        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
        { 
            /// Once the high-frequency current response reaches a steady state, perform digital signal processing (DSP).
            
            /// Fetch fresh current readings and apply Clarke/Park transforms to extract Id and Iq
            float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
            float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
            float Ialpha, Ibeta, Id, Iq;
            arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
            arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
            
            /// Perform Heterodyning: Multiply the current response by the carrier signal to extract amplitude and phase
            float cos_prod = Id * hfi_cos;
            float sin_prod = Id * hfi_sin;
            
            /// Apply an Infinite Impulse Response (IIR) Low-Pass Filter (LPF) to smooth the heterodyned signals
            SComm.id_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.id_cos_prod_2);
            SComm.id_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.id_sin_prod_2);
            SComm.iir_filtered_cnt++;
        }

        /// **Stage Exit Point:** Wait until the IIR filter settles
        if (SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
        {
            /// Proceed to Q-Axis Inductance (Lq) identification stage
            SComm.IDstage = SComm.IDStage::LQ_ID;
        }
    }
    break;
    case SComm.IDStage::LQ_ID:
    { 
        /// **Stage Entry Point:** Initialize Lq Identification
        if (SComm.IDstage != SComm.IDstage_last)
        {
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::LQ_ID;
            SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
            SComm.iq_cos_prod_2 = 0;
            SComm.iq_sin_prod_2 = 0;
            SComm.hfi_angle = 0;
            SComm.iir_filtered_cnt = 0;
        }
        
        /// Inject a High-Frequency (HF) sinusoidal voltage signal purely on the Q-axis (vd = 0)
        elec.theta = 0;
        float sin_val, cos_val, valpha, vbeta;
        float hfi_sin, hfi_cos;
        arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
        float vd = 0;
        float vq = SCOMM_HFI_VOLTAGE * hfi_sin;
        arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
        arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
        int16_t valpha_q15 = valpha / elec.vbus * (1 << 15);
        int16_t vbeta_q15 = vbeta / elec.vbus * (1 << 15);
        eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        
        /// Advance the internal high-frequency angle
        SComm.hfi_angle += 2 * PI * (ticks_to_ms(1) / 1000) * SCOMM_HFI_FREQ;

        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
        { 
            /// Process the high-frequency response on the Q-axis after transient settling
            
            /// Fetch and transform currents into the dq rotating reference frame
            float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
            float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
            float Ialpha, Ibeta, Id, Iq;
            arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
            arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
            
            /// Perform Heterodyning on the Q-axis current
            float cos_prod = Iq * hfi_cos;
            float sin_prod = Iq * hfi_sin;
            
            /// Smooth the resulting data using a Low-Pass IIR filter
            SComm.iq_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.iq_cos_prod_2);
            SComm.iq_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.iq_sin_prod_2);
            SComm.iir_filtered_cnt++;
        }

        /// **Stage Exit Point:** Wait for stable filtered readings
        if (SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
        { 
            /// Raw identification complete; trigger the low-frequency background task to calculate final electrical parameters
            SComm.IDstage = SComm.IDStage::ELEC_POSTPROCESS;
            
            /// Safely disable the inverter output phases until post-processing finishes
            eldriver_mc3p_write_float(&mc3p);
        }
    }
    break;
    default:
        break;
    }
}

/**
 * @brief Cross-domain low-frequency loop for post-processing identified parameters.
 * @details Executed at a lower priority/frequency than the PWM loop. It performs heavy floating-point math 
 *          (e.g., square roots and arctangents) to extract the final Rs, Ld, and Lq values safely outside 
 *          the time-critical fast PWM interrupt. It uses the heterodyned and filtered data collected during the HFI stages.
 */
void PmsmControl::SelfCommission_xmcLoop()
{
    switch (SComm.IDstage)
    {
    case SComm.IDStage::ELEC_POSTPROCESS:
    { 
        /// **Stage Entry Point:** Post-process raw heterodyned data
        /// Use volatile casts to guarantee a fresh memory fetch of the asynchronously updated filter accumulators
        float id_sin_prod = 2 * *(volatile float *)&SComm.id_sin_prod_2;
        float id_cos_prod = 2 * *(volatile float *)&SComm.id_cos_prod_2;
        float iq_sin_prod = 2 * *(volatile float *)&SComm.iq_sin_prod_2;
        float iq_cos_prod = 2 * *(volatile float *)&SComm.iq_cos_prod_2;

        /// Calculate current response amplitude via the Pythagorean theorem
        float id_amplitude = sqrtf(id_cos_prod * id_cos_prod + id_sin_prod * id_sin_prod);
        float iq_amplitude = sqrtf(iq_cos_prod * iq_cos_prod + iq_sin_prod * iq_sin_prod);
        /// Extract the phase angle delay using arctangent
        float id_phi = atan2f(-id_cos_prod, id_sin_prod);
        float iq_phi = atan2f(-iq_cos_prod, iq_sin_prod);
        /// Calculate the complex impedance magnitude (Z = V / I)
        float zd = SCOMM_HFI_VOLTAGE / id_amplitude;
        float zq = SCOMM_HFI_VOLTAGE / iq_amplitude;

        /// Cross-verify resistance values derived from the HFI impedance equations (Redundant but useful for diagnostics)
        float Rd = zd * cos(id_phi);
        float Rq = zq * cos(iq_phi);
        /// Derive the final inductances using the imaginary component of the impedance (Xl = 2 * PI * f * L)
        float Ld = (zd * sin(id_phi)) / (2 * M_PI * SCOMM_HFI_FREQ);
        float Lq = (zq * sin(iq_phi)) / (2 * M_PI * SCOMM_HFI_FREQ);

        /// Store the calculated motor parameters into the system model state
        model.Ld = Ld;
        model.Lq = Lq;
        model.Rs = SComm.rs_dc;
        
        /// @todo Expand post-processing to trigger Mechanical Inertia and Flux Linkage identification stages next.
    }
    break;
    default:
        break;
    }
}