#include "PmsmControl.h"

void PmsmControl::SComm_init()
{
    //SComm.IDstage = SComm.IDStage::RESET;
    //SComm.IDstage_last = SComm.IDStage::RESET;
    //control.mctype = MCType::SComm;
}
void PmsmControl::SComm_onEnter(MCType prev_mct)
{
}
void PmsmControl::SComm_onExit()
{
}

void PmsmControl::SComm_pwmLoop()
{
    switch (SComm.IDstage)
    {
    case SComm.IDStage::RESET:
    {
        SComm.IDstage = SComm.IDStage::DAXIS_ALIGN;
    }
    case SComm.IDStage::DAXIS_ALIGN:
    {
        // Entry Point
        if (SComm.IDstage != SComm.IDstage_last)
        {
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::DAXIS_ALIGN;
            state.eTheta_q31 = 0;
            float sin, cos, valpha, vbeta;
            float vd = SCOMM_ALIGN_VOLTAGE;
            float vq = 0;
            arm_sin_cos_f32(state.eTheta_q31, &sin, &cos);
            arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin, cos);
            int16_t valpha_q15 = valpha*1000 / state.Vbus_q31 * (1 << 15);
            int16_t vbeta_q15 = vbeta*1000 / state.Vbus_q31 * (1 << 15);
            eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        }

        // Exit point
        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_ALIGN_DURATION_MS)))
        {
            SComm.IDstage = SComm.IDStage::RS_ID;
        }
    }
    break;
    case SComm.IDStage::RS_ID:
    { // Entry Point
        if (SComm.IDstage != SComm.IDstage_last)
        {
            elcore_swttimer_reset(&SComm.timer, pwmTicks);
            SComm.IDstage_last = SComm.IDStage::RS_ID;
            SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
        }
        // After alignment current should have settled by now , measure DC current and DC resistance
        //  valpha = vd for theta equal zero,  Id = Ia for theta equal zero
        float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
        float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
        float Ic = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cw_q31);

        // We can also check for viability of current sensors or applied voltage here (Ia + Ib + Ic ~= 0)(and none equals zero) , Sanity check
        float nsum = Ia + Ib + Ic;
        if (((nsum >= SCOMM_CS_ERROR_MARGIN) || (nsum <= -SCOMM_CS_ERROR_MARGIN)) && Ia != 0 && Ib != 0 && Ic != 0)
        {
            // We shouldnt come here , handle this fault accordingly (TODO)
        }

        float Id, Iq, Ialpha, Ibeta, sin, cos;
        arm_sin_cos_f32(state.eTheta_q31, &sin, &cos);
        arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
        arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin, cos);
        model.Rs += (SCOMM_ALIGN_VOLTAGE) / (Id); // Vd/Id

        SComm.remaining_id_samples--;

        // Exist Point
        if (SComm.remaining_id_samples == 0)
        {
            SComm.rs_dc = model.Rs / SCOMM_ID_OVERSAMPLE;
            SComm.IDstage = SComm.IDStage::LD_ID;
        }
    }
    break;
    case SComm.IDStage::LD_ID:
    { // Entry Point
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
        // RUN HFI on the D-axis
        state.eTheta_q31 = 0;
        float sin_val, cos_val, valpha, vbeta;
        float hfi_sin, hfi_cos;
        arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
        float vd = SCOMM_HFI_VOLTAGE * hfi_sin;
        float vq = 0;
        arm_sin_cos_f32(state.eTheta_q31, &sin_val, &cos_val);
        arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
        int16_t valpha_q15 = valpha*1000 / state.Vbus_q31 * (1 << 15);
        int16_t vbeta_q15 = vbeta*1000 / state.Vbus_q31 * (1 << 15);
        eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        SComm.hfi_angle += 2 * PI * (ticks_to_ms(1) / 1000) * SCOMM_HFI_FREQ;

        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
        { // HF signal steady state , lets do some DSP
            // Get fresh currents and transform to dq
            float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
            float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
            float Ialpha, Ibeta, Id, Iq;
            arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
            arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
            // Lets do some good old heterodyning
            float cos_prod = Id * hfi_cos;
            float sin_prod = Id * hfi_sin;
            // LPF IIR filter
            SComm.id_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.id_cos_prod_2);
            SComm.id_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.id_sin_prod_2);
            SComm.iir_filtered_cnt++;
        }

        // Exit Point
        if (SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
        {
            SComm.IDstage = SComm.IDStage::LQ_ID;
        }
    }
    break;
    case SComm.IDStage::LQ_ID:
    { // Entry Point
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
        // RUN HFI on the D-axis
        state.eTheta_q31 = 0;
        float sin_val, cos_val, valpha, vbeta;
        float hfi_sin, hfi_cos;
        arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
        float vd = 0;
        float vq = SCOMM_HFI_VOLTAGE * hfi_sin;
        arm_sin_cos_f32(state.eTheta_q31, &sin_val, &cos_val);
        arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
        int16_t valpha_q15 = valpha*1000 / state.Vbus_q31 * (1 << 15);
        int16_t vbeta_q15 = vbeta*1000 / state.Vbus_q31 * (1 << 15);
        eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
        SComm.hfi_angle += 2 * PI * (ticks_to_ms(1) / 1000) * SCOMM_HFI_FREQ;

        if (elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
        { // HF signal steady state , lets do some DSP
            // Get fresh currents and transform to dq
            float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
            float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
            float Ialpha, Ibeta, Id, Iq;
            arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
            arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
            // Lets do some good old heterodyning
            float cos_prod = Iq * hfi_cos;
            float sin_prod = Iq * hfi_sin;
            // LPF IIR filter
            SComm.iq_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.iq_cos_prod_2);
            SComm.iq_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.iq_sin_prod_2);
            SComm.iir_filtered_cnt++;
        }

        // Exit Point
        if (SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
        { // Now we have electrical parameters, go ahead and post process them tune current PI controller , etc
            SComm.IDstage = SComm.IDStage::ELEC_POSTPROCESS;
            // Disable phases for now
            eldriver_mc3p_write_float(&mc3p);
        }
    }
    break;
    default:
        break;
    }
}

void PmsmControl::SComm_xmcLoop()
{
    switch (SComm.IDstage)
    {
    case SComm.IDStage::ELEC_POSTPROCESS:
    { // Entry Point (do Nothing)
        // Ensure a fresh Ram fetch
        float id_sin_prod = 2 * *(volatile float *)&SComm.id_sin_prod_2;
        float id_cos_prod = 2 * *(volatile float *)&SComm.id_cos_prod_2;
        float iq_sin_prod = 2 * *(volatile float *)&SComm.iq_sin_prod_2;
        float iq_cos_prod = 2 * *(volatile float *)&SComm.iq_cos_prod_2;

        float id_amplitude = sqrtf(id_cos_prod * id_cos_prod + id_sin_prod * id_sin_prod);
        float iq_amplitude = sqrtf(iq_cos_prod * iq_cos_prod + iq_sin_prod * iq_sin_prod);
        float id_phi = atan2f(-id_cos_prod, id_sin_prod);
        float iq_phi = atan2f(-iq_cos_prod, iq_sin_prod);
        float zd = SCOMM_HFI_VOLTAGE / id_amplitude;
        float zq = SCOMM_HFI_VOLTAGE / iq_amplitude;

        // Check the Resistance values we get from the Ld and Lq identification although they are reduntant
        float Rd = zd * cos(id_phi);
        float Rq = zq * cos(iq_phi);
        float Ld = (zd * sin(id_phi)) / (2 * M_PI * SCOMM_HFI_FREQ);
        float Lq = (zq * sin(iq_phi)) / (2 * M_PI * SCOMM_HFI_FREQ);

        model.Ld = Ld;
        model.Lq = Lq;
        model.Rs = SComm.rs_dc;
        // Exit point (Start mechanical and flux linkage identification for example)
    }
    break;
    default:
        break;
    }
}
