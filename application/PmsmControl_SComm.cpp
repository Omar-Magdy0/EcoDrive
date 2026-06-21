#include "PmsmControl.h"
// Builds upon FOC to run, tune and identify parameters

void PmsmControl::SComm_init()
{
}
void PmsmControl::SComm_onEnter(MCType prev_mct)
{
    control.ectype = ECType::Voltage; // Voltage control mode for now
    scomm.idstage = SComm::IDStage::RS_ID;
}

void PmsmControl::SComm_onExit()
{
}

void PmsmControl::SComm_pwmLoop()
{
    // Hfi added logic here......
    switch (scomm.idstage)
    {
    case SComm::IDStage::RS_ID:
        if (scomm.samples_counter && scomm.idsub == SComm::IDSubStage::Active_Sampling)
        {
            scomm.accumulate0 += foc.state.Id_q31;
            scomm.samples_counter = scomm.samples_counter - 1;
        }
        break;
    case SComm::IDStage::LD_ID:
        if (scomm.samples_counter)
        {
            q31_t sin, cos;
            arm_sin_cos_q31(scomm.hfi_angle_q31, &sin, &cos);
            foc.state.ECd_sp_q31 = ((int64_t)scomm.hfi_vinj_q31 * sin) >> 31;
            foc.state.ECq_sp_q31 = 0;
            // Heterodyne the read current.....
            q31_t hfid = ((int64_t)foc.state.Id_q31 * sin) >> 31; // Since they are multiplied by half already
            q31_t hfiq = ((int64_t)foc.state.Id_q31 * cos) >> 31; // Since they are multiplied by half already
            scomm.hfid_q31 = ((scomm.hfi_alpha) * (int64_t)hfid + (INT32_MAX - scomm.hfi_alpha) * (int64_t)scomm.hfid_q31) >> 31;
            scomm.hfiq_q31 = ((scomm.hfi_alpha) * (int64_t)hfiq + (INT32_MAX - scomm.hfi_alpha) * (int64_t)scomm.hfiq_q31) >> 31;
            if (scomm.idsub == SComm::IDSubStage::Active_Sampling)
            {
                scomm.accumulate0 += scomm.hfid_q31;
                scomm.accumulate1 += scomm.hfiq_q31;
                scomm.samples_counter = scomm.samples_counter - 1;
            }
            scomm.hfi_angle_q31 += scomm.hfi_Angv_RPT_q31;
        }
    case SComm::IDStage::LQ_ID:
        if (scomm.samples_counter)
        {
            q31_t sin, cos;
            arm_sin_cos_q31(scomm.hfi_angle_q31, &sin, &cos);
            foc.state.ECd_sp_q31 = 0;
            foc.state.ECq_sp_q31 = ((int64_t)scomm.hfi_vinj_q31 * sin) >> 31;
            // Heterodyne the read current.....
            q31_t hfid = ((int64_t)foc.state.Iq_q31 * sin) >> 30; // Since they are multiplied by half already
            q31_t hfiq = ((int64_t)foc.state.Iq_q31 * cos) >> 30; // Since they are multiplied by half already
            scomm.hfid_q31 = ((scomm.hfi_alpha) * (int64_t)hfid + (INT32_MAX - scomm.hfi_alpha) * (int64_t)scomm.hfid_q31) >> 31;
            scomm.hfiq_q31 = ((scomm.hfi_alpha) * (int64_t)hfiq + (INT32_MAX - scomm.hfi_alpha) * (int64_t)scomm.hfiq_q31) >> 31;
            if (scomm.idsub == SComm::IDSubStage::Active_Sampling)
            {
                scomm.accumulate0 += scomm.hfid_q31;
                scomm.accumulate1 += scomm.hfiq_q31;
                scomm.samples_counter = scomm.samples_counter - 1;
            }
            scomm.hfi_angle_q31 += scomm.hfi_Angv_RPT_q31;
        }
    default:
        break;
    }
    Foc_pwmLoop();
}

void PmsmControl::SComm_xmcLoop()
{
    // Go step by step into identification process...
    // First Identify DC resistance
    switch (scomm.idstage)
    {
    case SComm::IDStage::RS_ID:
        // Apply constant dc voltage on d-axis
        // Force angle and speed to 0
        state.eTheta_q31 = 0;
        state.eAngv_RPS = 0;
        foc.state.ECd_sp_q31 = scomm.dc_vinj_q31;
        foc.state.ECq_sp_q31 = 0;
        // wait for electrical settling
        switch (scomm.idsub)
        {
        case SComm::IDSubStage::ESettle_Start:
            scomm.eSettle_start_tick = pwmTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pwmTicks - scomm.eSettle_start_tick > scomm.eSettle_min_ticks)
            {
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                scomm.R = ELDRIVER_MC3P_VS_TO_FLOAT(scomm.dc_vinj_q31) / ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate0 >> scomm.oversample_bits));
                scomm.idstage = SComm::IDStage::LD_ID;
                scomm.idsub = SComm::IDSubStage::ESettle_Start;
            }
            break;
        default:
            break;
        }
        break;
    case SComm::IDStage::LD_ID:
        state.eTheta_q31 = 0;
        state.eAngv_RPS = 0;
        // wait for electrical settling
        switch (scomm.idsub)
        {
        case SComm::IDSubStage::ESettle_Start:
            scomm.hfi_Angv_RPT_q31 = q31_t((scomm.hfi_Angv_RPS / pwm_freq_hz) * (INT32_MAX / M_PI));
            scomm.eSettle_start_tick = pwmTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.accumulate1 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pwmTicks - scomm.eSettle_start_tick > scomm.eSettle_min_ticks)
            {
                scomm.idsub = SComm::IDSubStage::LPFSettle_Start;
            }
            break;
        case SComm::IDSubStage::LPFSettle_Start:
            if (pwmTicks - scomm.eSettle_start_tick > scomm.eSettle_min_ticks)
            {
                scomm.eSettle_start_tick = pwmTicks;
                scomm.idsub = SComm::IDSubStage::LPFSettle_Wait;
            }
            break;
        case SComm::IDSubStage::LPFSettle_Wait:
            if (pwmTicks - scomm.eSettle_start_tick > (scomm.eSettle_min_ticks > 1))
            {
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                scomm.Idd = ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate0 >> (scomm.oversample_bits) - 1));
                scomm.Idq = ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate1 >> (scomm.oversample_bits) - 1));
                scomm.idsub = SComm::IDSubStage::ESettle_Start;
                scomm.idstage = SComm::IDStage::LQ_ID;
            }
            break;
        default:
            break;
        }
    case SComm::IDStage::LQ_ID:
        state.eTheta_q31 = 0;
        state.eAngv_RPS = 0;
        // wait for electrical settling
        switch (scomm.idsub)
        {
        case SComm::IDSubStage::ESettle_Start:
            scomm.hfi_Angv_RPT_q31 = q31_t((scomm.hfi_Angv_RPS / pwm_freq_hz) * (INT32_MAX / M_PI));
            scomm.eSettle_start_tick = pwmTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.accumulate1 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pwmTicks - scomm.eSettle_start_tick > scomm.eSettle_min_ticks)
            {
                scomm.idsub = SComm::IDSubStage::LPFSettle_Start;
            }
            break;
        case SComm::IDSubStage::LPFSettle_Start:
            if (pwmTicks - scomm.eSettle_start_tick > scomm.eSettle_min_ticks)
            {
                scomm.eSettle_start_tick = pwmTicks;
                scomm.idsub = SComm::IDSubStage::LPFSettle_Wait;
            }
            break;
        case SComm::IDSubStage::LPFSettle_Wait:
            if (pwmTicks - scomm.eSettle_start_tick > (scomm.eSettle_min_ticks > 1))
            {
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                scomm.Iqd = ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate0 >> (scomm.oversample_bits - 1) ));
                scomm.Iqq = ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate1 >> (scomm.oversample_bits - 1) ));
            }
            break;
        default:
            break;
        }
    default:
        break;
    }
    state.eAngv_RPT_q31 = q31_t((state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX / M_PI));
}
