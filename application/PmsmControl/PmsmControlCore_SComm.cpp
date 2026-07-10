#include "PmsmControlCore.h"
// Builds upon FOC to run, tune and identify parameters
using namespace PmsmControlTypes;
void PmsmControlCore::SComm_init()
{
    
}
void PmsmControlCore::SComm_onEnter(MCMode prev_mct)
{
    scomm.idstage = SComm::IDStage::RS_ID;
    control.run_mode = RunMode::Override;
    control.elec_mode = ElecMode::Voltage;
}

void PmsmControlCore::SComm_onExit()
{
}

void PmsmControlCore::SComm_pwmLoop()
{
    q31_t sin, cos;
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
        arm_sin_cos_q31(scomm.hfi_angle_q31, &sin, &cos);
        foc.state.ECd_sp_q31 = ((int64_t)scomm.hfi_vinj_q31 * sin) >> 31;
        foc.state.ECq_sp_q31 = 0;
        if (scomm.samples_counter && scomm.idsub == SComm::IDSubStage::Active_Sampling)
        {
            // Heterodyne the read current.....
            q31_t hfid = ((int64_t)foc.state.Id_q31 * sin) >> 31;
            q31_t hfiq = ((int64_t)foc.state.Id_q31 * cos) >> 31;
            scomm.accumulate0 += hfid;
            scomm.accumulate1 += hfiq;
            scomm.samples_counter = scomm.samples_counter - 1;
        }
        scomm.hfi_angle_q31 += scomm.hfi_Angv_RPT_q31;
        break;
    case SComm::IDStage::LQ_ID:
        arm_sin_cos_q31(scomm.hfi_angle_q31, &sin, &cos);
        foc.state.ECd_sp_q31 = 0;
        foc.state.ECq_sp_q31 = ((int64_t)scomm.hfi_vinj_q31 * sin) >> 31;
        if (scomm.samples_counter && scomm.idsub == SComm::IDSubStage::Active_Sampling)
        {
            // Heterodyne the read current.....
            q31_t hfid = ((int64_t)foc.state.Iq_q31 * sin) >> 31;
            q31_t hfiq = ((int64_t)foc.state.Iq_q31 * cos) >> 31;
            scomm.accumulate0 += hfid;
            scomm.accumulate1 += hfiq;
            scomm.samples_counter = scomm.samples_counter - 1;
        }
        scomm.hfi_angle_q31 += scomm.hfi_Angv_RPT_q31;
    default:
        break;
    }
    Foc_pwmLoop();
}

void PmsmControlCore::SComm_xmcLoop()
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
            scomm.eSettle_start_tick = pTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pTicks - scomm.eSettle_start_tick > PmsmControlConf::ESETTLE_MIN_TICKS)
            {
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                scomm.model.R = ELDRIVER_MC3P_VS_TO_FLOAT(scomm.dc_vinj_q31) / ELDRIVER_MC3P_CS_TO_FLOAT((scomm.accumulate0 >> PmsmControlConf::OVERSAMPLE_BITS));
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
            scomm.eSettle_start_tick = pTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.accumulate1 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pTicks - scomm.eSettle_start_tick > PmsmControlConf::ESETTLE_MIN_TICKS)
            {
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                q31_t Idd = scomm.accumulate0 >> PmsmControlConf::OVERSAMPLE_BITS;
                q31_t Idq = scomm.accumulate1 >> PmsmControlConf::OVERSAMPLE_BITS;
                q31_t Imag2 = (((int64_t)Idd*Idd) + ((int64_t)Idq*Idq))>>31; 
                q31_t Imag;
                arm_sqrt_q31(Imag2, &Imag);
                float Zd = voltage_q31_to_float(scomm.hfi_vinj_q31)/(current_q31_to_float(2*Imag));
                float Xd = sqrt(Zd*Zd - scomm.model.R*scomm.model.R);
                scomm.model.Ld = (Xd)/(angle_q31_to_float(scomm.hfi_Angv_RPT_q31)*pwm_freq_hz);
                scomm.idsub = SComm::IDSubStage::ESettle_Start;
                scomm.idstage = SComm::IDStage::REST0;
            }
            break;
        default:
            break;
        }
        // Fall-through to REST0 intended
    case SComm::IDStage::REST0:
    {
        switch (scomm.idsub)
        {
        case SComm::IDSubStage::ESettle_Start:
            scomm.eSettle_start_tick = pTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.accumulate1 = 0;
            scomm.samples_counter = scomm.oversample;
            state.eTheta_q31 = 0;
            state.eAngv_RPS = 0;
            foc.state.ECd_sp_q31 = 0;
            foc.state.ECq_sp_q31 = 0;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pTicks - scomm.eSettle_start_tick > PmsmControlConf::ESETTLE_MIN_TICKS)
            {
                scomm.idsub = SComm::IDSubStage::ESettle_Start;
                scomm.idstage = SComm::IDStage::LQ_ID;
            }
            break;
        default:
            break;
        }
    }
    case SComm::IDStage::LQ_ID:
        state.eTheta_q31 = 0;
        state.eAngv_RPS = 0;
        // wait for electrical settling
        switch (scomm.idsub)
        {
        case SComm::IDSubStage::ESettle_Start:
            scomm.eSettle_start_tick = pTicks;
            scomm.idsub = SComm::IDSubStage::ESettle_Wait;
            scomm.accumulate0 = 0;
            scomm.accumulate1 = 0;
            scomm.samples_counter = scomm.oversample;
            break;
        case SComm::IDSubStage::ESettle_Wait:
            if (pTicks - scomm.eSettle_start_tick > PmsmControlConf::ESETTLE_MIN_TICKS)
            {
                scomm.eSettle_start_tick = pTicks;
                scomm.idsub = SComm::IDSubStage::Active_Sampling;
            }
            break;
        case SComm::IDSubStage::Active_Sampling:
            if (scomm.samples_counter == 0)
            {
                q31_t Iqd = scomm.accumulate0 >> (PmsmControlConf::OVERSAMPLE_BITS);
                q31_t Iqq = scomm.accumulate1 >> (PmsmControlConf::OVERSAMPLE_BITS);
                q31_t Imag2 = (((int64_t)Iqd*Iqd) + ((int64_t)Iqq*Iqq))>>31; 
                q31_t Imag;
                arm_sqrt_q31(Imag2, &Imag);
                float Zq = voltage_q31_to_float(scomm.hfi_vinj_q31)/(current_q31_to_float(2*Imag));
                float Xq = sqrt(Zq*Zq - scomm.model.R*scomm.model.R);
                scomm.model.Lq = (Xq)/(angle_q31_to_float(scomm.hfi_Angv_RPT_q31)*pwm_freq_hz);
                scomm.idsub = SComm::IDSubStage::ESettle_Start;
                scomm.idstage = SComm::IDStage::RS_ID;
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
