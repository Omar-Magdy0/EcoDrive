#include "PmsmControl.h"

constexpr q31_t a0 = (127/128.0)*INT32_MAX;
constexpr q31_t a1 = (27/32.0)*INT32_MAX;
constexpr q31_t b0 = (3/16.0)*INT32_MAX;
constexpr q31_t b1 = (71/128.0)*INT32_MAX;
constexpr q15_t SQRT3_Q3P12 = ((int32_t)(1.73205080757*INT16_MAX)>>3);

static inline q31_t maxa_minb_2_q31(q31_t i1,q31_t i2)
{
    q31_t max,min;
    i1 = i1<0?-i1:i1;
    i2 = i2<0?-i2:i2;
    if(i1>i2)
    {
        max = i1;
        min = i2;
    }else{
        max = i2;
        min = i1;
    }
    q31_t z0 = (((int64_t)a0*max + (int64_t)b0*min)>>31);
    q31_t z1 = (((int64_t)a1*max + (int64_t)b1*min)>>31);
    return z0>z1?z0:z1;
}



void PmsmControl::Foc_init()
{

}
void PmsmControl::Foc_onEnter(MCType prev_mct)
{

}
void PmsmControl::Foc_onExit()
{

}
void PmsmControl::Foc_olstup_cfg(float vbus_init,const float time_tb[Foc::Foc::OLSTUP_TABLE_SIZE],const float ec_tb[Foc::Foc::OLSTUP_TABLE_SIZE],const float rpm_tb[Foc::Foc::OLSTUP_TABLE_SIZE])
{
    if (state.Vbus_q31 == 0)
    {
        state.Vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(vbus_init);
    }
    for (int i = 0; i < Foc::Foc::OLSTUP_TABLE_SIZE; i++)
    {
        foc.olstup.time_tb[i] = time_tb[i];
        foc.olstup.EC_tb[i] = ec_tb[i];
        foc.olstup.eAngv_RPS_tb[i] = (rpm_tb[i] * 2 * M_PI / 60) * model.pole_pairs;
    }
}
void PmsmControl::Foc_olstup_start()
{
    foc.olstup.tb_index = 0;
    foc.olstup.time_start_us = ticks_to_us(pwmTicks);
    //Apply Align here..... lets say Set electrical angle to 0
    state.eTheta_q31 = 0;
    foc.olstup.complete = false;
    foc.run_mode = Foc::RunMode::OL;
}

void PmsmControl::Foc_pwmLoop()
{
    state.Vbus_q31 = mc3p_sync_meas.svm.vbus_q31;
    state.eTheta_q31 += state.eAngv_RPT_q31;
    q31_t ed, eq;
    q31_t sin,cos;
    q31_t Ialpha_q31, Ibeta_q31;
    q31_t valpha_q31, vbeta_q31;
    arm_sin_cos_q31(state.eTheta_q31, &sin, &cos);
    arm_clarke_q31(mc3p_sync_meas.svm.cu_q31, mc3p_sync_meas.svm.cv_q31, &Ialpha_q31, &Ibeta_q31);
    arm_park_q31(Ialpha_q31, Ibeta_q31, &foc.state.Id_q31, &foc.state.Iq_q31, sin, cos);
    if(control.ectype == ECType::Current)
    {
        ed = foc.state.ECd_sp_q31 - foc.state.Id_q31;
        eq = foc.state.ECq_sp_q31 - foc.state.Iq_q31;
    }else{
        ed = 0;
        eq = 0;
        foc.state.Id_pid.state[2] = foc.state.ECd_sp_q31;
        foc.state.Iq_pid.state[2] = foc.state.ECq_sp_q31;
    }
    ed = ed >> 2;
    eq = eq >> 2;
    foc.state.vd_q31 = arm_pid_q31(&foc.state.Id_pid, ed) + foc.state.Id_feedforward;
    foc.state.vq_q31 = arm_pid_q31(&foc.state.Iq_pid, eq) + foc.state.Iq_feedforward;
    //Circular overmodulation clamping + pid state update (anti-windup)
    //Here overmodulation logic is intended
    q31_t vmag_q31 = maxa_minb_2_q31(foc.state.vd_q31, foc.state.vq_q31);
    int32_t mod_idx_q3p12 = (((int32_t)(vmag_q31>>16) * SQRT3_Q3P12)/(state.Vbus_q31>>16));
    if(mod_idx_q3p12 > foc.state.mod_idx_max_q3p12)
    {
        //scale both Vd and Vq down 
        foc.state.vd_q31 =  (foc.state.vd_q31 >> 12) * (((int32_t)foc.state.mod_idx_max_q3p12 * (int16_t)((1<<12) - 1))/mod_idx_q3p12);
        foc.state.vq_q31 =  (foc.state.vq_q31 >> 12) * (((int32_t)foc.state.mod_idx_max_q3p12 * (int16_t)((1<<12) - 1))/mod_idx_q3p12);
        foc.state.Id_pid.state[2] = foc.state.vd_q31;
        foc.state.Iq_pid.state[2] = foc.state.vq_q31;
    }
    q15_t dalpha_q15, dbeta_q15;
    arm_inv_park_q31(foc.state.vd_q31, foc.state.vq_q31, &valpha_q31, &vbeta_q31, sin, cos);
    dalpha_q15 = (valpha_q31) / (state.Vbus_q31 >> 15);
    dbeta_q15 = (vbeta_q31)/ (state.Vbus_q31 >> 15);
    eldriver_mc3p_write_svm(&mc3p, dalpha_q15, dbeta_q15);
}

void PmsmControl::Foc_xmcLoop()
{
    if(foc.run_mode == Foc::RunMode::OL)
    {//Open loop operation
        if (!foc.olstup.complete)
        {//Startup
            // Interpolate angular velocity and duty cycle and do appropiate updates
            float et = (ticks_to_us(pwmTicks) - foc.olstup.time_start_us)/1'000'000;
            if (foc.olstup.tb_index < (Foc::OLSTUP_TABLE_SIZE - 1) && et > foc.olstup.time_tb[foc.olstup.tb_index + 1])
            {
                foc.olstup.tb_index++;
            }
            bool complete = et >= foc.olstup.time_tb[Foc::OLSTUP_TABLE_SIZE - 1];
            if (!complete)
            {
                // We interpolate here for duty cycle and angular velocity
                float ret_slp = (float)(et - foc.olstup.time_tb[foc.olstup.tb_index]) / (foc.olstup.time_tb[foc.olstup.tb_index + 1] - foc.olstup.time_tb[foc.olstup.tb_index]);
                float ECq_sp = foc.olstup.EC_tb[foc.olstup.tb_index] + ret_slp * (foc.olstup.EC_tb[foc.olstup.tb_index + 1] - foc.olstup.EC_tb[foc.olstup.tb_index]);
                state.eAngv_RPS = foc.olstup.eAngv_RPS_tb[foc.olstup.tb_index] + ret_slp * (foc.olstup.eAngv_RPS_tb[foc.olstup.tb_index + 1] - foc.olstup.eAngv_RPS_tb[foc.olstup.tb_index]);
                if(control.ectype == ECType::Voltage){
                    foc.state.ECq_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(ECq_sp);
                }else{
                    foc.state.ECq_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(ECq_sp);
                }
            }
            foc.olstup.complete = complete;
        }else{//Normal openloop
            // TODO : SUPPORT COMMANDING VOLTAGE , CURRENT AND FREQUENCY AND DOING NECCESSARY UPDATES


        }
    }else{//Closed loop operation
        //Torque, Speed, Position Controllers

    }
    state.eAngv_RPT_q31 = q31_t( (state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX/M_PI) );
}

