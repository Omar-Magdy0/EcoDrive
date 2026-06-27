#include "PmsmControlCore.h"

constexpr q31_t a0 = (127/128.0)*INT32_MAX;
constexpr q31_t a1 = (27/32.0)*INT32_MAX;
constexpr q31_t b0 = (3/16.0)*INT32_MAX;
constexpr q31_t b1 = (71/128.0)*INT32_MAX;
constexpr q15_t SQRT3_Q3P12 = ((int32_t)(1.73205080757*INT16_MAX)>>3);

using namespace PmsmControlTypes;
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

void PmsmControlCore::Foc_init()
{

}
void PmsmControlCore::Foc_onEnter(MCMode prev_mct)
{

}
void PmsmControlCore::Foc_onExit()
{

}

void PmsmControlCore::Foc_olstup_start()
{
    foc.olstup.tb_index = 0;
    foc.olstup.time_start_ms = xTicks_to_ms(xTicks);
    //Apply Align here..... lets say Set electrical angle to 0
    state.eTheta_q31 = 0;
    foc.olstup.complete = false;
    foc.run_mode = Foc::RunMode::OL;
}

void PmsmControlCore::Foc_pwmLoop()
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
    int64_t temp0 = (((int64_t)foc.state.Id_q31*foc.state.Vd_q31 + (int64_t)foc.state.Iq_q31*foc.state.Vq_q31)>>1)/(state.Vbus_q31);
    state.Ibus_q31 = temp0*3;

    if(control.elec_mode == ElecMode::Current)
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
    foc.state.Vd_q31 = arm_pid_q31(&foc.state.Id_pid, ed) + foc.state.Id_feedforward;
    foc.state.Vq_q31 = arm_pid_q31(&foc.state.Iq_pid, eq) + foc.state.Iq_feedforward;
    //Circular overmodulation clamping + pid state update (anti-windup)
    //Here overmodulation logic is intended
    q31_t vmag_q31 = maxa_minb_2_q31(foc.state.Vd_q31, foc.state.Vq_q31);
    int32_t vbus_lim_q31 = (state.Vbus_q31 * mc3p.duty_max_q15)>>15;
    int32_t mod_idx_q3p12 = (((int32_t)(vmag_q31>>16) * SQRT3_Q3P12)/(vbus_lim_q31>>16));
    if(mod_idx_q3p12 > foc.state.mod_idx_max_q3p12)
    {
        //scale both Vd and Vq down 
        foc.state.Vd_q31 =  (foc.state.Vd_q31 >> 12) * (((int32_t)foc.state.mod_idx_max_q3p12<<12)/mod_idx_q3p12);
        foc.state.Vq_q31 =  (foc.state.Vq_q31 >> 12) * (((int32_t)foc.state.mod_idx_max_q3p12<<12)/mod_idx_q3p12);
        foc.state.Id_pid.state[2] = foc.state.Vd_q31;
        foc.state.Iq_pid.state[2] = foc.state.Vq_q31;
    }
    q15_t dalpha_q15, dbeta_q15;
    arm_inv_park_q31(foc.state.Vd_q31, foc.state.Vq_q31, &valpha_q31, &vbeta_q31, sin, cos);
    dalpha_q15 = (valpha_q31) / (state.Vbus_q31 >> 15);
    dbeta_q15 = (vbeta_q31)/ (state.Vbus_q31 >> 15);
    eldriver_mc3p_write_svm(&mc3p, dalpha_q15, dbeta_q15);
}

void PmsmControlCore::Foc_xmcLoop()
{
    if(foc.run_mode == Foc::RunMode::OL)
    {//Open loop operation
        if (!foc.olstup.complete)
        {//Startup
            // Interpolate angular velocity and duty cycle and do appropiate updates
            float et_ms = (xTicks_to_ms(xTicks) - foc.olstup.time_start_ms);
            if (foc.olstup.tb_index < (OLSTUP_TABLE_SIZE - 1) && et_ms > foc.olstup.cfg.time_ms_tb[foc.olstup.tb_index + 1])
            {
                foc.olstup.tb_index++;
            }
            bool complete = et_ms >= foc.olstup.cfg.time_ms_tb[OLSTUP_TABLE_SIZE - 1];
            if (!complete)
            {
                // We interpolate here for duty cycle and angular velocity
                float ret_slp = (float)(et_ms - foc.olstup.cfg.time_ms_tb[foc.olstup.tb_index]) / (foc.olstup.cfg.time_ms_tb[foc.olstup.tb_index + 1] - foc.olstup.cfg.time_ms_tb[foc.olstup.tb_index]);
                float ecq_sp = foc.olstup.cfg.ec_tb[foc.olstup.tb_index] + ret_slp * (foc.olstup.cfg.ec_tb[foc.olstup.tb_index + 1] - foc.olstup.cfg.ec_tb[foc.olstup.tb_index]);
                float rpm = foc.olstup.cfg.rpm_tb[foc.olstup.tb_index] + ret_slp * (foc.olstup.cfg.rpm_tb[foc.olstup.tb_index + 1] - foc.olstup.cfg.rpm_tb[foc.olstup.tb_index]);
                state.eAngv_RPS = rpm * model.pole_pairs * (float)(60.0/2*M_PI);
                if(control.elec_mode == ElecMode::Voltage){
                    foc.state.ECq_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(ecq_sp);
                }else{
                    foc.state.ECq_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(ecq_sp);
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

