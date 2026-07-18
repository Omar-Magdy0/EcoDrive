#include "PmsmControlCore.h"

// Fixed-point filter coefficients for current magnitude estimation (max_a_min_b approximation)
constexpr q31_t MAXA_MINB_A0 = (127/128.0)*INT32_MAX;
constexpr q31_t MAXA_MINB_A1 = (27/32.0)*INT32_MAX;
constexpr q31_t MAXA_MINB_B0 = (3/16.0)*INT32_MAX;
constexpr q31_t MAXA_MINB_B1 = (71/128.0)*INT32_MAX;
constexpr q15_t SQRT3_Q3P12 = ((int32_t)(1.73205080757*INT16_MAX)>>3);

// Speed scaling: convert from Q32.31 delta-angle to Q7.24 angular velocity
constexpr int SPEED_FROM_DELTA_SHIFT = 7;

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
    q31_t z0 = (((int64_t)MAXA_MINB_A0*max + (int64_t)MAXA_MINB_B0*min)>>31);
    q31_t z1 = (((int64_t)MAXA_MINB_A1*max + (int64_t)MAXA_MINB_B1*min)>>31);
    return z0>z1?z0:z1;
}

void PmsmControlCore::Foc_init()
{

}
void PmsmControlCore::Foc_onEnter(MCMode prev_mct)
{
    //Foc_olstup_start();
}
void PmsmControlCore::Foc_onExit()
{

}

void PmsmControlCore::Foc_xmcLoop()
{
    posDriver.xTickUpdate();
    int64_t Theta_q32p31 = posDriver.getMechAng_q31p32();
    state.mechAngv_RPXT_q7p24 = int32_t((Theta_q32p31 - state.mechTheta_q32p31)>>7); //we don't saturate here since speed range is humungous and won't go beyond
    state.mechTheta_q32p31 = Theta_q32p31;
    if (control.run_mode == RunMode::ClosedLoop)
    { // Closed loop here , Controllers are active
        int32_t ep_q31 = 0;
        int32_t es_q7p24 = 0;
        if (control.mech_mode == MechMode::Position)
        {
            ep_q31 = __SSAT(state.mechTheta_sp_q32p31 - state.mechTheta_q32p31 , 32); //saturate the position error to be a max of 1 rev (for higher position errors, a profiler should exist for stepping)
        }
        else
        {
            ep_q31 = 0;
            control.position_pid.state[2] = state.mechAngv_RPXT_sp_q7p24;
        }
        state.mechAngv_RPXT_sp_q7p24 = arm_pid_q31(&control.position_pid, ep_q31);
        // Clamping logic here for speed
        if (control.mech_mode == MechMode::Position || control.mech_mode == MechMode::Speed)
        {
            es_q7p24 = state.mechAngv_RPXT_sp_q7p24 - state.mechAngv_RPXT_q7p24;
        }
        else
        {
            es_q7p24 = 0;
            control.speed_pid.state[2] = state.torque_sp;
        }
        // Clamping logic here for Torque based on active controller
        state.torque_sp = arm_pid_q31(&control.speed_pid, es_q7p24);
        //Convert Torque Setpoint To Id/Iq
        if(control.mpta_active)
        {
            foc.state.ECd_sp_q31 = 0;
            foc.state.ECq_sp_q31 = state.torque_sp;
        }else
        {
            foc.state.ECd_sp_q31 = 0;
            foc.state.ECq_sp_q31 = state.torque_sp;
        }
        //Clamping logic here for Id/Iq based on active controller
    }else//Override control pipeline
    {
        
    }
    state.eAngv_RPT_q31 = q31_t((state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX / M_PI));
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
    foc.state.Vd_q31 = arm_pid_q31(&foc.state.Id_pid, ed);
    foc.state.Vq_q31 = arm_pid_q31(&foc.state.Iq_pid, eq);
    //Circular overmodulation clamping + pid state update (anti-windup)
    //Here overmodulation logic is intended
    q31_t vmag_q31 = maxa_minb_2_q31(foc.state.Vd_q31, foc.state.Vq_q31);
    int32_t vbus_lim_q31 = ((int64_t)state.Vbus_q31 * mc3p.duty_max_q15)>>15;
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
    el_mc3p_write_svm(&mc3p, dalpha_q15, dbeta_q15);
}