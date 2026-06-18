#include "PmsmControl.h"




void PmsmControl::Foc_init()
{
    //Foc_olstup_start();
    //control.mctype = MCType::Foc;
}
void PmsmControl::Foc_onEnter(MCType prev_mct)
{

}
void PmsmControl::Foc_onExit()
{

}
void PmsmControl::Foc_olstup_cfg(float vbus_init, float time_tb[STUP_TABLE_SIZE], float voltage_tb[STUP_TABLE_SIZE], float rpm_tb[STUP_TABLE_SIZE])
{
    if (state.Vbus_q31 == 0)
    {
        state.Vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(vbus_init);
    }
    for (int i = 0; i < STUP_TABLE_SIZE; i++)
    {
        foc.olstup.time_tb[i] = time_tb[i];
        foc.olstup.voltage_tb[i] = voltage_tb[i];
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
    foc.state.run_mode = Foc::RunMode::OL;
}

void PmsmControl::Foc_pwmLoop()
{
    state.Vbus_q31 = mc3p_sync_meas.svm.vbus_q31;
    state.eTheta_q31 += state.eAngv_RPT_q31;
    q31_t ed, eq;
    q31_t sin,cos;
    q15_t valpha_q15, vbeta_q15, vd_q15, vq_q15;
    q31_t Ialpha_q31, Ibeta_q31;
    q31_t valpha_q31, vbeta_q31;
    arm_sin_cos_q31(state.eTheta_q31, &sin, &cos);
    if(control.ectype == ECType::Current)
    {
        arm_clarke_q31(mc3p_sync_meas.svm.cu_q31, mc3p_sync_meas.svm.cv_q31, &Ialpha_q31, &Ibeta_q31);
        arm_park_q31(Ialpha_q31, Ibeta_q31, &foc.state.Iq_q31, &foc.state.Iq_q31, sin, cos);
        ed = foc.state.ECd_sp_q31 - foc.state.Id_q31;
        eq = foc.state.ECq_sp_q31 - foc.state.Iq_q31;
    }else{
        ed = 0;
        eq = 0;
        foc.state.Id_feedforward = foc.state.ECd_sp_q31 >> 16;
        foc.state.Iq_feedforward = foc.state.ECq_sp_q31 >> 16;
    }
    ed = ed >> 16;
    eq = eq >> 16;
    vd_q15 = arm_pid_q15(&foc.state.Id_pid, ed) + foc.state.Id_feedforward;
    vq_q15 = arm_pid_q15(&foc.state.Iq_pid, eq) + foc.state.Iq_feedforward;
    foc.state.vd_q31 = (q31_t)vd_q15 << 16;
    foc.state.vq_q31 = (q31_t)vq_q15 << 16;
    arm_inv_park_q31(foc.state.vd_q31, foc.state.vq_q31, &valpha_q31, &vbeta_q31, sin, cos);
    valpha_q15 = valpha_q31 >> 16;
    vbeta_q15 = vbeta_q31 >> 16;
    eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
}

void PmsmControl::Foc_xmcLoop()
{

}