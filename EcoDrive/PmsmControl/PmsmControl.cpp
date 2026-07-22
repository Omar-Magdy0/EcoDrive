#include "PmsmControl.h"
#include "PmsmControlConf.h"

PmsmControl pmsmC1;
//Static Callback registration
void el_mc3p_sync_postScanCallback(){pmsmC1.mc.pwmLoop();}
void el_xmc3p_tickerCallback(){pmsmC1.mc.xmcLoop();}

using namespace PmsmControlTypes;
using namespace PmsmControlConf;
PmsmControlTypes::ERR PmsmControl::init()
{
    mc.init();
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::setSetpoint(float sp, PmsmControlTypes::MechMode mech_mode)
{
    if(mech_mode != PmsmControlTypes::MechMode::SKIP)
    {
        if(mech_mode < PmsmControlTypes::MechMode::Torque || mech_mode > PmsmControlTypes::MechMode::OpenSpeed){return ERR::CONFIG_BAD;}
        mc.control.mech_mode = mech_mode;
    }
    switch (mc.control.mech_mode)
    {
    case PmsmControlTypes::MechMode::Speed:
    case PmsmControlTypes::MechMode::OpenSpeed:
        mc.state.mechAngv_RPXT_sp_q7p24 = mc.rps_to_rpxt(sp);
        break;
    case PmsmControlTypes::MechMode::Position:
        break;
    case PmsmControlTypes::MechMode::Torque:
        break;
    default:
        break;
    }
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configControlMode(PmsmControlTypes::MCMode mc_mode, PmsmControlTypes::MechMode mech_mode)
{
    if(mc_mode != MCMode::Idle && mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    uint32_t mask = el_atomic_start();
    mc.setControlMode(mc_mode);
    mc.control.mech_mode = mech_mode;
    el_atomic_end(mask);
    return ERR::OK;
}


PmsmControlTypes::ERR PmsmControl::configElecLimits(PmsmControlTypes::ConfigElecLimits elim)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configMechLimits(PmsmControlTypes::ConfigMechLimits mechlim)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configPwm(PmsmControlTypes::ConfigPwm cfg)
{
    //if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    if(cfg.pwm_freq_hz > MAX_PWM_FREQUENCY || cfg.pwm_freq_hz < MIN_PWM_FREQUENCY){return ERR::CONFIG_BAD;}
    if(cfg.deadtime_ns > MAX_DEADTIME_NS || cfg.deadtime_ns < MIN_DEADTIME_NS){return ERR::CONFIG_BAD;}
    if(cfg.duty_max < 0 || cfg.duty_max > PmsmControlConf::PWM_MAX_DUTY){return ERR::CONFIG_BAD;}
    if(cfg.duty_min < 0 || cfg.duty_min > 1){return ERR::CONFIG_BAD;}
    if(cfg.duty_min > cfg.duty_max){return ERR::CONFIG_BAD;}
    //Atomic operation we do this quick
    uint32_t mask = el_atomic_start();
    mc.pwmConfigUpdate(cfg);
    el_atomic_end(mask);
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configFocPid(PmsmControlTypes::Pid Id_pid, PmsmControlTypes::Pid Iq_pid)
{
    //Sanity Checks
    if(Id_pid.Kp > 1 || Id_pid.Kp < -1){return ERR::CONFIG_BAD;}
    if(Id_pid.Ki > 1 || Id_pid.Ki < -1){return ERR::CONFIG_BAD;}
    if(Id_pid.Kd > 1 || Id_pid.Kd < -1){return ERR::CONFIG_BAD;}
    if(Iq_pid.Kp > 1 || Iq_pid.Kp < -1){return ERR::CONFIG_BAD;}
    if(Iq_pid.Ki > 1 || Iq_pid.Ki < -1){return ERR::CONFIG_BAD;}
    if(Iq_pid.Kd > 1 || Iq_pid.Kd < -1){return ERR::CONFIG_BAD;}
    //fixed point scaling and conversion
    mc.foc.state.Id_pid.Kp = Id_pid.Kp * INT32_MAX;
    mc.foc.state.Id_pid.Ki = Id_pid.Ki * INT32_MAX;
    mc.foc.state.Id_pid.Kd = Id_pid.Kd * INT32_MAX;
    mc.foc.state.Iq_pid.Kp = Iq_pid.Kp * INT32_MAX;
    mc.foc.state.Iq_pid.Ki = Iq_pid.Ki * INT32_MAX;
    mc.foc.state.Iq_pid.Kd = Iq_pid.Kd * INT32_MAX;
    //Atomic operation , We do this quick
    uint32_t mask = el_atomic_start();
    arm_pid_init_q31(&mc.foc.state.Id_pid, 0);
    arm_pid_init_q31(&mc.foc.state.Iq_pid, 0);
    el_atomic_end(mask);
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configFocOlstup(PmsmControlTypes::ConfigOlstup cfg)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    //Table sanity checks
    if(cfg.time_ms_tb[0] != 0){return ERR::CONFIG_BAD;}
    for(int i = 0; i < olstup_tb_size() - 1; i++)
    {
        if(cfg.time_ms_tb[i + 1] < cfg.time_ms_tb[i]){return ERR::CONFIG_BAD;}
    }
    for(int i = 0; i < olstup_tb_size(); i++)
    {
        if(cfg.ec_tb[i] < 0){return ERR::CONFIG_BAD;}
        if(cfg.rpm_tb[i] < 0){return ERR::CONFIG_BAD;}
    }
    if(cfg.vbus_init <= 0){return ERR::CONFIG_BAD;}
    mc.foc.olstup.cfg = cfg;
    return ERR::OK;
}



PmsmControlTypes::ERR PmsmControl::configSComm(PmsmControlTypes::ConfigSComm cfg)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    if(cfg.dc_vinj <= 0  || cfg.dc_vinj > EL_MC3P_VS_SCALE){return ERR::CONFIG_BAD;}
    if(cfg.hfi_vinj <= 0 || cfg.hfi_vinj > EL_MC3P_VS_SCALE){return ERR::CONFIG_BAD;}
    mc.scomm.hfi_N = cfg.hfi_N;
    mc.scomm.dc_vinj_q31 = EL_MC3P_FLOAT_TO_VS(cfg.dc_vinj);
    mc.scomm.hfi_vinj_q31 = EL_MC3P_FLOAT_TO_VS(cfg.hfi_vinj);
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::getFault() const
{
    return mc.fault;
}

PmsmControlTypes::ERR PmsmControl::setOnFault(void (*onFault)(void *))
{
    if(!onFault){return ERR::CONFIG_BAD;}
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::clearFault()
{
    mc.fault = ERR::OK;
    return ERR::OK;
}
