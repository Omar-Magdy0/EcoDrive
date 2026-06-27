#include "PmsmControl.h"
#include "PmsmControlConf.h"

PmsmControl pmsmC1;
//Static Callback registration
void eldriver_mc3p_sync_postScanCallback(){pmsmC1.mc.pwmLoop();}
void eldriver_xmc3p_tickerCallback(){pmsmC1.mc.xmcLoop();}

using namespace PmsmControlTypes;
using namespace PmsmControlConf;
PmsmControlTypes::ERR PmsmControl::init()
{
    mc.init();
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configControlMode(PmsmControlTypes::MCMode mc_mode, PmsmControlTypes::ElecMode elec_mode)
{
    if(mc_mode != MCMode::Idle && mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    if(elec_mode != ElecMode::Current && elec_mode != ElecMode::Voltage){return ERR::CONFIG_BAD;}
    mc.setControlMode(mc_mode, elec_mode);
    return ERR::OK;
}


PmsmControlTypes::ERR PmsmControl::configElecLimits(PmsmControlTypes::ConfigElecLimits elim)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configMechLimits(PmsmControlTypes::ConfigMechLimits mechlim)
{
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
    mc.pwmConfigUpdate(cfg);
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
    arm_pid_init_q31(&mc.foc.state.Id_pid, 0);
    arm_pid_init_q31(&mc.foc.state.Iq_pid, 0);
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
    if(cfg.elec_mode != ElecMode::Current && cfg.elec_mode != ElecMode::Voltage){return ERR::CONFIG_BAD;}
    if(cfg.vbus_init <= 0){return ERR::CONFIG_BAD;}
    mc.foc.olstup.cfg = cfg;
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configTrapPid(PmsmControlTypes::Pid Ibus_pid)
{
    //Sanity checks
    if(Ibus_pid.Kp > 1 || Ibus_pid.Kp < -1){return ERR::CONFIG_BAD;}
    if(Ibus_pid.Ki > 1 || Ibus_pid.Ki < -1){return ERR::CONFIG_BAD;}
    if(Ibus_pid.Kd > 1 || Ibus_pid.Kd < -1){return ERR::CONFIG_BAD;}
    //fixed point scaling and conversion
    mc.trap.state.I_pid.Kp = Ibus_pid.Kp * INT32_MAX;
    mc.trap.state.I_pid.Ki = Ibus_pid.Ki * INT32_MAX;
    mc.trap.state.I_pid.Kd = Ibus_pid.Kd * INT32_MAX;
    //Atomic operation , We do this quick
    arm_pid_init_q31(&mc.trap.state.I_pid, 0);
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configTrapOlstup(PmsmControlTypes::ConfigOlstup cfg)
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
    if(cfg.elec_mode != ElecMode::Current && cfg.elec_mode != ElecMode::Voltage){return ERR::CONFIG_BAD;}
    if(cfg.vbus_init <= 0){return ERR::CONFIG_BAD;}
    if(cfg.init_sector < ELDRIVER_MC3P_SECTOR_TRAP1 || cfg.init_sector > ELDRIVER_MC3P_SECTOR_TRAP6){return ERR::CONFIG_BAD;}
    mc.trap.olstup.cfg = cfg;
    return ERR::OK;
}

PmsmControlTypes::ERR PmsmControl::configSComm(PmsmControlTypes::ConfigSComm cfg)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    if(cfg.dc_vinj <= 0  || cfg.dc_vinj > ELDRIVER_MC3P_VS_SCALE){return ERR::CONFIG_BAD;}
    if(cfg.hfi_vinj <= 0 || cfg.hfi_vinj > ELDRIVER_MC3P_VS_SCALE){return ERR::CONFIG_BAD;}
    mc.scomm.hfi_N = cfg.hfi_N;
    mc.scomm.dc_vinj_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(cfg.dc_vinj);
    mc.scomm.hfi_vinj_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(cfg.hfi_vinj);
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
