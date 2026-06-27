#include "PmsmControlCore.h"

using namespace PmsmControlTypes;

void PmsmControlCore::init()
{
    pTicks = 0;
    xTicks = 0;
    /* instance bindings */
    
    /* mode initialization*/
    voltage_q31_to_float(state.Vbus_q31);
    current_q31_to_float(state.Ibus_q31);
    angle_q31_to_float(state.eTheta_q31);

    pTick_period_ns = 1'000'000'000.0f / static_cast<float>(pwm_freq_hz);
    mc3p.config.pwm_Hz = pwm_freq_hz;
    mc3p.offset_calibration = true;
    Trap_init();
    SComm_init();
    Foc_init();
    /* hardware initialization */
    pos_sensor.init(pwm_freq_hz);
    eldriver_mc3p_set_sync_scale(&mc3p, PmsmControlConf::MC3P_SYNC_SCALE);
    eldriver_mc3p_init(&mc3p);
    eldriver_mc3p_write_float(&mc3p);
};
void PmsmControlCore::xmcLoop()
{
    if (control.mc_mode == MCMode::None)
        return;
    switch (control.mc_mode)
    {
    case MCMode::Trap:
        Trap_xmcLoop();
        break;
    case MCMode::Foc:
        Foc_xmcLoop();
        break;
    case MCMode::SComm:
        SComm_xmcLoop();
        break;
    default:
        break;
    }
    xTicks = xTicks + 1;
}

void PmsmControlCore::pwmConfigUpdate(PmsmControlTypes::ConfigPwm cfg)
{
    pwm_freq_hz = cfg.pwm_freq_hz;
    pTick_period_ns = 1'000'000'000.0f / static_cast<float>(pwm_freq_hz);
    mc3p.config.pwm_Hz = pwm_freq_hz;
    mc3p.config.deadtime_nS = cfg.deadtime_ns;
    mc3p.config.duty_min = cfg.duty_min;
    mc3p.config.duty_max = cfg.duty_max;
    eldriver_mc3p_reconfigure_pwm(&mc3p);
}

void PmsmControlCore::pwmLoop()
{
    if (control.mc_mode == MCMode::None)
        return;
    uint32_t start = eldriver_core_prof_tick();
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    switch (control.mc_mode)
    {
    case MCMode::Trap:
        Trap_pwmLoop();
        break;
    case MCMode::Foc:
        Foc_pwmLoop();
        break;
    case MCMode::SComm:
        SComm_pwmLoop();
        break;
    default:
        break;
    }
    pTicks = pTicks + 1;
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

void PmsmControlCore::setControlMode(MCMode mc_mode, ElecMode elec_mode)
{
    //Todo: Sanity check, and probably don't allow all or any electrical or motor control changes while motor is running
    control.elec_mode = elec_mode;
    if (control.mc_mode != mc_mode)
    {
        // Exit current mode
        switch (control.mc_mode)
        {
        case MCMode::Trap:
            Trap_onExit();
            break;
        case MCMode::Foc:
            Foc_onExit();
            break;
        case MCMode::SComm:
            SComm_onExit();
            break;
        default:
            break;
        }
        // Enter new mode
        switch (mc_mode)
        {
        case MCMode::Trap:
            Trap_onEnter(control.mc_mode);
            break;
        case MCMode::Foc:
            Foc_onEnter(control.mc_mode);
            break;
        case MCMode::SComm:
            SComm_onEnter(control.mc_mode);
            break;
        default:
            break;
        }
        control.mc_mode = mc_mode;
    }
}


