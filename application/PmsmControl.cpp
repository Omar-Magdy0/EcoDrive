#include "PmsmControl.h"

void PmsmControl::init()
{
    pwmTicks = 0;
    /* instance bindings */
    
    /* mode initialization*/
    voltage_q31_to_float(state.Vbus_q31);
    current_q31_to_float(state.Ibus_q31);
    angle_q31_to_float(state.eTheta_q31);

    tick_period_ns = 1'000'000'000.0f / static_cast<float>(pwm_freq_hz);
    tick_period_us = tick_period_ns / 1000.0f;
    mc3p.config.pwm_Hz = pwm_freq_hz;
    Trap_init();
    SComm_init();
    Foc_init();
    /* hardware initialization */
    pos_sensor.init(pwm_freq_hz);
    eldriver_mc3p_init(&mc3p);
    eldriver_mc3p_write_float(&mc3p);
};
void PmsmControl::xmcLoop()
{
    if (control.mctype == MCType::None)
        return;
    switch (control.mctype)
    {
    case MCType::Trap:
        Trap_xmcLoop();
        break;
    case MCType::Foc:
        Foc_xmcLoop();
        break;
    case MCType::SComm:
        SComm_xmcLoop();
        break;
    default:
        break;
    }
}

void PmsmControl::pwmLoop()
{
    if (control.mctype == MCType::None)
        return;
    uint32_t start = eldriver_core_prof_tick();
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    switch (control.mctype)
    {
    case MCType::Trap:
        Trap_pwmLoop();
        break;
    case MCType::Foc:
        Foc_pwmLoop();
        break;
    case MCType::SComm:
        SComm_pwmLoop();
        break;
    default:
        break;
    }
    pwmTicks = pwmTicks + 1;
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

void PmsmControl::setControlMode(MCType mctype, ECType ectype)
{
    //Todo: Sanity check, and probably don't allow all or any electrical or motor control changes while motor is running
    control.ectype = ectype;
    if (control.mctype != mctype)
    {
        // Exit current mode
        switch (control.mctype)
        {
        case MCType::Trap:
            Trap_onExit();
            break;
        case MCType::Foc:
            Foc_onExit();
            break;
        case MCType::SComm:
            SComm_onExit();
            break;
        default:
            break;
        }
        // Enter new mode
        switch (mctype)
        {
        case MCType::Trap:
            Trap_onEnter(control.mctype);
            break;
        case MCType::Foc:
            Foc_onEnter(control.mctype);
            break;
        case MCType::SComm:
            SComm_onEnter(control.mctype);
            break;
        default:
            break;
        }
        control.mctype = mctype;
    }
}
//======================================================
//  GLOBAL INSTANCE
//======================================================
inline PmsmControl motor_c1; /** Global controller instance for motor channel 1. */
void eldriver_mc3p_sync_postScanCallback() { motor_c1.pwmLoop(); }
void eldriver_xmc3p_tickerCallback() { motor_c1.xmcLoop(); }
