#include "PmsmControlCore.h"

using namespace PmsmControlTypes;

void PmsmControlCore::olstup_lut_run(Olstup &stup, q31_t &ec_sp_q31, float &rps)
{
    // Interpolate angular velocity and duty cycle and do appropiate updates
    float et_ms = (xTicks_to_ms(xTicks) - stup.time_start_ms);
    bool is_complete = et_ms >= stup.cfg.time_ms_tb[OLSTUP_TABLE_SIZE - 1];
    if (!is_complete)
    {
        // We interpolate here for duty cycle and angular velocity
        float ret_slp = (float)(et_ms - stup.cfg.time_ms_tb[stup.tb_index]) / (stup.cfg.time_ms_tb[stup.tb_index + 1] - stup.cfg.time_ms_tb[stup.tb_index]);
        float ec_sp = stup.cfg.ec_tb[stup.tb_index] + ret_slp * (stup.cfg.ec_tb[stup.tb_index + 1] - stup.cfg.ec_tb[stup.tb_index]);
        float rpm = stup.cfg.rpm_tb[stup.tb_index] + ret_slp * (stup.cfg.rpm_tb[stup.tb_index + 1] - stup.cfg.rpm_tb[stup.tb_index]);
        rps = rpm * (float)(2 * M_PI / 60.0);
        ec_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(ec_sp);
        if (stup.tb_index < (OLSTUP_TABLE_SIZE - 1) && et_ms > stup.cfg.time_ms_tb[stup.tb_index + 1])
        {
            stup.tb_index++;
        }
    }else{

    }
    stup.is_complete = is_complete;
}

void PmsmControlCore::init()
{
    pTicks = 0;
    xTicks = 0;
    
    /* mode initialization*/
    voltage_q31_to_float(state.Vbus_q31);
    current_q31_to_float(state.Ibus_q31);
    angle_q31_to_float(state.eTheta_q31);

    mc3p.offset_calibration = true;
    SComm_init();
    Foc_init();
    /* hardware initialization */
    posDriver.init();
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
    pTick_period_ns = static_cast<uint32_t>(1'000'000'000 / static_cast<float>(pwm_freq_hz));
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
    uint32_t start = eldriver_prof_tick();
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    switch (control.mc_mode)
    {
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
    volatile uint32_t elapsed = eldriver_prof_tock(start);
}

void PmsmControlCore::setControlMode(MCMode mc_mode)
{
    //Todo: Sanity check, and probably don't allow all or any electrical or motor control changes while motor is running
    if (control.mc_mode != mc_mode)
    {
        // Exit current mode
        switch (control.mc_mode)
        {
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
