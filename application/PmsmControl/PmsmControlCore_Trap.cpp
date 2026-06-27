#include "PmsmControlCore.h"
using namespace PmsmControlTypes;
void PmsmControlCore::Trap_init()
{

}
void PmsmControlCore::Trap_onEnter(MCMode prev_mct){}
void PmsmControlCore::Trap_onExit(){}
//==========================================
// OPEN LOOP CODE
//==========================================
void PmsmControlCore::Trap_olstup_start()
{
    trap.olstup.tb_index = 0;
    trap.olstup.time_start_ms = xTicks_to_ms(xTicks);
    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(trap.olstup.cfg.ec_tb[0]);
    trap.olstup.comm_period_pwmTicks = (float)(trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index + 1] - trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index]) * pwm_freq_hz / 1000;
    pos_sensor.reset();
    state.eTheta_q31 = 0;
    trap.olstup.complete = false;
    trap.run_mode = Trap::RunMode::OL;
    trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
    trap.state.sector = trap.olstup.cfg.init_sector;
}

void PmsmControlCore::Trap_xmcLoop()
{
    if(trap.run_mode == Trap::RunMode::OL)
    {//Open loop operation
        if (!trap.olstup.complete)
        {//Startup
            // Interpolate angular velocity and duty cycle and do appropiate updates
            float et_ms = (xTicks_to_ms(pTicks) - trap.olstup.time_start_ms);
            if (trap.olstup.tb_index < (OLSTUP_TABLE_SIZE - 1) && et_ms > trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index + 1])
            {
                trap.olstup.tb_index++;
            }
            bool complete = et_ms >= trap.olstup.cfg.time_ms_tb[OLSTUP_TABLE_SIZE - 1];
            if (!complete)
            {
                // We interpolate here for duty cycle and angular velocity
                float ret_slp = (float)(et_ms - trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index]) / (trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index + 1] - trap.olstup.cfg.time_ms_tb[trap.olstup.tb_index]);
                float ec_sp = trap.olstup.cfg.ec_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.cfg.ec_tb[trap.olstup.tb_index + 1] - trap.olstup.cfg.ec_tb[trap.olstup.tb_index]);
                float rpm = trap.olstup.cfg.rpm_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.cfg.rpm_tb[trap.olstup.tb_index + 1] - trap.olstup.cfg.rpm_tb[trap.olstup.tb_index]);
                state.eAngv_RPS = rpm * model.pole_pairs * (float)(60.0/2*M_PI);
                if(control.elec_mode == ElecMode::Voltage){
                    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(ec_sp);
                }else{
                    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(ec_sp);
                }
                if(state.eAngv_RPS  > 0)trap.olstup.comm_period_pwmTicks = ((((M_PI / 3))) / (float)state.eAngv_RPS) * pwm_freq_hz;
            }
            trap.olstup.complete = complete;
        }else{//Normal openloop
            // TODO : SUPPORT COMMANDING VOLTAGE , CURRENT AND FREQUENCY AND DOING NECCESSARY UPDATES

        }
    }else{//Closed loop operation


    }
    state.eAngv_RPT_q31 = q31_t( (state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX/M_PI) );
}
//==========================================
// Closed LOOP CODE
//==========================================
void PmsmControlCore::Trap_pwmLoop()
{
    state.Vbus_q31 = (mc3p_sync_meas.trap.vbus_q31);
    state.Ibus_q31 = (mc3p_sync_meas.trap.cbus_q31);
    state.eTheta_q31 += state.eAngv_RPT_q31;
    q15_t duty;
    if(trap.run_mode == Trap::RunMode::OL)
    {
        // Detect Commutation event
        state.eTheta_q31 += state.eAngv_RPT_q31;
        if (trap.olstup.pwmTicks_till_comm == 0)
        {
            // Calculate ticks till next commutation event based on frequency
            trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
            trap.state.sector = TrapIncrement(trap.state.sector, state.dir);
        }
        trap.olstup.pwmTicks_till_comm = trap.olstup.pwmTicks_till_comm - 1;
    }else{

    }
    q31_t e = 0;
    if(control.elec_mode == ElecMode::Current)
    {
        //calculate error normally
        e = (trap.state.EC_sp_q31 - mc3p_sync_meas.trap.cbus_q31);
    }else
    {
        e = 0;
        trap.state.I_pid.state[2] = trap.state.EC_sp_q31;
    }
    //Always divide error by 4 to avoid overflow of pid accumulator (cmsis_pid reference)
    e = e>>2;
    trap.state.Vcomm_q31 = arm_pid_q31(&trap.state.I_pid, e) + trap.state.I_feedforward;
    //Saturate output
    int32_t vbus_lim_q31 = (state.Vbus_q31 * mc3p.duty_max_q15)>>15;
    if(trap.state.Vcomm_q31 > vbus_lim_q31){trap.state.Vcomm_q31 = vbus_lim_q31; trap.state.I_pid.state[2] = trap.state.Vcomm_q31;}
    duty = (int32_t)(trap.state.Vcomm_q31)/(state.Vbus_q31 >> 15);
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, duty);
}

