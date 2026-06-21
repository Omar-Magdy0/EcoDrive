#include "PmsmControl.h"

void PmsmControl::Trap_init()
{

}
void PmsmControl::Trap_onEnter(MCType prev_mct){}
void PmsmControl::Trap_onExit(){}
//==========================================
// OPEN LOOP CODE
//==========================================
void PmsmControl::Trap_olstup_cfg(float vbus_init,const float time_tb[Trap::OLSTUP_TABLE_SIZE],const float ec_tb[Trap::OLSTUP_TABLE_SIZE],const float rpm_tb[Trap::OLSTUP_TABLE_SIZE], eldriver_mc3p_sector_t init_sector)
{
    if (state.Vbus_q31 == 0)
    {
        state.Vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(vbus_init);
    }
    trap.olstup.init_sector = init_sector;
    for (int i = 0; i < Trap::OLSTUP_TABLE_SIZE; i++)
    {
        trap.olstup.time_tb[i] = time_tb[i];
        trap.olstup.EC_tb[i] = ec_tb[i];
        trap.olstup.eAngv_RPS_tb[i] = (rpm_tb[i] * 2 * M_PI / 60) * model.pole_pairs;
    }
}
void PmsmControl::Trap_olstup_start()
{
    trap.olstup.tb_index = 0;
    trap.olstup.time_start_us = ticks_to_us(pwmTicks);
    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(trap.olstup.EC_tb[0]);
    trap.olstup.comm_period_pwmTicks = (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]) * pwm_freq_hz;
    pos_sensor.reset();
    state.eTheta_q31 = 0;
    trap.olstup.complete = false;
    trap.run_mode = Trap::RunMode::OL;
    trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
    trap.state.sector = trap.olstup.init_sector;
}

void PmsmControl::Trap_xmcLoop()
{
    if(trap.run_mode == Trap::RunMode::OL)
    {//Open loop operation
        if (!trap.olstup.complete)
        {//Startup
            // Interpolate angular velocity and duty cycle and do appropiate updates
            float et = (ticks_to_us(pwmTicks) - trap.olstup.time_start_us)/1'000'000;
            if (trap.olstup.tb_index < (Trap::OLSTUP_TABLE_SIZE - 1) && et > trap.olstup.time_tb[trap.olstup.tb_index + 1])
            {
                trap.olstup.tb_index++;
            }
            bool complete = et >= trap.olstup.time_tb[Trap::OLSTUP_TABLE_SIZE - 1];
            if (!complete)
            {
                // We interpolate here for duty cycle and angular velocity
                float ret_slp = (float)(et - trap.olstup.time_tb[trap.olstup.tb_index]) / (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]);
                float EC_sp = trap.olstup.EC_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.EC_tb[trap.olstup.tb_index + 1] - trap.olstup.EC_tb[trap.olstup.tb_index]);
                state.eAngv_RPS = trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index + 1] - trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index]);
                if(control.ectype == ECType::Voltage){
                    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(EC_sp);
                }else{
                    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(EC_sp);
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
void PmsmControl::Trap_pwmLoop()
{
    state.Vbus_q31 = (mc3p_sync_meas.trap.vbus_q31);
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
    if(control.ectype == ECType::Current)
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
    if(trap.state.Vcomm_q31 > state.Vbus_q31){trap.state.Vcomm_q31 = state.Vbus_q31; trap.state.I_pid.state[2] = trap.state.Vcomm_q31;}
    duty = (int32_t)(trap.state.Vcomm_q31)/(state.Vbus_q31 >> 15);
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, duty);
}

