#include "PmsmControl.h"

void PmsmControl::Trap_init()
{
    Trap_olstup_start();
    control.mctype = MCType::Trap;
}
void PmsmControl::Trap_onEnter(MCType prev_mct){}
void PmsmControl::Trap_onExit(){}
//==========================================
// OPEN LOOP CODE
//==========================================
void PmsmControl::Trap_olstup_cfg(float vbus_init, float time_tb[STUP_TABLE_SIZE], float voltage_tb[STUP_TABLE_SIZE], float rpm_tb[STUP_TABLE_SIZE], eldriver_mc3p_sector_t init_sector)
{
    if (state.Vbus_q31 == 0)
    {
        state.Vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(vbus_init);
    }
    trap.olstup.init_sector = init_sector;
    for (int i = 0; i < STUP_TABLE_SIZE; i++)
    {
        trap.olstup.time_tb[i] = time_tb[i];
        trap.olstup.voltage_tb[i] = voltage_tb[i];
        trap.olstup.eAngv_RPS_tb[i] = (rpm_tb[i] * 2 * M_PI / 60) * model.pole_pairs;
    }
}
void PmsmControl::Trap_olstup_start()
{
    trap.olstup.tb_index = 0;
    trap.olstup.time_start_us = ticks_to_us(pwmTicks);
    trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(trap.olstup.voltage_tb[0]);
    trap.olstup.comm_period_pwmTicks = (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]) * pwm_freq_hz;
    pos_sensor.reset();
    state.eTheta_q31 = 0;
    trap.olstup.complete = false;
    arm_pid_reset_q15(&trap.state.I_pid);
    trap.state.run_mode = Trap::RunMode::OL;
    trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
    trap.state.sector = trap.olstup.init_sector;
}

void PmsmControl::Trap_xmcLoop()
{
    if(trap.state.run_mode == Trap::RunMode::OL)
    {
        if (!trap.olstup.complete)
        {
            /* STARTUP SECTION START */
            // Interpolate angular velocity and duty cycle and do appropiate updates
            float et = (ticks_to_us(pwmTicks) - trap.olstup.time_start_us)/1'000'000;
            if (trap.olstup.tb_index < (STUP_TABLE_SIZE - 1) && et > trap.olstup.time_tb[trap.olstup.tb_index + 1])
            {
                trap.olstup.tb_index++;
            }
            bool complete = et >= trap.olstup.time_tb[STUP_TABLE_SIZE - 1];
            if (!complete)
            {
                // We interpolate here for duty cycle and angular velocity
                float ret_slp = (float)(et - trap.olstup.time_tb[trap.olstup.tb_index]) / (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]);
                float voltage = trap.olstup.voltage_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.voltage_tb[trap.olstup.tb_index + 1] - trap.olstup.voltage_tb[trap.olstup.tb_index]);
                state.eAngv_RPS = trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index + 1] - trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index]);
                trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(voltage);
                if(state.eAngv_RPS  > 0)trap.olstup.comm_period_pwmTicks = ((((M_PI / 3))) / (float)state.eAngv_RPS) * pwm_freq_hz;
            }
            trap.olstup.complete = complete;
            /* STARTUP SECTION END */
        }else{
            // TODO : SUPPORT COMMANDING VOLTAGE , CURRENT AND FREQUENCY AND DOING NECCESSARY UPDATES


        }
    }else{


    }


    state.eAngv_RPT_q31 = q31_t( (state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX/M_PI) );
}
//==========================================
// Closed LOOP CODE
//==========================================
void PmsmControl::Trap_pwmLoop()
{
    state.Vbus_q31 = (mc3p_sync_meas.trap.vbus_q31);
    q15_t duty, Vcomm_q15;
    if(trap.state.run_mode == Trap::RunMode::OL)
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
    q15_t e = 0;
    if(control.ectype == ECType::Current)
    {
        //calculate error normally
        e = (trap.state.EC_sp_q31 - mc3p_sync_meas.trap.cbus_q31) >> 16;
    }else
    {
        e = 0;
        trap.state.I_feedforward = trap.state.EC_sp_q31 >> 16;
    }
    Vcomm_q15 = arm_pid_q15(&trap.state.I_pid, e) + trap.state.I_feedforward;
    trap.state.Vcomm_q31 = Vcomm_q15 << 16;
    duty = (((int64_t)trap.state.Vcomm_q31 * INT32_MAX)/ state.Vbus_q31 ) >> 16;
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, duty);
}


