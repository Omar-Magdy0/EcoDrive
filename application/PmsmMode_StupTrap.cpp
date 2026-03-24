#include "PmsmController.h"

void bemfzc_ComCallback()
{
    motor_c.elec.sector = TrapIncrement(motor_c.elec.sector, motor_c.mech.dir);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
}

void hall1_ComCallback()
{
    uint8_t hall_value = eldriver_hall1_read();
    motor_c.elec.sector = HALL_TO_TRAP_TABLE[hall_value];  
    motor_c.elec.sector = TrapIncrement(motor_c.elec.sector, motor_c.mech.dir);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
}


static inline void pmsm_stup_trapBemf(PmsmControl *cp)
{
    #ifdef ELDRIVER_MC3P_HALL1_ENABLED
    switch (cp->stup.stage_current)
    {
    case pmsm_stup_stage_t::Reset:
    {
        cp->stup.stage_last    = pmsm_stup_stage_t::Reset;
        cp->stup.stage_current = pmsm_stup_stage_t::Align;
        cp->stup.good_est_count = 0;
    }
    case pmsm_stup_stage_t::Align:
    {
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            //APPLY ALIGN VOLTAGE
            float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
            arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
            cp->elec.sector = cp->stup.cfg.align_sector; 
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
            cp->stup.stage_last = pmsm_stup_stage_t::Align;
        }
        
        if(elcore_swttimer_timout(&(cp->stup.stage_timer), cp->pwmTicks, static_cast<uint32_t>(cp->ms_to_ticks(cp->stup.cfg.align_duration_ms))))
        {
            cp->stup.stage_current = pmsm_stup_stage_t::Ramp;
        }
        else
        {
            break;
        }
    }
    case pmsm_stup_stage_t::Ramp: 
    {
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
            cp->stup.stage_last = pmsm_stup_stage_t::Ramp;
        }
        uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->pwmTicks, cp->stup.comm_ticks);
        if(cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
        {    
            //increment voltage
            float et_mS = cp->ticks_to_ms(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->pwmTicks));
            float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS) / cp->stup.cfg.bus_V;
            arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
            //increment frequency
            if(comm)
            {
                float32_t f = elmath_linearInterp(&cp->stup.cfg.freq_Hz[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS);
                cp->stup.comm_ticks = static_cast<uint32_t>(cp->ms_to_ticks(1000.0f / (f * 6.0f)));
                cp->elec.speed_hz = 2 * M_PI * (static_cast<float>(cp->pwm_freq_hz) / (cp->stup.comm_ticks * 6) );
            }
            if(et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx + 1]){
                cp->stup.ramp_idx++;
            }
        }else{
            cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, DirectionSign(cp->mech.dir));
            cp->stup.est_elec_speed = cp->pos_sensor.elec_speed();
            if(NEARLY_EQUAL(cp->stup.est_elec_speed, cp->elec.speed_hz, cp->elec.speed_hz*(STUP_BEMFZC_ERROR_MARGIN)))
            {
                cp->stup.good_est_count++;
            }else{
                cp->stup.good_est_count = 0;
            }
            if(cp->stup.good_est_count >= STUP_BEMFZC_GOOD_EST_COUNT)
            {
                ////Switchover and handle to closed loop commutation
                cp->pos_sensor.impl().takeover(bemfzc_ComCallback);
                cp->state = PmsmMode::ClosedTrap;
            }            
        }
        if(comm){
            cp->elec.sector = TrapIncrement(cp->elec.sector, cp->mech.dir);   
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
        }
        break;
    }
    default:
        break;
    }
    #endif
}



static inline void pmsm_stup_trapHall(PmsmControl *cp)
{
    //APPLY ALIGN VOLTAGE
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    uint8_t hall_value = eldriver_hall1_read();
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];  
    cp->elec.sector = TrapIncrement(cp->elec.sector, motor_c.mech.dir);
    cp->pos_sensor.set_com_delay_us(COMMUTATION_PHASE_DELAY);
    cp->pos_sensor.set_com_callback(hall1_ComCallback);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
    cp->mode = PmsmMode::ClosedTrap;
}


void PmsmControl::StupTrap_pwmLoop()
{
    #ifdef ELDRIVER_BEMFZC_ENABLED
    pmsm_stup_trapBemf(this);
    #endif

    #ifdef ELDRIVER_HALL1_ENABLED
    pmsm_stup_trapHall(this);
    #endif

    eldriver_mc3p_write_trap(&mc3p, elec.sector, elec.trap_duty_q15);
}
