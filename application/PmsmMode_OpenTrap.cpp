#include "PmsmController.h"



void PmsmControl::OpenTrap_pwmLoop()
{
    switch (stup.stage_current)
    {
    case pmsm_stup_stage_t::Reset:
        stup.stage_last    = pmsm_stup_stage_t::Reset;
        stup.stage_current = pmsm_stup_stage_t::Align;
        stup.good_est_count = 0;
    case pmsm_stup_stage_t::Align:
        if(stup.stage_current != stup.stage_last)
        {
            //APPLY ALIGN VOLTAGE
            float32_t r = (float32_t)(stup.cfg.align_V / stup.cfg.bus_V);
            arm_float_to_q15(&r, &(elec.trap_duty_q15), 1);
            elec.sector = stup.cfg.align_sector; 
            elcore_swttimer_reset(&(stup.stage_timer), xmc_ticks);
            stup.stage_last = pmsm_stup_stage_t::Align;
        }
        if(elcore_swttimer_timout(&(stup.stage_timer), xmc_ticks, static_cast<uint32_t>(XCPWM_MS_TO_TICKS(stup.cfg.align_duration_ms))))
        {
            stup.stage_current = pmsm_stup_stage_t::Ramp;
        }
        else
        {
            break;
        }
    case pmsm_stup_stage_t::Ramp: {
        if(stup.stage_current != stup.stage_last)
        {
            elcore_swttimer_reset(&(stup.stage_timer), xmc_ticks);
            elcore_swttimer_reset(&(stup.comm_timer), xmc_ticks);
            stup.stage_last = pmsm_stup_stage_t::Ramp;
        }
        uint8_t comm = elcore_swttimer_timout(&stup.comm_timer, xmc_ticks, stup.comm_ticks);
        //increment frequency
        if(comm)
        {
            mech.speed_rpm = mech.speed_sp_rpm;
            float32_t f = (mech.speed_sp_rpm / 60.0) * (pole_pairs);
            stup.comm_ticks = static_cast<uint32_t>(XCPWM_MS_TO_TICKS(1000.0f / (f * 6.0f)));
            elec.speed_hz = 2 * M_PI * (XCPWM_TICKFREQ / (stup.comm_ticks * 6) );
            elec.sector = TrapIncrement(elec.sector, mech.dir);   
            elcore_swttimer_reset(&(stup.comm_timer), xmc_ticks);
        }
        break;
    }
    default:
        break;
    }
}
