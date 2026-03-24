#include "PmsmController.h"



void PmsmControl::OpenTrap_pwmLoop()
{
    switch (stup.stage_current)
    {
    case pmsm_stup_stage_t::Reset:
    {
        stup.stage_last    = pmsm_stup_stage_t::Reset;
        stup.stage_current = pmsm_stup_stage_t::Ramp;
        stup.good_est_count = 0;
    }
    case pmsm_stup_stage_t::Ramp: 
    {
        if(stup.stage_current != stup.stage_last)
        {
            elcore_swttimer_reset(&(stup.stage_timer), pwmTicks);
            elcore_swttimer_reset(&(stup.comm_timer), pwmTicks);
            stup.stage_last = pmsm_stup_stage_t::Ramp;
        }
        uint8_t comm = elcore_swttimer_timout(&stup.comm_timer, pwmTicks, stup.comm_ticks);
        //increment frequency
        if(comm)
        {
            mech.speed_rpm = mech.speed_sp_rpm;
            float32_t f = (mech.speed_sp_rpm / 60.0) * (pole_pairs);
            stup.comm_ticks = static_cast<uint32_t>(ms_to_ticks(1000.0f / (f * 6.0f)));
            elec.speed_hz = 2 * M_PI * (static_cast<float>(pwm_freq_hz) / (stup.comm_ticks * 6) );
            elec.sector = TrapIncrement(elec.sector, mech.dir);   
            elcore_swttimer_reset(&(stup.comm_timer), pwmTicks);
        }
        break;
    }
    default:
        break;
    }
}
