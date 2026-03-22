#include "PmsmController.h"


void PmsmControl::ClosedTrap_pwmLoop()
{
    float32_t r = (float32_t)(2.5 / stup.cfg.bus_V);
    arm_float_to_q15(&r, &(elec.trap_duty_q15), 1);
    pos_sensor.update(mc3p_sync_data.trap.vbemf_q31, xmc_ticks, DirectionSign(mech.dir));
    elec.speed_hz = pos_sensor.elec_speed();
    eldriver_mc3p_write_trap(&mc3p, elec.sector, elec.trap_duty_q15);
}