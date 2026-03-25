#include "PmsmControl.h"

static inline void ResetHandler(PmsmControl *cp);
static inline void AlignHandler(PmsmControl *cp);
static inline void RampHandler(PmsmControl *cp);
static inline void ClosedHandler(PmsmControl *cp);

#ifndef ELDRIVER_HALL1_ENABLED
#define BEMFZC_ENABLED
#endif

void PmsmControl::ClosedTrap_pwmLoop()
{
    switch (stup.stage_current)
    {
    case PmsmControl::StupStage::Reset:
        ResetHandler(this);
        break;
    case PmsmControl::StupStage::Align:
        AlignHandler(this);
        break;
    case PmsmControl::StupStage::Ramp:
        RampHandler(this);
        break;
    case PmsmControl::StupStage::Closed:
        ClosedHandler(this);
        break;
    default:
        break;
    }
}

void bemfzc_ComCallback()
{
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}
void hall1_ComCallback()
{
    uint8_t hall_value = eldriver_hall1_read();
    motor_c1.elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

//================BemfZc Handlers===============
#ifdef BEMFZC_ENABLED
static inline void Bemfzc_ResetHandler(PmsmControl *cp)
{
    cp->stup.stage_last = PmsmControl::StupStage::Reset;
    cp->stup.stage_current = PmsmControl::StupStage::Align;
    cp->stup.good_est_count = 0;
}
static inline void Bemfzc_AlignHandler(PmsmControl *cp)
{
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        // APPLY ALIGN VOLTAGE
        float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
        arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
        cp->elec.sector = cp->stup.cfg.align_sector;
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        cp->stup.stage_last = PmsmControl::StupStage::Align;
    }

    if (elcore_swttimer_timout(&(cp->stup.stage_timer), cp->pwmTicks, static_cast<uint32_t>(cp->ms_to_ticks(cp->stup.cfg.align_duration_ms))))
    {
        cp->stup.stage_current = PmsmControl::StupStage::Ramp;
    }
}
static inline void Bemfzc_RampHandler(PmsmControl *cp)
{
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
        cp->stup.stage_last = PmsmControl::StupStage::Ramp;
    }
    uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->pwmTicks, cp->stup.comm_ticks);
    if (cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
    {
        // increment voltage
        float et_mS = cp->ticks_to_ms(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->pwmTicks));
        float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS) / cp->stup.cfg.bus_V;
        arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
        // increment frequency
        if (comm)
        {
            float32_t f = elmath_linearInterp(&cp->stup.cfg.freq_Hz[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS);
            cp->stup.comm_ticks = static_cast<uint32_t>(cp->ms_to_ticks(1000.0f / (f * 6.0f)));
            cp->elec.speed_hz = 2 * M_PI * (static_cast<float>(cp->pwm_freq_hz) / (cp->stup.comm_ticks * 6));
        }
        if (et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx + 1])
        {
            cp->stup.ramp_idx++;
        }
    }
    else
    {
        cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, cp->DirectionSign(cp->mech.dir));
        cp->stup.est_elec_speed = cp->pos_sensor.elec_speed();
        if (NEARLY_EQUAL(cp->stup.est_elec_speed, cp->elec.speed_hz, cp->elec.speed_hz * (STUP_BEMFZC_ERROR_MARGIN)))
        {
            cp->stup.good_est_count++;
        }
        else
        {
            cp->stup.good_est_count = 0;
        }
        if (cp->stup.good_est_count >= STUP_BEMFZC_GOOD_EST_COUNT)
        {
            ////Switchover and handle to closed loop commutation
            cp->pos_sensor.impl().takeover(bemfzc_ComCallback);
            cp->stup.stage_current = PmsmControl::StupStage::Closed;
        }
    }
    if (comm)
    {
        cp->elec.sector = cp->TrapIncrement(cp->elec.sector, cp->mech.dir);
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
    }
}
#endif

//================Hall Handlers===============
static inline void Hall_ResetHandler(PmsmControl *cp)
{
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    uint8_t hall_value = eldriver_hall1_read();
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    cp->elec.sector = PmsmControl::TrapIncrement(cp->elec.sector, cp->mech.dir);
    cp->pos_sensor.set_com_delay_us(COMMUTATION_PHASE_DELAY);
    cp->pos_sensor.set_com_callback(hall1_ComCallback);
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
    cp->stup.stage_current = PmsmControl::PmsmControl::StupStage::Closed;
}
//================Handlers Binding===============
static inline void ResetHandler(PmsmControl *cp)
{
#ifdef ELDRIVER_HALL1_ENABLED
    Hall_ResetHandler(cp);
#else
    Bemfzc_ResetHandler(cp);
#endif
}
static inline void AlignHandler(PmsmControl *cp)
{
#ifdef ELDRIVER_HALL1_ENABLED
#else
    Bemfzc_AlignHandler(cp);
#endif
}
static inline void RampHandler(PmsmControl *cp)
{
#ifdef ELDRIVER_HALL1_ENABLED
#else
    Bemfzc_RampHandler(cp);
#endif
}
static inline void ClosedHandler(PmsmControl *cp)
{
    float32_t r = (float32_t)(2.5 / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, cp->DirectionSign(cp->mech.dir));
    cp->elec.speed_hz = cp->pos_sensor.elec_speed();
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
}
