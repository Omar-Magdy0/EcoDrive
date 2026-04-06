#pragma once

#include "PmsmControl.h"

#ifndef ELDRIVER_HALL1_ENABLED
#define BEMFZC_ENABLED
#endif

// Shared trapezoidal startup helpers for open/closed trap modes.

static inline void Trap_Bemfzc_ComCallback()
{
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

static inline void Trap_Hall1_ComCallback()
{
    uint8_t hall_value = eldriver_hall1_read();
    motor_c1.elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

#ifdef BEMFZC_ENABLED
static inline void Trap_Bemfzc_ResetHandler(PmsmControl *cp)
{
    cp->stup.stage_last = PmsmControl::StupStage::Reset;
    cp->stup.stage_current = PmsmControl::StupStage::Align;
    cp->stup.good_est_count = 0;
}

static inline void Trap_Bemfzc_AlignHandler(PmsmControl *cp)
{
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
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

static inline void Trap_Bemfzc_RampHandler(PmsmControl *cp, bool allow_takeover)
{
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
        cp->stup.stage_last = PmsmControl::StupStage::Ramp;
    }

    uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->pwmTicks, cp->stup.comm_ticks);
    float et_mS = cp->ticks_to_ms(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->pwmTicks));

    if (cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
    {
        float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS) / cp->stup.cfg.bus_V;
        arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
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
        if (allow_takeover)
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
                cp->pos_sensor.impl().takeover(Trap_Bemfzc_ComCallback);
                cp->stup.stage_current = PmsmControl::StupStage::Closed;
            }
        }
        else if (et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx])
        {
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

static inline void Trap_Hall_ResetHandler(PmsmControl *cp)
{
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    uint8_t hall_value = eldriver_hall1_read();
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    cp->elec.sector = PmsmControl::TrapIncrement(cp->elec.sector, cp->mech.dir);
    cp->pos_sensor.set_com_delay_us(COMMUTATION_PHASE_DELAY);
    cp->pos_sensor.set_com_callback(Trap_Hall1_ComCallback);
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
    cp->stup.stage_current = PmsmControl::StupStage::Closed;
}

static inline void Trap_ResetHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
    Trap_Hall_ResetHandler(cp);
#else
    Trap_Bemfzc_ResetHandler(cp);
#endif
}

static inline void Trap_AlignHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
#else
    Trap_Bemfzc_AlignHandler(cp);
#endif
}

static inline void Trap_RampHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
#else
    Trap_Bemfzc_RampHandler(cp, allow_takeover);
#endif
}

static inline void Trap_StartupDispatch(PmsmControl *cp, bool allow_takeover, void (*closed_handler)(PmsmControl *))
{
    switch (cp->stup.stage_current)
    {
    case PmsmControl::StupStage::Reset:
        Trap_ResetHandler(cp, allow_takeover);
        break;
    case PmsmControl::StupStage::Align:
        Trap_AlignHandler(cp, allow_takeover);
        break;
    case PmsmControl::StupStage::Ramp:
        Trap_RampHandler(cp, allow_takeover);
        break;
    case PmsmControl::StupStage::Closed:
        closed_handler(cp);
        break;
    default:
        break;
    }
}
