#include "PmsmControl.h"

/**
 * @brief Resets the startup state machine to its initial state.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ResetHandler(PmsmControl *cp);

/**
 * @brief Handles the rotor alignment stage before starting commutation.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void AlignHandler(PmsmControl *cp);

/**
 * @brief Handles the open-loop voltage and frequency ramping stage.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void RampHandler(PmsmControl *cp);

/**
 * @brief Handles the closed-loop state after successful startup estimation.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ClosedHandler(PmsmControl *cp);

#ifndef ELDRIVER_HALL1_ENABLED
#define BEMFZC_ENABLED
#endif

/**
 * @brief Executes the PWM loop specifically for Closed-Loop Trapezoidal control.
 * @details Implements a state machine that transitions through Reset, Align, Ramp, and finally Closed-loop stages.
 */
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

/**
 * @brief Callback function executed upon Back-EMF Zero Crossing detection.
 * @details Advances the electrical commutation sector and updates the driver's trapezoidal outputs.
 */
void bemfzc_ComCallback()
{
    /// Increment the electrical sector based on the current mechanical rotation direction
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    /// Write the updated sector and the active duty cycle to the 3-phase driver
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

/**
 * @brief Callback function executed upon a Hall Sensor state change.
 * @details Reads the new Hall pattern, maps it to a trapezoidal sector, and commands the inverter.
 */
void hall1_ComCallback()
{
    /// Read the raw 3-bit value from the Hall effect sensors
    uint8_t hall_value = eldriver_hall1_read();
    /// Look up the corresponding initial electrical sector from the configured mapping table
    motor_c1.elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    /// Adjust the sector based on the commanded mechanical direction
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    /// Apply the calculated sector and duty cycle to the inverter hardware
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

/**
 * @name Sensorless Back-EMF Zero Crossing (BEMFZC) Handlers
 * @brief Logic for open-loop alignment, ramping, and switchover when no physical position sensors are present.
 */
///@{
#ifdef BEMFZC_ENABLED
/**
 * @brief Initializes the sensorless startup sequence.
 */
static inline void Bemfzc_ResetHandler(PmsmControl *cp)
{
    /// Set the last stage to Reset to mark the starting point
    cp->stup.stage_last = PmsmControl::StupStage::Reset;
    /// Transition the active stage directly to Rotor Alignment
    cp->stup.stage_current = PmsmControl::StupStage::Align;
    /// Reset the counter tracking consecutive successful BEMF estimations
    cp->stup.good_est_count = 0;
}

/**
 * @brief Applies a fixed DC voltage vector to force the rotor to a known stationary alignment.
 */
static inline void Bemfzc_AlignHandler(PmsmControl *cp)
{
    /// Check if this is the first entry into the Alignment stage
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        /// Calculate the ratio of the desired alignment voltage to the available DC bus voltage
        float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
        /// Convert the float voltage ratio to a fixed-point Q15 duty cycle representation
        arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
        /// Set the inverter to the configured static alignment sector
        cp->elec.sector = cp->stup.cfg.align_sector;
        /// Reset the stage timer to accurately measure the alignment duration
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        /// Update the last stage tracker
        cp->stup.stage_last = PmsmControl::StupStage::Align;
    }

    /// Check if the alignment duration has elapsed
    if (elcore_swttimer_timout(&(cp->stup.stage_timer), cp->pwmTicks, static_cast<uint32_t>(cp->ms_to_ticks(cp->stup.cfg.align_duration_ms))))
    {
        /// Transition to the open-loop Ramp stage once alignment is complete
        cp->stup.stage_current = PmsmControl::StupStage::Ramp;
    }
}

/**
 * @brief Accelerates the motor in open-loop by interpolating voltage and frequency against a time profile.
 */
static inline void Bemfzc_RampHandler(PmsmControl *cp)
{
    /// Initialization upon first entering the Ramp stage
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        /// Reset the stage duration timer
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        /// Reset the commutation interval timer
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
        /// Update the last stage tracker
        cp->stup.stage_last = PmsmControl::StupStage::Ramp;
    }
    /// Check if it's time to commutate based on the current synthetic frequency
    uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->pwmTicks, cp->stup.comm_ticks);
    
    /// Continue ramping if we haven't reached the end of the predefined lookup table
    if (cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
    {
        /// Calculate the elapsed time in milliseconds for the current ramp segment
        float et_mS = cp->ticks_to_ms(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->pwmTicks));
        
        /// Linearly interpolate the target voltage based on the elapsed time and convert to normalized duty cycle
        float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS) / cp->stup.cfg.bus_V;
        /// Apply the interpolated duty cycle into the Q15 control variable
        arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
        
        /// If a commutation event is triggered, update the operational frequency
        if (comm)
        {
            /// Linearly interpolate the target electrical frequency in Hz
            float32_t f = elmath_linearInterp(&cp->stup.cfg.freq_Hz[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS);
            /// Calculate the number of PWM ticks required for the next commutation step (1/6th of an electrical period)
            cp->stup.comm_ticks = static_cast<uint32_t>(cp->ms_to_ticks(1000.0f / (f * 6.0f)));
            /// Calculate the real-time electrical speed in Hz
            cp->elec.speed_hz = 2 * M_PI * (static_cast<float>(cp->pwm_freq_hz) / (cp->stup.comm_ticks * 6));
        }
        
        /// Advance to the next segment in the profile if the segment's target time is reached
        if (et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx + 1])
        {
            cp->stup.ramp_idx++;
        }
    }
    else
    {
        /// Ramp profile completed; begin observing the Back-EMF via the position sensor logic
        cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, cp->DirectionSign(cp->mech.dir));
        /// Fetch the estimated electrical speed from the sensor logic
        cp->stup.est_elec_speed = cp->pos_sensor.elec_speed();
        
        /// Compare the estimated Back-EMF speed against the synthetic open-loop speed within a defined error margin
        if (NEARLY_EQUAL(cp->stup.est_elec_speed, cp->elec.speed_hz, cp->elec.speed_hz * (STUP_BEMFZC_ERROR_MARGIN)))
        {
            /// Increment consecutive good estimates counter
            cp->stup.good_est_count++;
        }
        else
        {
            /// Reset counter if the estimate diverges
            cp->stup.good_est_count = 0;
        }
        
        /// Check if sufficient consecutive good estimates have been gathered to confidently close the loop
        if (cp->stup.good_est_count >= STUP_BEMFZC_GOOD_EST_COUNT)
        {
            /// Handover control to the BEMF callback mechanism, establishing closed-loop operation
            cp->pos_sensor.impl().takeover(bemfzc_ComCallback);
            /// Transition state machine to the standard Closed mode
            cp->stup.stage_current = PmsmControl::StupStage::Closed;
        }
    }
    
    /// Process the actual commutation step if flagged
    if (comm)
    {
        /// Step to the next electrical sector based on motor direction
        cp->elec.sector = cp->TrapIncrement(cp->elec.sector, cp->mech.dir);
        /// Reset the commutation timer for the next phase
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
    }
}
#endif
///@}

/**
 * @name Hall Sensor Handlers
 * @brief State machine handlers for sensored startup using discrete Hall effect sensors.
 */
///@{
/**
 * @brief Immediately reads the Hall state and transitions directly to closed-loop.
 * @details Sensored motors do not require open-loop ramping; they can be closed-loop commutated immediately from standstill.
 */
static inline void Hall_ResetHandler(PmsmControl *cp)
{
    /// Calculate starting voltage duty cycle based on configuration
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    
    /// Read the instantaneous Hall sensor value
    uint8_t hall_value = eldriver_hall1_read();
    /// Look up the corresponding initial trapezoidal sector
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    /// Advance the sector to produce driving torque in the desired direction
    cp->elec.sector = PmsmControl::TrapIncrement(cp->elec.sector, cp->mech.dir);
    
    /// Set the phase delay required for optimal commutation switching
    cp->pos_sensor.set_com_delay_us(COMMUTATION_PHASE_DELAY);
    /// Register the Hall sensor callback to handle future edge interrupts
    cp->pos_sensor.set_com_callback(hall1_ComCallback);
    
    /// Write the first closed-loop step to the hardware
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
    
    /// Transition the state machine straight to Closed mode
    cp->stup.stage_current = PmsmControl::PmsmControl::StupStage::Closed;
}
///@}

/**
 * @name Startup Handler Bindings
 * @brief Wrapper functions that route execution to either Hall or BEMF logic based on preprocessor configurations.
 */
///@{
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
    /// Calculate a fixed target duty cycle based on a predetermined closed-loop operational voltage
    float32_t r = (float32_t)(2.5 / cp->stup.cfg.bus_V);
    /// Convert the target voltage ratio to a fixed-point Q15 value suitable for the inverter driver
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    
    /// Continuously feed real-time BEMF data and direction into the position sensing algorithm
    cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, cp->DirectionSign(cp->mech.dir));
    /// Retrieve and update the instantaneous electrical speed measurement
    cp->elec.speed_hz = cp->pos_sensor.elec_speed();
    
    /// Continuously apply the updated sector and duty cycle to maintain closed-loop operation
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
}
