#include "PmsmControl.h"

/**
 * @brief Resets the startup state machine and prepares hardware for the commutation sequence.
 * @details This function acts as the entry point for motor startup. Depending on the active 
 *          sensor configuration (Hall effect sensors vs. Sensorless Back-EMF), it delegates 
 *          the initialization to the appropriate specific handler (e.g., Hall_ResetHandler or 
 *          Bemfzc_ResetHandler). It ensures that all staging variables, timers, and error 
 *          counters are cleared before energizing the motor phases.
 * @param cp Pointer to the primary PMSM control structure containing the motor's state context.
 */
static inline void ResetHandler(PmsmControl *cp);

/**
 * @brief Handles the rotor alignment stage to establish a known initial electrical angle.
 * @details In sensorless (Back-EMF) control schemes, the absolute rotor position is unknown at 
 *          standstill. This handler injects a fixed DC voltage vector into a specific electrical 
 *          sector for a predetermined duration. This creates a stationary magnetic field that forces 
 *          the permanent magnet rotor to align itself, guaranteeing a safe and predictable starting 
 *          point for the subsequent open-loop ramp stage.
 * @note This stage is typically bypassed when absolute position sensors (like Hall sensors) are available.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void AlignHandler(PmsmControl *cp);

/**
 * @brief Manages the open-loop forced commutation ramping stage for sensorless startup.
 * @details Since Back-EMF amplitude is proportional to speed, it cannot be reliably detected at 
 *          very low speeds. This handler synthesizes a rotating magnetic field by applying a 
 *          pre-configured Voltage/Frequency (V/f) profile. It linearly interpolates voltage and 
 *          frequency targets over time, gradually accelerating the rotor open-loop until the 
 *          generated Back-EMF is strong enough for reliable zero-crossing detection.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void RampHandler(PmsmControl *cp);

/**
 * @brief Maintains steady-state closed-loop commutation.
 * @details Once the motor has successfully started (either immediately via Hall sensors or after 
 *          a successful open-loop ramp in sensorless mode), this handler takes over. It constantly 
 *          evaluates real-time position feedback to dynamically update the active electrical sector 
 *          and applies the appropriate PWM duty cycles to the inverter bridge, ensuring optimal 
 *          torque generation and synchronous rotation.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void ClosedHandler(PmsmControl *cp);

#ifndef ELDRIVER_HALL1_ENABLED
#define BEMFZC_ENABLED
#endif

/**
 * @brief Executes the primary high-frequency PWM control loop for Closed-Loop Trapezoidal operation.
 * @details This method acts as the central execution hub for the trapezoidal state machine. It is 
 *          typically invoked by a hardware timer interrupt or an ADC synchronization callback at the 
 *          PWM carrier frequency (e.g., 20 kHz to 25 kHz). By evaluating `stup.stage_current`, it 
 *          routes the execution flow to the appropriate stage handler: Reset, Align, Ramp, or Closed.
 *          It ensures that state transitions occur seamlessly without dropping PWM cycles.
 * @warning This function is executed in a time-critical Interrupt Service Routine (ISR) context. 
 *          Blocking operations, heavy math, or lengthy loops must be strictly avoided to prevent 
 *          control loop instability.
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
 * @brief Hardware/Software callback triggered upon successful Back-EMF Zero-Crossing (ZCD) detection.
 * @details In sensorless trapezoidal control, a zero-crossing of the un-driven phase's Back-EMF 
 *          indicates that the rotor's magnetic axis is perfectly aligned with the current commutation 
 *          sector. To maximize torque, the actual commutation (phase switching) must be delayed by 
 *          30 electrical degrees after this crossing. When this delayed callback fires, it calculates 
 *          the next electrical sector based on the commanded mechanical direction (forward or reverse) 
 *          and immediately updates the 3-phase inverter outputs.
 */
void bemfzc_ComCallback()
{
    /// Increment the electrical sector based on the current mechanical rotation direction
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    /// Write the updated sector and the active duty cycle to the 3-phase driver
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

/**
 * @brief Hardware interrupt callback triggered upon any state change of the physical Hall sensors.
 * @details Hall effect sensors provide discrete, absolute rotor position feedback every 60 electrical 
 *          degrees. When a state change triggers an interrupt, this callback executes immediately to 
 *          minimize commutation latency. It reads the raw 3-bit binary pattern from the GPIO pins, 
 *          uses a lookup table (`HALL_TO_TRAP_TABLE`) to determine the corresponding electrical sector, 
 *          applies a directional offset for forward/reverse rotation, and updates the inverter's PWM 
 *          duty cycles. This ensures highly responsive and robust torque control, especially at low speeds.
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
 * @brief Initializes the internal state machine variables for a sensorless (Back-EMF) startup sequence.
 * @details This function is the first step in starting a motor without physical position sensors. 
 *          It forces the internal state trackers to acknowledge the `Reset` stage, clears the consecutive 
 *          good BEMF estimates counter (`good_est_count` = 0) to prevent premature closed-loop handover, 
 *          and queues the `Align` stage as the immediate next operation.
 * @param cp Pointer to the active PMSM control structure.
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
 * @brief Executes the sensorless DC alignment protocol to lock the rotor into a known position.
 * @details When the state machine first enters this handler, it calculates the necessary PWM duty 
 *          cycle fraction by dividing the target alignment voltage (`align_V`) by the measured DC bus 
 *          voltage (`bus_V`). This ratio is converted into a highly efficient Q15 fixed-point format 
 *          (`arm_float_to_q15`) and applied to a predefined, static electrical sector (`align_sector`). 
 *          A software timer is started, and the handler continuously monitors elapsed time until the 
 *          configured `align_duration_ms` is reached, allowing any mechanical oscillations to dampen 
 *          out before transitioning to the `Ramp` stage.
 * @param cp Pointer to the active PMSM control structure.
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
 * @brief Orchestrates the complex open-loop V/f ramping profile to accelerate the rotor.
 * @details At a standstill, Back-EMF is zero. To generate enough Back-EMF for sensorless observation, 
 *          the motor must be forcefully spun up. This handler steps through a multi-point configuration 
 *          table (`stup.cfg`), mapping time milestones (`time_mS`) to specific voltage (`volt_V`) and 
 *          frequency (`freq_Hz`) targets. 
 *          
 *          Key operations per cycle:
 *          - **Time Tracking**: Calculates elapsed stage time in milliseconds.
 *          - **Voltage Interpolation**: Uses `elmath_linearInterp` to calculate a smooth voltage curve, 
 *            converts it to a normalized Q15 duty cycle, and applies it to the active sector.
 *          - **Frequency Commutation**: Simulates an electrical frequency by triggering forced phase 
 *            commutations every `comm_ticks`.
 *          - **Profile Advancement**: Moves to the next index in the ramp table as time passes.
 *          - **BEMF Handover**: Once the mechanical ramp profile is exhausted, it compares the synthetically 
 *            driven frequency against the real-time Back-EMF speed estimate. If they match within a margin, it 
 *            enters `Closed` mode.
 * @param cp Pointer to the active PMSM control structure.
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
 * @brief Initializes sensored startup by instantly resolving absolute rotor position and transitioning to closed-loop.
 * @details Unlike sensorless techniques, a Hall-sensored motor does not require an alignment or open-loop 
 *          ramp phase. This handler exploits this advantage by immediately reading the current Hall sensor 
 *          pattern, deriving the correct starting electrical sector, and calculating the necessary PWM duty 
 *          cycle based on the alignment voltage configuration. It pre-configures the phase delay for the 
 *          optimal commutation point, registers the `hall1_ComCallback` for future interrupts, writes the 
 *          initial pulse to the inverter, and instantly promotes the state machine to `Closed` mode.
 * @param cp Pointer to the active PMSM control structure.
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
