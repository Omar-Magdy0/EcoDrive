/**
 * @file PmsmTrapStartup.h
 * @brief Shared architectural helpers for Trapezoidal Motor Startup (Open-Loop and Closed-Loop).
 * @details This header defines the core inline state machine functions necessary to successfully 
 *          boot up a Permanent Magnet Synchronous Motor (PMSM) using 6-step trapezoidal commutation. 
 *          It abstracts the heavily timing-dependent stages—such as DC Alignment, Open-Loop V/f 
 *          Ramping, and seamless Handover to Closed-Loop—into modular handlers. 
 *          By keeping these handlers `static inline` within a header, the compiler can heavily 
 *          optimize them directly into the fast PWM interrupt loops (`pwmLoop`) of the respective 
 *          control modes (e.g., `PmsmMode_ClosedTrap` or `PmsmMode_OpenTrap`), achieving zero 
 *          function-call overhead during critical real-time execution.
 */
#pragma once

#include "PmsmControl.h"

#ifndef ELDRIVER_HALL1_ENABLED
#define BEMFZC_ENABLED
#endif

/**
 * @brief Hardware/Software interrupt callback for Sensorless Back-EMF Zero Crossing (ZCD).
 * @details In a sensorless system, this function is dynamically registered to fire exactly 30 
 *          electrical degrees *after* a zero-crossing event is detected on the un-driven (floating) 
 *          motor phase. This 30-degree delay is mechanically optimal for maximizing torque. 
 *          Upon firing, this routine immediately advances the electrical commutation sector based 
 *          on the target mechanical direction, and applies the pre-calculated PWM duty cycle directly 
 *          to the 3-phase inverter driver, forcing the rotor to continue spinning.
 */
static inline void Trap_Bemfzc_ComCallback()
{
    /// Increment the electrical sector depending on the active mechanical direction
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    /// Apply the new sector and current duty cycle to the motor driver
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

/**
 * @brief Hardware interrupt callback for Sensored Hall Effect operation.
 * @details For motors equipped with physical Hall effect sensors, absolute rotor position is known 
 *          in discrete 60-degree increments. When the rotor turns and trips a Hall sensor, a GPIO 
 *          interrupt immediately triggers this callback. It reads the raw 3-bit binary state from 
 *          the sensors, passes it through a static lookup table (`HALL_TO_TRAP_TABLE`) to resolve the 
 *          fundamental electrical sector, artificially shifts this sector forward to maintain a 90-degree 
 *          lead (for optimal torque generation), and instantly writes the new state to the inverter hardware.
 */
static inline void Trap_Hall1_ComCallback()
{
    /// Read raw state from Hall sensors (1 to 6)
    uint8_t hall_value = eldriver_hall1_read();
    /// Map the physical Hall state to the corresponding fundamental electrical sector
    motor_c1.elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    /// Advance the sector by 90 electrical degrees to produce maximum torque in the desired direction
    motor_c1.elec.sector = PmsmControl::TrapIncrement(motor_c1.elec.sector, motor_c1.mech.dir);
    /// Apply the new sector and current duty cycle to the motor driver
    eldriver_mc3p_write_trap(&motor_c1.mc3p, motor_c1.elec.sector, motor_c1.elec.trap_duty_q15);
}

#ifdef BEMFZC_ENABLED
/**
 * @brief Resets internal tracking variables to cleanly initiate a sensorless startup sequence.
 * @details A sensorless motor must be carefully marshaled through strict sequential phases 
 *          (Reset -> Align -> Ramp -> Closed) to prevent stalling. This handler acts as the 
 *          entry point. It ensures any residual tracking metrics from previous runs (such as the 
 *          consecutive stable BEMF estimations counter) are zeroed out. It then actively pushes 
 *          the state machine into the `Align` stage for the next execution cycle.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void Trap_Bemfzc_ResetHandler(PmsmControl *cp)
{
    cp->stup.stage_last = PmsmControl::StupStage::Reset;
    /// Force the state machine into the Alignment stage
    cp->stup.stage_current = PmsmControl::StupStage::Align;
    /// Reset the consecutive good BEMF estimates counter
    cp->stup.good_est_count = 0;
}

/**
 * @brief Executes the static DC alignment protocol to lock the sensorless rotor into a known position.
 * @details Because the absolute position of a sensorless rotor is entirely unknown at 0 RPM, starting 
 *          commutation blindly could cause the motor to twitch backward or stall entirely. This handler 
 *          safely solves this by injecting a fixed, low-power DC voltage vector into a single, predetermined 
 *          electrical sector. This generates a static magnetic stator field. The permanent magnets on the 
 *          rotor will physically pull themselves into alignment with this field. The handler maintains this 
 *          state until a software timer dictates that the physical mechanical ringing has dampened 
 *          (`align_duration_ms`), after which it advances the system to the open-loop `Ramp` stage.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void Trap_Bemfzc_AlignHandler(PmsmControl *cp)
{
    /// Stage Entry Point: Initialize alignment parameters on the first call
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        /// Calculate the ratio of the desired alignment voltage against the available DC bus voltage
        float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
        /// Convert the voltage ratio to a fixed-point Q15 duty cycle representation
        arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
        /// Assign the statically configured alignment sector
        cp->elec.sector = cp->stup.cfg.align_sector;
        /// Reset the software timer to track the alignment duration
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        cp->stup.stage_last = PmsmControl::StupStage::Align;
    }

    /// Stage Exit Point: Check if the alignment duration (e.g., 100ms) has fully elapsed
    if (elcore_swttimer_timout(&(cp->stup.stage_timer), cp->pwmTicks, static_cast<uint32_t>(cp->ms_to_ticks(cp->stup.cfg.align_duration_ms))))
    {
        /// Move to the open-loop ramp acceleration stage
        cp->stup.stage_current = PmsmControl::StupStage::Ramp;
    }
}

/**
 * @brief Orchestrates the open-loop V/f acceleration ramp and manages the BEMF observer takeover.
 * @details This is the most complex stage of sensorless startup. Since Back-EMF relies on velocity, 
 *          the motor must be artificially dragged up to a minimum speed. This handler reads a configured 
 *          multipoint V/f profile table (`stup.cfg`). By evaluating the elapsed time in milliseconds, it 
 *          linearly interpolates both the target driving voltage and the synthetic electrical commutation 
 *          frequency needed for the present moment. 
 * 
 *          If `allow_takeover` is enabled, this handler concurrently feeds real-time voltage measurements 
 *          to the position observer algorithm. Once the observer successfully tracks the synthetic frequency 
 *          within a tight error margin for a consistent number of cycles, this handler abandons the rigid 
 *          open-loop profile and seamlessly transfers control to the Closed-Loop BEMF zero-crossing callbacks.
 *          If disabled, it strictly enforces the entire V/f profile before blindly jumping to the closed state.
 * @param cp Pointer to the primary PMSM control structure.
 * @param allow_takeover Flag determining if the BEMF observer is permitted to dynamically preempt the ramp.
 */
static inline void Trap_Bemfzc_RampHandler(PmsmControl *cp, bool allow_takeover)
{
    /// Stage Entry Point: Reset timers for both the overall stage duration and the individual commutation steps
    if (cp->stup.stage_current != cp->stup.stage_last)
    {
        elcore_swttimer_reset(&(cp->stup.stage_timer), cp->pwmTicks);
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
        cp->stup.stage_last = PmsmControl::StupStage::Ramp;
    }

    /// Check if it is time to force the next commutation step based on the synthetic frequency
    uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->pwmTicks, cp->stup.comm_ticks);
    /// Calculate the total elapsed time in milliseconds to determine our position on the ramp profile
    float et_mS = cp->ticks_to_ms(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->pwmTicks));

    /// Execute if we are still within the boundaries of the predefined V/F profile table
    if (cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
    {
        /// Interpolate the target voltage for the current exact millisecond
        float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS) / cp->stup.cfg.bus_V;
        /// Convert the normalized voltage to a Q15 duty cycle value
        arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
        
        if (comm)
        {
            /// Interpolate the target synthetic frequency for the current exact millisecond
            float32_t f = elmath_linearInterp(&cp->stup.cfg.freq_Hz[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS);
            /// Calculate the number of ticks required to wait before the next commutation (1/6th of a cycle)
            cp->stup.comm_ticks = static_cast<uint32_t>(cp->ms_to_ticks(1000.0f / (f * 6.0f)));
            /// Track the synthetic electrical speed in Hz to compare it later with the BEMF observer
            cp->elec.speed_hz = 2 * M_PI * (static_cast<float>(cp->pwm_freq_hz) / (cp->stup.comm_ticks * 6));
        }
        
        /// Advance the ramp index if the elapsed time crosses into the next profile segment
        if (et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx + 1])
        {
            cp->stup.ramp_idx++;
        }
    }
    else
    {
        /// The acceleration ramp profile has finished. Handle the transition phase.
        if (allow_takeover)
        {
            /// Feed the latest BEMF voltage and mechanical direction into the sensorless observer
            cp->pos_sensor.update(cp->mc3p_sync_meas.trap.vbemf_q31, cp->pwmTicks, cp->DirectionSign(cp->mech.dir));
            /// Retrieve the estimated electrical speed from the observer
            cp->stup.est_elec_speed = cp->pos_sensor.elec_speed();
            
            /// Verify if the observed BEMF speed is nearly equal to our synthetic ramp speed (within a margin)
            if (NEARLY_EQUAL(cp->stup.est_elec_speed, cp->elec.speed_hz, cp->elec.speed_hz * (STUP_BEMFZC_ERROR_MARGIN)))
            {
                cp->stup.good_est_count++;
            }
            else
            {
                /// Reset the counter if the observer diverges, ensuring we only take over when fully stable
                cp->stup.good_est_count = 0;
            }
            
            /// Trigger a closed-loop takeover if we have accumulated enough consecutive stable estimates
            if (cp->stup.good_est_count >= STUP_BEMFZC_GOOD_EST_COUNT)
            {
                /// Bind the BEMF zero-crossing detection to the commutation callback
                cp->pos_sensor.impl().takeover(Trap_Bemfzc_ComCallback);
                /// Formally transition the state machine to the Closed stage
                cp->stup.stage_current = PmsmControl::StupStage::Closed;
            }
        }
        else if (et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx])
        {
            /// In pure open-loop mode, simply transition to the Closed stage without BEMF takeover
            cp->stup.stage_current = PmsmControl::StupStage::Closed;
        }
    }

    /// Execute the physical commutation if the synthetic timer expired
    if (comm)
    {
        /// Advance the electrical sector based on the direction of rotation
        cp->elec.sector = cp->TrapIncrement(cp->elec.sector, cp->mech.dir);
        /// Restart the timer for the next commutation interval
        elcore_swttimer_reset(&(cp->stup.comm_timer), cp->pwmTicks);
    }
}
#endif

/**
 * @brief Immediately resolves position and kicks off sensored (Hall) operation without ramping.
 * @details Sensored motors inherently bypass the most difficult aspects of startup. Because absolute 
 *          rotor position is available immediately via the Hall effect sensors, there is no need to guess, 
 *          align, or artificially ramp the motor. This handler takes advantage of this by instantly 
 *          sampling the Hall sensors, deriving the correct leading commutation sector, calculating the 
 *          starting duty cycle, applying the initial inverter pulse, and instantly promoting the state 
 *          machine directly to the `Closed` stage in a single tick.
 * @param cp Pointer to the primary PMSM control structure.
 */
static inline void Trap_Hall_ResetHandler(PmsmControl *cp)
{
    /// Calculate the initial starting duty cycle
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    
    /// Read the physical Hall effect sensors to determine the rotor's current position
    uint8_t hall_value = eldriver_hall1_read();
    /// Map the sensor reading to the base electrical sector
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];
    /// Advance the sector to apply leading torque in the desired direction
    cp->elec.sector = PmsmControl::TrapIncrement(cp->elec.sector, cp->mech.dir);
    
    /// Configure hardware interrupts with the necessary commutation phase delay
    cp->pos_sensor.set_com_delay_us(COMMUTATION_PHASE_DELAY);
    /// Bind the Hall sensor hardware interrupt to the commutation callback
    cp->pos_sensor.set_com_callback(Trap_Hall1_ComCallback);
    
    /// Instantly energize the motor coils with the calculated sector and duty cycle
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
    /// Jump directly into the Closed-loop state machine stage
    cp->stup.stage_current = PmsmControl::StupStage::Closed;
}

/**
 * @brief Hardware-agnostic dispatcher for the Reset stage of the startup state machine.
 * @details Evaluates compile-time hardware definitions to automatically bind the correct underlying 
 *          implementation (Hall-based vs. Sensorless-based) for the reset procedure.
 * @param cp Pointer to the primary PMSM control structure.
 * @param allow_takeover Flag indicating whether a BEMF takeover should be anticipated (applicable to sensorless).
 */
static inline void Trap_ResetHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
    Trap_Hall_ResetHandler(cp);
#else
    Trap_Bemfzc_ResetHandler(cp);
#endif
}

/**
 * @brief Hardware-agnostic dispatcher for the static DC Alignment stage.
 * @details Resolves the correct alignment behavior based on the hardware configuration. If physical 
 *          Hall sensors are enabled at compile-time, this stage acts as an empty stub, effectively 
 *          bypassing the unnecessary alignment step entirely.
 * @param cp Pointer to the primary PMSM control structure.
 * @param allow_takeover Optional configuration passed down to the active implementation.
 */
static inline void Trap_AlignHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
#else
    Trap_Bemfzc_AlignHandler(cp);
#endif
}

/**
 * @brief Hardware-agnostic dispatcher for the Open-Loop Acceleration Ramp stage.
 * @details Resolves the correct acceleration profile execution. Like the alignment stage, if physical 
 *          Hall sensors are present, this stage is entirely bypassed and resolved as a compile-time empty stub.
 * @param cp Pointer to the primary PMSM control structure.
 * @param allow_takeover Instructs the underlying ramp implementation whether to attempt a dynamic closed-loop takeover.
 */
static inline void Trap_RampHandler(PmsmControl *cp, bool allow_takeover)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)allow_takeover;
#else
    Trap_Bemfzc_RampHandler(cp, allow_takeover);
#endif
}

/**
 * @brief Central execution multiplexer for the entire Trapezoidal Startup State Machine.
 * @details This core function acts as the "switchboard" that drives the sequential motor startup logic. 
 *          It continuously monitors `cp->stup.stage_current` and routes the execution flow into the 
 *          corresponding inline handler (Reset, Align, Ramp, or Closed). By accepting a function pointer 
 *          for the `closed_handler`, this dispatcher becomes highly reusable across entirely different 
 *          operating modes (e.g., standard Closed-Loop Trapezoidal vs. Open-Loop Forced Commutation), 
 *          reducing code duplication significantly.
 * @param cp Pointer to the primary PMSM control structure.
 * @param allow_takeover Dictates if the state machine should use dynamic BEMF synchronization during ramping.
 * @param closed_handler Function pointer to the specific implementation to execute once the startup completes.
 */
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
