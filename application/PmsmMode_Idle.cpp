/**
 * @file PmsmMode_Idle.cpp
 * @brief Implementation of the Idle (Safe/Disabled) control mode for the PMSM controller.
 * @details This file contains the implementation of the idle state loop, which is 
 *          typically the default state upon startup or when a fault condition is 
 *          detected. In this mode, the motor is un-driven and allowed to coast.
 */
#include "PmsmControl.h"

/**
 * @brief Executes the high-frequency PWM loop when the controller is in the Idle state.
 * @details This function acts as a safety fallback and the default resting state of the 
 *          motor controller. When invoked, it explicitly commands the hardware inverter 
 *          driver to turn off all high-side and low-side power switches (e.g., MOSFETs/IGBTs). 
 *          This "floats" the motor phases, preventing any active current from flowing through the 
 *          stator windings. Consequently, the motor produces zero electrical torque and 
 *          will freely coast to a mechanical stop if it was previously spinning. 
 * 
 *          Key use cases for Idle Mode:
 *          - System initialization and waiting state before a valid run command is issued.
 *          - Immediate reaction to hardware fault conditions (e.g., Over-Current, Over-Voltage).
 *          - Intentional freewheeling or coast-to-stop operations.
 */
void PmsmControl::Idle_pwmLoop()
{
    /// Explicitly disable all inverter switches, placing the 3 motor phases into a high-impedance (floating) state.
    eldriver_mc3p_write_float(&mc3p);
}