#include "PmsmController.h"

/**
 * @brief Closed Loop Trapezoidal Control with Speed PID
 * This replaces the previous static duty cycle logic.
 */
void PmsmControl::ClosedTrap_pwmLoop()
{
    // 1. Update speed measurement from Back-EMF (BEMF) feedback
    // We use the raw ADC data (vbemf_q31) to calculate how fast the motor is actually spinning
    pos_sensor.update(mc3p_sync_data.trap.vbemf_q31, xmc_ticks, DirectionSign(mech.dir));
    elec.speed_hz = pos_sensor.elec_speed();
    
    // 2. Calculate the Setpoint (Target) in Electrical Hz
    // Conversion: (RPM / 60) / pole_pairs
    float speed_setpoint_hz = (mech.speed_sp_rpm / 60.0f) / pole_pairs;
    
    // 3. Calculate Error
    // Difference between where we want to be and where we are
    speed_loop.error = speed_setpoint_hz - elec.speed_hz;
    
    // 4. Run PID Controller
    // The PID "brain" calculates how much to adjust the duty cycle based on the error
    float pid_output = arm_pid_f32(&speed_loop.pid, speed_loop.error);
    
    // 5. Calculate Final Duty Cycle
    // Start with a base voltage (feed-forward) and add the PID correction
    float base_duty = 2.5f / stup.cfg.bus_V;
    float duty = base_duty + pid_output;
    
    // 6. Safety Clamping
    // Ensure the duty cycle stays within the hardware limits (5% to 95%)
    if (duty > 0.95f) duty = 0.95f;
    if (duty < 0.05f) duty = 0.05f;
    
    // 7. Output to Hardware
    // Convert the floating point result to Q15 format and send to the motor driver
    float32_t duty_f32 = duty;
    arm_float_to_q15(&duty_f32, &(elec.trap_duty_q15), 1);
    
    eldriver_mc3p_write_trap(&mc3p, elec.sector, elec.trap_duty_q15);
}