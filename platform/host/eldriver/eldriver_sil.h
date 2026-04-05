#ifndef ELDRIVER_SIL_H
#define ELDRIVER_SIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "eldriver_conf.h"

// ==================== SIL Motor/Inverter State ====================
// This struct holds both the inverter and motor simulation state
typedef struct {
    // --- MOTOR STATE (PMSM) ---
    // State variables from pmsm_model.h
    float motor_i[3];           // Phase currents [A, B, C]
    float motor_v[3];           // Phase voltages [A, B, C]
    float motor_vn;             // Neutral point voltage
    float motor_theta;          // Rotor angle (electrical) [rad]
    float motor_omega;          // Rotor angular velocity [rad/s]
    
    // --- INVERTER STATE ---
    // State variables from inverter_model.h
    float inv_i[3];             // Inverter output currents [A, B, C]
    float inv_v[3];             // Inverter output voltages [A, B, C]
    
    // --- INVERTER INPUT (duty cycle commands from eldriver) ---
    float inv_duty[3];          // PWM duty cycles [0-1] for phases A, B, C
    float inv_vbus;             // Bus voltage [V]
    
    // --- SIMULATION PARAMETERS ---
    float dt;                   // Simulation timestep [s]
    float t;                    // Current simulation time [s]
    
} eldriver_sil_state_t;

// ==================== Initialization ====================
/**
 * Initialize SIL motor/inverter simulation
 * Call this once during platform_init()
 */
void eldriver_sil_init(void);

// ==================== Control Input (from eldriver) ====================
/**
 * Set inverter duty cycles (called by eldriver_mc3p_write_phase_duty)
 * @param duty_u, duty_v, duty_w: Duty cycle per phase [0-1]
 * @param vbus: DC bus voltage [V]
 */
void eldriver_sil_set_inverter_duty(float duty_u, float duty_v, float duty_w, float vbus);

// ==================== Simulation Step ====================
/**
 * Step the SIL models forward by dt
 * Call this at every motor control interrupt (4 kHz for this system)
 * This does: inverter_step() -> pmsm_step()
 */
void eldriver_sil_step(void);

// ==================== Output/Readback (to eldriver) ====================
/**
 * Get simulated phase currents (what ADC would read)
 * @param out_iu, out_iv, out_iw: Pointers to store phase currents [A]
 */
void eldriver_sil_get_currents(float *out_iu, float *out_iv, float *out_iw);

/**
 * Get simulated phase voltages
 * @param out_vu, out_vv, out_vw: Pointers to store phase voltages [V]
 */
void eldriver_sil_get_voltages(float *out_vu, float *out_vv, float *out_vw);

/**
 * Get rotor electrical angle and speed
 * @param out_theta, out_omega: Pointers to store angle [rad] and speed [rad/s]
 */
void eldriver_sil_get_rotor_state(float *out_theta, float *out_omega);

// ==================== GUI/Visualization ====================
/**
 * Write current sample to oscilloscope buffer for GUI visualization
 * Call this after eldriver_sil_step() to record waveforms
 */
void eldriver_sil_write_scope(void);

/**
 * Get the global SIL state (for diagnostics)
 */
eldriver_sil_state_t* eldriver_sil_get_state(void);

#ifdef __cplusplus
}
#endif

#endif // ELDRIVER_SIL_H
