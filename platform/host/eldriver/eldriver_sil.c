#include "eldriver_sil.h"
#include "pmsm_model.h"
#include "inverter_model.h"
#include "eldriver_conf.h"
#include <string.h>
#include <stdio.h>

// ==================== GLOBAL STATE ====================
static eldriver_sil_state_t g_sil_state = {0};

// Motor parameters (PMSM - tune these to match your actual motor!)
static const MotorParams motor_params = {
    .Ld = 0.00015f,             // d-axis inductance [H]
    .Lq = 0.00015f,             // q-axis inductance [H]
    .Rs = 0.5f,                 // Stator resistance [Ω]
    .pp = 7,                    // Pole pairs
    .Ke = 0.03f,                // Back-EMF constant [V·s/rad]
    .J = 0.00001f,              // Moment of inertia [kg·m²]
    .B = 0.00001f               // Damping coefficient [N·m·s/rad]
};

// Inverter parameters (tune to match your inverter!)
static const InverterParams inv_params = {
    .Rdson = 0.01f,             // On-state resistance [Ω]
    .RdsOff = 1000000.0f,       // Off-state resistance [Ω]
    .FWmin = 0.7f               // Forward voltage drop [V]
};

// Motor and inverter state structs (from model headers)
static MotorState motor_state = {0};
static MotorInput motor_input = {0};
static MotorOutput motor_output = {0};

static InverterState inv_state = {0};
static InverterInput inv_input = {0};
static InverterOutput inv_output = {0};

// ==================== INITIALIZATION ====================
void eldriver_sil_init(void)
{
    // Initialize global state
    memset(&g_sil_state, 0, sizeof(eldriver_sil_state_t));
    
    // Initialize motor state
    memset(&motor_state, 0, sizeof(MotorState));
    motor_state.theta = 0.0f;
    motor_state.omega = 0.0f;
    
    // Initialize inverter state
    memset(&inv_state, 0, sizeof(InverterState));
    
    // Set simulation timestep (4 kHz motor control frequency)
    g_sil_state.dt = 1.0f / ELDRIVER_XMC3P_TICKFREQ;  // 250 μs for 4 kHz
    g_sil_state.t = 0.0f;
    
    // Initialize inverter DC bus voltage
    g_sil_state.inv_vbus = 24.0f;  // 24V nominal
    
    printf("[SIL] Initialized. dt=%.6f s, f=%d Hz\n", 
           g_sil_state.dt, ELDRIVER_XMC3P_TICKFREQ);
}

// ==================== CONTROL INPUTS ====================
void eldriver_sil_set_inverter_duty(float duty_u, float duty_v, float duty_w, float vbus)
{
    // Store duty cycles and bus voltage
    g_sil_state.inv_duty[0] = duty_u;
    g_sil_state.inv_duty[1] = duty_v;
    g_sil_state.inv_duty[2] = duty_w;
    g_sil_state.inv_vbus = vbus;
    
    // Prepare inverter input for next step
    inv_input.vbus = vbus;
    inv_input.duty[0] = duty_u;
    inv_input.duty[1] = duty_v;
    inv_input.duty[2] = duty_w;
    
    // Note: drive[] array (gate drive signals) would be set separately if needed
    // For now, we assume duty cycle directly controls the switch
    inv_input.drive[0] = (duty_u > 0.01f) ? 1 : 0;
    inv_input.drive[1] = (duty_v > 0.01f) ? 1 : 0;
    inv_input.drive[2] = (duty_w > 0.01f) ? 1 : 0;
}

// ==================== SIMULATION STEP ====================
void eldriver_sil_step(void)
{
    // --- STEP 1: Run inverter model ---
    // Takes duty cycles → outputs Thevenin voltage and resistance
    inverter_step(&inv_state, &inv_input, &inv_params, &inv_output);
    
    // --- STEP 2: Prepare motor input from inverter output ---
    motor_input.vthev[0] = inv_output.vthev[0];
    motor_input.vthev[1] = inv_output.vthev[1];
    motor_input.vthev[2] = inv_output.vthev[2];
    
    motor_input.rthev[0] = inv_output.rthev[0];
    motor_input.rthev[1] = inv_output.rthev[1];
    motor_input.rthev[2] = inv_output.rthev[2];
    
    // --- STEP 3: Run motor (PMSM) model ---
    // Takes voltages from inverter → outputs currents and torque
    pmsm_step(&motor_state, &motor_input, &motor_params, 
              g_sil_state.dt, &motor_state, &motor_output);
    
    // --- STEP 4: Update inverter state with motor currents (feedback) ---
    inv_state.i[0] = motor_state.i[0];
    inv_state.i[1] = motor_state.i[1];
    inv_state.i[2] = motor_state.i[2];
    
    // --- STEP 5: Copy motor state to global for readout ---
    g_sil_state.motor_i[0] = motor_state.i[0];
    g_sil_state.motor_i[1] = motor_state.i[1];
    g_sil_state.motor_i[2] = motor_state.i[2];
    
    g_sil_state.motor_v[0] = motor_state.v[0];
    g_sil_state.motor_v[1] = motor_state.v[1];
    g_sil_state.motor_v[2] = motor_state.v[2];
    
    g_sil_state.motor_vn = motor_state.vn;
    g_sil_state.motor_theta = motor_state.theta;
    g_sil_state.motor_omega = motor_state.omega;
    
    // --- STEP 6: Advance simulation time ---
    g_sil_state.t += g_sil_state.dt;
}

// ==================== OUTPUT READBACK ====================
void eldriver_sil_get_currents(float *out_iu, float *out_iv, float *out_iw)
{
    if (out_iu) *out_iu = g_sil_state.motor_i[0];
    if (out_iv) *out_iv = g_sil_state.motor_i[1];
    if (out_iw) *out_iw = g_sil_state.motor_i[2];
}

void eldriver_sil_get_voltages(float *out_vu, float *out_vv, float *out_vw)
{
    if (out_vu) *out_vu = g_sil_state.motor_v[0];
    if (out_vv) *out_vv = g_sil_state.motor_v[1];
    if (out_vw) *out_vw = g_sil_state.motor_v[2];
}

void eldriver_sil_get_rotor_state(float *out_theta, float *out_omega)
{
    if (out_theta) *out_theta = g_sil_state.motor_theta;
    if (out_omega) *out_omega = g_sil_state.motor_omega;
}

eldriver_sil_state_t* eldriver_sil_get_state(void)
{
    return &g_sil_state;
}

// ==================== VISUALIZATION ====================
// Note: SimHelper is a C++ class defined in virtual_pmsm.h
// Scope writing is accessed from C++ via sim_gui.cpp using eldriver_sil_get_state()

void eldriver_sil_write_scope(void)
{
    // Placeholder - scope data is accessed via eldriver_sil_get_state()
    // from the C++ visualization layer (sim_gui.cpp)
}
