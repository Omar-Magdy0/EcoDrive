#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "inverter_model.h"
#include "pmsm_model.h"

// --- SIMULATION PARAMETERS (same as your original) ---
#define SIM_TIME_S     0.5f
#define PWM_FREQ_HZ    20000.0f
#define SIM_STEP_S     0.00005f     // 50 µs
#define V_BUS          24.0f
#define MODULATION_IDX 0.05f        // 0..1  (0.85 gives good utilization without overmodulation)
#define ELEC_FREQ_HZ   10.0f
#define PLOT_DECIMATION 1           // log every step → can increase to 20 later

// Helper: Sinusoidal PWM with 3rd harmonic injection
void apply_spwm(float theta_e, InverterInput* in, float mod_idx) {
    // References (per unit)
    float va_ref = sinf(theta_e);
    float vb_ref = sinf(theta_e - 2.0f * M_PI / 3.0f);
    float vc_ref = sinf(theta_e + 2.0f * M_PI / 3.0f);

    // Third harmonic injection → increases linear range to ~1.1547 × pure sine
    float vmin = fminf(fminf(va_ref, vb_ref), vc_ref);
    float vmax = fmaxf(fmaxf(va_ref, vb_ref), vc_ref);
    float vcom = (vmax + vmin) / 2.0f;               // ≈ third harmonic / 6 in many implementations

    va_ref -= vcom;
    vb_ref -= vcom;
    vc_ref -= vcom;

    // Scale to modulation index and convert to duty cycle [0..1]
    // Center around 0.5 (bipolar PWM style, common in many MCUs)
    in->duty[0] = 0.5f + mod_idx * va_ref * 0.5f;
    in->duty[1] = 0.5f + mod_idx * vb_ref * 0.5f;
    in->duty[2] = 0.5f + mod_idx * vc_ref * 0.5f;

    // Clamp to valid PWM range
    for (int k = 0; k < 3; k++) {
        if (in->duty[k] > 1.0f) in->duty[k] = 1.0f;
        if (in->duty[k] < 0.0f) in->duty[k] = 0.0f;
    }

    // Assume complementary drive (high-side = drive[k]==1 means active high)
    // In your inverter_model.h we use drive[k]==1 → active high with Rdson
    for (int k = 0; k < 3; k++) {
        in->drive[k] = 1;           // always active PWM (no high-Z except dead-time, which we ignore)
    }
}

int main() {
    // 1. Parameters (same as your test)
    MotorParams mp = { .Ld=0.001f, .Lq=0.001f, .Rs=0.05f, .pp=7.0f, .Ke=0.01f, .J=0.005f, .B=0.001f };
    InverterParams ip = { .Rdson=0.02f, .RdsOff=1000000.0f, .FWmin=0.2f };

    // 2. States
    MotorState m_state = {0};
    MotorState m_next  = {0};
    MotorOutput m_out  = {0};

    InverterState  i_state = {0};
    InverterInput  i_in    = { .vbus = V_BUS };
    InverterOutput i_out   = {0};

    // 3. CSV output
    FILE *f = fopen("system_results_spwm.csv", "w");
    if (!f) {
        printf("Cannot open CSV file\n");
        return 1;
    }

    fprintf(f, "time,ia,ib,ic,vthev_a,v_a,v_n,omega,torque,theta_elec,theta_mech\n");
    long steps = (long)(SIM_TIME_S / SIM_STEP_S + 0.5f);
    printf("Simulating sinusoidal drive (%ld steps, ~%.1f Hz electrical)...\n", steps, ELEC_FREQ_HZ);

    for (long s = 0; s < steps; s++) {
        float t = s * SIM_STEP_S;

        // Electrical angle (open-loop)
        float theta_e = 2.0f * M_PI * ELEC_FREQ_HZ * t;

        // --- Apply SPWM ---
        apply_spwm(theta_e, &i_in, MODULATION_IDX);

        // --- Inverter model ---
        for (int k = 0; k < 3; k++) {
            i_state.i[k] = m_state.i[k];
        }
        inverter_step(&i_state, &i_in, &ip, &i_out);

        // --- PMSM model ---
        MotorInput m_in;
        for (int k = 0; k < 3; k++) {
            m_in.vthev[k] = i_out.vthev[k];
            m_in.rthev[k] = i_out.rthev[k];
        }
        pmsm_step(&m_state, &m_in, &mp, SIM_STEP_S, &m_next, &m_out);

        // --- Simple mechanical step (constant load) ---
        float T_load = 0.01f;
        float alpha = (m_out.T - T_load - mp.B * m_state.omega) / mp.J;
        m_next.omega = m_state.omega + alpha * SIM_STEP_S;
        m_next.theta = m_state.theta + m_next.omega * SIM_STEP_S;   // ← use NEW omega

        // --- Logging ---
        if (s % PLOT_DECIMATION == 0) {
            fprintf(f, "%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                m_state.i[0], m_state.i[1], m_state.i[2],
                i_out.vthev[0],
                m_next.v[0],
                m_next.vn,
                m_state.omega,
                m_out.T,
                theta_e,
                m_state.theta
            );
        }

        m_state = m_next;
    }

    fclose(f);
    printf("Sinusoidal test finished → data saved to system_results_spwm.csv\n");
    return 0;
}