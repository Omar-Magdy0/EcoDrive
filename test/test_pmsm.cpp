#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

// --- SIMULATION PARAMETERS ---
#define SIM_TIME_S       0.5f
#define PWM_FREQ_HZ      20000.0f
#define SIM_STEP_S       0.00005f  // 50us (Oversampling 20kHz PWM)
#define V_BUS            24.0f
#define TARGET_DUTY      0.05f       // 80% Power
#define PLOT_DECIMATION  1         // Only log every 20th step to CSV (100us resolution)

// Missing Output struct for Inverter (if not in header)

#include "inverter_model.h"
#include "pmsm_model.h"

// Helper: 6-Step Trapezoidal Commutation Logic
void apply_trapezoidal_sector(float theta_e, InverterInput* in, float duty) {
    // Normalize theta to [0, 2pi]
    float angle = fmodf(theta_e, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;

    int sector = (int)(angle / (M_PI / 3.0f));

    switch (sector) {
        case 0: // A+, B- (C High-Z)
            in->drive[0]=1; in->duty[0]=duty; in->drive[1]=1; in->duty[1]=0; in->drive[2]=0; break;
        case 1: // A+, C- (B High-Z)
            in->drive[0]=1; in->duty[0]=duty; in->drive[1]=0; in->drive[2]=1; in->duty[2]=0; break;
        case 2: // B+, C- (A High-Z)
            in->drive[0]=0; in->drive[1]=1; in->duty[1]=duty; in->drive[2]=1; in->duty[2]=0; break;
        case 3: // B+, A- (C High-Z)
            in->drive[0]=1; in->duty[0]=0; in->drive[1]=1; in->duty[1]=duty; in->drive[2]=0; break;
        case 4: // C+, A- (B High-Z)
            in->drive[0]=1; in->duty[0]=0; in->drive[1]=0; in->drive[2]=1; in->duty[2]=duty; break;
        case 5: // C+, B- (A High-Z)
            in->drive[0]=0; in->drive[1]=1; in->duty[1]=0; in->drive[2]=1; in->duty[2]=duty; break;
    }
}

int main() {
    // 1. Initialize Params
    MotorParams mp = { .Ld=0.001f, .Lq=0.001f, .Rs=0.05f, .pp=7.0f, .Ke=0.05f, .J=0.005f, .B=0.001f };
    InverterParams ip = { .Rdson=0.02f, .RdsOff=1000000.0f, .FWmin=0.2f };

    // 2. Initialize States
    MotorState m_state = {0};
    MotorState m_next = {0};
    MotorOutput m_out = {0};
    
    InverterState i_state = {0};
    InverterInput i_in = { .vbus = V_BUS };
    InverterOutput i_out = {0};

    // 3. Prepare CSV
    FILE *f = fopen("system_results.csv", "w");
    fprintf(f, "time,ia,ib,ic,vthev_a,v_a,v_n, omega,torque,theta_com, theta_e\n");

    long steps = (long)(SIM_TIME_S / SIM_STEP_S);
    printf("Simulating EcoDrive System (%ld steps)...\n", steps);

    for (long s = 0; s < steps; s++) {
        float t = s * SIM_STEP_S;
        float theta_com = 2*M_PI*10*t;
        //theta_com = m_state.theta * mp.pp;

        // --- Commutation ---
        apply_trapezoidal_sector(theta_com, &i_in, TARGET_DUTY);

        // --- STEP 1: Inverter Model ---
        // Feed current from motor state back to inverter for diode logic
        for(int k=0; k<3; k++) i_state.i[k] = m_state.i[k];
        inverter_step(&i_state, &i_in, &ip, &i_out);

        // --- STEP 2: PMSM Model ---
        // Feed Inverter Thevenin outputs into PMSM
        MotorInput m_in;
        for(int k=0; k<3; k++) {
            m_in.vthev[k] = i_out.vthev[k];
            m_in.rthev[k] = i_out.rthev[k];
        }

        pmsm_step(&m_state, &m_in, &mp, SIM_STEP_S, &m_next, &m_out);

        float T_load = 0.01;
        // --- STEP 3: Mechanical Integration ---
        float alpha = (m_out.T - T_load - mp.B * m_state.omega) / mp.J;
        m_next.omega = m_state.omega + alpha * SIM_STEP_S;
        m_next.theta = m_state.theta + m_state.omega * SIM_STEP_S;

        // Log Decimated Data
        if (s % PLOT_DECIMATION == 0) {
            fprintf(f, "%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f\n",
                    t, m_state.i[0], m_state.i[1], m_state.i[2], 
                    i_out.vthev[0], m_next.v[0], m_next.vn ,m_state.omega, m_out.T, theta_com, m_state.theta);
        }

        m_state = m_next;
    }

    fclose(f);
    printf("Simulation Finished. Data saved to system_results.csv\n");
    return 0;
}