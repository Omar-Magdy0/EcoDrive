#include "virtual_pmsm.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A simple RL load model for testing the inverter.
typedef struct
{
    float R; // Resistance in Ohms
    float L; // Inductance in Henrys
    elec_bus *bus;
    float theta;
    float e[3];
    float vn;
} rl_load_model_t;

// Derivative function di/dt = (v - R*i) / L for the RL load.
// This will be used with the RK4 integrator.
void rle_load_deriv(const float *i, float t, const void *params, float *didt)
{
    (void)t; // time is not used in this simple model
    const rl_load_model_t *load = (const rl_load_model_t *)params;

    // Calculate Neutral Point Voltage (vn)
    // vn is derived from the constraint that sum(di/dt) = 0 for conducting phases.
    // vn = sum(v_term - e - R*i) / N_conducting
    for(int i =0; i < 3; i++)
    {
        if(load->bus->driven[i])
        {
            didt[i] = (load->bus->v[i] - load->e[i] - load->R * load->i[i]) / load->L;
        }else{
            didt[i] = 0;
        }
    }
    
}

// Helper to generate Trapezoidal BEMF shape based on angle (0..2PI)
// Returns normalized value in range [-1, 1]
float get_emf(float theta)
{
    // Normalize to 0..2PI
    float a = fmodf(theta, 2.0f * M_PI);
    if (a < 0) a += 2.0f * M_PI;

    return sin(theta);                                 // 300..360 deg: Flat Top
}

// Main simulation function
void run_simulation_and_save_csv()
{
    // --- Simulation Parameters ---
    const float vbus = 24.0f;
    const float speed_hz = 50.0f;        // Electrical Hz
    const float load_R = 0.5f;           // 0.5 Ohm
    const float load_L = 1.0f / 10000.0f; // 0.1 mH
    const float bemf_k = 3;          // Back-EMF peak magnitude

    // --- Simulation State ---
    float time_accum = 0.0f;
    const float sample_rate = 100000.0f; // 100kHz simulation
    const float dt = 1.0f / sample_rate;
    const float sim_duration = 0.1f; // Simulate for 100ms

    // --- Simulation Models ---
    elec_bus bus = {{0}}; // Initialize currents and voltages to zero
    inverter_model inv = {vbus, 0.01f, &bus};
    rl_load_model_t load = {load_R, load_L, &bus, 0.0f, {0}};

    // --- CSV File Output ---
    std::ofstream csv_file("simulation_results.csv");
    if (!csv_file.is_open())
    {
        std::cerr << "Error: Could not open simulation_results.csv for writing." << std::endl;
        return;
    }

    // Write header
    csv_file << "Time,Va,Vb,Vc,Ia,Ib,Ic,Ea,Eb,Ec,Theta,Sector\n";
    csv_file << std::fixed << std::setprecision(4);

    std::cout << "Running simulation for " << sim_duration << " seconds..." << std::endl;

    // --- Simulation Loop ---
    while (time_accum < sim_duration)
    {
        // 1. Update Mechanics (Constant Speed)
        load.theta = 2.0f * M_PI * speed_hz * time_accum;

        // 2. Update Back-EMF (Trapezoidal)
        load.e[0] = bemf_k * get_emf(load.theta);
        load.e[1] = bemf_k * get_emf(load.theta - 2.0f * M_PI / 3.0f);
        load.e[2] = bemf_k * get_emf(load.theta + 2.0f * M_PI / 3.0f);

        // 3. Determine Sector and Commutation (6-step)
        // Map angle to 6 sectors (0..5)
        float angle_deg = fmodf(load.theta * 180.0f / M_PI, 360.0f);
        if (angle_deg < 0) angle_deg += 360.0f;
        int sector = (int)(angle_deg / 60.0f);

        float duty[3] = {0, 0, 0};
        int drive[3] = {0, 0, 0};
        float target_duty = 0.5f; // 50% PWM Duty

        // Standard 6-step commutation sequence based on the BEMF sectors
        switch (sector)
        {
        case 0: // Sector 1: A+ C- (B float)
            duty[0] = target_duty; drive[0] = 1;
            duty[2] = 0.0f;        drive[2] = 1;
            break;
        case 1: // Sector 2: B+ C- (A float)
            duty[1] = target_duty; drive[1] = 1;
            duty[2] = 0.0f;        drive[2] = 1;
            break;
        case 2: // Sector 3: B+ A- (C float)
            duty[1] = target_duty; drive[1] = 1;
            duty[0] = 0.0f;        drive[0] = 1;
            break;
        case 3: // Sector 4: C+ A- (B float)
            duty[2] = target_duty; drive[2] = 1;
            duty[0] = 0.0f;        drive[0] = 1;
            break;
        case 4: // Sector 5: C+ B- (A float)
            duty[2] = target_duty; drive[2] = 1;
            duty[1] = 0.0f;        drive[1] = 1;
            break;
        case 5: // Sector 6: A+ B- (C float)
            duty[0] = target_duty; drive[0] = 1;
            duty[1] = 0.0f;        drive[1] = 1;
            break;
        }

        // 4. Inverter Step (Apply voltages)
        inverter_step(&inv, duty, drive, dt, time_accum);
        int driven_cnt = 0;
        int vsum = 0;
        for(int i = 0; i < 3; i++)
        {
            if(inv.e_bus->driven[i])
            {
                vsum += inv.e_bus->v[i] - load.e[i];
                driven_cnt++;
            }
        }
        load.vn = vsum / driven_cnt;

        // 5. Load Step (Calculate currents)
        rk4_step(bus.i, 3, dt, rle_load_deriv, time_accum, &load);

        for(int i = 0; i < 3; i++)
        {
            if(!inv.e_bus->driven[i])
            {
                bus.i[i] = 0;
                bus.v[i] = load.vn - load.e[i];
            }
        }
        
        // 6. Write data to CSV
        csv_file << time_accum << ","
                 << bus.v[0] << "," << bus.v[1] << "," << bus.v[2] << ","
                 << bus.i[0] << "," << bus.i[1] << "," << bus.i[2] << ","
                 << load.e[0] << "," << load.e[1] << "," << load.e[2] << ","
                 << load.theta << "," << sector << "\n";

        time_accum += dt;
    }

    csv_file.close();
    std::cout << "Simulation finished. Data saved to simulation_results.csv" << std::endl;
}

int main()
{
    run_simulation_and_save_csv();
    return 0;
}