#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

// --- SIMULATION PARAMETERS ---
#define PWM_FREQ_HZ      20000.0f
#define DT               (1.0f / PWM_FREQ_HZ)   
#define V_BUS            48.0f
//Open Circuit test
static const float OC_TEST_TIME = 1;
static const float OC_TEST_OMEGA = 30;
const float R_LOAD = 10;
const float GEN_OMEGA = 100;
const float RAMP_TIME   = 8.0f;      // seconds to reach target speed
const float OLR_TIME    = 15;
const float MAX_FREQ_HZ = 3.0f;     
// Missing Output struct for Inverter (if not in header)

#include "inverter_model.h"
#include "pmsm_model.h"
#include <fstream>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>   // for setprecision (optional)




class PmsmTestLogger {
private:
    std::ofstream file;
    
    // references to the objects you want to log
    MotorState     &m_state;
    MotorOutput    &m_out;
    InverterState  &i_state;
    InverterOutput &i_out;

public:
    PmsmTestLogger(const char* filename,
                   MotorState&     m_state_ref,
                   MotorOutput&    m_out_ref,
                   InverterState&  i_state_ref,
                   InverterOutput& i_out_ref)
        : m_state(m_state_ref),
          m_out(m_out_ref),
          i_state(i_state_ref),
          i_out(i_out_ref),
          file(filename, std::ios::out)
    {
        if (!file.is_open()) {
            // you may want to handle error differently
            return;
        }

        // Write CSV header once (in constructor)
        file << std::fixed << std::setprecision(6);
        file << "time,"
             << "ia,ib,ic,"
             << "vthev_a,vthev_b,vthev_c,"
             << "rthev_a,rthev_b,rthev_c,"
             << "va,vb,vc,"
             << "vn,"
             << "omega," << "theta,"
             << "torque\n";
    }

    ~PmsmTestLogger() {
        if (file.is_open()) {
            file.close();
        }
    }

    // Call this in the loop — no arguments needed
    void log(float time) {
        if (!file.is_open()) return;

        file << time << ','
             << m_state.i[0] << ',' << m_state.i[1] << ',' << m_state.i[2] << ','
             << i_out.vthev[0] << ',' << i_out.vthev[1] << ',' << i_out.vthev[2] << ','
             << i_out.rthev[0] << ',' << i_out.rthev[1] << ',' << i_out.rthev[2] << ','
             << m_state.v[0]   << ',' << m_state.v[1]   << ',' << m_state.v[2]   << ','
             << m_state.vn     << ','
             << m_state.omega  << ',' << m_state.theta  << ','
             << m_out.T        << '\n';
    }
};

class MechSolver {
public:
    float &J;   // inertia pointer
    float &B;   // viscous friction pointer

    MechSolver(float &J_ptr, float &B_ptr) : J(J_ptr), B(B_ptr) {}

    // T     = electromagnetic torque (input)   [Nm]
    // alpha = angular acceleration (output)    [rad/s²]
    // omega = angular velocity  (in/out)       [rad/s]
    // theta = mechanical angle  (in/out)       [rad]
    // dt    = time step                        [s]
    void solve(float T_em, float T_load, float dt,
                 float &omega, float &theta)
    {
        // alpha = (T_em - T_load - B * omega) / J
        float alpha = (T_em - T_load - (B) * omega) / (J);

        // semi-implicit Euler (more stable than pure forward)
        omega += alpha * dt;
        theta += omega * dt;          // use updated omega
    }
};

void generator_test(InverterParams &ip, MotorParams &mp);
void lrt_test(InverterParams &ip, MotorParams &mp);
void ocv_test(InverterParams &ip, MotorParams &mp);
void olr_test(InverterParams &ip, MotorParams &mp);
void apply_spwm(float theta_e, InverterInput* in, float mod_idx);
void apply_trapezoidal_sector(float theta_e, InverterInput* in, float duty);



int main()
{
    // 1. Initialize Params
    MotorParams mp = { .Ld=0.0012f, .Lq=0.0012f, .Rs=0.35f, .pp=23.0f, .Ke=0.085f, .J=0.08f, .B=0.035f };
    InverterParams ip = { .Rdson=0.02f, .RdsOff=1000000.0f, .FWmin=0.2f };


    ocv_test(ip, mp);
    lrt_test(ip, mp);
    generator_test(ip, mp);
    olr_test(ip, mp);
    return 0;
}



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

void apply_spwm(float theta_e, InverterInput* in, float mod_idx) {
    float va = sinf(theta_e);
    float vb = sinf(theta_e - 2.0f * M_PI / 3);
    float vc = sinf(theta_e + 2.0f * M_PI / 3);

    // simple 3rd harmonic injection
    float vmin = fminf(fminf(va,vb),vc);
    float vmax = fmaxf(fmaxf(va,vb),vc);
    float vcom = (vmax + vmin) / 2.0f;
    va -= vcom; vb -= vcom; vc -= vcom;

    in->duty[0] = 0.5f + mod_idx * va * 0.5f;
    in->duty[1] = 0.5f + mod_idx * vb * 0.5f;
    in->duty[2] = 0.5f + mod_idx * vc * 0.5f;
    in->drive[0] = in->drive[1] = in->drive[2] = 1;
}


void ocv_test(InverterParams &ip, MotorParams &mp)
{
    // 2. Initialize States
    MotorState m_state = {0};
    MotorOutput m_out = {0};
    MotorInput m_in = {};
    InverterState i_state = {0};
    InverterInput i_in = { .vbus = V_BUS };
    InverterOutput i_out = {0};
    PmsmTestLogger logger("pmsm_oc_test.csv", 
                      m_state, m_out, i_state, i_out);
    MechSolver mech(mp.J, mp.B);
    float t = 0;
    m_state.omega = OC_TEST_OMEGA;
    m_state.theta = 0;
    for (int p = 0; p < PWM_FREQ_HZ * OC_TEST_TIME; p++)
    {
        for (int i = 0; i < 3; i++)
        {
            i_state.i[i] = m_state.i[i];
            i_state.v[i] = m_state.v[i];
        }
        i_in.drive[0] = 0;
        i_in.drive[1] = 0;
        i_in.drive[2] = 0;
        i_in.duty[0] = 0;
        i_in.duty[1] = 0;
        i_in.duty[2] = 0;
        inverter_step(&i_state, &i_in, &ip, &i_out);
        for (int i = 0; i < 3; i++)
        {
            m_in.rthev[i] = i_out.rthev[i];
            m_in.vthev[i] = i_out.vthev[i];
        }
        pmsm_step(&m_state, &m_in, &mp, DT, &m_state, &m_out);
        m_state.theta += m_state.omega * DT;
        logger.log(t);
        t += DT;
    }
    std::cout << "DONE! OPEN CIRCUIT TEST" << std::endl;
}


void lrt_test(InverterParams &ip, MotorParams &mp)
{
    // 2. Initialize States
    MotorState m_state = {0};
    MotorOutput m_out = {0};
    MotorInput m_in = {};
    InverterState i_state = {0};
    InverterInput i_in = { .vbus = V_BUS };
    InverterOutput i_out = {0};
    PmsmTestLogger logger("pmsm_lrt_test.csv", 
                      m_state, m_out, i_state, i_out);
    MechSolver mech(mp.J, mp.B);
    float t = 0;
    m_state.omega = 0;
    m_state.theta = 0;
    for (int p = 0; p < PWM_FREQ_HZ * OC_TEST_TIME; p++)
    {
        for (int i = 0; i < 3; i++)
        {
            i_state.i[i] = m_state.i[i];
            i_state.v[i] = m_state.v[i];
        }
        i_in.drive[0] = 1;
        i_in.drive[1] = 1;
        i_in.drive[2] = 1;
        i_in.duty[0] = 0.05 + 0.5;
        i_in.duty[1] = -0.05 + 0.5;
        i_in.duty[2] = -0.05 + 0.5;
        inverter_step(&i_state, &i_in, &ip, &i_out);
        for (int i = 0; i < 3; i++)
        {
            m_in.rthev[i] = i_out.rthev[i];
            m_in.vthev[i] = i_out.vthev[i];
        }
        pmsm_step(&m_state, &m_in, &mp, DT, &m_state, &m_out);
        m_state.theta += m_state.omega * DT;
        logger.log(t);
        t += DT;
    }
    std::cout << "DONE! LOCKED ROTOR TORQUE TEST" << std::endl;
}

void olr_test(InverterParams &ip, MotorParams &mp)
{
    // 2. Initialize States
    MotorState m_state = {0};
    MotorOutput m_out = {0};
    MotorInput m_in = {};
    InverterState i_state = {0};
    InverterInput i_in = { .vbus = V_BUS };
    InverterOutput i_out = {0};
    PmsmTestLogger logger("pmsm_olr_test.csv", 
                      m_state, m_out, i_state, i_out);
    MechSolver mech(mp.J, mp.B);
    float t = 0;
    m_state.omega = 0;
    m_state.theta = 0;
    for (int p = 0; p < OLR_TIME/DT; p++)
    {
        float progress = t/RAMP_TIME;
        if (progress > 1) progress = 1;
        float f_e = MAX_FREQ_HZ * progress;
        float theta_e = 2 * M_PI * f_e * t;
        float mod_idx = 0.05 + 0.05 * progress;

        for (int i = 0; i < 3; i++)
        {
            i_state.i[i] = m_state.i[i];
            i_state.v[i] = m_state.v[i];
        }
        apply_spwm(theta_e, &i_in, mod_idx);
        inverter_step(&i_state, &i_in, &ip, &i_out);
        for (int i = 0; i < 3; i++)
        {
            m_in.rthev[i] = i_out.rthev[i];
            m_in.vthev[i] = i_out.vthev[i];
        }
        pmsm_step(&m_state, &m_in, &mp, DT, &m_state, &m_out);
        mech.solve(m_out.T, 0, DT, m_state.omega, m_state.theta);
        logger.log(t);
        t += DT;
    }
    std::cout << "DONE! OPEN LOOP RAMP TEST" << std::endl;
}

void generator_test(InverterParams &ip, MotorParams &mp)
{
    // 2. Initialize States
    MotorState m_state = {0};
    MotorOutput m_out = {0};
    MotorInput m_in = {};
    InverterState i_state = {0};
    InverterInput i_in = { .vbus = V_BUS };
    InverterOutput i_out = {0};
    PmsmTestLogger logger("pmsm_gen_test.csv", 
                      m_state, m_out, i_state, i_out);
    MechSolver mech(mp.J, mp.B);
    float t = 0;
    m_state.omega = GEN_OMEGA;
    m_state.theta = 0;
    for (int p = 0; p < 3/DT; p++)
    {
        for (int i = 0; i < 3; i++)
        {
            i_state.i[i] = m_state.i[i];
            i_state.v[i] = m_state.v[i];
        }
        for (int i = 0; i < 3; i++)
        {
            m_in.rthev[i] = R_LOAD;
            m_in.vthev[i] = 0;
        }
        pmsm_step(&m_state, &m_in, &mp, DT, &m_state, &m_out);
        m_state.theta += m_state.omega * DT;
        logger.log(t);
        t += DT;
    }
    std::cout << "DONE! GENERATOR TEST" << std::endl;
}