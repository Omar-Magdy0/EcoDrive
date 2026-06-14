#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>


#include <fstream>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>   // for setprecision (optional)

#include "sil.h"

extern Sil sil;
constexpr double PWM_FREQ = 20000;
constexpr double TEST_SEC = 10;
constexpr double OCV_OMEGA = 10;
constexpr double OLR_OMEGA = 50;
constexpr double OLR_VOLTAGE = 4.5;
constexpr double LRT_VOLTAGE = 2;
constexpr double LRT_OMEGA = 50;

void write_svpwm(Sil::Input& in, double V_alpha, double V_beta)
{
    // All phases driven for SVPWM
    for(int i = 0; i < 3; i++) {
        in.drive[i] = 1;
    }
    
    // Clarke transform to phase voltages
    double va = V_alpha;
    double vb = -0.5 * V_alpha + 0.8660254 * V_beta;   // -0.5*Va + sqrt(3)/2 * Vb
    double vc = -0.5 * V_alpha - 0.8660254 * V_beta;   // -0.5*Va - sqrt(3)/2 * Vb
    
    // Normalize to duty cycles [0,1] assuming V_alpha,V_beta range is [-Vcc/2, Vcc/2]
    // SVPWM with common mode injection
    double vmin = std::min({va, vb, vc});
    double vmax = std::max({va, vb, vc});
    double vcom = (vmin + vmax) / 2.0;
    
    // Add common mode and convert to duty
    in.duty[0] = (va - vcom) / in.vcc + 0.5;
    in.duty[1] = (vb - vcom) / in.vcc + 0.5;
    in.duty[2] = (vc - vcom) / in.vcc + 0.5;
    
    // Clamp duties to [0,1]
    for(int i = 0; i < 3; i++) {
        in.duty[i] = std::clamp(in.duty[i], 0.0, 1.0);
    }
}

void write_trap(Sil::Input& in, unsigned int sector, double V_sect)
{
    sector = (sector - 1)%6 + 1;
    double duty = V_sect/in.vcc;
    duty = std::clamp(duty, 0.0, 1.0);
    switch(sector)
    {
        case 1:
            in.drive[0] = 1; in.drive[1] = 1; in.drive[2] = 0;
            in.duty[0] = duty; in.duty[1] = 0; in.duty[2] = 0;
            break;
        case 2:
            in.drive[0] = 1; in.drive[1] = 0; in.drive[2] = 1;
            in.duty[0] = duty; in.duty[1] = 0; in.duty[2] = 0;
            break;
        case 3:
            in.drive[0] = 0; in.drive[1] = 1; in.drive[2] = 1;
            in.duty[0] = 0; in.duty[1] = duty; in.duty[2] = 0;
            break;
        case 4:
            in.drive[0] = 1; in.drive[1] = 1; in.drive[2] = 0;
            in.duty[0] = 0; in.duty[1] = duty; in.duty[2] = 0;
            break;
        case 5:
            in.drive[0] = 1; in.drive[1] = 0; in.drive[2] = 1;
            in.duty[0] = 0; in.duty[1] = 0; in.duty[2] = duty;
            break;
        case 6:
            in.drive[0] = 0; in.drive[1] = 1; in.drive[2] = 1;
            in.duty[0] = 0; in.duty[1] = 0; in.duty[2] = duty;
            break;
        default:
            break;
    }
}

inline double trapezoidal_wave(double theta)
{
    static const double angles[] = {0, M_PI/6, 5*M_PI/6, M_PI, 7*M_PI/6, 11*M_PI/6, 12*M_PI/6};
    static const double values[] = {0, 1     , 1       , 0   , -1      , -1       , 0        };
    theta = fmod(theta, 2*M_PI);
    if (theta < 0) theta += 2*M_PI;
    int cval_idx, nval_idx = 0;
    int arr_size = sizeof(angles)/sizeof(double);
    for(int i = 0; i < arr_size; i++) {
        if(theta >= angles[i]) {
            cval_idx = i;
        }
    }
    nval_idx = (cval_idx+1)%arr_size;
    double m = (values[nval_idx] - values[cval_idx]) / (angles[nval_idx] - angles[cval_idx]);
    return values[cval_idx] + m * (theta - angles[cval_idx]);
}
inline void dPsim_dTheta(double theta_e, double dPsi_dTheta[3])
{
    double amplitude = sil.param.motor_fluxLinkage; // Back EMF amplitude
    #ifdef SINE_BEMF
    dPsi_dTheta[0] = - amplitude * sin(theta_e);
    dPsi_dTheta[1] = - amplitude * sin(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * sin(theta_e + 2*M_PI/3);
    #else
    // Trapezoidal waveform function (120° flat top, 60° rising/falling edges)
    dPsi_dTheta[0] = - amplitude * trapezoidal_wave(theta_e);
    dPsi_dTheta[1] = - amplitude * trapezoidal_wave(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * trapezoidal_wave(theta_e + 2*M_PI/3);
    #endif
};



void gen_test();
void lrt_test();
void ocv_test();
void olr1_test();
void olr2_test();
void foc_test();
void dc_test();

void trapezoidal_wave_test()
{
    std::ofstream file("trapezoidal_wave.csv");
    file << "Theta,Value\n";
    for(double theta = 0; theta <= 10*M_PI; theta += 0.01) {
        file << theta << "," << trapezoidal_wave(theta) << "\n";
    }
    file.close();
    std::cout << "Trapezoidal wave test complete. Data saved to trapezoidal_wave.csv\n";
}

int main()
{
    // Configure motor parameters
    sil.param.motor_Rs = 0.2;
    sil.param.motor_Ls = 0.001;
    sil.param.motor_Lm = 0.0001;
    sil.param.motor_Ms = 0.0003;
    sil.param.motor_fluxLinkage = 0.1;
    sil.param.motor_pp = 4;
    sil.param.motor_J = 0.001;
    sil.param.motor_B = 0.001;
    sil.param.inv_Ron = 0.01;
    sil.param.inv_Roff = 1e5;
    sil.param.motor_rotorOffset = 0;
    sil.param.dPsim_dTheta = dPsim_dTheta;

    trapezoidal_wave_test();
    ocv_test();
    dc_test();
    lrt_test();
    gen_test();
    olr1_test();
    olr2_test();
    return 0;
}

void ocv_test()
{
    Sil::Input in = {0};
    sil.state = Sil::State();
    in.dt = 1.0 / PWM_FREQ;
    in.drive[0] = 0;  // All phases floating
    in.drive[1] = 0;
    in.drive[2] = 0;
    in.duty[0] = 0;
    in.duty[1] = 0;
    in.duty[2] = 0;
    in.vcc = 0;  // No applied voltage
    in.load_torque = 0;
    sil.param.load_J = 0;
    sil.param.load_B = 0;
    
    // Initialize at rest
    sil.state.theta = 0;
    double sil_theta = 0;
    sil.state.omega = 0;
    sil.state.ip[0] = sil.state.ip[1] = sil.state.ip[2] = 0;
    
    // Log OCV data
    sil.logger.start("ocv_test.csv");
    
    //Start Test by forcing omega to a value
    for(int i = 0; i < TEST_SEC * PWM_FREQ; i++)
    {
        sil.step(in);
        sil.logger.log(sil.state);
        sil.state.omega = OCV_OMEGA;
        sil_theta += sil.state.omega * in.dt;
        sil.state.theta = sil_theta;
    }

    sil.logger.stop();
    std::cout << "\nOCV test complete. Data saved to ocv_test.csv\n";
}

void lrt_test()
{
    Sil::Input in = {0};
    sil.state = Sil::State();
    in.dt = 1.0 / PWM_FREQ;
    in.load_torque = 1;
    sil.param.load_J = 1000;
    sil.param.load_B = 1000;
    in.vcc = 12;
    
    // Initialize at rest
    sil.state.theta = 0;
    sil.state.omega = 0;
    sil.state.ip[0] = sil.state.ip[1] = sil.state.ip[2] = 0;
    
    // Log data
    sil.logger.start("lrt_test.csv");
    double ramp_time = TEST_SEC * 0.8;
    double omega_ef = 0;
    double theta_e_f = 0;
    double v_mag = 0;
    unsigned int sector = 1;

    omega_ef = LRT_OMEGA * sil.param.motor_pp;
    v_mag = LRT_VOLTAGE;
    //Start Test by forcing omega to a value
    for(int i = 0; i < TEST_SEC * PWM_FREQ; i++)
    {
        theta_e_f += omega_ef * in.dt;
        double v_alpha = v_mag * cos(theta_e_f);
        double v_beta  = v_mag * sin(theta_e_f);
        write_svpwm(in, v_alpha, v_beta);
        sil.step(in);
        sil.logger.log(sil.state);
    }

    sil.logger.stop();
    std::cout << "\nLRT test complete. Data saved to lrt_test.csv\n";
}

void olr1_test()
{
    Sil::Input in = {0};
    sil.state = Sil::State();
    in.dt = 1.0 / PWM_FREQ;
    in.load_torque = 0.05;
    sil.param.load_J = 0.001;
    sil.param.load_B = 0.001;
    in.vcc = 12;
    
    // Initialize at rest
    sil.state.theta = 0;
    sil.state.omega = 0;
    sil.state.ip[0] = sil.state.ip[1] = sil.state.ip[2] = 0;
    
    // Log data
    sil.logger.start("olr1_test.csv");
    double ramp_time = TEST_SEC * 0.8;
    double omega_ef = 0;
    double theta_e_f = 0;
    double v_mag = 0;
    unsigned int sector = 1;


    //Start Test by forcing omega to a value
    for(int i = 0; i < TEST_SEC * PWM_FREQ; i++)
    {
        if(sil.state.time <= ramp_time)
        {
            omega_ef = (OLR_OMEGA*sil.param.motor_pp) * (sil.state.time / ramp_time);
            v_mag = OLR_VOLTAGE * (0.6 + 0.4 * (sil.state.time / ramp_time));
        }
        theta_e_f += omega_ef * in.dt;
        if(theta_e_f >= 2 * M_PI) theta_e_f -= 2 * M_PI;
        // Determine sector (1-6) based on electrical angle
        sector = (unsigned int)(theta_e_f / (M_PI / 3.0)) + 1;
        if(sector > 6) sector = 1;
        write_trap(in, sector, v_mag);
        sil.step(in);
        sil.logger.log(sil.state);
    }

    sil.logger.stop();
    std::cout << "\nOLR1 test complete. Data saved to olr1_test.csv\n";
}

void olr2_test()
{
    Sil::Input in = {0};
    sil.state = Sil::State();
    in.dt = 1.0 / PWM_FREQ;
    in.load_torque = 0.05;
    sil.param.load_J = 0.001;
    sil.param.load_B = 0.001;
    in.vcc = 12;
    
    // Initialize at rest
    sil.state.theta = 0;
    sil.state.omega = 0;
    sil.state.ip[0] = sil.state.ip[1] = sil.state.ip[2] = 0;
    
    // Log data
    sil.logger.start("olr2_test.csv");
    double ramp_time = TEST_SEC * 0.8;
    double omega_ef = 0;
    double theta_e_f = 0;
    double v_mag = 0;
    unsigned int sector = 1;


    //Start Test by forcing omega to a value
    for(int i = 0; i < TEST_SEC * PWM_FREQ; i++)
    {
        if(sil.state.time <= ramp_time)
        {
            omega_ef = (OLR_OMEGA*sil.param.motor_pp) * (sil.state.time / ramp_time);
            v_mag = OLR_VOLTAGE * (0.6 + 0.4 * (sil.state.time / ramp_time));
        }
        theta_e_f += omega_ef * in.dt;
        double v_alpha = v_mag * cos(theta_e_f);
        double v_beta  = v_mag * sin(theta_e_f);
        write_svpwm(in, v_alpha, v_beta);
        sil.step(in);
        sil.logger.log(sil.state);
    }

    sil.logger.stop();
    std::cout << "\nOLR2 test complete. Data saved to olr2_test.csv\n";
}

void dc_test()
{
    Sil::Input in = {0};
    sil.state = Sil::State();
    in.dt = 1.0 / PWM_FREQ;
    in.drive[0] = 1;  
    in.drive[1] = 1;
    in.drive[2] = 1;
    in.duty[0] = 1.0/12;
    in.duty[1] = 0;
    in.duty[2] = 0;
    in.vcc = 12;  
    in.load_torque = 0;
    sil.param.load_J = 0;
    sil.param.load_B = 0;
    
    // Initialize at rest
    sil.state.theta = 0;
    sil.state.omega = 0;
    sil.state.ip[0] = sil.state.ip[1] = sil.state.ip[2] = 0;
    
    // Log OCV data
    sil.logger.start("dc_test.csv");
    
    //Start Test by forcing omega to a value
    for(int i = 0; i < TEST_SEC * PWM_FREQ; i++)
    {
        sil.step(in);
        sil.logger.log(sil.state);
    }

    sil.logger.stop();
    std::cout << "\nDC test complete. Data saved to ocv_test.csv\n";
}

void gen_test()
{

}