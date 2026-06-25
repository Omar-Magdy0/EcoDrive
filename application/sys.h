#pragma once

#include "PmsmControlCore.h"
#include "platform.h"
#include "aebfStream.h"
#include <cstdint>
#include <array>
#include <string.h>
#include <stdio.h>
//=======================================================


#include "eldriver/eldriver_core.h"

class Sys
{
    inline static eldriver_core_t core;

    public:
    static void init(void)
    {   
        platform_init();
        eldriver_core_init(&core);
        //enable motor control function

        //Initial Control mode and pwm frequency  
        motor_c1.pwm_freq_hz = 25000;
        //mc3p hardware and pwm generation configuration
        motor_c1.mc3p.offset_calibration = true;
        motor_c1.mc3p.config.deadtime_nS = 1000;
        motor_c1.mc3p.config.duty_min = 0;
        motor_c1.mc3p.config.duty_max = 0.97;
        //sensor configurations
        
        //Control modes configuration 
        //Open loop startup sequence
        constexpr float time[PmsmControlCore::Trap::OLSTUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        constexpr float EC_TRAP[PmsmControlCore::Trap::OLSTUP_TABLE_SIZE] = {3, 6, 12, 16, 20, 24};
        constexpr float EC_FOC[PmsmControlCore::Foc::OLSTUP_TABLE_SIZE] = {3, 5, 7, 9, 10.5, 11.8};
        constexpr float RPM[PmsmControlCore::Foc::OLSTUP_TABLE_SIZE] = {0, 100, 200, 300, 400, 500};

        //Motor model
        motor_c1.model.pole_pairs = 6;
    
        //Trapezoidal
        motor_c1.trap.run_mode = PmsmControlCore::Trap::RunMode::OL;
        motor_c1.trap.I_Kp = 0;
        motor_c1.trap.I_Kd = 0;
        motor_c1.trap.I_Ki = 0;

        //Foc
        motor_c1.foc.run_mode = PmsmControlCore::Foc::RunMode::OL;
        motor_c1.foc.Id_Kp = 0;
        motor_c1.foc.Id_Ki = 0;
        motor_c1.foc.Id_Kd = 0;
        motor_c1.foc.Iq_Kp = 0;
        motor_c1.foc.Iq_Ki = 0;
        motor_c1.foc.Iq_Kd = 0;

        //Config methods
        motor_c1.Trap_olstup_cfg(24, time, EC_TRAP, RPM);
        motor_c1.Foc_olstup_cfg(24, time, EC_FOC, RPM);
        motor_c1.setControlMode(PmsmControlCore::MCType::SComm,
                                 PmsmControlCore::ECType::Voltage);
        motor_c1.init();
        //motor_c1.Foc_olstup_start();
        #ifndef PLATFORM_HOST

        #else
        freeRtos_init();
        gui_loop();
        #endif
    }
};



