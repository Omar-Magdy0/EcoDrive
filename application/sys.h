#pragma once

#include "PmsmControl/PmsmControl.h"
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

        ////Initial Control mode and pwm frequency  
        //cntrl1.pwm_freq_hz = 25000;
        ////mc3p hardware and pwm generation configuration
        //cntrl1.mc3p.offset_calibration = true;
        //cntrl1.mc3p.config.deadtime_nS = 1000;
        //cntrl1.mc3p.config.duty_min = 0;
        //cntrl1.mc3p.config.duty_max = 0.97;
        ////sensor configurations
        
        //Control modes configuration 
        //Open loop startup sequence
        constexpr float time[PmsmControl::olstup_tb_size()] = {0, 2, 4, 6, 8, 10};
        constexpr float EC_TRAP[PmsmControl::olstup_tb_size()] = {3, 6, 12, 16, 20, 24};
        constexpr float EC_FOC[PmsmControl::olstup_tb_size()] = {3, 5, 7, 9, 10.5, 11.8};
        constexpr float RPM[PmsmControl::olstup_tb_size()] = {0, 100, 200, 300, 400, 500};

        //Motor model
        //cntrl1.model.pole_pairs = 6;
    //
        ////Trapezoidal
        //cntrl1.trap.run_mode = PmsmControlCore::Trap::RunMode::OL;
        //cntrl1.trap.I_Kp = 0;
        //cntrl1.trap.I_Kd = 0;
        //cntrl1.trap.I_Ki = 0;
//
        ////Foc
        //cntrl1.foc.run_mode = PmsmControlCore::Foc::RunMode::OL;
        //cntrl1.foc.Id_Kp = 0;
        //cntrl1.foc.Id_Ki = 0;
        //cntrl1.foc.Id_Kd = 0;
        //cntrl1.foc.Iq_Kp = 0;
        //cntrl1.foc.Iq_Ki = 0;
        //cntrl1.foc.Iq_Kd = 0;
//
        ////Config methods
        //cntrl1.Trap_olstup_cfg(24, time, EC_TRAP, RPM);
        //cntrl1.Foc_olstup_cfg(24, time, EC_FOC, RPM);
        //cntrl1.setControlMode(PmsmControlCore::MCType::SComm,
        //                         PmsmControlCore::ECType::Voltage);
        //cntrl1.init();
        //cntrl1.Foc_olstup_start();
        #ifndef PLATFORM_HOST

        #else
        gui_loop();
        #endif
    }
};



