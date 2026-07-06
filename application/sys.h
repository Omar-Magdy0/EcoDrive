#pragma once

#include "PmsmControl/PmsmControl.h"
#include "platform.h"
#include "ABFStream.h"
#include "eldriver/eldriver_usbxch.h"
#include <cstdint>
#include <array>
#include <string.h>
#include <stdio.h>
//=======================================================
#include "IDV.h"

#include "eldriver/eldriver_core.h"

inline eldriver_usbxch_handle_t usbxch;

class Sys
{
    inline static eldriver_core_t core;

    public:
    static void init(void)
    {   
        platform_init();
        eldriver_usbxch_init(&usbxch);
        eldriver_core_init(&core);
        //enable motor control function
        
        //Control modes configuration 
        //Open loop startup sequence
        constexpr uint16_t time[PmsmControl::olstup_tb_size()] = {0, 1000, 3000, 5000, 7000, 9000};
        constexpr float EC_FOC[PmsmControl::olstup_tb_size()] = {3, 5, 7, 9, 10.5, 13};
        constexpr float RPM[PmsmControl::olstup_tb_size()] = {0, 100, 200, 300, 400, 500};
        PmsmControlTypes::ConfigPwm pwm_cfg;
        PmsmControlTypes::ConfigOlstup foc_olstup_cfg;
        PmsmControlTypes::ConfigSComm scomm_cfg;
        PmsmControlTypes::ERR err = PmsmControlTypes::ERR::OK;

        pwm_cfg.pwm_freq_hz = 25000;
        pwm_cfg.deadtime_ns = 1000;
        pwm_cfg.duty_max = 0.95;
        pwm_cfg.duty_min = 0;
        foc_olstup_cfg.vbus_init = 20;
        memcpy(foc_olstup_cfg.time_ms_tb, time, sizeof(time));
        memcpy(foc_olstup_cfg.ec_tb, EC_FOC, sizeof(EC_FOC));
        memcpy(foc_olstup_cfg.rpm_tb, RPM, sizeof(RPM));
        scomm_cfg.dc_vinj = 1;
        scomm_cfg.hfi_N = 20;
        scomm_cfg.hfi_vinj = 12;
        err = pmsmC1.configControlMode(PmsmControlTypes::MCMode::Idle, PmsmControlTypes::MechMode::OpenSpeed);while(err != PmsmControlTypes::ERR::OK);
        err = pmsmC1.configPwm(pwm_cfg);while(err != PmsmControlTypes::ERR::OK);
        err = pmsmC1.configFocOlstup(foc_olstup_cfg);while(err != PmsmControlTypes::ERR::OK);
        err = pmsmC1.configSComm(scomm_cfg);while(err != PmsmControlTypes::ERR::OK);
        pmsmC1.init();
        err = pmsmC1.configControlMode(PmsmControlTypes::MCMode::SComm, PmsmControlTypes::MechMode::OpenSpeed);while(err != PmsmControlTypes::ERR::OK);
        err = pmsmC1.setSetpoint(300.0*(2*M_PI/60.0), PmsmControlTypes::MechMode::OpenSpeed);
        int madsad = sizeof(PmsmControl);

        while (true) {
            char sad[] = "*_SAD_*";
            eldriver_usbxch_write(&usbxch, (uint8_t*)sad, sizeof(sad));
            eldriver_usbxch_update(&usbxch);
            //eldriver_delay(1000);
            #ifdef PLATFORM_HOST
                gui_loop();
            #endif
        }
    }
};



