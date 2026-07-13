#include "sys.h"





void Sys::init()
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
    
    DAQChannel channels[9];
    DAQStream streams[3];
    constexpr char chan[] = "CHAN";
    constexpr char str[] = "STR";
    
    for(int i = 0; i < 9; i++)
    {
        channels[i].Meta = chan;
        channels[i].meta_size = 4;
        channels[i].scale = i;
        channels[i].offset = -i;
    }

    
    for(int i = 0; i < 3; i++)
    {
        streams[i].channels = channels + i*3;
        streams[i].channels_num = 3;
        streams[i].Meta = str;
        streams[i].meta_size = 3;    
    }

    daq.streams = streams;
    daq.streams_num = 3;

    while (true) {
        eldriver_usbxch_update(&usbxch);
        uint16_t read_len = eldriver_usbxch_read(&usbxch, abfStream_rx_buffer, sizeof(abfStream_rx_buffer));
        abfStream.process(abfStream_rx_buffer, read_len);
        daq.process(nullptr, 0);
        eldriver_delay(1);
        #ifdef PLATFORM_HOST
            gui_loop();
        #endif
    }
}
