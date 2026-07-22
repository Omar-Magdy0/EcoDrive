#pragma once
#include "PmsmControlCore.h"
#include "PmsmControlTypes.h"
#include "DAQSessionAPP.h"

class PmsmControl
{
private:
    PmsmControlCore mc;
    friend void el_mc3p_sync_postScanCallback();
    friend void el_xmc3p_tickerCallback();
    void(*onFault)(void *ctx);
public:
    static constexpr uint8_t olstup_tb_size() {return PmsmControlTypes::OLSTUP_TABLE_SIZE;};
    // Exposed API
    PmsmControlTypes::ERR init();

    // Control
    PmsmControlTypes::ERR setSetpoint(float sp, PmsmControlTypes::MechMode mech_mode = PmsmControlTypes::MechMode::SKIP);
    PmsmControlTypes::ERR estop(PmsmControlTypes::ConfigEstop cfg);

    // Configuration
    PmsmControlTypes::ERR configControlMode(PmsmControlTypes::MCMode mc_mode, PmsmControlTypes::MechMode mech_mod);
    PmsmControlTypes::ERR configElecLimits(PmsmControlTypes::ConfigElecLimits elim);
    PmsmControlTypes::ERR configMechLimits(PmsmControlTypes::ConfigMechLimits mechlim);
    PmsmControlTypes::ERR configPwm(PmsmControlTypes::ConfigPwm cfg);
    PmsmControlTypes::ERR configFocPid(PmsmControlTypes::Pid Id_pid, PmsmControlTypes::Pid Iq_pid);
    PmsmControlTypes::ERR configFocOlstup(PmsmControlTypes::ConfigOlstup cfg);
    PmsmControlTypes::ERR configSComm(PmsmControlTypes::ConfigSComm cfg);

    // Status and Error
    PmsmControlTypes::ERR getFault() const;
    PmsmControlTypes::ERR setOnFault(void(*onFault)(void*));
    PmsmControlTypes::ERR clearFault();
};

extern PmsmControl pmsmC1;