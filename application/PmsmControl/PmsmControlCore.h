
#pragma once
#include "arm_math.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "eldriver/eldriver_core.h"
#include "PosDriver.h"
#include "elmath.h"
#include <array>
#include <cstdint>
#include <cstddef>
#include "PmsmControlTypes.h"
#include "PmsmControlConf.h"

#define READ_VOLATILE(x) (*(volatile typeof(x) *)&(x))

#ifdef ELDRIVER_HALL1_ENABLED
#define HALL_ENABLED
#endif
class PmsmControl;
class PmsmControlCore
{
    //Api Layer
    friend PmsmControl;
    //Position drivers
    friend PosDriver;
    friend PosOpen;

    #define RPM_TO_RPS(rpm)(rpm * (2*M_PI/60.0))
    volatile uint32_t xTicks;
    const uint32_t xTicks_period_us = (uint32_t)ELDRIVER_XMC3P_TICKPERIOD_US;
    volatile uint32_t pTicks{};                                                       /** PWM period in ticks (timer ticks). */
    uint32_t pwm_freq_hz;                                               /** PWM carrier frequency (Hz). */
    uint32_t pTick_period_ns{ (uint32_t)(1'000'000'000.0f / static_cast<float>(10000)) }; /** PWM tick period (us). */
    float voltage_q31_to_float(q31_t voltage_q31) const { return ELDRIVER_MC3P_VS_TO_FLOAT(voltage_q31); }
    float current_q31_to_float(q31_t current_q31) const { return ELDRIVER_MC3P_CS_TO_FLOAT(current_q31); }
    float angle_q31_to_float(q31_t eAngle_q31) const { return (float)eAngle_q31 * (M_PI / INT32_MAX);}
    inline uint32_t xTicks_to_us(uint32_t ticks)const{return ticks*xTicks_period_us;};
    inline uint32_t xTicks_to_ms(uint32_t ticks)const{return xTicks_to_us(ticks)/1000;};
    inline int32_t rps_to_rpxt(float rps){    
        return  (int32_t)((rps) * (ELDRIVER_XMC3P_TICKPERIOD_US / 1000000.0f / PI) * (1<<24));
    }
    inline float rpxt_to_rps(int32_t rpxt){
        return ((float)rpxt * ((M_PI*1000000.0f)/ELDRIVER_XMC3P_TICKPERIOD_US))/(1<<24);
    }
    // Owned hardware elements
    eldriver_mc3p_handle_t mc3p{}; /** Underlying MC3P driver instance. */
    PosDriver posDriver;
    union
    {
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    } mc3p_sync_meas{};

    // Control variables
    struct
    {
        volatile q31_t Vbus_q31;                                               /** DC bus voltage (V). */
        volatile q31_t Ibus_q31;
        volatile float eAngv_RPS{};                                        /** Electrical speed (rad per second). */
        volatile q31_t eTheta_q31;                                      /** Electrical angle (rad) (-Pi to Pi https://arm-software.github.io/CMSIS-DSP/v1.14.0/group__SinCos.html#gae9e4ddebff9d4eb5d0a093e28e0bc504). */
        volatile q31_t eAngv_RPT_q31{};                                       /** Electrical speed (rad per pwm tick). */
        volatile int64_t mechTheta_q32p31;
        volatile int64_t mechTheta_sp_q32p31;
        volatile int32_t mechAngv_RPXT_q7p24;                              /** Mechanical Angular velocity rad per xTick timebase */
        volatile int32_t mechAngv_RPXT_sp_q7p24;
        volatile int32_t torque_sp;
    } state;

    struct Olstup
    {
        bool is_complete = false;
        uint32_t time_start_ms = 0;
        uint8_t tb_index = 0;
        PmsmControlTypes::ConfigOlstup cfg;
    };

    PmsmControlTypes::Model model;
    PmsmControlTypes::ConfigElecLimits eleclim;
    PmsmControlTypes::ConfigMechLimits mechlim;
    PmsmControlTypes::ERR fault;

    struct
    {
        arm_pid_instance_q31 position_pid;
        arm_pid_instance_q31 speed_pid;
        PmsmControlTypes::MCMode mc_mode; /** Current control mode. */
        PmsmControlTypes::MechMode mech_mode; /** Mechanical control type */
        PmsmControlTypes::ElecMode elec_mode; /** Current error type. */
        PmsmControlTypes::RunMode run_mode;
        bool mpta_active = false;
    } control;

//#include "PmsmControlCore_Trap.h"
#include "PmsmControlCore_Foc.h"
#include "PmsmControlCore_SComm.h"

    void init();
    void setControlMode(PmsmControlTypes::MCMode mc_mode);
    void pwmConfigUpdate(PmsmControlTypes::ConfigPwm cfg);
    void olstup_lut_run(Olstup&, q31_t&, float&);
    PmsmControlCore(): posDriver(*this){};
    
    public:
    void pwmLoop();
    void xmcLoop();
};
