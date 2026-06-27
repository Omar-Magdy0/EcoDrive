
#pragma once
#include "arm_math.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "eldriver/eldriver_core.h"
#include "sensor.h"
#include "elmath.h"
#include <array>
#include <cstdint>
#include <cstddef>
#include "PmsmControlTypes.h"
#include "PmsmControlConf.h"

#define READ_VOLATILE(x) (*(volatile typeof(x) *)&(x))

#ifdef ELDRIVER_HALL1_ENABLED
#define HALL_ENABLED
#else
#define BEMFZC_ENABLED
#endif
class PmsmControl;
class PmsmControlCore
{
    friend PmsmControl;
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

    // Owned hardware elements
    eldriver_mc3p_t mc3p{}; /** Underlying MC3P driver instance. */
    PosSensor pos_sensor{}; /** Rotor position sensor interface. */
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
        volatile PmsmControlTypes::Direction dir;               /** Commanded rotation direction. */
        volatile float eAngv_RPS{};                                        /** Electrical speed (rad per second). */
        volatile q31_t eTheta_q31;                                      /** Electrical angle (rad) (-Pi to Pi https://arm-software.github.io/CMSIS-DSP/v1.14.0/group__SinCos.html#gae9e4ddebff9d4eb5d0a093e28e0bc504). */
        volatile q31_t eAngv_RPT_q31{};                                       /** Electrical speed (rad per pwm tick). */
    } state;

    PmsmControlTypes::Model model;
    PmsmControlTypes::ConfigElecLimits eleclim;
    PmsmControlTypes::ConfigMechLimits mechlim;
    PmsmControlTypes::ERR fault;

    protected:
    struct
    {
        PmsmControlTypes::MCMode mc_mode; /** Current control mode. */
        PmsmControlTypes::MechMode mech_mode; /** Mechanical control type */
        PmsmControlTypes::ElecMode elec_mode; /** Current error type. */
    } control;

    public:

#include "PmsmControlCore_Trap.h"
#include "PmsmControlCore_Foc.h"
#include "PmsmControlCore_SComm.h"

    void init();
    void setControlMode(PmsmControlTypes::MCMode mc_mode, PmsmControlTypes::ElecMode ectype);
    void pwmLoop();
    void xmcLoop();
    void pwmConfigUpdate(PmsmControlTypes::ConfigPwm cfg);
};
