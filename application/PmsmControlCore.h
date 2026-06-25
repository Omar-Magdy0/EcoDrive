
#pragma once
#include "arm_math.h"
#include "core/elmath.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "eldriver/eldriver_core.h"
#include "sensor.h"
#include <array>
#include <cstdint>
#include <cstddef>
#include "PmsmControl_Conf.h"

#define READ_VOLATILE(x) (*(volatile typeof(x) *)&(x))

#ifdef ELDRIVER_HALL1_ENABLED
#define HALL_ENABLED
#else
#define BEMFZC_ENABLED
#endif

struct PmsmControlCore
{
    enum class Direction : int8_t
    {
        Forward = 1,  /** Forward rotation. */
        Backward = -1 /** Reverse rotation. */
    };

    enum class MCType : uint8_t
    {
        None = 0,
        Idle,
        Fault,
        Foc, /** Open-loop V/Hz (V/F) commutation. */
        Trap,
        SComm /** Self-commissioning / parameter identification. */
    };

    enum class ECType : uint8_t
    {
        Voltage = 0,
        Current
    };

    enum class MECType : uint8_t
    {
        None = 0,
        Speed,
        Torque,
        Position,
        Power
    };

    enum class ERRType : uint8_t
    {
        None = 0,
        Overcurrent,
        Overvoltage,
        Overtemperature
    };

    enum SyncErrBits
    {
        SYNCBITS_SUCCESS = 0,
        SYNCBITS_BAD_PWM_FREQ = 1 << 1
    };

    enum DirtyPits
    {
        DIRTYBITS_PWM    = 1 << 0,
        DIRTYBITS_ADC    = 1 << 1,  // 0x02
        DIRTYBITS_CTRL   = 1 << 2,  // 0x04
        DIRTYBITS_SP     = 1 << 3,  // 0x08
    };

    #define RPM_TO_RPS(rpm)(rpm * (2*M_PI/60.0))

    volatile uint32_t pwmTicks{};                                                       /** PWM period in ticks (timer ticks). */
    uint32_t pwm_freq_hz{10000};                                    /** PWM carrier frequency (Hz). */
    uint32_t tick_period_ns{ (uint32_t)(1'000'000'000.0f / static_cast<float>(10000)) }; /** PWM tick period (us). */
    uint32_t tick_period_us{tick_period_ns / 1000};                         /** PWM tick period (us). */
    uint32_t pwm_freq() const { return pwm_freq_hz; }                          /** Get PWM frequency (Hz). */
    float pwm_tick_period_us() const { return tick_period_us; }                /** Get PWM tick period (us). */
    float pwm_tick_period_ms() const { return tick_period_ns/1000'000.0f; }                /** Get PWM tick period (ms). */
    float us_to_ticks(float us) const { return us / tick_period_us; }          /** us -> ticks. */
    float ms_to_ticks(float ms) const { return us_to_ticks(ms * 1000.0f); }    /** ms -> ticks. */
    float ticks_to_us(float ticks) const { return ticks * tick_period_us; }    /** ticks -> us. */
    float ticks_to_ms(float ticks) const { return ticks * pwm_tick_period_ms(); }    /** ticks -> ms. */
    float voltage_q31_to_float(q31_t voltage_q31) const { return ELDRIVER_MC3P_VS_TO_FLOAT(voltage_q31); }
    float current_q31_to_float(q31_t current_q31) const { return ELDRIVER_MC3P_CS_TO_FLOAT(current_q31); }
    float angle_q31_to_float(q31_t eAngle_q31) const { return (float)eAngle_q31 * (M_PI / INT32_MAX);}

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
        volatile q31_t Vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(15);                                               /** DC bus voltage (V). */
        volatile q31_t Ibus_q31;
        volatile Direction dir{Direction::Forward};               /** Commanded rotation direction. */
        volatile float eAngv_RPS{};                                        /** Electrical speed (rad per second). */
        volatile q31_t eTheta_q31;                                      /** Electrical angle (rad) (-Pi to Pi https://arm-software.github.io/CMSIS-DSP/v1.14.0/group__SinCos.html#gae9e4ddebff9d4eb5d0a093e28e0bc504). */
        volatile q31_t eAngv_RPT_q31{};                                       /** Electrical speed (rad per pwm tick). */
    } state;

    struct
    {
        float Rs;           /** Stator resistance (Ohm). */
        float Ld;           /** D-axis inductance (H). */
        float Lq;           /** Q-axis inductance (H). */
        float Ke;           /** Back-EMF constant. */
        float J;            /** Rotor inertia. */
        float B;            /** Viscous friction coefficient. */
        uint8_t pole_pairs = 1; /** Motor pole pairs (electrical). */
    } model;

    protected:
    struct
    {
        MCType mctype{MCType::None}; /** Current control mode. */
        ECType ectype{ECType::Voltage}; /** Current error type. */
    } control;

    public:

#include "PmsmControlCore_Trap.h"
#include "PmsmControlCore_Foc.h"
#include "PmsmControlCore_SComm.h"

    void init();
    void setControlMode(MCType mctype, ECType ectype);
    void pwmLoop();
    void xmcLoop();
    uint32_t isync();
};

extern PmsmControlCore motor_c1;