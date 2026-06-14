
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

struct PmsmControl
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
        Commission /** Self-commissioning / parameter identification. */
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


    #define RPM_TO_RPS(rpm)(rpm * (2*M_PI/60.0))

    volatile uint32_t pwmTicks{};                                                       /** PWM period in ticks (timer ticks). */
    uint32_t pwm_freq_hz{PWM_FREQ_DEFAULT};                                    /** PWM carrier frequency (Hz). */
    uint32_t tick_period_ns{ (uint32_t)(1'000'000'000.0f / static_cast<float>(PWM_FREQ_DEFAULT)) }; /** PWM tick period (us). */
    uint32_t tick_period_us{tick_period_ns / 1000.0f};                         /** PWM tick period (us). */
    uint32_t pwm_freq() const { return pwm_freq_hz; }                          /** Get PWM frequency (Hz). */
    float pwm_tick_period_us() const { return tick_period_us; }                /** Get PWM tick period (us). */
    float pwm_tick_period_ms() const { return tick_period_ns/1000'000.0f; }                /** Get PWM tick period (ms). */
    float us_to_ticks(float us) const { return us / tick_period_us; }          /** us -> ticks. */
    float ms_to_ticks(float ms) const { return us_to_ticks(ms * 1000.0f); }    /** ms -> ticks. */
    float ticks_to_us(float ticks) const { return ticks * tick_period_us; }    /** ticks -> us. */
    float ticks_to_ms(float ticks) const { return ticks * pwm_tick_period_ms(); }    /** ticks -> ms. */

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
        uint32_t vbus_mV = 12'000;                                               /** DC bus voltage (V). */
        uint32_t ibus_mA{};
        volatile Direction dir{Direction::Forward};                 /** Commanded rotation direction. */
        uint32_t eAngv_mRPS{};                                        /** Electrical speed (Hz). */
        uint32_t eTheta_cmR{};                                               /** Electrical angle (rad). */
    } state;

    struct
    {
        float Rs;           /** Stator resistance (Ohm). */
        float Ld;           /** D-axis inductance (H). */
        float Lq;           /** Q-axis inductance (H). */
        float Ke;           /** Back-EMF constant. */
        float J;            /** Rotor inertia. */
        float B;            /** Viscous friction coefficient. */
        uint8_t pole_pairs = DEFAULT_POLE_PAIRS; /** Motor pole pairs (electrical). */
    } model;

    struct
    {
        MCType mctype{MCType::None}; /** Current control mode. */
    } control;

#include "PmsmControl_Trap.h"

    void init()
    {
        pwmTicks = 0;
        control.mctype = MCType::None;

        pwm_freq_hz = PWM_FREQ_DEFAULT;
        tick_period_ns = 1'000'000'000.0f / static_cast<float>(pwm_freq_hz);
        tick_period_us = tick_period_ns / 1000.0f;
        /* instance bindings */

        /* mode initialization*/
        Trap_init();

        /* hardware initialization */
        pos_sensor.init(pwm_freq_hz);
        mc3p.offset_calibration = true;
        mc3p.config.pwm_Hz = pwm_freq_hz;
        mc3p.config.duty_max = 0.95;
        mc3p.config.duty_min = 0.05;
        mc3p.config.deadtime_nS = 1500;
        eldriver_mc3p_init(&mc3p);
        eldriver_mc3p_write_float(&mc3p);
    };

    void setControlMode(MCType mctype)
    {
        if (control.mctype != mctype)
        {
            // Exit current mode
            switch (control.mctype)
            {
            case MCType::Trap:
                Trap_onExit();
                break;
            case MCType::Foc:
                break;
            case MCType::Commission:
                break;
            default:
                break;
            }
            // Enter new mode
            switch (mctype)
            {
            case MCType::Trap:
                Trap_onEnter(control.mctype);
                break;
            case MCType::Foc:
                break;
            case MCType::Commission:
                break;
            default:
                break;
            }
            control.mctype = mctype;
        }
    }

    void pwmLoop()
    {
        if (control.mctype == MCType::None)
            return;
        uint32_t start = eldriver_core_prof_tick();
        eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
        switch (control.mctype)
        {
        case MCType::Trap:
            Trap_pwmLoop();
            break;
        default:
            break;
        }
        pwmTicks++;
        volatile uint32_t elapsed = eldriver_core_prof_tock(start);
    }

    void xmcLoop()
    {
        if (control.mctype == MCType::None)
            return;
        switch (control.mctype)
        {
        case MCType::Trap:
            Trap_xmcLoop();
            break;
        default:
            break;
        }
    }
};


extern PmsmControl motor_c1;