#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include "arm_math.h"

namespace PmsmControlTypes
{
    enum class Direction : int8_t
    {
        Forward = 1,  /** Forward rotation. */
        Backward = -1 /** Reverse rotation. */
    };

    enum class MechMode : uint8_t
    {
        None = 0,
        Torque,
        Speed,
        Position
    };

    enum class MCMode : uint8_t
    {
        None = 0,
        Idle,
        Fault,
        Foc, /** Open-loop V/Hz (V/F) commutation. */
        Trap,
        SComm /** Self-commissioning / parameter identification. */
    };

    enum class ElecMode : uint8_t
    {
        Voltage = 0,
        Current
    };

    enum class EStopMode : uint8_t
    {
        COAST,
        RAMP,
        EMERGENCY_PLUG
    };

    enum class ERR : uint8_t
    {
        OK = 0,
        UNKNOWN_ERROR = 1,
        NOT_INITIALIZED = 2,
        NOT_IMPLEMENTED = 3,
        

        CONFIG_NOT_ALLOWED_MOTOR_RUNNING = 64,
        CONFIG_BAD = 65
    };

    enum DirtyPits
    {
        DIRTYBITS_PWM    = 1 << 0,
        DIRTYBITS_ADC    = 1 << 1,  // 0x02
        DIRTYBITS_CTRL   = 1 << 2,  // 0x04
        DIRTYBITS_SP     = 1 << 3,  // 0x08
    }; 

    struct ConfigPwm
    {
        uint32_t pwm_freq_hz;
        uint32_t deadtime_ns;
        float    duty_min;
        float    duty_max;
    };

    struct ConfigEstop
    {
        EStopMode mode;
    };

    struct ConfigSComm
    {
        uint8_t hfi_N = 20;
        float dc_vinj = 2.5;
        float hfi_vinj = 12;
    };

    struct Model
    {
        float R;           /** Stator resistance (Ohm). */
        float Ld;           /** D-axis inductance (H). */
        float Lq;           /** Q-axis inductance (H). */
        float Kt;           /** Torque constant. */
        float J;            /** Rotor inertia. */
        float B;            /** Viscous friction coefficient. */
        uint8_t pole_pairs = 1; /** Motor pole pairs (electrical). */
    };

    inline static constexpr size_t OLSTUP_TABLE_SIZE = 6; /** STUP profile table size. */
    struct ConfigOlstup
    {
        float vbus_init;
        uint16_t time_ms_tb[OLSTUP_TABLE_SIZE];
        float ec_tb[OLSTUP_TABLE_SIZE];
        float rpm_tb[OLSTUP_TABLE_SIZE];
        eldriver_mc3p_sector_t init_sector;
        ElecMode elec_mode;
    };

    struct ConfigElecLimits
    {
        float currentLimit; 
        float underVoltageLimit;
        float overVoltageLimit;
    };

    struct ConfigMechLimits
    {

    };

    struct Pid
    {
        float Kp;
        float Ki;
        float Kd;
    };
};