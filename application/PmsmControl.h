/**
 * @file PmsmControl.h
 * @brief PMSM control data structures and control-loop entry points.
 *
 * This header defines the main controller state container (`PmsmControl`),
 * startup (STUP) configuration, self-commissioning context, and a small
 * streaming buffer used for PWM telemetry capture.
 *
 * The controller is structured around two periodic loops:
 * - `pwmLoop()` runs at the PWM interrupt rate (fast loop).
 * - `xmcLoop()` runs at a slower housekeeping rate (slow loop).
 *
 * Control modes supported:
 * - Idle (outputs disabled)
 * - Open-loop trapezoidal
 * - Open-loop V/F
 * - Closed-loop trapezoidal
 * - Self-commissioning
 *
 * @note This header is intentionally free of implementation details.
 *       Functional behavior, timing, and state transitions are defined
 *       in the corresponding `.cpp` files.
 *
 * @par Threading / Concurrency
 * The fast loop (`pwmLoop`) is typically executed from an ISR context.
 * Any fields accessed by both fast and slow loops are either `volatile`
 * or updated in a way that is safe for the target architecture. Review
 * platform-specific interrupt safety when extending this header.
 *
 * @par Units Convention
 * - Time: `*_us`, `*_ms` for microseconds and milliseconds.
 * - Speed: electrical `Hz`, mechanical `RPM`.
 * - Voltages in `V`, angles in `rad`.
 */
#pragma once
#include "core/elmath.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "sensor.h"
#include "arm_math.h"
#include <array>
#include <cstdint>
#include <cstddef>
//===================================================
// CONFIG
//==================================================

constexpr uint32_t PWM_FREQ_DEFAULT = 25000; /** Default PWM frequency (Hz). */
constexpr float STUP_BEMFZC_ERROR_MARGIN = 0.05f; /** Max relative BEMF ZC error. */
constexpr uint8_t STUP_BEMFZC_GOOD_EST_COUNT = 10; /** Consecutive good ZC estimates. */
constexpr size_t STUP_TABLE_SIZE = 4; /** STUP profile table size. */
static constexpr std::array<eldriver_mc3p_sector_t, 8> HALL_TO_TRAP_TABLE = /** Hall->trap sector LUT (idx=hall 0..7). */
    {
        ELDRIVER_MC3P_SECTOR_FLOAT,
        ELDRIVER_MC3P_SECTOR_TRAP5,
        ELDRIVER_MC3P_SECTOR_TRAP3,
        ELDRIVER_MC3P_SECTOR_TRAP4,
        ELDRIVER_MC3P_SECTOR_TRAP1,
        ELDRIVER_MC3P_SECTOR_TRAP6,
        ELDRIVER_MC3P_SECTOR_TRAP2,
        ELDRIVER_MC3P_SECTOR_FLOAT
    };

/**
 * @brief Main PMSM control state and entry points.
 *
 * This struct holds all runtime state for motor control, including electrical,
 * mechanical, startup (STUP), and self-commissioning data.
 *
 * @details
 * The controller is designed around a minimal data-only interface that can
 * be accessed from time-critical ISR code. Most members are plain data with
 * deterministic layout. Configuration is provided via `init()` and runtime
 * updates are performed by the loop entry points.
 */
struct PmsmControl
{
    enum class ControlMode : uint8_t
    {
        /** 
         * @brief Safe resting state (Outputs Disabled).
         * @details All inverter switches are turned off, leaving the motor phases floating 
         *          (high-impedance). This is the default system state upon power-up and the 
         *          fallback state during hardware faults (e.g., Over-Current or Over-Voltage).
         */
        Idle = 0,    
        /** 
         * @brief Closed-loop 6-step trapezoidal commutation.
         * @details Dynamically determines the active commutation sector using real-time absolute 
         *          rotor position feedback (from Hall sensors or BEMF zero-crossing observers). 
         *          Provides optimal torque generation and is the primary running mode.
         */
        ClosedTrap, 
        /** 
         * @brief Open-loop 6-step trapezoidal commutation.
         * @details Blindly applies a rotating 6-step voltage pattern without relying on actual 
         *          rotor position feedback. Useful for diagnostic testing or initial forced rotation.
         */
        OpenTrap,   
        /** 
         * @brief Open-loop Voltage/Frequency (V/f) sinusoidal commutation.
         * @details Synthesizes a rotating voltage vector (Space Vector Modulation) at a fixed V/f 
         *          ratio. Often used for pumps/fans or as a preliminary step before closing the 
         *          loop in Field Oriented Control (FOC).
         */
        OpenFocIF,  
        /** 
         * @brief Automated Self-Commissioning and Parameter Identification.
         * @details Executes a sequence of static and high-frequency signal injections to physically 
         *          measure motor characteristics (Rs, Ld, Lq) while the rotor is stationary. 
         *          Crucial for auto-tuning the controller to unknown motors.
         */
        Commission  
    };

    /**
     * @brief Synchronized measurement cache from the MC3P hardware driver.
     * @details Declared as a `union` to significantly save RAM. Because the motor can only operate 
     *          in one physical commutation mode at a time (either Sinusoidal/SVM or Trapezoidal), 
     *          the raw ADC readings are parsed and stored in the specific struct that matches the 
     *          active `ControlMode`. This guarantees that high-frequency control loops always 
     *          have instantaneous access to the freshest currents and voltages without memory overhead.
     */
    union
    {
        eldriver_mc3p_svm_data_t svm;   /**< Active when Space Vector Modulation (SVM) is used (e.g., OpenFocIF, Commission). */
        eldriver_mc3p_trap_data_t trap; /**< Active when 6-step block commutation is used (e.g., ClosedTrap, OpenTrap). */
    } mc3p_sync_meas{};

    enum class StupStage : uint8_t
    {
        /** 
         * @brief Initialization and preparation state.
         * @details Clears all internal software timers, error tracking counters, and tracking 
         *          variables. Acts as the entry gateway before attempting to energize the motor.
         */
        Reset = 0, 
        /** 
         * @brief Static rotor alignment phase (DC Injection).
         * @details Used primarily in sensorless control. It injects a static DC voltage into a 
         *          specific electrical sector, creating a stationary magnetic field that physically 
         *          pulls the rotor into a known 0-degree starting position.
         */
        Align,     
        /** 
         * @brief Open-loop forced acceleration phase (V/f Ramp).
         * @details Gradually accelerates the synthesized magnetic field using a predefined Voltage/Frequency 
         *          profile. This physically drags the rotor up to a minimum velocity where Back-EMF 
         *          signals become strong enough to be reliably measured by the ADC.
         */
        Ramp,      
        /** 
         * @brief Steady-state operational phase (Closed-Loop).
         * @details Indicates that the startup sequence has successfully completed. The system is 
         *          now fully relying on real-time feedback (Hall or BEMF) to commutate the motor.
         */
        Closed     
    };

    enum class Direction : uint8_t
    {
        Forward = 0, /** Forward rotation. */
        Backward     /** Reverse rotation. */
    };

    /**
     * @brief Startup (STUP) configuration profile.
     *
     * Defines the strict physical parameters for DC alignment and the precise 
     * multi-point acceleration ramp used during open-loop sensorless startup.
     *
     * @details
     * Since Back-EMF is absent at 0 RPM, the motor must be blindly accelerated. 
     * This structure defines a piecewise linear interpolation profile. By matching 
     * an elapsed `time_mS` to a target `volt_V` and `freq_Hz`, the controller can 
     * synthesize a smooth acceleration curve that avoids stalling or drawing 
     * excessive overcurrent.
     */
    struct StupConfig
    {
        uint16_t align_duration_ms{}; /**< Total time to hold the DC alignment vector to allow mechanical ringing to dampen (ms). */
        eldriver_mc3p_sector_t align_sector{ELDRIVER_MC3P_SECTOR_FLOAT}; /**< The specific stationary electrical sector used for alignment. */
        float bus_V{}; /**< Nominal DC bus voltage used as a mathematical base to normalize duty cycles (V). */
        float align_V{}; /**< The precise, low-level DC voltage applied during the alignment phase (V). */
        float start_current_A{}; /**< Estimated open-loop acceleration current setpoint (A). */
        std::array<float, STUP_TABLE_SIZE> time_mS{}; /**< Array of time milestones (in milliseconds) defining the X-axis of the ramp profile. */
        std::array<float, STUP_TABLE_SIZE> volt_V{};  /**< Array of target voltage amplitudes mapped to each time milestone. */
        std::array<float, STUP_TABLE_SIZE> freq_Hz{}; /**< Array of target electrical commutation frequencies mapped to each time milestone. */
    };

    constexpr int8_t DirectionSign(Direction dir)
    {
        return (dir == Direction::Forward) ? 1 : -1;
    }

    // For trap sectors specifically (1-6)
    static inline eldriver_mc3p_sector_t TrapIncrement(eldriver_mc3p_sector_t sector, Direction dir)
    {
        return static_cast<eldriver_mc3p_sector_t>(
            (dir == Direction::Forward)
                ? elmath_increment_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6)
                : elmath_decrement_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6));
    }

    uint32_t pwmTicks{}; /** PWM period in ticks (timer ticks). */
    bool initialized{false}; /** True after init() successfully configures internal state. */
    ControlMode mode{ControlMode::Idle}; /** Current control mode. */
    uint32_t pwm_freq_hz{PWM_FREQ_DEFAULT}; /** PWM carrier frequency (Hz). */
    float tick_period_us{1'000'000.0f / static_cast<float>(PWM_FREQ_DEFAULT)}; /** PWM tick period (us). */
    float tick_period_ms{tick_period_us / 1000.0f}; /** PWM tick period (ms). */
    eldriver_mc3p_t mc3p{}; /** Underlying MC3P driver instance. */
    PosSensor pos_sensor{}; /** Rotor position sensor interface. */

    /**
     * @brief Startup (STUP) runtime state.
     *
     * Holds timing, ramp index, and zero-crossing estimate quality.
     */
    struct
    {
        StupConfig cfg{ /** Startup configuration (default). */
            .align_duration_ms = 100,
            .align_sector = ELDRIVER_MC3P_SECTOR_TRAP3,
            .bus_V = 12,
            .align_V = 1,
            .start_current_A = 2,
            .time_mS = {0, 1000, 1500, 2000},
            .volt_V = {2, 2.25f, 2.5f, 2.75f},
            .freq_Hz = {100, 125, 200, 250},
        };
        elcore_swttimer_t stage_timer{}; /** Timer for current STUP stage. */
        elcore_swttimer_t comm_timer{}; /** Commutation timer during STUP. */
        uint32_t comm_ticks{}; /** Commutation interval in ticks. */
        StupStage stage_last{StupStage::Reset}; /** Previous STUP stage. */
        StupStage stage_current{StupStage::Reset}; /** Current STUP stage. */
        uint8_t ramp_idx{}; /** Ramp index into the STUP tables. */
        float est_elec_speed{}; /** Estimated electrical speed (Hz). */
        uint8_t good_est_count{}; /** Consecutive good BEMF estimates. */
    } stup{};

    /**
     * @brief High-frequency electrical state, coordinate transforms, and commutation metrics.
     * @details This structure encapsulates all real-time electrical variables derived directly 
     *          from the ADC measurements or generated by the control loops. It uses efficient 
     *          Q15 fixed-point math for duty cycles and voltages to maximize execution speed 
     *          on microcontrollers without native floating-point units.
     */
    struct
    {
        float vbus{}; /**< Instantaneous measured DC link bus voltage (V). */
        q15_t alpha_q15{}; /**< Stationary alpha-axis component of the Clarke transform (normalized Q15 format). */
        q15_t beta_q15{};  /**< Stationary beta-axis component of the Clarke transform (normalized Q15 format). */
        q15_t trap_duty_q15{}; /**< Commanded PWM duty cycle for block commutation (normalized Q15 format). */
        eldriver_mc3p_sector_t sector{ELDRIVER_MC3P_SECTOR_FLOAT}; /**< The currently active 6-step commutation sector (1 to 6, or Float). */
        float speed_hz{}; /**< Instantaneous electrical speed of the rotating stator field (Hz). */
        float theta; /**< Real-time electrical angle of the rotor (in radians, from 0 to 2*PI). */
    } elec{};

    /**
     * @brief Mechanical state and setpoints.
     */
    struct
    {
        volatile Direction dir{Direction::Forward}; /** Commanded rotation direction. */
        volatile float speed_rpm{}; /** Measured mechanical speed (RPM). */
        volatile float speed_sp_rpm{}; /** Speed setpoint (RPM). */
    } mech{};

    /**
     * @brief Comprehensive motor physics model parameters.
     * @details This structure holds the fundamental electrical and mechanical constants of the 
     *          connected PMSM. These values are typically populated by the `SelfCommission` routines 
     *          and are absolutely vital for advanced closed-loop algorithms like Field Oriented Control (FOC), 
     *          current decoupling, and state observers (like SMO or PLL).
     */
    struct
    {
        float Rs; /**< Stator phase resistance (Ohms). Dictates static copper losses and DC voltage drop. */
        float Ld; /**< Direct-axis synchronous inductance (Henries). Affects flux-weakening capability. */
        float Lq; /**< Quadrature-axis synchronous inductance (Henries). Primary contributor to reluctance torque. */
        float Ke; /**< Back-EMF constant (V*s/rad). Dictates the voltage generated by the motor per unit of speed. */
        float J;  /**< Mechanical rotor inertia (kg*m^2). Used to tune speed loop PID gains. */
        float B;  /**< Viscous friction coefficient (N*m*s). Represents mechanical drag. */
    } model;

    /**
     * @brief Self-commissioning / hardware identification state machine context.
     * @details Contains all staging variables, software timers, and mathematical accumulators 
     *          required to run the automated motor identification sequence (measuring Rs, Ld, Lq). 
     *          It specifically handles the complex High-Frequency Injection (HFI) heterodyning states.
     */
    struct
    {
        enum class IDStage
        {
            RESET = 0,       /** Initial / reset stage. */
            DAXIS_ALIGN,     /** D-axis alignment stage. */
            RS_ID,           /** Stator resistance identification. */
            LD_ID,           /** D-axis inductance identification. */
            LQ_ID,           /** Q-axis inductance identification. */
            ELEC_POSTPROCESS /** Final post-processing stage. */
        };
        volatile IDStage IDstage;
        IDStage IDstage_last;
        elcore_swttimer_t timer{}; /** Stage timing for self-commissioning. */
        uint8_t remaining_id_samples; /** Remaining samples for current step. */
        float id_sin_prod_2; /** D-axis correlation accumulator. */
        float id_cos_prod_2; /** D-axis correlation accumulator. */
        float iq_sin_prod_2; /** Q-axis correlation accumulator. */
        float iq_cos_prod_2; /** Q-axis correlation accumulator. */
        float rs_dc; /** Estimated DC resistance. */
        uint16_t iir_filtered_cnt; /** IIR filter counter for smoothing. */
        float hfi_angle; /** High-frequency injection angle estimate. */
    } SComm;

    uint8_t pole_pairs{}; /** Motor pole pairs (electrical). */

    void init(); /** Init state and apply STUP config. */
    void set_pwm_freq(uint32_t pwm_hz); /** Set PWM frequency and tick periods. */
    uint32_t pwm_freq() const { return pwm_freq_hz; } /** Get PWM frequency (Hz). */
    float pwm_tick_period_us() const { return tick_period_us; } /** Get PWM tick period (us). */
    float pwm_tick_period_ms() const { return tick_period_ms; } /** Get PWM tick period (ms). */
    float us_to_ticks(float us) const { return us / tick_period_us; } /** us -> ticks. */
    float ms_to_ticks(float ms) const { return us_to_ticks(ms * 1000.0f); } /** ms -> ticks. */
    float ticks_to_us(float ticks) const { return ticks * tick_period_us; } /** ticks -> us. */
    float ticks_to_ms(float ticks) const { return ticks * tick_period_ms; } /** ticks -> ms. */
    void Idle_pwmLoop(); /** Idle PWM loop (outputs disabled). */
    void ClosedTrap_pwmLoop(); /** Closed-loop trapezoidal PWM loop. */
    void OpenFocIF_pwmLoop(); /** Open-loop V/F PWM loop. */
    void OpenTrap_pwmLoop(); /** Open-loop trapezoidal PWM loop. */
    void SelfCommission_init(); /** Initialize self-commissioning. */
    void SelfCommission_pwmLoop(); /** Self-commissioning PWM loop. */
    void SelfCommission_xmcLoop(); /** Self-commissioning slow loop. */
    void pwmLoop(); /** Dispatch fast PWM loop based on mode. */
    void xmcLoop(); /** Dispatch slow loop based on mode. */
};

extern PmsmControl motor_c1; /** Global controller instance for motor channel 1. */

#include "core/elcore.h"

constexpr uint8_t SAMPLE_LEN = 5; /** Values per PWM sample (schema). */
constexpr uint8_t SAMPLES_PER_FRAME = 24; /** Samples per telemetry frame. */
constexpr uint8_t FRAME_BUFFER_COUNT = 8; /** Frames stored in ring buffer. */
constexpr uint8_t FRAME_BUFFER_NOTIFY_THRESHOLD = 6; /** Frame notify threshold. */
using pwmSample_t = std::array<int16_t, SAMPLE_LEN>; /** One PWM telemetry sample. */

/**
 * @brief Fixed-size telemetry frame.
 *
 * A frame contains multiple PWM samples and a monotonic sample counter.
 */
struct PwmDataFrame_t
{
    uint32_t sample_counter{}; /** Monotonic sample counter at frame start. */
    std::array<pwmSample_t, SAMPLES_PER_FRAME> samples{}; // 5 values per sample as per schema
};

/**
 * @brief Lock-free ring buffer designed for high-speed PWM telemetry capture.
 * @details In high-frequency motor control (e.g., 25 kHz), extracting debug data without blocking 
 *          the fast PWM loop is extremely challenging. This struct implements a zero-copy, lock-free 
 *          ring buffer. The fast ISR writes raw samples into a `currentFrame`. Once the frame fills 
 *          up to `SAMPLES_PER_FRAME`, it is atomically committed to the underlying `elcore_rstream_t`, 
 *          allowing a slower background RTOS task (like USB/UART) to transmit the data to a host GUI 
 *          without ever interrupting the critical motor control execution.
 */
struct pwmDataBuffer_t
{
    std::array<PwmDataFrame_t, FRAME_BUFFER_COUNT> frames{}; /** Storage for frames. */
    elcore_rstream_t buffer{}; /** Streaming buffer metadata. */
    PwmDataFrame_t *currentFrame{}; /** Current frame being filled. */
    uint32_t frame_sample_idx{}; /** Next sample index in current frame. */
    uint32_t sample_count{}; /** Total samples pushed since init. */
    uint32_t overflowCount{}; /** Samples dropped due to overflow. */

    void init(); /** Init ring buffer. */
    pwmSample_t *sample(uint8_t *len); /** Get writable sample slot. */
    void pushSample(); /** Commit current sample. */
    bool readFrame(PwmDataFrame_t **outFrame); /** Read next available frame. */
};

extern pwmDataBuffer_t pwmDataBuffer; /** Global telemetry buffer instance. */
