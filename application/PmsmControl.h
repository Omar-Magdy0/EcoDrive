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
        Idle = 0,    /** Outputs disabled, no commutation. */
        ClosedTrap, /** Closed-loop trapezoidal commutation. */
        OpenTrap,   /** Open-loop trapezoidal commutation. */
        OpenFocIF,  /** Open-loop V/Hz (V/F) commutation. */
        Commission  /** Self-commissioning / parameter identification. */
    };

    /**
     * @brief Synchronized measurement cache from MC3P driver.
     *
     * Stores either SVM or trapezoidal measurements depending on mode.
     * The active interpretation is controlled by the current `ControlMode`.
     */
    union
    {
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    } mc3p_sync_meas{};

    enum class StupStage : uint8_t
    {
        Reset = 0, /** STUP inactive or reset state. */
        Align,     /** Rotor alignment stage. */
        Ramp,      /** Open-loop ramp stage. */
        Closed     /** Ready to transition to closed-loop. */
    };

    enum class Direction : uint8_t
    {
        Forward = 0, /** Forward rotation. */
        Backward     /** Reverse rotation. */
    };

    /**
     * @brief Startup (STUP) configuration.
     *
     * Defines alignment parameters and the ramp profile used for
     * open-loop startup.
     *
     * @details
     * The ramp is defined as a piecewise profile over equally-indexed
     * time, voltage, and frequency points. Each index represents a
     * target operating point for the startup sequence.
     */
    struct StupConfig
    {
        uint16_t align_duration_ms{}; /** Alignment dwell time (ms). */
        eldriver_mc3p_sector_t align_sector{ELDRIVER_MC3P_SECTOR_FLOAT}; /** Alignment sector. */
        float bus_V{}; /** Bus voltage (V). */
        float align_V{}; /** Alignment voltage (V). */
        float start_current_A{}; /** Open-loop accel current setpoint (A). */
        std::array<float, STUP_TABLE_SIZE> time_mS{}; /** Ramp time points (ms). */
        std::array<float, STUP_TABLE_SIZE> volt_V{}; /** Ramp voltage points (V). */
        std::array<float, STUP_TABLE_SIZE> freq_Hz{}; /** Ramp frequency points (Hz). */
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
     * @brief Electrical (alpha/beta) state and commutation data.
     */
    struct
    {
        float vbus{}; /** DC bus voltage (V). */
        q15_t alpha_q15{}; /** Alpha-axis voltage/current (Q15). */
        q15_t beta_q15{}; /** Beta-axis voltage/current (Q15). */
        q15_t trap_duty_q15{}; /** Trapezoidal duty (Q15). */
        eldriver_mc3p_sector_t sector{ELDRIVER_MC3P_SECTOR_FLOAT}; /** Electrical sector. */
        float speed_hz{}; /** Electrical speed (Hz). */
        float theta; /** Electrical angle (rad). */
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
     * @brief Motor model parameters used for control and commissioning.
     */
    struct
    {
        float Rs; /** Stator resistance (Ohm). */
        float Ld; /** D-axis inductance (H). */
        float Lq; /** Q-axis inductance (H). */
        float Ke; /** Back-EMF constant. */
        float J; /** Rotor inertia. */
        float B; /** Viscous friction coefficient. */
    } model;

    /**
     * @brief Self-commissioning / identification state.
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
 * @brief Ring buffer for PWM telemetry frames.
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
