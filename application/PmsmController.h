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

constexpr uint32_t PWM_FREQ = 40000;
constexpr float STUP_BEMFZC_ERROR_MARGIN = 0.05f;
constexpr uint8_t STUP_BEMFZC_GOOD_EST_COUNT = 10;

constexpr uint32_t XCPWM_TICKFREQ = PWM_FREQ;
constexpr float XCPWM_TICKPERIOD_US = 1'000'000.0f / static_cast<float>(XCPWM_TICKFREQ);
constexpr float XCPWM_TICKPERIOD_MS = XCPWM_TICKPERIOD_US / 1000.0f;
constexpr float XCPWM_US_TO_TICKS(float us) { return us / XCPWM_TICKPERIOD_US; }
constexpr float XCPWM_MS_TO_TICKS(float ms) { return XCPWM_US_TO_TICKS(ms * 1000.0f); }
constexpr float XCPWM_TICKS_TO_US(float ticks) { return ticks * XCPWM_TICKPERIOD_US; }
constexpr float XCPWM_TICKS_TO_MS(float ticks) { return ticks * XCPWM_TICKPERIOD_MS; }

constexpr size_t STUP_TABLE_SIZE = 4;

static constexpr std::array<eldriver_mc3p_sector_t, 8> HALL_TO_TRAP_TABLE = 
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


struct elmotor_pmsm_stup_config_t {
    uint16_t align_duration_ms{};
    eldriver_mc3p_sector_t align_sector{ELDRIVER_MC3P_SECTOR_FLOAT};
    float bus_V{};
    float align_V{};
    std::array<float, STUP_TABLE_SIZE> time_mS{};
    std::array<float, STUP_TABLE_SIZE> volt_V{};
    std::array<float, STUP_TABLE_SIZE> freq_Hz{};
};

enum class pmsm_stup_stage_t : uint8_t {
    Reset = 0,
    Align,
    Ramp,
    StBemfSwitchover
};

enum class Direction : uint8_t {
    Forward = 0,
    Backward
};

constexpr int8_t DirectionSign(Direction dir)
{
    return (dir == Direction::Forward) ? 1 : -1;
}
// For trap sectors specifically (1-6)
inline eldriver_mc3p_sector_t TrapIncrement(eldriver_mc3p_sector_t sector, Direction dir)
{
    return static_cast<eldriver_mc3p_sector_t>(
        (dir == Direction::Forward)
            ? elmath_increment_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6)
            : elmath_decrement_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6));
}

enum class PmsmMode : uint8_t {
    Idle = 0,
    StartupTrap,
    ClosedTrap,
    OpenTrap,
    Commission
};

struct PmsmControl {
    uint32_t xmc_ticks{};
    bool initialized{false};
    PmsmMode state{PmsmMode::Idle};
    eldriver_mc3p_t mc3p{};
    PosSensor pos_sensor{};

    union {
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    } mc3p_sync_data{};

    struct {
        elmotor_pmsm_stup_config_t cfg{};
        elcore_swttimer_t stage_timer{};
        elcore_swttimer_t comm_timer{};
        uint32_t          comm_ticks{};
        pmsm_stup_stage_t stage_last{pmsm_stup_stage_t::Reset};
        pmsm_stup_stage_t stage_current{pmsm_stup_stage_t::Reset};
        uint8_t           ramp_idx{};
        float             est_elec_speed{};
        uint8_t           good_est_count{};
    } stup{};
    
    struct {
        float vbus{};
        q15_t alpha_q15{};
        q15_t beta_q15{};
        q15_t trap_duty_q15{};
        eldriver_mc3p_sector_t sector{ELDRIVER_MC3P_SECTOR_FLOAT};
        float speed_hz{};
    } elec{};
    
    struct {
        volatile Direction dir{Direction::Forward};
        volatile float         speed_rpm{};
        volatile float         speed_sp_rpm{};
    } mech{};

    uint8_t pole_pairs{};

    void init(const elmotor_pmsm_stup_config_t& stup_cfg);
    void ClosedTrap_pwmLoop();
    void OpenTrap_pwmLoop();
    void StupTrap_pwmLoop();
    void Commission_pwmLoop();
    void pwmLoop();
    void set_speed(uint16_t speed_rpm);
    void freewheel();
};

extern PmsmControl motor_c;



#include "core/elcore.h"

constexpr uint8_t SAMPLE_LEN = 5;
constexpr uint8_t SAMPLES_PER_FRAME = 24;
constexpr uint8_t FRAME_BUFFER_COUNT = 8;
constexpr uint8_t FRAME_BUFFER_NOTIFY_THRESHOLD = 6;

using pwmSample_t = std::array<int16_t, SAMPLE_LEN>;

struct PwmDataFrame_t {
    uint32_t sample_counter{};
    std::array<pwmSample_t, SAMPLES_PER_FRAME> samples{}; // 5 values per sample as per schema
};

struct pwmDataBuffer_t {
    std::array<PwmDataFrame_t, FRAME_BUFFER_COUNT> frames{};
    elcore_rstream_t buffer{};
    PwmDataFrame_t* currentFrame{};
    uint32_t frame_sample_idx{};
    uint32_t sample_count{};
    uint32_t overflowCount{};

    void init();
    pwmSample_t* sample(uint8_t* len);
    void pushSample();
    bool readFrame(PwmDataFrame_t** outFrame);
};


extern pwmDataBuffer_t pwmDataBuffer;
