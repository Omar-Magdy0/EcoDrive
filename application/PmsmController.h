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

constexpr uint32_t PWM_FREQ_DEFAULT = 25000;
constexpr float STUP_BEMFZC_ERROR_MARGIN = 0.05f;
constexpr uint8_t STUP_BEMFZC_GOOD_EST_COUNT = 10;

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
    uint32_t pwmTicks{};
    bool initialized{false};
    PmsmMode mode{PmsmMode::Idle};
    uint32_t pwm_freq_hz{PWM_FREQ_DEFAULT};
    float tick_period_us{1'000'000.0f / static_cast<float>(PWM_FREQ_DEFAULT)};
    float tick_period_ms{tick_period_us / 1000.0f};
    eldriver_mc3p_t mc3p{};
    PosSensor pos_sensor{};

    union {
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    } mc3p_sync_meas{};

    struct StupConfig {
        uint16_t align_duration_ms{};
        eldriver_mc3p_sector_t align_sector{ELDRIVER_MC3P_SECTOR_FLOAT};
        float bus_V{};
        float align_V{};
        std::array<float, STUP_TABLE_SIZE> time_mS{};
        std::array<float, STUP_TABLE_SIZE> volt_V{};
        std::array<float, STUP_TABLE_SIZE> freq_Hz{};
    };

    struct {
        StupConfig cfg{};
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
        float theta;
    } elec{};
    
    struct {
        volatile Direction dir{Direction::Forward};
        volatile float         speed_rpm{};
        volatile float         speed_sp_rpm{};
    } mech{};

    struct {
        float Rs;
        float Ld;
        float Lq;
        float Ke;
        float J;
        float B;
    } model;

    enum class SCommStage{
        RESET = 0,
        DAXIS_ALIGN,
        RS_ID,
        LD_ID,
        LQ_ID,
        KE_ID
    };

    struct {
        SCommStage stage;
        SCommStage stage_last;
        elcore_swttimer_t timer{};
        uint8_t remaining_id_samples;
        float id_sin_prod_2;
        float id_cos_prod_2;
        float iq_sin_prod_2;
        float iq_cos_prod_2;
        uint16_t iir_filtered_cnt;
        float hfi_angle;
    }SComm;

    uint8_t pole_pairs{};

    void init(const StupConfig& stup_cfg);
    void set_pwm_freq(uint32_t pwm_hz);
    uint32_t pwm_freq() const { return pwm_freq_hz; }
    float pwm_tick_period_us() const { return tick_period_us; }
    float pwm_tick_period_ms() const { return tick_period_ms; }
    float us_to_ticks(float us) const { return us / tick_period_us; }
    float ms_to_ticks(float ms) const { return us_to_ticks(ms * 1000.0f); }
    float ticks_to_us(float ticks) const { return ticks * tick_period_us; }
    float ticks_to_ms(float ticks) const { return ticks * tick_period_ms; }
    void Idle_pwmLoop();
    void ClosedTrap_pwmLoop();
    void OpenTrap_pwmLoop();
    void StupTrap_pwmLoop();
    void SelfCommission_init();
    void SelfCommission_pwmLoop();
    void SelfCommission_xmcLoop();
    void pwmLoop();
    void xmcLoop();
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
