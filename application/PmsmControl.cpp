#include "PmsmControl.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "arm_math.h"
#include <array>

#define DEBUG_PIN (16 * 1 + 2)

void bemfzc_ComCallback();
void hall1_ComCallback();

PmsmControl motor_c1;

void PmsmControl::init()
{
    pwmTicks = 0;
    set_pwm_freq(PWM_FREQ_DEFAULT);
    mc3p.config.pwm_Hz = pwm_freq_hz;
    mc3p.config.duty_max = 0.95;
    mc3p.config.duty_min = 0.05;
    mc3p.config.deadtime_nS = 1500;

    stup.comm_ticks = 0;
    elec.sector = ELDRIVER_MC3P_SECTOR_FLOAT;
    stup.comm_timer.tick_period = tick_period_us;
    stup.stage_timer.tick_period = tick_period_us;
    mode = ControlMode::ClosedTrap;
    stup.stage_current = StupStage::Reset;
    pos_sensor.init(pwm_freq_hz);
    mc3p.offset_calibration = true;
    eldriver_mc3p_init(&mc3p);
    initialized = true;
}

void PmsmControl::set_pwm_freq(uint32_t pwm_hz)
{
    if (pwm_hz == 0)
        return;
    pwm_freq_hz = pwm_hz;
    tick_period_us = 1'000'000.0f / static_cast<float>(pwm_freq_hz);
    tick_period_ms = tick_period_us / 1000.0f;
    mc3p.config.pwm_Hz = pwm_freq_hz;
    stup.comm_timer.tick_period = tick_period_us;
    stup.stage_timer.tick_period = tick_period_us;
    pos_sensor.init(pwm_freq_hz);
}

/// @brief
void PmsmControl::pwmLoop()
{
    if (!initialized)
        return;
    uint32_t start = eldriver_core_prof_tick();
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    elec.vbus = mc3p_sync_meas.svm.vbus_q31;

    switch (mode)
    {
    case ControlMode::Idle:
        Idle_pwmLoop();
        break;
    case ControlMode::ClosedTrap:
        ClosedTrap_pwmLoop();
        break;
    case ControlMode::OpenTrap:
        OpenTrap_pwmLoop();
        break;
    case ControlMode::OpenFocIF:
        OpenFocIF_pwmLoop();
        break;
    case ControlMode::Commission:
        SelfCommission_pwmLoop();
        break;

    default:
        break;
    }
    pwmTicks++;
    uint8_t len;

    pwmSample_t *sample_ptr = pwmDataBuffer.sample(&len);
    if (mc3p.mode == ELDRIVER_MC3P_MODE_TRAP && sample_ptr)
    {
        (*sample_ptr)[0] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbus_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[1] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbemf_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[2] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.cbus_q31) * ELDRIVER_MC3P_CS_SCALE * 1000) >> 31);
        (*sample_ptr)[4] = (int16_t)elec.speed_hz;
    }
    pwmDataBuffer.pushSample();
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

void PmsmControl::xmcLoop()
{
    switch (mode)
    {
    case ControlMode::Idle:
        break;
    case ControlMode::ClosedTrap:
        break;
    case ControlMode::OpenTrap:
        break;
    case ControlMode::OpenFocIF:
        break;
    case ControlMode::Commission:
        SelfCommission_xmcLoop();
        break;

    default:
        break;
    }
}

//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void eldriver_mc3p_sync_postScanCallback()
{
    motor_c1.pwmLoop();
}

void eldriver_xmc3p_tickerCallback()
{
    motor_c1.xmcLoop();
}

//========================================================
void pwmDataBuffer_t::init()
{
    elcore_rstream_init(&buffer, (void *)frames.data(), sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    frame_sample_idx = 0;
    sample_count = 0;
    overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2);
    currentFrame->sample_counter = 0;
    // update curernt frame value
}

pwmSample_t *pwmDataBuffer_t::sample(uint8_t *sample_len)
{
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        *sample_len = SAMPLE_LEN; // 5 floats per sample
        return &currentFrame->samples[frame_sample_idx];
    }
    return NULL; // No available sample slot
}

void pwmDataBuffer_t::pushSample()
{
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        frame_sample_idx++;
    }
    sample_count++;
    if (frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        elcore_rstream_commitWrite(&buffer, 1); // Commit the current frame
        // Current frame is full, move to next frame
        uint8_t *w2;
        uint16_t c1, c2;
        // Commit the current full frame
        if (elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2))
        {
            currentFrame->sample_counter = sample_count;
            frame_sample_idx = 0;
        }
        else
        {
            // Buffer overflow, data loss occurs
            overflowCount++;
        }
    }
}
bool pwmDataBuffer_t::readFrame(PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    if (elcore_rstream_peekRead(&buffer, (void **)frame, &c1, (void **)&r2, &c2))
    {
        // Successfully read a frame
        // Process the frame data as needed
        // After processing, commit the read to move the tail forward
        elcore_rstream_releaseRead(&buffer, 1);
        return true;
    }
    else
    {
        return false;
    }
}
