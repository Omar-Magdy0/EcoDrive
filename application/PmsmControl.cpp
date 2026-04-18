/**
 * @file PmsmControl.cpp
 * @brief Implementation of the Permanent Magnet Synchronous Motor (PMSM) controller.
 * @details Contains the primary control loops, hardware configuration, and telemetry data buffering logic.
 * @details Detailed Doxygen documentation and code analysis by Bishoy Youssef.
 */
 
#include "PmsmControl.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "arm_math.h"
#include <array>

/**
 * @def DEBUG_PIN
 * @brief GPIO pin definition used for debugging execution timing.
 */
#define DEBUG_PIN (16 * 1 + 2)

/**
 * @brief External callback function for Back-EMF Zero Crossing commutation.
 */
void bemfzc_ComCallback();

/**
 * @brief External callback function for Hall Sensor 1 commutation.
 */
void hall1_ComCallback();

/**
 * @brief Global instance of the PMSM control structure for motor C1.
 */
PmsmControl motor_c1;

void PmsmControl::init()
{
    /// Reset the PWM tick counter to zero
    pwmTicks = 0;
    /// Set the initial PWM frequency to the default value
    set_pwm_freq(PWM_FREQ_DEFAULT);
    /// Configure the 3-phase motor driver's PWM frequency
    mc3p.config.pwm_Hz = pwm_freq_hz;
    /// Set the maximum allowed duty cycle to 95%
    mc3p.config.duty_max = 0.95;
    /// Set the minimum allowed duty cycle to 5%
    mc3p.config.duty_min = 0.05;
    /// Set the deadtime for the inverter switches to 1500 nanoseconds
    mc3p.config.deadtime_nS = 1500;

    stup.comm_ticks = 0;
    /// Initialize the electrical sector to floating state (all switches off)
    elec.sector = ELDRIVER_MC3P_SECTOR_FLOAT;
    /// Sync commutation timer period with the calculated microsecond tick period
    stup.comm_timer.tick_period = tick_period_us;
    /// Sync staging timer period with the calculated microsecond tick period
    stup.stage_timer.tick_period = tick_period_us;
    /// Set the default control mode to Closed-Loop Trapezoidal
    mode = ControlMode::ClosedTrap;
    /// Initialize the startup stage machine to the Reset stage
    stup.stage_current = StupStage::Reset;
    /// Initialize the position sensor with the current PWM frequency
    pos_sensor.init(pwm_freq_hz);
    /// Enable hardware offset calibration for phase currents
    mc3p.offset_calibration = true;
    /// Initialize the underlying motor driver hardware
    eldriver_mc3p_init(&mc3p);
    /// Flag the controller as successfully initialized
    initialized = true;
}

/**
 * @brief Updates the PWM frequency and recalculates dependent timing periods.
 * @param pwm_hz The target PWM frequency in Hertz.
 */
void PmsmControl::set_pwm_freq(uint32_t pwm_hz)
{
    /// Prevent division by zero if frequency is invalid
    if (pwm_hz == 0)
        return;
    /// Update the internal frequency variable
    pwm_freq_hz = pwm_hz;
    /// Calculate the time period of one tick in microseconds
    tick_period_us = 1'000'000.0f / static_cast<float>(pwm_freq_hz);
    /// Calculate the time period of one tick in milliseconds
    tick_period_ms = tick_period_us / 1000.0f;
    /// Update the driver configuration with the new frequency
    mc3p.config.pwm_Hz = pwm_freq_hz;
    /// Update timing periods for the software timers
    stup.comm_timer.tick_period = tick_period_us;
    stup.stage_timer.tick_period = tick_period_us;
    /// Reinitialize the position sensor to adapt to the new timing
    pos_sensor.init(pwm_freq_hz);
}

/// @brief
/**
 * @brief Main high-frequency Pulse Width Modulation (PWM) control loop.
 * @details Executed periodically (usually bound to ADC synchronization). It evaluates the active control mode and drives the inverter.
 */
void PmsmControl::pwmLoop()
{
    /// Abort execution if the controller has not been initialized
    if (!initialized)
        return;
    /// Start performance profiling for the loop execution time
    uint32_t start = eldriver_core_prof_tick();
    /// Read the latest synchronized measurements from the hardware driver
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    /// Update the local electrical bus voltage variable
    elec.vbus = mc3p_sync_meas.svm.vbus_q31;

    /// Route execution to the appropriate handler based on the active control mode
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
    /// Increment the global PWM loop tick counter
    pwmTicks++;
    uint8_t len;

    /// Request a new telemetry sample slot from the data buffer
    pwmSample_t *sample_ptr = pwmDataBuffer.sample(&len);
    /// Populate the telemetry sample if the driver is in Trapezoidal mode and a valid pointer is returned
    if (mc3p.mode == ELDRIVER_MC3P_MODE_TRAP && sample_ptr)
    {
        /// Store scaled bus voltage data
        (*sample_ptr)[0] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbus_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        /// Store scaled Back-EMF voltage data
        (*sample_ptr)[1] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbemf_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        /// Store scaled bus current data
        (*sample_ptr)[2] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.cbus_q31) * ELDRIVER_MC3P_CS_SCALE * 1000) >> 31);
        /// Store the calculated electrical speed in Hertz
        (*sample_ptr)[4] = (int16_t)elec.speed_hz;
    }
    /// Commit the populated sample to the buffer
    pwmDataBuffer.pushSample();
    /// End performance profiling and record the elapsed ticks
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

/**
 * @brief Cross-domain/Low-frequency auxiliary control loop.
 * @details Used to execute slower tasks such as post-processing during self-commissioning.
 */
void PmsmControl::xmcLoop()
{
    /// Handle auxiliary routines based on the active control mode
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
        /// Execute self-commissioning post-process calculations
        SelfCommission_xmcLoop();
        break;

    default:
        break;
    }
}

/**
 * @name CALLBACK DEFINITIONS
 * @brief System-level interrupt service routines and hardware callbacks.
 */
///@{

/**
 * @brief Callback triggered after the synchronous ADC scan is complete.
 */
void eldriver_mc3p_sync_postScanCallback()
{
    /// Execute the main PWM control loop for motor C1
    motor_c1.pwmLoop();
}

/**
 * @brief Periodic timer callback for cross-domain motor control tasks.
 */
void eldriver_xmc3p_tickerCallback()
{
    /// Execute the auxiliary control loop for motor C1
    motor_c1.xmcLoop();
}
///@}

//========================================================
/**
 * @brief Initializes the telemetry PWM data ring buffer.
 */
void pwmDataBuffer_t::init()
{
    /// Initialize the underlying reliable stream (rstream) buffer with the frame array structure
    elcore_rstream_init(&buffer, (void *)frames.data(), sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    /// Reset the active sample index within the current frame
    frame_sample_idx = 0;
    /// Reset the absolute global sample counter
    sample_count = 0;
    /// Reset the buffer overflow event counter
    overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    /// Reserve the initial block of memory within the stream for writing the first data frame
    elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2);
    /// Set the base sample counter for the first reserved frame
    currentFrame->sample_counter = 0;
    /// @note The current frame value is initialized to reflect the base sample state.
}

/**
 * @brief Requests the next available sample pointer in the current data frame.
 * @param sample_len Output pointer indicating the data length of the returned sample.
 * @return Pointer to the telemetry sample buffer, or NULL if the frame is full.
 */
pwmSample_t *pwmDataBuffer_t::sample(uint8_t *sample_len)
{
    /// Verify there is still capacity inside the current frame
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        /// Set output length assigning the expected number of parameters (e.g., 5 data points per sample)
        *sample_len = SAMPLE_LEN;
        /// Return the memory address of the targeted sample array slot within the active frame
        return &currentFrame->samples[frame_sample_idx];
    }
    /// Return NULL to indicate that there are no available sample slots left in the active frame
    return NULL;
}

/**
 * @brief Commits the populated telemetry sample and manages frame rollover.
 */
void pwmDataBuffer_t::pushSample()
{
    /// Increment the local frame index if there is still room
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        frame_sample_idx++;
    }
    /// Increment the absolute system sample counter
    sample_count++;
    /// Check if the current frame has reached its maximum sample capacity
    if (frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        /// Commit the completely filled frame into the readable stream ring buffer for consumer processing
        elcore_rstream_commitWrite(&buffer, 1);
        
        /// Allocate pointers and length variables required for the next memory stream reservation
        uint8_t *w2;
        uint16_t c1, c2;
        /// Attempt to reserve a new block of memory for the next incoming data frame
        if (elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2))
        {
            /// Reservation successful; timestamp the new frame with the absolute sample count for synchronization
            currentFrame->sample_counter = sample_count;
            /// Reset the local frame index to start populating from the beginning of the new frame
            frame_sample_idx = 0;
        }
        else
        {
            /// Reservation failed due to buffer overflow; increment the overflow counter (telemetry data is dropped)
            overflowCount++;
        }
    }
}

/**
 * @brief Attempts to retrieve a complete telemetry frame from the read buffer.
 * @param frame Output double pointer to the fetched PWM data frame.
 * @return true if a frame was successfully read, false otherwise.
 */
bool pwmDataBuffer_t::readFrame(PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    /// Peek into the read stream buffer to check for an available frame without moving the read pointer yet
    if (elcore_rstream_peekRead(&buffer, (void **)frame, &c1, (void **)&r2, &c2))
    {
        /// Successfully located an available frame; the caller will process its data payload
        /// Immediately release the read buffer segment so the memory can be reclaimed by the writer
        elcore_rstream_releaseRead(&buffer, 1);
        return true;
    }
    else
    {
        /// No frames currently available to read
        return false;
    }
}
