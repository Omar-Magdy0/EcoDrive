/**
 * @file PmsmControl.cpp
 * @brief Implementation of the Permanent Magnet Synchronous Motor (PMSM) controller.
 * @details Contains the primary control loops, hardware configuration, state machine routing,
 *          and telemetry data buffering logic required to drive a PMSM.
 * @details Detailed Doxygen documentation and code analysis by Bishoy Youssef.
 */
 
#include "PmsmControl.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "arm_math.h"
#include <array>

/**
 * @def DEBUG_PIN
 * @brief GPIO pin definition used for toggling signals to debug execution timing and measure interrupt latency.
 */
#define DEBUG_PIN (16 * 1 + 2)

/**
 * @brief External callback function triggered upon detecting a Back-EMF Zero Crossing.
 * @details Used in sensorless trapezoidal control to advance the commutation sector at the right electrical angle.
 */
void bemfzc_ComCallback();

/**
 * @brief External callback function triggered upon a change in the Hall Sensor state.
 * @details Used in sensored trapezoidal control to instantly apply the next commutation sector based on rotor position.
 */
void hall1_ComCallback();

/**
 * @brief Global instance of the primary PMSM control structure for motor channel 1.
 */
PmsmControl motor_c1;

/**
 * @brief Initializes the overall PMSM controller, state machine, and associated hardware drivers.
 * @details Resets the global PWM tick counter, applies the default PWM frequency, configures the
 *          inverter's maximum/minimum duty cycles and deadtime, and initializes the MC3P driver.
 *          It also prepares the commutation and staging software timers and sets the initial mode.
 */
void PmsmControl::init()
{
    /// Reset the PWM tick counter to zero to establish a fresh time reference
    pwmTicks = 0;
    /// Set the initial PWM carrier frequency to the system's default value (typically 25kHz)
    set_pwm_freq(PWM_FREQ_DEFAULT);
    /// Apply the selected PWM frequency to the 3-phase motor driver configuration
    mc3p.config.pwm_Hz = pwm_freq_hz;
    /// Constrain the maximum allowed PWM duty cycle to 95% to leave a window for current sensing
    mc3p.config.duty_max = 0.95;
    /// Constrain the minimum allowed PWM duty cycle to 5% to ensure bootstrap capacitors stay charged
    mc3p.config.duty_min = 0.05;
    /// Set the switching deadtime to 1500 nanoseconds to prevent shoot-through in the inverter bridge
    mc3p.config.deadtime_nS = 1500;

    stup.comm_ticks = 0;
    /// Initialize the electrical sector to a floating state to ensure all power switches are safely off initially
    elec.sector = ELDRIVER_MC3P_SECTOR_FLOAT;
    /// Synchronize the commutation timer's software period with the calculated microsecond tick period
    stup.comm_timer.tick_period = tick_period_us;
    /// Synchronize the staging timer's software period with the calculated microsecond tick period
    stup.stage_timer.tick_period = tick_period_us;
    /// Set the default control mode to Closed-Loop Trapezoidal as the standard operational state
    mode = ControlMode::ClosedTrap;
    /// Initialize the startup state machine to the Reset stage, waiting for a start command
    stup.stage_current = StupStage::Reset;
    /// Initialize the rotor position sensor logic (Hall or Sensorless) with the active PWM frequency
    pos_sensor.init(pwm_freq_hz);
    /// Enable automated hardware offset calibration for the phase current sensors during driver startup
    mc3p.offset_calibration = true;
    /// Initialize the underlying MC3P hardware motor driver
    eldriver_mc3p_init(&mc3p);
    /// Flag the controller state as successfully initialized to allow main loop execution
    initialized = true;
}

/**
 * @brief Updates the system PWM frequency and recalculates all dependent hardware and software timing periods.
 * @param pwm_hz The target PWM carrier frequency in Hertz.
 * @details This recalculates the tick period in both microseconds and milliseconds. It updates the underlying
 *          MC3P hardware driver configuration and resynchronizes the software timers (commutation and stage timers)
 *          to ensure timing accuracy is maintained across frequency changes.
 */
void PmsmControl::set_pwm_freq(uint32_t pwm_hz)
{
    /// Prevent division by zero crashes if an invalid frequency (0 Hz) is requested
    if (pwm_hz == 0)
        return;
    /// Update the internal active frequency tracking variable
    pwm_freq_hz = pwm_hz;
    /// Calculate the precise time period of one PWM tick in microseconds
    tick_period_us = 1'000'000.0f / static_cast<float>(pwm_freq_hz);
    /// Calculate the precise time period of one PWM tick in milliseconds
    tick_period_ms = tick_period_us / 1000.0f;
    /// Update the underlying driver configuration payload with the newly requested frequency
    mc3p.config.pwm_Hz = pwm_freq_hz;
    /// Resynchronize the timing periods for the startup phase commutation and stage software timers
    stup.comm_timer.tick_period = tick_period_us;
    stup.stage_timer.tick_period = tick_period_us;
    /// Reinitialize the position sensor logic to adapt filtering and delays to the new timing characteristics
    pos_sensor.init(pwm_freq_hz);
}

/**
 * @brief Main high-frequency Pulse Width Modulation (PWM) control loop.
 * @details Executed periodically (usually bound to an ADC synchronization interrupt or a hardware timer).
 *          It forms the core of the fast motor control loop. Depending on the active `ControlMode`, it delegates
 *          execution to specific routines (e.g., Idle, OpenTrap, ClosedTrap, OpenFocIF, Commission).
 *          It also captures high-frequency electrical telemetry (Bus voltage, BEMF, current, speed) and pushes it to a diagnostic data buffer.
 */
void PmsmControl::pwmLoop()
{
    /// Abort execution safely if the controller initialization sequence has not been completed
    if (!initialized)
        return;
    /// Start performance profiling to measure the execution time of this time-critical fast loop
    uint32_t start = eldriver_core_prof_tick();
    /// Retrieve the latest synchronized analog measurements (currents, voltages) from the hardware driver
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_meas);
    /// Update the local state with the newly measured DC bus voltage for accurate duty cycle scaling
    elec.vbus = mc3p_sync_meas.svm.vbus_q31;

    /// Route execution to the appropriate specific control handler based on the current active motor control mode
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
    /// Increment the global PWM loop tick counter to track absolute time for software timers
    pwmTicks++;
    uint8_t len;

    /// Request a fresh, writable telemetry sample slot from the data ring buffer
    pwmSample_t *sample_ptr = pwmDataBuffer.sample(&len);
    /// Populate the telemetry sample structure only if the driver is actively in Trapezoidal mode and buffer space is available
    if (mc3p.mode == ELDRIVER_MC3P_MODE_TRAP && sample_ptr)
    {
        /// Store scaled bus voltage data, converted to a signed 16-bit integer for compact transmission
        (*sample_ptr)[0] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbus_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        /// Store scaled Back-EMF voltage data, converted to a signed 16-bit integer
        (*sample_ptr)[1] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.vbemf_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        /// Store scaled DC bus return current data, converted to a signed 16-bit integer
        (*sample_ptr)[2] = (int16_t)(((int64_t)(mc3p_sync_meas.trap.cbus_q31) * ELDRIVER_MC3P_CS_SCALE * 1000) >> 31);
        /// Store the instantaneous calculated electrical speed of the rotor in Hertz
        (*sample_ptr)[4] = (int16_t)elec.speed_hz;
    }
    /// Commit the populated sample data to the active telemetry frame
    pwmDataBuffer.pushSample();
    /// End performance profiling and calculate the total ticks elapsed during this control cycle
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

/**
 * @brief Cross-domain or low-frequency auxiliary control loop.
 * @details Designed to execute slower, non-time-critical tasks that should not block the fast PWM interrupt.
 *          This includes complex mathematical post-processing during self-commissioning (e.g., floating-point
 *          square roots and arctangents) or managing high-level mechanical speed control loops.
 */
void PmsmControl::xmcLoop()
{
    /// Route execution to specific auxiliary routines based on the currently active motor control mode
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
        /// Execute complex, low-priority self-commissioning post-process math calculations
        SelfCommission_xmcLoop();
        break;

    default:
        break;
    }
}

/**
 * @name CALLBACK DEFINITIONS
 * @brief System-level interrupt service routines and hardware synchronization callbacks.
 */
///@{

/**
 * @brief Hardware callback triggered immediately after a synchronous ADC scan completes.
 * @details This is the entry point for the fast motor control loop. Triggering it post-ADC scan ensures
 *          that the control algorithm uses the freshest possible phase current and voltage measurements,
 *          minimizing latency in the feedback loop.
 */
void eldriver_mc3p_sync_postScanCallback()
{
    /// Execute the primary high-frequency PWM control loop for motor channel 1
    motor_c1.pwmLoop();
}

/**
 * @brief Periodic software timer callback for slower, cross-domain motor control tasks.
 * @details Typically executed at a much lower frequency compared to the PWM loop.
 *          It dispatches to `xmcLoop()` to handle tasks like parameter estimation math or state transitions.
 */
void eldriver_xmc3p_tickerCallback()
{
    /// Execute the auxiliary low-frequency control loop for motor channel 1
    motor_c1.xmcLoop();
}
///@}

//========================================================
/**
 * @brief Initializes the telemetry PWM data ring buffer and reserves the first frame.
 * @details Prepares the underlying reliable stream (rstream) to manage a fixed number of frames (`FRAME_BUFFER_COUNT`).
 *          It resets global sample tracking counters and immediately attempts to reserve the first block of memory.
 */
void pwmDataBuffer_t::init()
{
    /// Initialize the underlying reliable stream (rstream) ring buffer with the predefined memory block of frames
    elcore_rstream_init(&buffer, (void *)frames.data(), sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    /// Reset the active sample index pointer to the start of the current frame
    frame_sample_idx = 0;
    /// Reset the absolute global telemetry sample tracking counter
    sample_count = 0;
    /// Reset the counter that tracks how many telemetry data points were dropped due to buffer overflows
    overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    /// Attempt to reserve the very first initial block of memory within the stream for writing the first data frame
    elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2);
    /// Stamp the base sample counter value for the first newly reserved frame to guarantee chronologically synced data
    currentFrame->sample_counter = 0;
    /// @note The pointer `currentFrame` is now successfully initialized and ready for immediate writing by the fast loop.
}

/**
 * @brief Requests a writable pointer for the next available sample slot in the active telemetry frame.
 * @param sample_len Output pointer that will be populated with the expected length (number of data points) per sample.
 * @return Pointer to the `pwmSample_t` array slot within the active frame, or NULL if the frame has reached maximum capacity.
 * @details Called by the fast control loop to obtain a memory location for storing real-time telemetry.
 *          It performs a bounds check against `SAMPLES_PER_FRAME` to prevent buffer overflows within the frame.
 */
pwmSample_t *pwmDataBuffer_t::sample(uint8_t *sample_len)
{
    /// Verify there is still allocated capacity remaining inside the currently active frame
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        /// Set the output length variable, assigning the expected number of parameters (e.g., 5 raw data points per sample)
        *sample_len = SAMPLE_LEN;
        /// Return the direct memory address of the targeted sample array slot within the active frame for fast data assignment
        return &currentFrame->samples[frame_sample_idx];
    }
    /// Return NULL to safely indicate to the caller that there are no available sample slots left in the active frame
    return NULL;
}

/**
 * @brief Commits the currently populated telemetry sample and handles frame rollover.
 * @details Increments the active sample index. If the current frame becomes completely filled
 *          (i.e., `frame_sample_idx` reaches `SAMPLES_PER_FRAME`), the frame is committed to the read stream
 *          for consumer processing. It then immediately attempts to reserve a new frame.
 */
void pwmDataBuffer_t::pushSample()
{
    /// Increment the local intra-frame tracking index if there is still room before saturation
    if (frame_sample_idx < SAMPLES_PER_FRAME)
    {
        frame_sample_idx++;
    }
    /// Always increment the absolute global system sample counter for synchronized time-stamping
    sample_count++;
    /// Check if the current telemetry frame has now reached its predetermined maximum sample capacity
    if (frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        /// Commit the completed, thoroughly filled frame into the readable stream ring buffer for asynchronous consumer processing
        elcore_rstream_commitWrite(&buffer, 1);
        
        /// Allocate auxiliary pointers and length variables required by the stream interface for the next memory reservation
        uint8_t *w2;
        uint16_t c1, c2;
        /// Attempt to aggressively reserve a brand new block of memory from the stream for the next incoming data frame
        if (elcore_rstream_reserveWrite(&buffer, 1, (void **)&currentFrame, &c1, (void **)&w2, &c2))
        {
            /// Memory reservation successful; timestamp the new frame with the current absolute sample count to maintain chronological synchronization
            currentFrame->sample_counter = sample_count;
            /// Reset the local intra-frame index to start cleanly populating from the beginning of the new memory block
            frame_sample_idx = 0;
        }
        else
        {
            /// Memory reservation failed due to the consumer being too slow (buffer overflow); increment the drop counter to reflect lost telemetry data
            overflowCount++;
        }
    }
}

/**
 * @brief Attempts to retrieve a completely populated telemetry frame from the read buffer.
 * @param frame Output double pointer that will point to the fetched `PwmDataFrame_t` if successful.
 * @return `true` if a completed frame was successfully peeked and released for reading, `false` otherwise.
 * @details Intended to be called by a lower-priority consumer task (like a communications thread).
 *          It peeks at the buffer to get the data, and immediately releases the read sector so the high-frequency
 *          producer can reuse the memory block in the ring buffer.
 */
bool pwmDataBuffer_t::readFrame(PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    /// Non-destructively peek into the read stream buffer to check for an available full frame without immediately moving the read pointer
    if (elcore_rstream_peekRead(&buffer, (void **)frame, &c1, (void **)&r2, &c2))
    {
        /// Successfully located an available, fully populated frame; the calling consumer will now process its data payload
        /// Immediately release the read buffer segment from the stream so the memory block can be swiftly reclaimed by the high-frequency writer
        elcore_rstream_releaseRead(&buffer, 1);
        return true;
    }
    else
    {
        /// No fully populated frames are currently available to be read by the consumer
        return false;
    }
}
