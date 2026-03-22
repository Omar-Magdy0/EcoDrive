#include "PmsmController.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "arm_math.h"
#include <array>


#define DEBUG_PIN (16*1 + 2)


void bemfzc_ComCallback();
void hall1_ComCallback();

static constexpr float mc3p_sync_scales[][2] = 
{
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {(50) , (3.3/2)},
    {(50) , (3.3/2)},
    {(50) , (3.3/2)}
};

PmsmControl motor_c;

void PmsmControl::init(const elmotor_pmsm_stup_config_t& stup_cfg)
{
    xmc_ticks = 0;
    mc3p.config.pwm_Hz = PWM_FREQ;
    mc3p.config.duty_max = 0.95;
    mc3p.config.duty_min = 0.05;
    mc3p.config.deadtime_nS = 1500;

    stup.cfg    = stup_cfg;
    stup.comm_ticks = 0;
    elec.sector     = ELDRIVER_MC3P_SECTOR_FLOAT;
    stup.comm_timer.tick_period = XCPWM_TICKPERIOD_US;
    stup.stage_timer.tick_period = XCPWM_TICKPERIOD_US;
    state = PmsmMode::StartupTrap;
    stup.stage_current = pmsm_stup_stage_t::Reset;
    pos_sensor.init(XCPWM_TICKFREQ);
    eldriver_mc3p_init(&mc3p, mc3p_sync_scales);
    initialized = true;
}


/// @brief 
void PmsmControl::pwmLoop()
{
    if(!initialized)return;
    uint32_t start = eldriver_core_prof_tick();
    eldriver_mc3p_read_sync(&mc3p, &mc3p_sync_data);
    
    switch (state)
    {
    case PmsmMode::Idle:
        /* code */
        break;   
    case PmsmMode::StartupTrap:
        StupTrap_pwmLoop();
        break;
    case PmsmMode::ClosedTrap:
        ClosedTrap_pwmLoop();
        break;
    case PmsmMode::OpenTrap:
        OpenTrap_pwmLoop();
        break;
    case PmsmMode::Commission:
        Commission_pwmLoop();
        break;

    default:
        break;
    }
    xmc_ticks++;
    uint8_t len;
    
    pwmSample_t *sample_ptr = pwmDataBuffer.sample(&len);
    if(mc3p.mode == ELDRIVER_MC3P_MODE_TRAP && sample_ptr){
        (*sample_ptr)[0] = (int16_t)(((int64_t)(mc3p_sync_data.trap.vbus_q31 ) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[1] = (int16_t)(((int64_t)(mc3p_sync_data.trap.vbemf_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[2] = (int16_t)(((int64_t)(mc3p_sync_data.trap.cbus_q31) * ELDRIVER_MC3P_CS_SCALE * 1000) >> 31);
        (*sample_ptr)[4] = (int16_t)elec.speed_hz;
    }
    pwmDataBuffer.pushSample();
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void eldriver_mc3p_sync_postScanCallback()
{
    motor_c.pwmLoop();
}

void eldriver_xmc3p_tickerCallback()
{

}

//========================================================

void PmsmControl::set_speed(uint16_t speed_rpm)
{
    mech.speed_sp_rpm = speed_rpm;
}

void PmsmControl::freewheel()
{

}

void pwmDataBuffer_t::init()
{
    elcore_rstream_init(&buffer, (void *)frames.data(), sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    frame_sample_idx = 0;
    sample_count = 0;
    overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    elcore_rstream_reserveWrite(&buffer, 1, (void**)&currentFrame, &c1, (void**)&w2, &c2);
    currentFrame->sample_counter = 0;
    //update curernt frame value
}

pwmSample_t* pwmDataBuffer_t::sample(uint8_t *sample_len)
{
    if(frame_sample_idx < SAMPLES_PER_FRAME)
    {
        *sample_len = SAMPLE_LEN; //5 floats per sample
        return &currentFrame->samples[frame_sample_idx];
    }
    return NULL; // No available sample slot
}

void pwmDataBuffer_t::pushSample()
{
    if(frame_sample_idx < SAMPLES_PER_FRAME)
    {
        frame_sample_idx++;
    }
    sample_count++;
    if(frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        elcore_rstream_commitWrite(&buffer, 1); // Commit the current frame
        //Current frame is full, move to next frame
        uint8_t *w2;
        uint16_t c1, c2;
        //Commit the current full frame
        if(elcore_rstream_reserveWrite(&buffer, 1, (void**)&currentFrame, &c1, (void**)&w2, &c2))
        {
            currentFrame->sample_counter = sample_count;
            frame_sample_idx = 0;
        }else{
            //Buffer overflow, data loss occurs
            overflowCount++;
        }
    }
}
bool pwmDataBuffer_t::readFrame(PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    if(elcore_rstream_peekRead(&buffer, (void**)frame, &c1, (void**)&r2, &c2))
    {
        //Successfully read a frame
        //Process the frame data as needed
        //After processing, commit the read to move the tail forward
        elcore_rstream_releaseRead(&buffer, 1);
        return true;
    }else{
        return false;
    }
}
