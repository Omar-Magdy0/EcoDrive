#include "elmotor_pmsm.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "arm_math.h"


#define DEBUG_PIN (16*1 + 2)

// For trap sectors specifically (1-6)
#define TRAP_INCREMENT(sector, dir)((dir == ELMOTOR_DIR_FORWARD)?elmath_increment_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6):elmath_decrement_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6))

uint8_t HALL_TO_TRAP_TABLE[8] = 
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

void bemfzc_ComCallback();
void hall1_ComCallback();

float mc3p_sync_scales[][2] = 
{
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {((47+2.2)/(2.2)) ,0},
    {(50) , (3.3/2)},
    {(50) , (3.3/2)},
    {(50) , (3.3/2)}
};

elmotor_pmsm_t motor_c;

void elmotor_pmsm_init(elmotor_pmsm_t *cp, elmotor_pmsm_stup_config_t stup_cfg)
{
    cp->xmc_ticks = 0;
    cp->mc3p.config.pwm_Hz = PWM_FREQ;
    cp->mc3p.config.duty_max = 0.95;
    cp->mc3p.config.duty_min = 0.05;
    cp->mc3p.config.deadtime_nS = 1500;

    cp->stup.cfg    = stup_cfg;
    cp->stup.comm_ticks = 0;
    cp->elec.sector     = ELDRIVER_MC3P_SECTOR_FLOAT;
    cp->stup.comm_timer.tick_period = XCPWM_TICKPERIOD_US;
    cp->stup.stage_timer.tick_period = XCPWM_TICKPERIOD_US;
    cp->state = ELMOTOR_STUP_TRAP;
    cp->stup.stage_current = STUP_STAGE_RESET;
    pos_init(&cp->pos_sensor, XCPWM_TICKFREQ);
    eldriver_mc3p_init(&(cp->mc3p), mc3p_sync_scales);
    cp->initialized = 1;
}


void pmsm_idle(elmotor_pmsm_t *cp)
{
}

static inline void pmsm_stup_trapBemf(elmotor_pmsm_t *cp)
{
    switch (cp->stup.stage_current)
    {
    case STUP_STAGE_RESET:
        cp->stup.stage_last    = STUP_STAGE_RESET;
        cp->stup.stage_current = STUP_STAGE_ALIGN;
        cp->stup.good_est_count = 0;
    case STUP_STAGE_ALIGN:
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            //APPLY ALIGN VOLTAGE
            float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
            arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
            cp->elec.sector = cp->stup.cfg.align_sector; 
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->xmc_ticks);
            cp->stup.stage_last = STUP_STAGE_ALIGN;
        }
        
        if(elcore_swttimer_timout(&(cp->stup.stage_timer), cp->xmc_ticks, XCPWM_MS_TO_TICKS(cp->stup.cfg.align_duration_ms)))
        {
            cp->stup.stage_current = STUP_STAGE_RAMP;
        }
        else
        {
            break;
        }
    case STUP_STAGE_RAMP:
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->xmc_ticks);
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->xmc_ticks);
            cp->stup.stage_last = STUP_STAGE_RAMP;
        }
        uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->xmc_ticks, cp->stup.comm_ticks);
        if(cp->stup.ramp_idx < STUP_TABLE_SIZE - 1)
        {    
            //increment voltage
            float et_mS = XCPWM_TICKS_TO_MS(elcore_swttimer_elapsed_ticks(&cp->stup.stage_timer, cp->xmc_ticks));
            float32_t v_norm = elmath_linearInterp(&cp->stup.cfg.volt_V[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS)/cp->stup.cfg.bus_V;
            arm_float_to_q15((&v_norm), &cp->elec.trap_duty_q15, 1);
            //increment frequency
            if(comm)
            {
                float32_t f = elmath_linearInterp(&cp->stup.cfg.freq_Hz[cp->stup.ramp_idx], &cp->stup.cfg.time_mS[cp->stup.ramp_idx], et_mS);
                cp->stup.comm_ticks = XCPWM_MS_TO_TICKS(1000.0 / (f * 6));
                cp->elec.speed_hz = 2 * M_PI * (XCPWM_TICKFREQ / (cp->stup.comm_ticks * 6) );
            }
            if(et_mS > cp->stup.cfg.time_mS[cp->stup.ramp_idx + 1]){
                cp->stup.ramp_idx++;
            }
        }else{
            bemfzc_update(&cp->pos_sensor.bemf, cp->mc3p_sync_data.trap.vbemf_q31, cp->xmc_ticks, cp->mech.dir);
            cp->stup.est_elec_speed = bemfzc_elec_speed(&cp->pos_sensor.bemf);
            if(NEARLY_EQUAL(cp->stup.est_elec_speed, cp->elec.speed_hz, cp->elec.speed_hz*(STUP_BEMFZC_ERROR_MARGIN)))
            {
                cp->stup.good_est_count++;
            }else{
                cp->stup.good_est_count = 0;
            }
            if(cp->stup.good_est_count >= STUP_BEMFZC_GOOD_EST_COUNT)
            {
                ////Switchover and handle to closed loop commutation
                //bemfzc_takeover(&cp->pos_sensor.bemf, bemfzc_ComCallback);
                //cp->state = ELMOTOR_CL_TRAP;
            }            
        }
        if(comm){
            cp->elec.sector = TRAP_INCREMENT(cp->elec.sector, cp->mech.dir);   
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->xmc_ticks);
        }
        break;
    default:
        break;
    }
}

static inline void pmsm_stup_trapHall(elmotor_pmsm_t *cp)
{
    //APPLY ALIGN VOLTAGE
    float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    uint8_t hall_value = eldriver_hall1_read();
    cp->elec.sector = HALL_TO_TRAP_TABLE[hall_value];  
    cp->elec.sector = TRAP_INCREMENT(cp->elec.sector, motor_c.mech.dir);
    pos_set_com_delay_uS(&cp->pos_sensor, COMMUTATION_PHASE_DELAY);
    pos_set_com_callback(&cp->pos_sensor, hall1_ComCallback);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
    cp->state = ELMOTOR_CL_TRAP;
}

static inline void pmsm_cl_trap(elmotor_pmsm_t *cp)
{
    float32_t r = (float32_t)(2.5 / cp->stup.cfg.bus_V);
    arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
    pos_update(&cp->pos_sensor, cp->mc3p_sync_data.trap.vbemf_q31, cp->xmc_ticks, cp->mech.dir);
    cp->elec.speed_hz = pos_elec_speed(&cp->pos_sensor);
    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
}
static inline void pmsm_ol_trap(elmotor_pmsm_t *cp)
{
    switch (cp->stup.stage_current)
    {
    case STUP_STAGE_RESET:
        cp->stup.stage_last    = STUP_STAGE_RESET;
        cp->stup.stage_current = STUP_STAGE_ALIGN;
        cp->stup.good_est_count = 0;
    case STUP_STAGE_ALIGN:
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            //APPLY ALIGN VOLTAGE
            float32_t r = (float32_t)(cp->stup.cfg.align_V / cp->stup.cfg.bus_V);
            arm_float_to_q15(&r, &(cp->elec.trap_duty_q15), 1);
            cp->elec.sector = cp->stup.cfg.align_sector; 
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->xmc_ticks);
            cp->stup.stage_last = STUP_STAGE_ALIGN;
        }
        if(elcore_swttimer_timout(&(cp->stup.stage_timer), cp->xmc_ticks, XCPWM_MS_TO_TICKS(cp->stup.cfg.align_duration_ms)))
        {
            cp->stup.stage_current = STUP_STAGE_RAMP;
        }
        else
        {
            break;
        }
    case STUP_STAGE_RAMP:
        if(cp->stup.stage_current != cp->stup.stage_last)
        {
            elcore_swttimer_reset(&(cp->stup.stage_timer), cp->xmc_ticks);
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->xmc_ticks);
            cp->stup.stage_last = STUP_STAGE_RAMP;
        }
        uint8_t comm = elcore_swttimer_timout(&cp->stup.comm_timer, cp->xmc_ticks, cp->stup.comm_ticks);
        //increment frequency
        if(comm)
        {
            cp->mech.speed_rpm = cp->mech.speed_sp_rpm;
            float32_t f = (cp->mech.speed_sp_rpm / 60.0) * (cp->pole_pairs);
            cp->stup.comm_ticks = XCPWM_MS_TO_TICKS(1000.0 / (f * 6));
            cp->elec.speed_hz = 2 * M_PI * (XCPWM_TICKFREQ / (cp->stup.comm_ticks * 6) );
            cp->elec.sector = TRAP_INCREMENT(cp->elec.sector, cp->mech.dir);   
            elcore_swttimer_reset(&(cp->stup.comm_timer), cp->xmc_ticks);
        }
        break;
    default:
        break;
    }
}
static inline void pmsm_stup_trap(elmotor_pmsm_t *cp)
{
    #ifdef ELDRIVER_BEMFZC_ENABLED
    pmsm_stup_trapBemf(cp);
    #endif

    #ifdef ELDRIVER_HALL1_ENABLED
    pmsm_stup_trapHall(cp);
    #endif

    eldriver_mc3p_write_trap(&cp->mc3p, cp->elec.sector, cp->elec.trap_duty_q15);
}
static inline void pmsm_pwm_loop(elmotor_pmsm_t *cp)
{
    if(!cp->initialized)return;
    uint32_t start = eldriver_core_prof_tick();
    eldriver_mc3p_read_sync(&cp->mc3p, &cp->mc3p_sync_data);
    
    switch (cp->state)
    {
    case ELMOTOR_IDLE:
        /* code */
        break;   
    case ELMOTOR_STUP_TRAP:
        /* code */
        pmsm_stup_trap(cp);
        break;
    case ELMOTOR_CL_TRAP:
        pmsm_cl_trap(cp);
        break;
    case ELMOTOR_OL_TRAP:
        pmsm_ol_trap(cp);
        break;

    default:
        break;
    }
    motor_c.xmc_ticks++;
    uint8_t len;
    
    pwmSample_t *sample_ptr = pwmDataBuffer_sample(&pwmDataBuffer, &len);
    if(cp->mc3p.mode == ELDRIVER_MC3P_MODE_TRAP && sample_ptr){
        (*sample_ptr)[0] = (int16_t)(((int64_t)(cp->mc3p_sync_data.trap.vbus_q31 )* ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[1] = (int16_t)(((int64_t)(cp->mc3p_sync_data.trap.vbemf_q31) * ELDRIVER_MC3P_VS_SCALE * 1000) >> 31);
        (*sample_ptr)[2] = (int16_t)(((int64_t)(cp->mc3p_sync_data.trap.cbus_q31) * ELDRIVER_MC3P_CS_SCALE * 1000) >> 31);
        (*sample_ptr)[4] = (int16_t)cp->elec.speed_hz;
    }
    pwmDataBuffer_pushSample(&pwmDataBuffer);
    volatile uint32_t elapsed = eldriver_core_prof_tock(start);
}

//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void eldriver_mc3p_sync_postScanCallback()
{
    pmsm_pwm_loop(&motor_c);
}

void eldriver_xmc3p_tickerCallback()
{

}

void bemfzc_ComCallback()
{
    motor_c.elec.sector = TRAP_INCREMENT(motor_c.elec.sector, motor_c.mech.dir);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
}

void hall1_ComCallback()
{
    uint8_t hall_value = eldriver_hall1_read();
    motor_c.elec.sector = HALL_TO_TRAP_TABLE[hall_value];  
    motor_c.elec.sector = TRAP_INCREMENT(motor_c.elec.sector, motor_c.mech.dir);
    eldriver_mc3p_write_trap(&motor_c.mc3p, motor_c.elec.sector, motor_c.elec.trap_duty_q15);
}
//========================================================

void pwmDataBuffer_init(pwmDataBuffer_t *cp)
{
    elcore_rstream_init(&cp->buffer, (void *)cp->frames, sizeof(PwmDataFrame_t), FRAME_BUFFER_COUNT);
    cp->frame_sample_idx = 0;
    cp->sample_count = 0;
    cp->overflowCount = 0;
    uint8_t *w2;
    uint16_t c1, c2;
    elcore_rstream_reserveWrite(&cp->buffer, 1, (void**)&cp->currentFrame, &c1, (void**)&w2, &c2);
    cp->currentFrame->sample_counter = 0;
    //update curernt frame value
}
pwmSample_t* pwmDataBuffer_sample(pwmDataBuffer_t *cp, uint8_t *sample_len)
{
    if(cp->frame_sample_idx < SAMPLES_PER_FRAME)
    {
        *sample_len = SAMPLE_LEN; //5 floats per sample
        return &cp->currentFrame->samples[cp->frame_sample_idx];
    }
    return NULL; // No available sample slot
}
void pwmDataBuffer_pushSample(pwmDataBuffer_t *cp)
{
    if(cp->frame_sample_idx < SAMPLES_PER_FRAME)
    {
        cp->frame_sample_idx++;
    }
    cp->sample_count++;
    if(cp->frame_sample_idx >= SAMPLES_PER_FRAME)
    {
        elcore_rstream_commitWrite(&cp->buffer, 1); // Commit the current frame
        //Current frame is full, move to next frame
        uint8_t *w2;
        uint16_t c1, c2;
        //Commit the current full frame
        if(elcore_rstream_reserveWrite(&cp->buffer, 1, (void**)&cp->currentFrame, &c1, (void**)&w2, &c2))
        {
            cp->currentFrame->sample_counter = cp->sample_count;
            cp->frame_sample_idx = 0;
        }else{
            //Buffer overflow, data loss occurs
            cp->overflowCount++;
        }
    }
}
bool pwmDataBuffer_readFrame(pwmDataBuffer_t *cp, PwmDataFrame_t **frame)
{
    uint8_t *r2;
    uint16_t c1, c2;
    if(elcore_rstream_peekRead(&cp->buffer, (void**)frame, &c1, (void**)&r2, &c2))
    {
        //Successfully read a frame
        //Process the frame data as needed
        //After processing, commit the read to move the tail forward
        elcore_rstream_releaseRead(&cp->buffer, 1);
        return true;
    }else{
        return false;
    }
}


