#pragma once
#include "core/elmath.h"
#include "core/elcore.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include "sensor.h"
#include "sys.h"
#include "arm_math.h"
//===================================================
// CONFIG
//==================================================

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_FREQ                   40000
#define STUP_BEMFZC_ERROR_MARGIN    0.05 
#define STUP_BEMFZC_GOOD_EST_COUNT    10

#define XCPWM_TICKFREQ            PWM_FREQ
#define XCPWM_TICKPERIOD_US       (1000000.0/XCPWM_TICKFREQ)
#define XCPWM_TICKPERIOD_MS       (XCPWM_TICKPERIOD_US/1000.0)
#define XCPWM_US_TO_TICKS(us)     (us/XCPWM_TICKPERIOD_US)
#define XCPWM_MS_TO_TICKS(ms)     (XCPWM_US_TO_TICKS(ms*1000))
#define XCPWM_TICKS_TO_US(ticks)  (ticks * XCPWM_TICKPERIOD_US)
#define XCPWM_TICKS_TO_MS(ticks)  (ticks * XCPWM_TICKPERIOD_MS)

#define STUP_TABLE_SIZE 4

typedef struct
{
    uint16_t align_duration_ms;
    eldriver_mc3p_sector_t align_sector;
    float bus_V;
    float align_V;
    float time_mS[STUP_TABLE_SIZE];
    float volt_V[STUP_TABLE_SIZE];
    float freq_Hz[STUP_TABLE_SIZE];
} elmotor_pmsm_stup_config_t;

typedef enum
{
    STUP_STAGE_RESET = 0,
    STUP_STAGE_ALIGN,
    STUP_STAGE_RAMP,
    STUP_STAGE_STBEMF_SWITCHOVER
} pmsm_stup_stage_t;

typedef enum
{
    ELMOTOR_DIR_FORWARD = 0,
    ELMOTOR_DIR_BACKWARD
} elmotor_dir_t;


typedef enum{
    ELMOTOR_IDLE = 0,
    ELMOTOR_STUP_TRAP,
    ELMOTOR_CL_TRAP,
    ELMOTOR_OL_TRAP
}elmotor_pmsm_state;


typedef struct{
    uint32_t xmc_ticks;
    bool initialized;
    elmotor_pmsm_state state;
    eldriver_mc3p_t mc3p;
    pos_sensor_t pos_sensor;

    union{
        eldriver_mc3p_svm_data_t svm;
        eldriver_mc3p_trap_data_t trap;
    }mc3p_sync_data;

    struct{
        elmotor_pmsm_stup_config_t cfg;
        elcore_swttimer_t stage_timer;
        elcore_swttimer_t comm_timer;
        uint32_t          comm_ticks;
        pmsm_stup_stage_t stage_last;
        pmsm_stup_stage_t stage_current;
        uint8_t           ramp_idx;
        float             est_elec_speed;
        uint8_t           good_est_count;
    }stup;
    
    struct{
        float vbus;
        q15_t alpha_q15;
        q15_t beta_q15;
        q15_t trap_duty_q15;
        eldriver_mc3p_sector_t sector;
        float speed_hz;
    }elec;
    
    struct{
        volatile elmotor_dir_t dir;
        volatile float         speed_rpm;
        volatile float         speed_sp_rpm;
    }mech;

    uint8_t pole_pairs;
} elmotor_pmsm_t;

extern elmotor_pmsm_t motor_c;
void elmotor_pmsm_init(elmotor_pmsm_t *cp, elmotor_pmsm_stup_config_t stup_cfg);
void elmotor_pmsm_setSpeed(elmotor_pmsm_t *cp, uint16_t speed_rpm);
void elmotor_pmsm_freewheel(elmotor_pmsm_t *cp);



#include "core/elcore.h"

#define SAMPLE_LEN 5
#define SAMPLES_PER_FRAME 24
#define FRAME_BUFFER_COUNT 8 
#define FRAME_BUFFER_NOTIFY_THRESHOLD 6

typedef int16_t pwmSample_t[SAMPLE_LEN];
typedef struct
{
    uint32_t sample_counter;
    pwmSample_t samples[SAMPLES_PER_FRAME]; // 5 floats per sample as per schema
}PwmDataFrame_t;


typedef struct
{
    PwmDataFrame_t frames[FRAME_BUFFER_COUNT];
    elcore_rstream_t buffer;
    PwmDataFrame_t* currentFrame;
    uint32_t frame_sample_idx;
    uint32_t sample_count;
    uint32_t overflowCount; // Count of how many times data was lost due to overflow
}pwmDataBuffer_t;


void pwmDataBuffer_init(pwmDataBuffer_t *cp);
pwmSample_t *pwmDataBuffer_sample(pwmDataBuffer_t *cp, uint8_t *len);
void pwmDataBuffer_pushSample(pwmDataBuffer_t *cp);
bool pwmDataBuffer_readFrame(pwmDataBuffer_t *cp, PwmDataFrame_t **outFrame);
extern pwmDataBuffer_t pwmDataBuffer;


#ifdef __cplusplus
}
#endif
