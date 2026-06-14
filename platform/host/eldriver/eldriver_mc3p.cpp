#include "eldriver_mc3p.h"
#include "platform.h"
#include "eldriver_conf.h"
#include "sil.h"
#include "silgui.h"
#include "eldriver_hall.h"

#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))
#define Q15_HALF        16384   // 0.5 × 32768
#define Q15_SQRT3_BY_2  28378   // 0.8660254 × 32768

extern float vtime;
float eldriver_mc3p_adc_read_single(eldriver_mc3p_t *h, uint32_t channel){}

void postScanMethod()
{
    eldriver_mc3p_sync_postScanCallback();
    sil_input.load_torque = (dummy_load.K * sil.state.omega + dummy_load.B) * sil.state.omega + dummy_load.Tc;
    for(int i = 0; i <  SIL_TO_PWM_FREQ; i++) {
        sil.step(sil_input);
        sil_hall_update();
        silgui_updateData();
        silLogger.log(sil, sil_input);
    }
}


inline double trapezoidal_wave(double theta)
{
    static const double angles[] = {0, M_PI/6, 5*M_PI/6, M_PI, 7*M_PI/6, 11*M_PI/6, 12*M_PI/6};
    static const double values[] = {0, 1     , 1       , 0   , -1      , -1       , 0        };
    theta = fmod(theta, 2*M_PI);
    if (theta < 0) theta += 2*M_PI;
    int cval_idx, nval_idx = 0;
    int arr_size = sizeof(angles)/sizeof(double);
    for(int i = 0; i < arr_size; i++) {
        if(theta >= angles[i]) {
            cval_idx = i;
        }
    }
    nval_idx = (cval_idx+1)%arr_size;
    double m = (values[nval_idx] - values[cval_idx]) / (angles[nval_idx] - angles[cval_idx]);
    return values[cval_idx] + m * (theta - angles[cval_idx]);
}
inline void dPsim_dTheta(double theta_e, double dPsi_dTheta[3])
{
    double amplitude = sil.param.motor_fluxLinkage; // Back EMF amplitude
    #ifdef SINE_BEMF
    dPsi_dTheta[0] = - amplitude * sin(theta_e);
    dPsi_dTheta[1] = - amplitude * sin(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * sin(theta_e + 2*M_PI/3);
    #else
    // Trapezoidal waveform function (120° flat top, 60° rising/falling edges)
    dPsi_dTheta[0] = - amplitude * trapezoidal_wave(theta_e);
    dPsi_dTheta[1] = - amplitude * trapezoidal_wave(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * trapezoidal_wave(theta_e + 2*M_PI/3);
    #endif
};

void eldriver_mc3p_init(eldriver_mc3p_t *h)
{
    sil_input.dt = 1.0f / h->config.pwm_Hz / SIL_TO_PWM_FREQ;
    sil_input.load_torque = 0.00;
    sil_input.vcc = SIL_DEFAULT_VCC;
    sil.param.dPsim_dTheta = dPsim_dTheta;
    sil.param.inv_Ron = 0.008;
    sil.param.inv_Roff = 1e6;
    sil.param.motor_pp = 6;
    sil.param.motor_Rs = 0.25;
    sil.param.motor_Ls = 0.0008;
    sil.param.motor_Lm = 0;
    sil.param.motor_Ms = 0;
    sil.param.motor_rotorOffset = 0;
    sil.param.motor_fluxLinkage = 0.036;
    sil.param.motor_B = 1e-2;
    sil.param.motor_J = 8e-5;
    dummy_load.K = 1.2e-5;
    dummy_load.J = 1e-3;
    dummy_load.B = 5e-2;
    sil.param.load_J = dummy_load.J;
    register_timer(&timer_manager, postScanMethod, (uint64_t)(1e9/h->config.pwm_Hz));
    register_timer(&timer_manager, eldriver_xmc3p_tickerCallback, (uint64_t)(1e9/ELDRIVER_XMC3P_TICKFREQ));
}

void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float* scanData){}
uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h){}

void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void* data)
{    
    uint8_t is_svm = IS_SVM_SECTOR(h->sector_last);
    uint8_t is_trap = IS_TRAP_SECTOR(h->sector_last);
    
    if(is_svm)
    {       
        // Convert float currents to Q31 format for the control algorithm
        // Scaling: 1.0A = X counts (adjust ELDRIVER_MC3P_CS_SCALE to tune sensitivity)
        ((eldriver_mc3p_svm_data_t *)(data))->vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(sil_input.vcc);
        ((eldriver_mc3p_svm_data_t *)(data))->cu_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[0]);
        ((eldriver_mc3p_svm_data_t *)(data))->cv_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[1]);
        ((eldriver_mc3p_svm_data_t *)(data))->cw_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[2]);
    }
    else if (is_trap)
    {
        // Back-EMF would be: Ke * ωe where ωe = pp * ω_mech
        ((eldriver_mc3p_svm_data_t *)(data))->vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(sil_input.vcc);
        float adc_syncRank1, adc_syncRank2;
        switch(h->sector_last){
            case ELDRIVER_MC3P_SECTOR_TRAP1:
                adc_syncRank1 = sil.state.vp[2] - sil.state.vn;
                adc_syncRank2 = sil.state.ip[0];
                break;
            case ELDRIVER_MC3P_SECTOR_TRAP2:
                adc_syncRank1 = sil.state.vp[1];
                adc_syncRank2 = sil.state.ip[0];
                break;
            case ELDRIVER_MC3P_SECTOR_TRAP3:
                adc_syncRank1 = sil.state.vp[0];
                adc_syncRank2 = sil.state.ip[1];
                break;
            case ELDRIVER_MC3P_SECTOR_TRAP4:
                adc_syncRank1 = sil.state.vp[2];
                adc_syncRank2 = sil.state.ip[1];
                break;
            case ELDRIVER_MC3P_SECTOR_TRAP5:
                adc_syncRank1 = sil.state.vp[1];
                adc_syncRank2 = sil.state.ip[2];
                break;
            case ELDRIVER_MC3P_SECTOR_TRAP6:
                adc_syncRank1 = sil.state.vp[0];
                adc_syncRank2 = sil.state.ip[2];
                break;
            default:
                break;
        }
        ((eldriver_mc3p_trap_data_t *)(data))->vbemf_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(adc_syncRank1);
        ((eldriver_mc3p_trap_data_t *)(data))->cbus_q31  = ELDRIVER_MC3P_FLOAT_TO_CS(adc_syncRank2);
    }else{
        ((eldriver_mc3p_svm_data_t *)(data))->vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(sil_input.vcc);
    }
}

void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w)
{
    sil_input.drive[0] = (state_u != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
    sil_input.drive[1] = (state_v != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
    sil_input.drive[2] = (state_w != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
    h->phase_state[0] = state_u;
    h->phase_state[1] = state_v;
    h->phase_state[2] = state_w;

}

// ===== KEY CHANGE: Send duty cycles to SIL =====
void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, uint16_t duty_u_q15, uint16_t duty_v_q15, uint16_t duty_w_q15)
{
    float duty[3] = {((float)duty_u_q15/INT16_MAX), ((float)duty_v_q15/INT16_MAX), ((float)duty_w_q15/INT16_MAX)};
    for(int i = 0; i < 3; i++)
    {
        if(h->phase_state[i] == ELDRIVER_MC3P_PHASE_L_ON)
        {
            duty[i] = 0;
        }else if (h->phase_state[i] == ELDRIVER_MC3P_PHASE_H_ON)
        {
            duty[i] = 1;
        }
        sil_input.duty[i] = duty[i];
    }
}

void eldriver_mc3p_write_float(eldriver_mc3p_t *h)
{
    eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_FLOAT);
    eldriver_mc3p_write_phase_duty(h, 0, 0, 0);
    h->sector_last = ELDRIVER_MC3P_SECTOR_FLOAT;
}


typedef struct {
    eldriver_mc3p_phase_state_t phase_state[3]; // A,B,C: COMP, L_ON, FLOAT
} mc3p_trap_sector_map_t;

static const mc3p_trap_sector_map_t trap_table[6] = {
    [ELDRIVER_MC3P_SECTOR_TRAP1 - 1] = { {ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_FLOAT}},
    [ELDRIVER_MC3P_SECTOR_TRAP2 - 1] = { {ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_L_ON}},
    [ELDRIVER_MC3P_SECTOR_TRAP3 - 1] = { {ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_L_ON}},
    [ELDRIVER_MC3P_SECTOR_TRAP4 - 1] = { {ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_FLOAT}},
    [ELDRIVER_MC3P_SECTOR_TRAP5 - 1] = { {ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_COMP}},
    [ELDRIVER_MC3P_SECTOR_TRAP6 - 1] = { {ELDRIVER_MC3P_PHASE_FLOAT, ELDRIVER_MC3P_PHASE_L_ON, ELDRIVER_MC3P_PHASE_COMP}},
};

void eldriver_mc3p_write_trap(eldriver_mc3p_t *h, eldriver_mc3p_sector_t sector, uint16_t duty_q15)
{
    if(!IS_TRAP_SECTOR(h->sector_last))
    {
        h->mode        = ELDRIVER_MC3P_MODE_TRAP;
    }
 

    if(sector < ELDRIVER_MC3P_SECTOR_TRAP1 || sector > ELDRIVER_MC3P_SECTOR_TRAP6)
        return;

    const mc3p_trap_sector_map_t *map = &trap_table[sector - 1];

    // Only update phase state & ADC if sector changed
    if(h->sector_last != sector)
    {
        eldriver_mc3p_write_phase_state(h, map->phase_state[0], map->phase_state[1], map->phase_state[2]);
        h->sector_last = sector;
    }
    eldriver_mc3p_write_phase_duty(h, duty_q15, duty_q15, duty_q15);
}

void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15)
{
    int32_t vmax, vmin, voff;
    if(!IS_SVM_SECTOR(h->sector_last))
    {
        eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP);
        h->mode        = ELDRIVER_MC3P_MODE_SVM;
    }

    /* αβ → phase (normalized Q15) */
    h->dutyu_q15 = alpha_q15;

    h->dutyv_q15 = (-(int32_t)Q15_HALF * alpha_q15
         + (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    h->dutyw_q15 = (-(int32_t)Q15_HALF * alpha_q15
         - (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    uint8_t b0 = (h->dutyu_q15 >= 0);
    uint8_t b1 = (h->dutyv_q15 >= 0);
    uint8_t b2 = (h->dutyw_q15 >= 0);

    // 3-bit code
    uint8_t code = (b2 << 2) | (b1 << 1) | b0;
    eldriver_mc3p_sector_t sector;
    switch(code)
    {
        case 0b001: sector = ELDRIVER_MC3P_SECTOR_SVM1; break;
        case 0b011: sector = ELDRIVER_MC3P_SECTOR_SVM2; break;
        case 0b010: sector = ELDRIVER_MC3P_SECTOR_SVM3; break;
        case 0b110: sector = ELDRIVER_MC3P_SECTOR_SVM4; break;
        case 0b100: sector = ELDRIVER_MC3P_SECTOR_SVM5; break;
        case 0b101: sector = ELDRIVER_MC3P_SECTOR_SVM6; break;
        default: sector = ELDRIVER_MC3P_SECTOR_FLOAT; break; // should not happen
    }
    
    /* SVPWM zero-sequence injection */
    vmax = h->dutyu_q15;
    if (h->dutyv_q15 > vmax) vmax = h->dutyv_q15;
    if (h->dutyw_q15 > vmax) vmax = h->dutyw_q15;

    vmin = h->dutyu_q15;
    if (h->dutyv_q15 < vmin) vmin = h->dutyv_q15;
    if (h->dutyw_q15 < vmin) vmin = h->dutyw_q15;

    voff = (vmax + vmin) >> 1;

    h->dutyu_q15 = (h->dutyu_q15 - voff) + Q15_HALF;
    h->dutyv_q15 = (h->dutyv_q15 - voff) + Q15_HALF;
    h->dutyw_q15 = (h->dutyw_q15 - voff) + Q15_HALF;
    eldriver_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
    h->sector_last = sector;
}

void eldriver_mc3p_setGain(eldriver_mc3p_t *h, eldriver_mc3p_sync s, float gain){}
