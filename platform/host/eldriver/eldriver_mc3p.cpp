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
#define SSAT(val, bits) \
    ({ \
        int32_t _v = (int32_t)(val); \
        int32_t _max = (1 << ((bits) - 1)) - 1; \
        int32_t _min = -(1 << ((bits) - 1)); \
        if (_v > _max) _v = _max; \
        if (_v < _min) _v = _min; \
        (int32_t)_v; \
    })


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

static void dPsim_dTheta_Sine(double theta_e, double dPsi_dTheta[3])
{
    double amplitude = sil.param.motor_fluxLinkage; // Back EMF amplitude
    dPsi_dTheta[0] = - amplitude * sin(theta_e);
    dPsi_dTheta[1] = - amplitude * sin(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * sin(theta_e + 2*M_PI/3);
};
static void dPsim_dTheta_Trapezoidal(double theta_e, double dPsi_dTheta[3])
{
    double amplitude = sil.param.motor_fluxLinkage; // Back EMF amplitude
    dPsi_dTheta[0] = - amplitude * Sil::trapezoidal_wave(theta_e);
    dPsi_dTheta[1] = - amplitude * Sil::trapezoidal_wave(theta_e - 2*M_PI/3);
    dPsi_dTheta[2] = - amplitude * Sil::trapezoidal_wave(theta_e + 2*M_PI/3);
}

void eldriver_mc3p_init(eldriver_mc3p_t *h)
{
    sil_input.dt = 1.0f / h->config.pwm_Hz / SIL_TO_PWM_FREQ;
    sil_input.load_torque = 0.00;
    sil_input.vcc = SIL_DEFAULT_VCC;
    sil.param.dPsim_dTheta = dPsim_dTheta_Sine;
    sil.param.inv_Ron = 0.008;
    sil.param.inv_Roff = 1e6;
    sil.param.motor_pp = 6;
    sil.param.motor_Rs = 0.25;
    sil.param.motor_Ls = 0.0008;
    sil.param.motor_Lm = 0.0002;
    sil.param.motor_Ms = sil.param.motor_Ls/3;
    sil.param.motor_rotorOffset = 0;
    sil.param.motor_fluxLinkage = 0.025;
    sil.param.motor_B = 1e-2;
    sil.param.motor_J = 8e-5;
    sil.param.inv_deatime_ns = h->config.deadtime_nS;
    sil.param.inv_pwm_freq = h->config.pwm_Hz;
    dummy_load.K = 1.2e-5;
    dummy_load.J = 1e-3;
    dummy_load.B = 5e-2;
    dummy_load.Tc = 0;
    sil.param.load_J = dummy_load.J;
    //compute deadtme compensation
    #ifdef ELDRIVER_MC3P_DTC_ACTIVE
    h->dtc_comp_q15 = (int16_t)(((float)h->config.deadtime_nS*h->config.pwm_Hz/1e9)*INT16_MAX + 0.5);
    #endif
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
#define DTC_UPDATE_POLARITY(current, bit)         \
do                                            \
{                                             \
    if ((current) > (int32_t)ELDRIVER_MC3P_DTC_CTHRESH)       \
    {                                         \
        h->dtc_state |= (bit);             \
    }                                         \
    else if ((current) < -(int32_t)ELDRIVER_MC3P_DTC_CTHRESH) \
    {                                         \
        h->dtc_state &= ~(bit);            \
    }                                         \
} while (0)
    if(is_svm)
    {       
        // Convert float currents to Q31 format for the control algorithm
        // Scaling: 1.0A = X counts (adjust ELDRIVER_MC3P_CS_SCALE to tune sensitivity)
        ((eldriver_mc3p_svm_data_t *)(data))->vbus_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(sil_input.vcc);
        ((eldriver_mc3p_svm_data_t *)(data))->cu_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[0]);
        ((eldriver_mc3p_svm_data_t *)(data))->cv_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[1]);
        ((eldriver_mc3p_svm_data_t *)(data))->cw_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(sil.state.ip[2]);
        #ifdef ELDRIVER_MC3P_DTC_ACTIVE
        DTC_UPDATE_POLARITY(((eldriver_mc3p_svm_data_t *)(data))->cu_q31, (1 << 0));
        DTC_UPDATE_POLARITY(((eldriver_mc3p_svm_data_t *)(data))->cv_q31, (1 << 1));
        DTC_UPDATE_POLARITY(((eldriver_mc3p_svm_data_t *)(data))->cw_q31, (1 << 2));
        #endif
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
        #ifdef ELDRIVER_MC3P_DTC_ACTIVE
        DTC_UPDATE_POLARITY(((eldriver_mc3p_trap_data_t *)(data))->cbus_q31 , (1 << 3));
        #endif
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
void eldriver_mc3p_write_phase_duty(eldriver_mc3p_t *h, int16_t duty_u_q15, int16_t duty_v_q15, int16_t duty_w_q15)
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
    #ifdef ELDRIVER_MC3P_DTC_ACTIVE
    h->dutyu_q15 = SSAT((int32_t)duty_q15 + ((h->dtc_state & (1<<3))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyv_q15 = h->dutyu_q15;
    h->dutyw_q15 = h->dutyu_q15;
    #else
    h->dutyu_q15 = duty_q15;
    h->dutyv_q15 = h->dutyu_q15;
    h->dutyw_q15 = h->dutyu_q15;
    #endif 
    eldriver_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
}

void eldriver_mc3p_write_svm(eldriver_mc3p_t *h, int16_t alpha_q15, int16_t beta_q15)
{
    int32_t vmax, vmin, voff;
    if(!IS_SVM_SECTOR(h->sector_last))
    {
        eldriver_mc3p_write_phase_state(h, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP, ELDRIVER_MC3P_PHASE_COMP);
        h->mode        = ELDRIVER_MC3P_MODE_SVM;
    }
    int32_t dutyu_q15, dutyv_q15, dutyw_q15;
    /* αβ → phase (normalized Q15) */
    dutyu_q15 = alpha_q15;

    dutyv_q15 = (-(int32_t)Q15_HALF * alpha_q15
         + (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    dutyw_q15 = (-(int32_t)Q15_HALF * alpha_q15
         - (int32_t)Q15_SQRT3_BY_2 * beta_q15) >> 15;

    uint8_t b0 = (dutyu_q15 >= 0);
    uint8_t b1 = (dutyv_q15 >= 0);
    uint8_t b2 = (dutyw_q15 >= 0);

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
    vmax = dutyu_q15;
    if (dutyv_q15 > vmax) vmax = dutyv_q15;
    if (dutyw_q15 > vmax) vmax = dutyw_q15;

    vmin = dutyu_q15;
    if (dutyv_q15 < vmin) vmin = dutyv_q15;
    if (dutyw_q15 < vmin) vmin = dutyw_q15;

    voff = (vmax + vmin) >> 1;

    dutyu_q15 = (dutyu_q15 - voff) + Q15_HALF;
    dutyv_q15 = (dutyv_q15 - voff) + Q15_HALF;
    dutyw_q15 = (dutyw_q15 - voff) + Q15_HALF;
    #ifdef ELDRIVER_MC3P_DTC_ACTIVE
    h->dutyu_q15 = SSAT(dutyu_q15 + ((h->dtc_state & (1<<0))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyv_q15 = SSAT(dutyv_q15 + ((h->dtc_state & (1<<1))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyw_q15 = SSAT(dutyw_q15 + ((h->dtc_state & (1<<2))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    #else
    h->dutyu_q15 = dutyu_q15;
    h->dutyv_q15 = dutyv_q15;
    h->dutyw_q15 = dutyw_q15;
    #endif
    eldriver_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
    h->sector_last = sector;
}

void eldriver_mc3p_setGain(eldriver_mc3p_t *h, eldriver_mc3p_sync s, float gain){}
