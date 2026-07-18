#include "el_mc3p.h"
#include "platform.h"
#include "eld_conf.h"
#include "sil.h"
#include "silgui.h"
#include "el_hall.h"

#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))
#define Q15_HALF        16384   // 0.5 × 32768
#define Q15_SQRT3_BY_2  28378   // 0.8660254 × 32768

extern float vtime;
unsigned int sil_pwm_timer;
unsigned int sil_to_pwm_freq = 1;

float el_mc3p_adc_read_single(el_mc3p_handle_t *h, uint32_t channel) { return 0.0; }
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
    el_mc3p_sync_postScanCallback();
    sil_input.load_torque = (dummy_load.K * sil.state.omega + dummy_load.B) * sil.state.omega + dummy_load.Tc;
    for(int i = 0; i < sil_to_pwm_freq; i++) {
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

void el_mc3p_init(el_mc3p_handle_t *h)
{
    sil_input.dt = 1.0f / h->config.pwm_Hz / sil_to_pwm_freq;
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
    h->duty_max_q15 = (uint16_t)((h->config.duty_max) * 0x7FFF);
    h->duty_min_q15 = (uint16_t)((h->config.duty_min) * 0x7FFF);
    //compute deadtme compensation
    #ifdef EL_MC3P_DTC_ACTIVE
    h->dtc_comp_q15 = (int16_t)(((float)h->config.deadtime_nS*h->config.pwm_Hz/1e9)*INT16_MAX + 0.5);
    #endif
    sil_pwm_timer = register_timer(&timer_manager, postScanMethod, (uint64_t)(1e9/h->config.pwm_Hz));
    register_timer(&timer_manager, el_xmc3p_tickerCallback, (uint64_t)(1e9/EL_XMC3P_TICKFREQ));
}

void el_mc3p_reconfigure_pwm(el_mc3p_handle_t *h)
{
    h->duty_max_q15 = (uint16_t)((h->config.duty_max) * 0x7FFF);
    h->duty_min_q15 = (uint16_t)((h->config.duty_min) * 0x7FFF);
    double min_sil_freq = 1.0/SIL_MAX_TIMESTEP;
    if(h->config.pwm_Hz < min_sil_freq)
    {
        //Find the nearest integer multiple of pwm frequency larger than minimum sil frequency
        double next_freq = 0;
        for(int i = 1; i <= 1000; i++)
        {
            if(i * h->config.pwm_Hz > min_sil_freq)
            {
                sil_to_pwm_freq = i;
            }
        }
    }
    sil_input.dt = 1.0f / h->config.pwm_Hz / sil_to_pwm_freq;
    sil.param.inv_pwm_freq = h->config.pwm_Hz;
    sil.param.inv_deatime_ns = h->config.deadtime_nS;
    configure_timer_timestep(&timer_manager, sil_pwm_timer, (uint64_t)(1e9/h->config.pwm_Hz));
}

void el_mc3p_bg_startConv(el_mc3p_handle_t *h){}
uint8_t el_mc3p_bg_channels(el_mc3p_handle_t *h){return 0;}
uint8_t el_mc3p_read_bg(el_mc3p_handle_t *h, float* scanData){return 0;}
uint8_t el_mc3p_bg_isReady(el_mc3p_handle_t *h){return 0;}

void el_mc3p_read_sync(el_mc3p_handle_t *h, void* data)
{    
    uint8_t is_svm = IS_SVM_SECTOR(h->sector_last);
    uint8_t is_trap = IS_TRAP_SECTOR(h->sector_last);
#define DTC_UPDATE_POLARITY(current, bit)         \
do                                            \
{                                             \
    if ((current) > (int32_t)EL_MC3P_DTC_CTHRESH)       \
    {                                         \
        h->dtc_state |= (bit);             \
    }                                         \
    else if ((current) < -(int32_t)EL_MC3P_DTC_CTHRESH) \
    {                                         \
        h->dtc_state &= ~(bit);            \
    }                                         \
} while (0)
    if(is_svm)
    {       
        // Convert float currents to Q31 format for the control algorithm
        // Scaling: 1.0A = X counts (adjust EL_MC3P_CS_SCALE to tune sensitivity)
        ((el_mc3p_svm_data_t *)(data))->vbus_q31 = EL_MC3P_FLOAT_TO_VS(sil_input.vcc);
        ((el_mc3p_svm_data_t *)(data))->cu_q31 = EL_MC3P_FLOAT_TO_CS(sil.state.ip[0]);
        ((el_mc3p_svm_data_t *)(data))->cv_q31 = EL_MC3P_FLOAT_TO_CS(sil.state.ip[1]);
        ((el_mc3p_svm_data_t *)(data))->cw_q31 = EL_MC3P_FLOAT_TO_CS(sil.state.ip[2]);
        #ifdef EL_MC3P_DTC_ACTIVE
        DTC_UPDATE_POLARITY(((el_mc3p_svm_data_t *)(data))->cu_q31, (1 << 0));
        DTC_UPDATE_POLARITY(((el_mc3p_svm_data_t *)(data))->cv_q31, (1 << 1));
        DTC_UPDATE_POLARITY(((el_mc3p_svm_data_t *)(data))->cw_q31, (1 << 2));
        #endif
    }
    else if (is_trap)
    {
        // Back-EMF would be: Ke * ωe where ωe = pp * ω_mech
        ((el_mc3p_svm_data_t *)(data))->vbus_q31 = EL_MC3P_FLOAT_TO_VS(sil_input.vcc);
        float adc_syncRank1, adc_syncRank2;
        switch(h->sector_last){
            case EL_MC3P_SECTOR_TRAP1:
                adc_syncRank1 = sil.state.vp[2] - sil.state.vn;
                adc_syncRank2 = sil.state.ip[0];
                break;
            case EL_MC3P_SECTOR_TRAP2:
                adc_syncRank1 = sil.state.vp[1];
                adc_syncRank2 = sil.state.ip[0];
                break;
            case EL_MC3P_SECTOR_TRAP3:
                adc_syncRank1 = sil.state.vp[0];
                adc_syncRank2 = sil.state.ip[1];
                break;
            case EL_MC3P_SECTOR_TRAP4:
                adc_syncRank1 = sil.state.vp[2];
                adc_syncRank2 = sil.state.ip[1];
                break;
            case EL_MC3P_SECTOR_TRAP5:
                adc_syncRank1 = sil.state.vp[1];
                adc_syncRank2 = sil.state.ip[2];
                break;
            case EL_MC3P_SECTOR_TRAP6:
                adc_syncRank1 = sil.state.vp[0];
                adc_syncRank2 = sil.state.ip[2];
                break;
            default:
                break;
        }
        ((el_mc3p_trap_data_t *)(data))->vbemf_q31 = EL_MC3P_FLOAT_TO_VS(adc_syncRank1);
        ((el_mc3p_trap_data_t *)(data))->cbus_q31  = EL_MC3P_FLOAT_TO_CS(adc_syncRank2);
        #ifdef EL_MC3P_DTC_ACTIVE
        DTC_UPDATE_POLARITY(((el_mc3p_trap_data_t *)(data))->cbus_q31 , (1 << 3));
        #endif
    }else{
        ((el_mc3p_svm_data_t *)(data))->vbus_q31 = EL_MC3P_FLOAT_TO_VS(sil_input.vcc);
    }
}

void el_mc3p_write_phase_state(el_mc3p_handle_t *h, el_mc3p_phase_state_t state_u, el_mc3p_phase_state_t state_v, el_mc3p_phase_state_t state_w)
{
    sil_input.drive[0] = (state_u != EL_MC3P_PHASE_FLOAT)? 1 : 0;
    sil_input.drive[1] = (state_v != EL_MC3P_PHASE_FLOAT)? 1 : 0;
    sil_input.drive[2] = (state_w != EL_MC3P_PHASE_FLOAT)? 1 : 0;
    h->phase_state[0] = state_u;
    h->phase_state[1] = state_v;
    h->phase_state[2] = state_w;
}

// ===== KEY CHANGE: Send duty cycles to SIL =====
void el_mc3p_write_phase_duty(el_mc3p_handle_t *h, int16_t duty_u_q15, int16_t duty_v_q15, int16_t duty_w_q15)
{
    duty_u_q15 = (uint32_t)SATURATE(duty_u_q15, h->duty_min_q15, h->duty_max_q15);
    duty_v_q15 = (uint32_t)SATURATE(duty_v_q15, h->duty_min_q15, h->duty_max_q15);
    duty_w_q15 = (uint32_t)SATURATE(duty_w_q15, h->duty_min_q15, h->duty_max_q15);
    float duty[3] = {((float)duty_u_q15/INT16_MAX), ((float)duty_v_q15/INT16_MAX), ((float)duty_w_q15/INT16_MAX)};
    for(int i = 0; i < 3; i++)
    {
        if(h->phase_state[i] == EL_MC3P_PHASE_L_ON)
        {
            duty[i] = 0;
        }else if (h->phase_state[i] == EL_MC3P_PHASE_H_ON)
        {
            duty[i] = 1;
        }
        sil_input.duty[i] = duty[i];
    }
}

void el_mc3p_write_float(el_mc3p_handle_t *h)
{
    el_mc3p_write_phase_state(h, EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_FLOAT);
    el_mc3p_write_phase_duty(h, 0, 0, 0);
    h->sector_last = EL_MC3P_SECTOR_FLOAT;
}


typedef struct {
    el_mc3p_phase_state_t phase_state[3]; // A,B,C: COMP, L_ON, FLOAT
} mc3p_trap_sector_map_t;

static const mc3p_trap_sector_map_t trap_table[6] = {
    [EL_MC3P_SECTOR_TRAP1 - 1] = { {EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_L_ON, EL_MC3P_PHASE_FLOAT}},
    [EL_MC3P_SECTOR_TRAP2 - 1] = { {EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_L_ON}},
    [EL_MC3P_SECTOR_TRAP3 - 1] = { {EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_L_ON}},
    [EL_MC3P_SECTOR_TRAP4 - 1] = { {EL_MC3P_PHASE_L_ON, EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_FLOAT}},
    [EL_MC3P_SECTOR_TRAP5 - 1] = { {EL_MC3P_PHASE_L_ON, EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_COMP}},
    [EL_MC3P_SECTOR_TRAP6 - 1] = { {EL_MC3P_PHASE_FLOAT, EL_MC3P_PHASE_L_ON, EL_MC3P_PHASE_COMP}},
};

void el_mc3p_write_trap(el_mc3p_handle_t *h, el_mc3p_sector_t sector, uint16_t duty_q15)
{
    if(!IS_TRAP_SECTOR(h->sector_last))
    {
        h->mode        = EL_MC3P_MODE_TRAP;
    }
    if(sector < EL_MC3P_SECTOR_TRAP1 || sector > EL_MC3P_SECTOR_TRAP6)
        return;

    const mc3p_trap_sector_map_t *map = &trap_table[sector - 1];

    // Only update phase state & ADC if sector changed
    if(h->sector_last != sector)
    {
        el_mc3p_write_phase_state(h, map->phase_state[0], map->phase_state[1], map->phase_state[2]);
        h->sector_last = sector;
    }   
    #ifdef EL_MC3P_DTC_ACTIVE
    h->dutyu_q15 = SSAT((int32_t)duty_q15 + ((h->dtc_state & (1<<3))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyv_q15 = h->dutyu_q15;
    h->dutyw_q15 = h->dutyu_q15;
    #else
    h->dutyu_q15 = duty_q15;
    h->dutyv_q15 = h->dutyu_q15;
    h->dutyw_q15 = h->dutyu_q15;
    #endif 
    el_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
}

void el_mc3p_write_svm(el_mc3p_handle_t *h, int16_t alpha_q15, int16_t beta_q15)
{
    int32_t vmax, vmin, voff;
    if(!IS_SVM_SECTOR(h->sector_last))
    {
        el_mc3p_write_phase_state(h, EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_COMP, EL_MC3P_PHASE_COMP);
        h->mode        = EL_MC3P_MODE_SVM;
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
    el_mc3p_sector_t sector;
    switch(code)
    {
        case 0b001: sector = EL_MC3P_SECTOR_SVM1; break;
        case 0b011: sector = EL_MC3P_SECTOR_SVM2; break;
        case 0b010: sector = EL_MC3P_SECTOR_SVM3; break;
        case 0b110: sector = EL_MC3P_SECTOR_SVM4; break;
        case 0b100: sector = EL_MC3P_SECTOR_SVM5; break;
        case 0b101: sector = EL_MC3P_SECTOR_SVM6; break;
        default: sector = EL_MC3P_SECTOR_FLOAT; break; // should not happen
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
    #ifdef EL_MC3P_DTC_ACTIVE
    h->dutyu_q15 = SSAT(dutyu_q15 + ((h->dtc_state & (1<<0))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyv_q15 = SSAT(dutyv_q15 + ((h->dtc_state & (1<<1))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    h->dutyw_q15 = SSAT(dutyw_q15 + ((h->dtc_state & (1<<2))? h->dtc_comp_q15 : -h->dtc_comp_q15), 16);
    #else
    h->dutyu_q15 = dutyu_q15;
    h->dutyv_q15 = dutyv_q15;
    h->dutyw_q15 = dutyw_q15;
    #endif
    el_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
    h->sector_last = sector;
}

void el_mc3p_set_gain(el_mc3p_handle_t *h,  el_mc3p_sync s, float gain)
{
    float scale = (s >= EL_MC3P_CSU && s <= EL_MC3P_CSW)? EL_MC3P_CS_SCALE : EL_MC3P_VS_SCALE;
    h->sync_scale_q31[s][0] = ((gain * h->adc_to_uV * (INT32_MAX) + 0.5)/(1000000.0 * scale));
}
void el_mc3p_set_sync_scale(el_mc3p_handle_t *h, const float scales[MC3P_SYNC_CHANNELS][2])
{
    for(uint8_t i = EL_MC3P_VSBUS; i < MC3P_SYNC_CHANNELS; i++)
    {
        //Gains
        el_mc3p_set_gain(h, (el_mc3p_sync)i, scales[i][0]);
        //Offsets
        if(i >= EL_MC3P_VSBUS || i <= EL_MC3P_VSW)
        {
            h->sync_scale_q31[i][1] = EL_MC3P_FLOAT_TO_VS(scales[i][1]);
        }
        else if(i >= EL_MC3P_CSV || i <= EL_MC3P_CSW)
        {
            h->sync_scale_q31[i][1] = EL_MC3P_FLOAT_TO_CS(scales[i][1]);
        }
    }
}