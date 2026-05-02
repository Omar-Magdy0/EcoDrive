#include "eldriver_mc3p.h"
#include "platform.h"
#include "virtual_pmsm.h"
#include "eldriver_sil.h"              // ADD THIS
#include "eldriver_conf.h"

#define SATURATE(v , min , max)(( v > max)?max: ((v < min)?0:v))
#define Q15_HALF        16384   // 0.5 × 32768
#define Q15_SQRT3_BY_2  28378   // 0.8660254 × 32768

extern float vtime;

float eldriver_mc3p_adc_read_single(eldriver_mc3p_t *h, uint32_t channel){}

void eldriver_mc3p_init(eldriver_mc3p_t *h)
{
    eldriver_sil_init(1.0/h->config.pwm_Hz);        // STEP 1: Initialize SIL layer
    register_timer(&timer_manager, eldriver_mc3p_sync_postScanCallback, (uint64_t)(1e9/h->config.pwm_Hz));
}


void eldriver_mc3p_bg_startConv(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_bg_channels(eldriver_mc3p_t *h){}
uint8_t eldriver_mc3p_read_bg(eldriver_mc3p_t *h, float* scanData){}
uint8_t eldriver_mc3p_bg_isReady(eldriver_mc3p_t *h){}

// ===== KEY CHANGE: Read from SIL instead of zeros =====
void eldriver_mc3p_read_sync(eldriver_mc3p_t *h, void* data)
{
    // STEP 2: Step the SIL motor/inverter simulation
    eldriver_sil_step();
    
    // STEP 3: Write simulation data to oscilloscope GUI
    eldriver_sil_write_scope();
    
    uint8_t is_svm = IS_SVM_SECTOR(h->sector_last);
    uint8_t is_trap = IS_TRAP_SECTOR(h->sector_last);
    
    if(is_svm)
    {
        // Get simulated currents from SIL
        float iu, iv, iw;
        eldriver_sil_get_currents(&iu, &iv, &iw);
        
        // Convert float currents to Q31 format for the control algorithm
        // Scaling: 1.0A = X counts (adjust ELDRIVER_MC3P_CS_SCALE to tune sensitivity)
        ((eldriver_mc3p_svm_data_t *)(data))->cu_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(iu);
        ((eldriver_mc3p_svm_data_t *)(data))->cv_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(iv);
        ((eldriver_mc3p_svm_data_t *)(data))->cw_q31 = ELDRIVER_MC3P_FLOAT_TO_CS(iw);
    }
    else if (is_trap)
    {
        // For trapezoidal commutation, we could return back-EMF, bus current, etc.
        // For now, just return the phase currents
        float iu, iv, iw;
        eldriver_sil_get_currents(&iu, &iv, &iw);
        
        // Back-EMF would be: Ke * ωe where ωe = pp * ω_mech
        // For now, just use a simple representation
        float theta, omega;
        eldriver_sil_get_rotor_state(&theta, &omega);
        float bemf_est = 0.03f * 7.0f * omega;  // Ke=0.03, pp=7
        
        ((eldriver_mc3p_trap_data_t *)(data))->vbemf_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(bemf_est);
        ((eldriver_mc3p_trap_data_t *)(data))->cbus_q31  = ELDRIVER_MC3P_FLOAT_TO_CS((iu + iv + iw) / 3.0f);
    }
}

void eldriver_mc3p_write_phase_state(eldriver_mc3p_t *h, eldriver_mc3p_phase_state_t state_u, eldriver_mc3p_phase_state_t state_v, eldriver_mc3p_phase_state_t state_w)
{
    h->switch_state[0] = (state_u != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
    h->switch_state[1] = (state_v != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
    h->switch_state[2] = (state_w != ELDRIVER_MC3P_PHASE_FLOAT)? 1 : 0;
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
    }
    
    // STEP 4: Send duty cycles to SIL inverter model
    eldriver_sil_set_inverter_duty(duty[0], duty[1], duty[2], 24.0f);  // 24V bus nominal
    
    float dt = 1.0/h->config.pwm_Hz;
    vtime += dt;
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
        case 0b001: sector = 1; break;
        case 0b011: sector = 2; break;
        case 0b010: sector = 3; break;
        case 0b110: sector = 4; break;
        case 0b100: sector = 5; break;
        case 0b101: sector = 6; break;
        default: sector = 0; break; // should not happen
    }
    
    /* SVPWM zero-sequence injection */
    vmax = h->dutyu_q15;
    if (h->dutyv_q15 > vmax) vmax = h->dutyv_q15;
    if (h->dutyw_q15 > vmax) vmax = h->dutyw_q15;

    vmin = h->dutyu_q15;
    if (h->dutyv_q15 < vmin) vmin = h->dutyv_q15;
    if (h->dutyw_q15 < vmin) vmin = h->dutyw_q15;

    voff = (vmax + vmin) >> 1;

    h->dutyu_q15 -= voff;
    h->dutyv_q15 -= voff;
    h->dutyw_q15 -= voff;
    eldriver_mc3p_write_phase_duty(h, h->dutyu_q15, h->dutyv_q15, h->dutyw_q15);
    h->sector_last = sector;
}

void eldriver_mc3p_setGain(eldriver_mc3p_t *h, eldriver_mc3p_sync s, float gain){}
