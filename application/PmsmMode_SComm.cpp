#include "PmsmController.h"
#include <arm_math.h>

constexpr float SCOMM_ALIGN_VOLTAGE = 2;
constexpr float SCOMM_ALIGN_DURATION_MS = 100; 
constexpr float SCOMM_HFI_FREQ = 1000;
constexpr float SCOMM_HFI_VOLTAGE = 2;
constexpr uint8_t SCOMM_ID_OVERSAMPLE = 20;
constexpr float SCOMM_HFI_SETTLE_TIME_MS = 50; 
constexpr float SCOMM_HFI_DEMOD_ALPHA = 0.005;
constexpr uint16_t SCOMM_HFI_IIR_SETTLE = (4.6/SCOMM_HFI_DEMOD_ALPHA);

void PmsmControl::SelfCommission_init()
{
    SComm.stage = SCommStage::RESET;
    SComm.stage_last = SCommStage::RESET;
    
}

void PmsmControl::SelfCommission_pwmLoop()
{
    switch (SComm.stage)
    {
        case SCommStage::RESET :
        {
            SComm.stage = SCommStage::DAXIS_ALIGN;    
        }
        case SCommStage::DAXIS_ALIGN:
        {
            //Entry Point
            if(SComm.stage != SComm.stage_last){
                elcore_swttimer_reset(&SComm.timer, pwmTicks);
                SComm.stage_last = SCommStage::DAXIS_ALIGN;
                elec.theta = 0;
                float sin,cos,valpha,vbeta;
                float vd = SCOMM_ALIGN_VOLTAGE;
                float vq = 0;
                arm_sin_cos_f32(elec.theta, &sin, &cos);
                arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin, cos);
                int16_t valpha_q15 = valpha/elec.vbus * (1<<15);
                eldriver_mc3p_write_svm(&mc3p, valpha_q15, (int16_t)0);
            }


            //Exit point
            if(elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_ALIGN_DURATION_MS)))
            {
                SComm.stage = SComm.stage = SCommStage::RS_ID;
            }
        }
            break;
        case SCommStage::RS_ID :
        {//Entry Point
            if(SComm.stage != SComm.stage_last){
                elcore_swttimer_reset(&SComm.timer, pwmTicks);
                SComm.stage_last = SCommStage::RS_ID;
                SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
            }
            //After alignment current should have settled by now , measure DC current and DC resistance
            // valpha = vd for theta equal zero,  Id = Ia for theta equal zero
            float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
            float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31); 
            float Id, Iq, Ialpha, Ibeta, sin, cos;
            arm_sin_cos_f32(elec.theta, &sin, &cos);
            arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
            arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin, cos);
            model.Rs += (SCOMM_ALIGN_VOLTAGE)/(Id) ; // Vd/Id
            SComm.remaining_id_samples--;
            
            //Exist Point
            if(SComm.remaining_id_samples == 0)
            {
                model.Rs = model.Rs/SCOMM_ID_OVERSAMPLE;
                SComm.stage = SCommStage::LD_ID;
            }
        }
            break;
        case SCommStage::LD_ID :
        {//Entry Point
            if(SComm.stage != SComm.stage_last){
                elcore_swttimer_reset(&SComm.timer, pwmTicks);
                SComm.stage_last = SCommStage::LD_ID;
                SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
                SComm.id_cos_prod_2 = 0;
                SComm.id_sin_prod_2 = 0;
                SComm.hfi_angle = 0;
                SComm.iir_filtered_cnt = 0;
            }
            // RUN HFI on the D-axis
            elec.theta = 0;
            float sin_val, cos_val,valpha,vbeta;
            float hfi_sin, hfi_cos;
            arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
            float vd = SCOMM_ALIGN_VOLTAGE + SCOMM_HFI_VOLTAGE * hfi_sin;
            float vq = 0;
            arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
            arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
            int16_t valpha_q15 = valpha/elec.vbus * (1<<15);
            eldriver_mc3p_write_svm(&mc3p, valpha_q15, (int16_t)0);
            SComm.hfi_angle += 2*PI*(ticks_to_ms(1)/1000)*SCOMM_HFI_FREQ;
 
            if(elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
            {//HF signal steady state , lets do some DSP
                // Get fresh currents and transform to dq
                float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
                float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
                float Ialpha, Ibeta, Id, Iq;
                arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
                arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
                //Lets do some good old heterodyning
                float cos_prod = Id * hfi_cos;
                float sin_prod = Id * hfi_sin;
                // LPF IIR filter
                SComm.id_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.id_cos_prod_2);
                SComm.id_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.id_sin_prod_2);
                SComm.iir_filtered_cnt++;
            }
            
            // Exit Point
            if(SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
            {
                SComm.stage = SCommStage::LQ_ID;
            }
        }
            break;
        case SCommStage::LQ_ID :
        {//Entry Point
            if(SComm.stage != SComm.stage_last){
                elcore_swttimer_reset(&SComm.timer, pwmTicks);
                SComm.stage_last = SCommStage::LQ_ID;
                SComm.remaining_id_samples = SCOMM_ID_OVERSAMPLE;
                SComm.iq_cos_prod_2 = 0;
                SComm.iq_sin_prod_2 = 0;
                SComm.hfi_angle = 0;
                SComm.iir_filtered_cnt = 0;
            }
            // RUN HFI on the D-axis
            elec.theta = 0;
            float sin_val, cos_val,valpha,vbeta;
            float hfi_sin, hfi_cos;
            arm_sin_cos_f32(SComm.hfi_angle, &hfi_sin, &hfi_cos);
            float vd = SCOMM_ALIGN_VOLTAGE;
            float vq = SCOMM_HFI_VOLTAGE * hfi_sin;
            arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
            arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
            int16_t valpha_q15 = valpha/elec.vbus * (1<<15);
            eldriver_mc3p_write_svm(&mc3p, valpha_q15, (int16_t)0);
            SComm.hfi_angle += 2*PI*(ticks_to_ms(1)/1000)*SCOMM_HFI_FREQ;
 
            if(elcore_swttimer_timout(&SComm.timer, pwmTicks, ms_to_ticks(SCOMM_HFI_SETTLE_TIME_MS)))
            {//HF signal steady state , lets do some DSP
                // Get fresh currents and transform to dq
                float Ia = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cu_q31);
                float Ib = ELDRIVER_MC3P_CS_TO_FLOAT(mc3p_sync_meas.svm.cv_q31);
                float Ialpha, Ibeta, Id, Iq;
                arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
                arm_park_f32(Ialpha, Ibeta, &Id, &Iq, sin_val, cos_val);
                //Lets do some good old heterodyning
                float cos_prod = Iq * hfi_cos;
                float sin_prod = Iq * hfi_sin;
                // LPF IIR filter
                SComm.iq_cos_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (cos_prod - SComm.iq_cos_prod_2);
                SComm.iq_sin_prod_2 += SCOMM_HFI_DEMOD_ALPHA * (sin_prod - SComm.iq_sin_prod_2);
                SComm.iir_filtered_cnt++;
            }
            
            // Exit Point
            if(SComm.iir_filtered_cnt >= SCOMM_HFI_IIR_SETTLE)
            {
                SComm.stage = SCommStage::KE_ID;
            }
        }
            break;
        case SCommStage::KE_ID :
        {
            OpenTrap_pwmLoop();
            
        }
            break;
        default:
            break;
    }
}

void PmsmControl::SelfCommission_xmcLoop()
{

}