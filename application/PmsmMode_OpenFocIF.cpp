#include "PmsmControl.h"

static inline void ResetHandler(PmsmControl *cp) {}
static inline void AlignHandler(PmsmControl *cp) {}
static inline void RampHandler(PmsmControl *cp) {}
static inline void ClosedHandler(PmsmControl *cp) {}

void PmsmControl::OpenFocIF_pwmLoop()
{
    switch (stup.stage_current)
    {
    case PmsmControl::StupStage::Reset:
        ResetHandler(this);
        break;
    case PmsmControl::StupStage::Align:
        AlignHandler(this);
        break;
    case PmsmControl::StupStage::Ramp:
        RampHandler(this);
        break;
    case PmsmControl::StupStage::Closed:
        ClosedHandler(this);
        break;
    default:
        break;
    }
    float vd = 0;
    float vq = 0;
    float sin_val, cos_val, valpha, vbeta;
    arm_sin_cos_f32(elec.theta, &sin_val, &cos_val);
    arm_inv_park_f32(vd, vq, &valpha, &vbeta, sin_val, cos_val);
    int16_t valpha_q15 = (valpha / elec.vbus) * (1 << 15);
    int16_t vbeta_q15 = (vbeta / elec.vbus) * (1 << 15);
    eldriver_mc3p_write_svm(&mc3p, valpha_q15, vbeta_q15);
}