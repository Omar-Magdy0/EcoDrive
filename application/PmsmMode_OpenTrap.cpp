#include "PmsmControl.h"

static inline void ResetHandler(PmsmControl *cp) {}
static inline void AlignHandler(PmsmControl *cp) {}
static inline void RampHandler(PmsmControl *cp) {}
static inline void ClosedHandler(PmsmControl *cp) {}

void PmsmControl::OpenTrap_pwmLoop()
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
}
