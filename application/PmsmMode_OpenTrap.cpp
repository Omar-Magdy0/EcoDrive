/**
 * @file PmsmMode_OpenTrap.cpp
 * @brief Implementation of the Open-Loop Trapezoidal control mode.
 * @details Provides a framework for driving the motor using 6-step trapezoidal commutation
 *          without relying on BEMF or Hall sensor feedback. Currently implemented as a skeleton.
 */
#include "PmsmControl.h"

/**
 * @brief Empty stub for the Reset stage in Open-Loop Trapezoidal mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ResetHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Alignment stage in Open-Loop Trapezoidal mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void AlignHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Ramp stage in Open-Loop Trapezoidal mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void RampHandler(PmsmControl *cp) {}

/**
 * @brief Empty stub for the Closed stage in Open-Loop Trapezoidal mode.
 * @param cp Pointer to the PMSM control structure.
 */
static inline void ClosedHandler(PmsmControl *cp) {}

/**
 * @brief Executes the PWM loop specifically for Open-Loop Trapezoidal control.
 * @details Evaluates the startup state machine and routes execution to the corresponding stage handler.
 *          Currently, the handlers are empty stubs, acting as a placeholder for future implementation.
 */
void PmsmControl::OpenTrap_pwmLoop()
{
    /// Route execution based on the current startup stage
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
