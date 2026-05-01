#include "PmsmControl.h"


void PmsmControl::Idle_pwmLoop()
{
    //Do nothing just float all phases
    eldriver_mc3p_write_float(&mc3p);
}