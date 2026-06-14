#include "PmsmControl.h"


//======================================================
//  GLOBAL INSTANCE
//======================================================
inline PmsmControl motor_c1; /** Global controller instance for motor channel 1. */
void eldriver_mc3p_sync_postScanCallback() {motor_c1.pwmLoop();}
void eldriver_xmc3p_tickerCallback() { motor_c1.xmcLoop();}