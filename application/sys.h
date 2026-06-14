#pragma once

#include "PmsmControl.h"
#include "platform.h"
#include "aebfStream.h"
#include <cstdint>
#include <array>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//=======================================================


#include "eldriver/eldriver_core.h"

class Sys
{
    inline static eldriver_core_t core;

    public:
    static void init(void)
    {   
        platform_init();
        eldriver_core_init(&core);
        //enable motor control function

        motor_c1.init();
        #ifndef PLATFORM_HOST
        vTaskStartScheduler();
        #else
        freeRtos_init();
        gui_loop();
        #endif
    }
};



