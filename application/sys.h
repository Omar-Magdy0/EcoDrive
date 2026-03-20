#pragma once

#include "PmsmController.h"
#include "platform.h"
#include "middleware/aebfStream.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>


constexpr uint8_t AEBF_DEVICE_ID = 0x01;
constexpr uint8_t AEBF_PWMDATA_SERVICEID = 0xC1;

// AEBF_PWMSCAN structure 


/**
 * PWM Data Schema
 * 
 * All packets: 5 floats max, unused = 0/NAN
 * 
 * TYPE 1: FOC (5 floats)
 * [0] vbus  - Bus voltage (V)
 * [1] vq    - Q-axis voltage (V)
 * [2] vd    - D-axis voltage (V)  
 * [3] iq    - Q-axis current (A)
 * [4] id    - D-axis current (A)
 * 
 * TYPE 3: TRAP_WITH_BEMF (4 floats)
 * [0] vbus  - Bus voltage (V)
 * [1] vduty - Duty cycle (0-1)
 * [2] ibus  - Bus current (A)
 * [3] bemf  - Back-EMF (V)
 * [4] (unused)
 */




//=======================================================



void sys_init();

#include "eldriver/eldriver_core.h"
extern eldriver_core_t core;
