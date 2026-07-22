#ifndef DRV_CONF_H
#define DRV_CONF_H

//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F4xx Series MCU's
//########################################################################
// UART1 for Serial (B6, B7)
// SWD (A13, A14)
// USB1 (A11, A12)
// CAN1 (B8, B9) (if exists (doesnt exist for F401, F411))
// TIM1 for complementary PWM generation (A8, A9, A10, B13, B14, B15)
// TIM2 for Hall sensor interface / sensorless commutation timing (mocked triggers) (A15, B3, B10)
// TIM3 for encoder interface(B4, B5)

// ANALOG PINS(A0, A1, A2, A3, A4, A5, A6, A7, B0, B1, C0, C1, C2, C3, C4, C5)  
//Will Need 9 Pins Min For the Case of Triple Shunt + Phase Voltages + Bus Voltage + 1 Temperature + 1 Throttle

// USED ANALOG PINS (A0, A1, A2, A3, A4, A5, A6, A7, B0, B1)

// (BLACKPILL_F401 : FREE) : C13, C14, C15, B2. B12
//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F4xx Series MCU's #########END#######
//########################################################################
#include <stdint.h>

#define EL_NONE 0
//================================================
// UART1 CONFIGURATION
//================================================
//#define EL_UART1_ENABLED
#define EL_UART1_RX_PIN        6
#define EL_UART1_RX_PORT       EL_NONE
#define EL_UART1_TX_PIN        7
#define EL_UART1_TX_PORT       EL_NONE
#define EL_UART1_TX_BUFFER_SIZE 256
#define EL_UART1_RX_BUFFER_SIZE 256
//================================================
// USBCDC CONFIGURATION
//================================================
#define EL_USBCDC_ENABLED
#define EL_USBCDC_TX_BUFFER_SIZE 4096
#define EL_USBCDC_RX_BUFFER_SIZE 1024
//================================================
// MCADCPWM3P CONFIGURATION    
//================================================
#define EL_MC3P_ENABLED        
#define EL_MC3P_CS_SCALE 50
#define EL_MC3P_VS_SCALE 60
#define CONF_MC3P_FLOAT_TO_VS(f) ((int32_t)(((float)(f) / EL_MC3P_VS_SCALE) * INT32_MAX))
#define CONF_MC3P_FLOAT_TO_CS(f) ((int32_t)(((float)(f) / EL_MC3P_CS_SCALE) * INT32_MAX))

#define EL_MC3P_DTC_ACTIVE
#define EL_MC3P_DTC_CTHRESH CONF_MC3P_FLOAT_TO_CS(0.05)
#define EL_MC3P_CS_NONE               0
#define EL_MC3P_CS_TRIPLE_SHUNT       1
#define EL_MC3P_CS_DOUBLE_SHUNT       2
#define EL_MC3P_CS_SINGLE_SHUNT       3
#define EL_MC3P_CS_INLINE             4
#define EL_MC3P_CS                    EL_MC3P_CS_TRIPLE_SHUNT
//Motor control tasking frequency
#define EL_XMC3P_TICKFREQ              4000
#define EL_XMC3P_TICKPERIOD_US          (1000000/EL_XMC3P_TICKFREQ)
#define EL_MC3P_ADCRES                 12
//#define EL_MC3P_VREFEXT                2.495f
#define EL_MC3P_HIN_ACTIVE            1
#define EL_MC3P_LIN_ACTIVE            0




//================================================
// SIL SIMULATION CONFIGURATION
//================================================
#define SIL_DEFAULT_VCC 36.0f
#define SIL_MAX_TIMESTEP (1.0/20000.0)
// Number of SIL steps to run for each PWM update 
//(for better resolution in the scope plots, better simulation quality and for marginally low time constants)
#define SIL_HALL_OFFSET 0
static const uint8_t SIL_HALL_TABLE_PI_3[6] = {
    ((1 << 2) | (1 << 1) | (0 << 0)), // 110
    ((0 << 2) | (1 << 1) | (0 << 0)), // 010
    ((0 << 2) | (1 << 1) | (1 << 0)), // 011
    ((0 << 2) | (0 << 1) | (1 << 0)), // 001
    ((1 << 2) | (0 << 1) | (1 << 0)), // 101
    ((1 << 2) | (0 << 1) | (0 << 0))  // 100
};

#endif//eld_conf.h