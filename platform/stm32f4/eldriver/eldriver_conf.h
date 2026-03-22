/**
 * @file    eldriver_conf.h
 * @author  Carol Nasser
 * @brief   Global Configuration for STM32F4 eldrivers.
 * @details Defines hardware mappings, buffer sizes, and peripheral enable 
 * flags for the entire EcoDrive driver suite.
 */

#ifndef DRV_CONF_H
#define DRV_CONF_H
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"

/** @name Hardware Mapping Reference
 * Fixed mappings for STM32F4xx series (UART, SWD, USB, CAN, Timers).
 * @{ */
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
/** @} */

/** @name UART1 Configuration */
//================================================
// UART1 CONFIGURATION
//================================================
//#define ELDRIVER_UART1_ENABLED
#define ELDRIVER_UART1_RX_PIN        6
#define ELDRIVER_UART1_RX_PORT       GPIOB
#define ELDRIVER_UART1_TX_PIN        7
#define ELDRIVER_UART1_TX_PORT       GPIOB
#define ELDRIVER_UART1_TX_BUFFER_SIZE 256
#define ELDRIVER_UART1_RX_BUFFER_SIZE 256

/** @name USB CDC Configuration */
//================================================
// USBCDC CONFIGURATION
//================================================
#define ELDRIVER_USBCDC_ENABLED
#define ELDRIVER_USBCDC_TX_BUFFER_SIZE 4096
#define ELDRIVER_USBCDC_RX_BUFFER_SIZE 1024

/** @name Motor Control (MC3P) Configuration */
//================================================
// MCADCPWM3P CONFIGURATION    
//================================================
#define ELDRIVER_MC3P_ENABLED        
#define ELDRIVER_MC3P_CS_NONE               0
#define ELDRIVER_MC3P_CS_TRIPLE_SHUNT       1
#define ELDRIVER_MC3P_CS_DOUBLE_SHUNT       2
#define ELDRIVER_MC3P_CS_SINGLE_SHUNT       3
#define ELDRIVER_MC3P_CS_INLINE             4
#define ELDRIVER_MC3P_CS                    ELDRIVER_MC3P_CS_TRIPLE_SHUNT
//Motor control tasking frequency
#define ELDRIVER_XMC3P_TICKFREQ              4000
#define ELDRIVER_XMC3P_TICKPERIOD_US          (1000000/ELDRIVER_XMC3P_TICKFREQ)
#define ELDRIVER_MC3P_ADCRES                 12
//#define ELDRIVER_MC3P_VREFEXT                2.495f
#define ELDRIVER_MC3P_HIN_ACTIVE            1
#define ELDRIVER_MC3P_LIN_ACTIVE            0

#define ELDRIVER_MC3P_UH_PIN         LL_GPIO_PIN_8
#define ELDRIVER_MC3P_UH_PORT        GPIOA
#define ELDRIVER_MC3P_UL_PIN         LL_GPIO_PIN_13
#define ELDRIVER_MC3P_UL_PORT        GPIOB
#define ELDRIVER_MC3P_VH_PIN         LL_GPIO_PIN_9
#define ELDRIVER_MC3P_VH_PORT        GPIOA
#define ELDRIVER_MC3P_VL_PIN         LL_GPIO_PIN_14
#define ELDRIVER_MC3P_VL_PORT        GPIOB
#define ELDRIVER_MC3P_WH_PIN         LL_GPIO_PIN_10
#define ELDRIVER_MC3P_WH_PORT        GPIOA
#define ELDRIVER_MC3P_WL_PIN         LL_GPIO_PIN_15
#define ELDRIVER_MC3P_WL_PORT        GPIOB


#define ELDRIVER_MC3P_CS_SCALE 50
#define ELDRIVER_MC3P_VS_SCALE 60


#define ELDRIVER_MC3P_VSBUS_PIN               LL_GPIO_PIN_0
#define ELDRIVER_MC3P_VSBUS_PORT              GPIOA
#define ELDRIVER_MC3P_VSBUS_ADC_CHANNEL       LL_ADC_CHANNEL_0   // PA1 - ADC1_IN2
#define ELDRIVER_MC3P_VSU_PIN                 LL_GPIO_PIN_1
#define ELDRIVER_MC3P_VSU_PORT                GPIOA
#define ELDRIVER_MC3P_VSU_ADC_CHANNEL         LL_ADC_CHANNEL_1   // PA1 - ADC1_IN2
#define ELDRIVER_MC3P_VSV_PIN                 LL_GPIO_PIN_2
#define ELDRIVER_MC3P_VSV_PORT                GPIOA
#define ELDRIVER_MC3P_VSV_ADC_CHANNEL         LL_ADC_CHANNEL_2   // PA4 - ADC1_IN3
#define ELDRIVER_MC3P_VSW_PIN                 LL_GPIO_PIN_3
#define ELDRIVER_MC3P_VSW_PORT                GPIOA
#define ELDRIVER_MC3P_VSW_ADC_CHANNEL         LL_ADC_CHANNEL_3   // PA5 - ADC1_IN3

#if (ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_TRIPLE_SHUNT || ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_DOUBLE_SHUNT || ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_INLINE)
    // Three shunt resistors - one per phase , (for double shunt and inline comment out unnused sensor)
    #define ELDRIVER_MC3P_CSU_PIN            4
    #define ELDRIVER_MC3P_CSU_PORT           GPIOA
    #define ELDRIVER_MC3P_CSU_ADC_CHANNEL    LL_ADC_CHANNEL_4   // PA3 - ADC1_IN4
    #define ELDRIVER_MC3P_CSV_PIN            5
    #define ELDRIVER_MC3P_CSV_PORT           GPIOA
    #define ELDRIVER_MC3P_CSV_ADC_CHANNEL    LL_ADC_CHANNEL_5   // PA4 - ADC1_IN5
    #define ELDRIVER_MC3P_CSW_PIN            6
    #define ELDRIVER_MC3P_CSW_PORT           GPIOA
    #define ELDRIVER_MC3P_CSW_ADC_CHANNEL    LL_ADC_CHANNEL_6   // PA5 - ADC1_IN6
    
#elif (ELDRIVER_MC3P_CS == ELDRIVER_MC3P_CS_SINGLE_SHUNT)
    // Single shunt resistor on DC bus
    #define ELDRIVER_MC3P_CSBUS_PIN            3
    #define ELDRIVER_MC3P_CSBUS_PORT           GPIOA
    #define ELDRIVER_MC3P_CSBUS_ADC_CHANNEL    LL_ADC_CHANNEL_5   // PA3 - ADC1_IN4

#endif

#define ELDRIVER_MC3P_BG_CHANNELS 0
#if ELDRIVER_MC3P_BG_CHANNELS > 0
    #define ELDRIVER_MC3P_BG1_PIN
    #define ELDRIVER_MC3P_BG1_PORT
    #define ELDRIVER_MC3P_BG1_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG1_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 1
    #define ELDRIVER_MC3P_BG2_PIN
    #define ELDRIVER_MC3P_BG2_PORT
    #define ELDRIVER_MC3P_BG2_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG2_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 2
    #define ELDRIVER_MC3P_BG1_PIN
    #define ELDRIVER_MC3P_BG1_PORT
    #define ELDRIVER_MC3P_BG1_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG1_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 3
    #define ELDRIVER_MC3P_BG2_PIN
    #define ELDRIVER_MC3P_BG2_PORT
    #define ELDRIVER_MC3P_BG2_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG2_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 4
    #define ELDRIVER_MC3P_BG1_PIN
    #define ELDRIVER_MC3P_BG1_PORT
    #define ELDRIVER_MC3P_BG1_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG1_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 5
    #define ELDRIVER_MC3P_BG2_PIN
    #define ELDRIVER_MC3P_BG2_PORT
    #define ELDRIVER_MC3P_BG2_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG2_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 6
    #define ELDRIVER_MC3P_BG2_PIN
    #define ELDRIVER_MC3P_BG2_PORT
    #define ELDRIVER_MC3P_BG2_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG2_RANK
#endif
#if ELDRIVER_MC3P_BG_CHANNELS > 7
    #define ELDRIVER_MC3P_BG2_PIN
    #define ELDRIVER_MC3P_BG2_PORT
    #define ELDRIVER_MC3P_BG2_ADC_CHANNEL
    #define ELDRIVER_MC3P_BG2_RANK
#endif


/** @name Hall Sensor Configuration */
//================================================
// HALL SENSOR CONFIGURATION    
//================================================
#define ELDRIVER_HALL1_ENABLED            
#define ELDRIVER_HALL1_A_PIN              LL_GPIO_PIN_15
#define ELDRIVER_HALL1_A_PORT             GPIOA
#define ELDRIVER_HALL1_B_PIN              LL_GPIO_PIN_3
#define ELDRIVER_HALL1_B_PORT             GPIOB
#define ELDRIVER_HALL1_C_PIN              LL_GPIO_PIN_10
#define ELDRIVER_HALL1_C_PORT             GPIOB



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/** @name DMA & NVIC Configuration */
//================================================
//DMA CONFIGURATION
//================================================
//NEVER FORGET 
//THAT YOU EVER CHANGE THESE VALUES FOR DMA YOU STILL HAVE TO CHANGE IMPLEMENTATION AND IRQ APPROPIATELY
#define UART1_DMA_TX_STREAM             LL_DMA_STREAM_7
#define UART1_DMA_RX_STREAM             LL_DMA_STREAM_5
#define UART1_DMA_INSTANCE              DMA2
#define UART1_DMA_CHANNEL               LL_DMA_CHANNEL_4
#define MC3P_DMA_BACKGROUND_STREAM      LL_DMA_STREAM_0
#define MC3P_DMA_INSTANCE               DMA2
#define MC3P_DMA_CHANNEL                LL_DMA_CHANNEL_0
//================================================
// NVIC CONFIGURATION
//================================================
#define UART1_NVIC_PRIORITY 4


#endif//eldriver_conf.h