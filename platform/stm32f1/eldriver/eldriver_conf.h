/**
 * @file    eldriver_conf.h
 * @author  Carol Nasser (adapted for STM32F1)
 * @brief   Global Configuration for STM32F1 eldrivers.
 * @details Defines hardware mappings, buffer sizes, and peripheral enable
 * flags for the entire EcoDrive driver suite on STM32F1.
 */

#ifndef DRV_CONF_H
#define DRV_CONF_H

/** @name Hardware Mapping Reference
 * Fixed mappings for STM32F103xB series (UART, SWD, USB, CAN, Timers).
 * @{ */
//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F103xB Series MCU's
//########################################################################
// UART1 for Serial (A9, A10)
// SWD (A13, A14)
// USB1 (A11, A12)
// CAN1 (A11, A12) - shared with USB
// TIM1 for complementary PWM generation (A8, A9, A10, B13, B14, B15)
// TIM2 for Hall sensor interface / sensorless commutation timing (A0, A1, A2)
// TIM3 for encoder interface (A6, A7)

// ANALOG PINS(A0, A1, A2, A3, A4, A5, A6, A7, B0, B1, C0, C1, C2, C3, C4, C5)
// Will Need 9 Pins Min For the Case of Triple Shunt + Phase Voltages + Bus Voltage + 1 Temperature + 1 Throttle

// USED ANALOG PINS (A0, A1, A2, A3, A4, A5, A6, A7, B0, B1)

// (BLUEPILL_F103 : FREE) : B2, B10, B11, B12, C13, C14, C15
//########################################################################
// FIXED HARDWARE MAPPING FOR STM32F103xB Series MCU's #########END#######
//########################################################################
/** @} */

/** @name UART1 Configuration */
//================================================
// UART1 CONFIGURATION
//================================================
//#define ELDRIVER_UART1_ENABLED
#define ELDRIVER_UART1_RX_PIN        10
#define ELDRIVER_UART1_RX_PORT       GPIOA
#define ELDRIVER_UART1_TX_PIN        9
#define ELDRIVER_UART1_TX_PORT       GPIOA
#define ELDRIVER_UART1_TX_BUFFER_SIZE 256

/** @name USB CDC Configuration */
//================================================
// USB CDC CONFIGURATION
//================================================
//#define ELDRIVER_USBCDC_ENABLED
#define ELDRIVER_USBCDC_TX_BUFFER_SIZE 256
#define ELDRIVER_USBCDC_RX_BUFFER_SIZE 256

/** @name MC3P (3-Phase Motor Control) Configuration */
//================================================
// MC3P CONFIGURATION
//================================================
#define ELDRIVER_MC3P_ENABLED

// PWM Timer Configuration
#define ELDRIVER_MC3P_TIM1_PRESCALER  0
#define ELDRIVER_MC3P_TIM1_PERIOD     4095  // 16-bit resolution
#define ELDRIVER_MC3P_PWM_FREQ        16000 // 16kHz PWM frequency

// ADC Configuration
#define ELDRIVER_MC3P_ADC_PRESCALER   ADC_CLOCK_SYNC_PCLK_DIV4
#define ELDRIVER_MC3P_ADC_RESOLUTION  ADC_RESOLUTION_12B

// Current Sensor Configuration
#define ELDRIVER_MC3P_CS ELDRIVER_MC3P_CS_NONE
#define ELDRIVER_MC3P_CS_NONE         0
#define ELDRIVER_MC3P_CS_SINGLE_SHUNT 1
#define ELDRIVER_MC3P_CS_DOUBLE_SHUNT 2
#define ELDRIVER_MC3P_CS_TRIPLE_SHUNT 3

// ADC Scaling factors for voltage and current measurements
#define ELDRIVER_MC3P_VS_SCALE 60  // Voltage scale factor
#define ELDRIVER_MC3P_CS_SCALE 50  // Current scale factor

// Hall Sensor Configuration
#define ELDRIVER_HALL_ENABLED
#define ELDRIVER_HALL_TIM2_PRESCALER 71  // 1MHz timer clock (72MHz/72)
#define ELDRIVER_HALL_TIM2_PERIOD    65535

#endif /* DRV_CONF_H */