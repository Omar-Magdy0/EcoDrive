/**
 * @file    eldriver_core.h
 * @author  Carol Nasser
 * @brief   Core Hardware Abstraction Layer for STM32F4.
 * @details This file contains core definitions, profiling utilities, and 
 * GPIO low-level driver implementations for the EcoDrive platform.
 */

#pragma once

#include <stdint.h>
#include "stm32f4xx.h"


#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief Core handle structure for eldriver state management.
 */
typedef struct{

}eldriver_core_t;

/**
 * @brief  Initializes the core driver handle.
 * @param  h: Pointer to eldriver_core_t handle.
 */
void eldriver_core_init(eldriver_core_t *h);

/**
 * @brief  Starts a profiling tick using the DWT Cycle Count Register.
 * @retval uint32_t: Current cycle count.
 */
static inline uint32_t eldriver_core_prof_tick()
{
    return DWT->CYCCNT;
};

/**
 * @brief  Calculates cycles elapsed since the start tick.
 * @param  start: The initial cycle count from prof_tick.
 * @retval uint32_t: Elapsed cycles.
 */
static inline uint32_t eldriver_core_prof_tock(uint32_t start)
{
    return (DWT->CYCCNT - start);
};


#ifndef ELDRIVER_STM32_LL_GPIO_H
#define ELDRIVER_STM32_LL_GPIO_H

#include "stm32f4xx_ll_gpio.h" // Replace with your specific family header if needed

/**
 * @brief Macro to map a global pin number to an STM32 GPIO Port.
 * Optimized for static evaluation by the compiler.
 */
#ifndef GPIOF 
#define GPIOF 0
#endif

#ifndef GPIOG 
#define GPIOG 0
#endif

#ifndef GPIOH 
#define GPIOH 0
#endif

#define ELDRIVER_GPIO_TO_PORT(n) \
    (((n) < 16)  ? GPIOA :           \
     ((n) < 32)  ? GPIOB :           \
     ((n) < 48)  ? GPIOC :           \
     ((n) < 64)  ? GPIOD :           \
     ((n) < 80)  ? GPIOE :           \
     ((n) < 96)  ? GPIOF :           \
     ((n) < 112) ? GPIOG : GPIOH)

/**
 * @brief Macro to map a global pin number to an STM32 LL Pin Mask.
 */
#define ELDRIVER_GPIO_TO_PIN(n) (1 << ((n) % 16))

/* Types */
/** @brief Type definition for global pin indexing. */
typedef uint16_t eldriver_pin_t;

/** @brief GPIO Mode enumeration mapped to LL Driver. */
typedef enum {
    ELDRIVER_GPIO_MODE_INPUT  = LL_GPIO_MODE_INPUT,
    ELDRIVER_GPIO_MODE_OUTPUT = LL_GPIO_MODE_OUTPUT,
    ELDRIVER_GPIO_MODE_ALT    = LL_GPIO_MODE_ALTERNATE,
    ELDRIVER_GPIO_MODE_ANALOG = LL_GPIO_MODE_ANALOG
} eldriver_gpio_mode_t;

/** @brief Logic state enumeration for GPIO pins. */
typedef enum {
    ELDRIVER_GPIO_LOW  = 0,
    ELDRIVER_GPIO_HIGH = 1
} eldriver_gpio_state_t;

/* Static Inline Implementation */

/**
 * @brief Sets the pin mode for a specific global pin number.
 */
static inline void eldriver_gpio_mode(eldriver_pin_t pin_num, eldriver_gpio_mode_t mode) {
    LL_GPIO_SetPinMode(ELDRIVER_GPIO_TO_PORT(pin_num), 
                       ELDRIVER_GPIO_TO_PIN(pin_num), 
                       (uint32_t)mode);
}

/**
 * @brief Writes to a pin using the BSRR register (Atomic).
 * If state is HIGH, it writes to the lower 16 bits; if LOW, the upper 16 bits.
 */
static inline void eldriver_gpio_write(eldriver_pin_t pin_num, eldriver_gpio_state_t state) {
    if (state == ELDRIVER_GPIO_HIGH) {
        LL_GPIO_SetOutputPin(ELDRIVER_GPIO_TO_PORT(pin_num), 
                             ELDRIVER_GPIO_TO_PIN(pin_num));
    } else {
        LL_GPIO_ResetOutputPin(ELDRIVER_GPIO_TO_PORT(pin_num), 
                               ELDRIVER_GPIO_TO_PIN(pin_num));
    }
}

/**
 * @brief Toggles the pin state.
 */
static inline void eldriver_gpio_toggle(eldriver_pin_t pin_num) {
    LL_GPIO_TogglePin(ELDRIVER_GPIO_TO_PORT(pin_num), 
                      ELDRIVER_GPIO_TO_PIN(pin_num));
}

/**
 * @brief Reads the current input state of the pin.
 */
static inline eldriver_gpio_state_t eldriver_gpio_read(eldriver_pin_t pin_num) {
    return (eldriver_gpio_state_t)LL_GPIO_IsInputPinSet(ELDRIVER_GPIO_TO_PORT(pin_num), 
                                                       ELDRIVER_GPIO_TO_PIN(pin_num));
}

#endif // ELDRIVER_STM32_LL_GPIO_H



#ifdef __cplusplus
}
#endif