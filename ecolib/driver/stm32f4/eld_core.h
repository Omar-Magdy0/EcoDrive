/**
 * @file    el_core.h
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

}el_core_t;

/**
 * @brief  Initializes the core driver handle.
 * @param  h: Pointer to el_core_t handle.
 */
void el_core_init(el_core_t *h);

/**
 * @brief  Starts a profiling tick using the DWT Cycle Count Register.
 * @retval uint32_t: Current cycle count.
 */
static inline uint32_t el_prof_tick()
{
    return DWT->CYCCNT;
};

/**
 * @brief  Calculates cycles elapsed since the start tick.
 * @param  start: The initial cycle count from prof_tick.
 * @retval uint32_t: Elapsed cycles.
 */
static inline uint32_t el_prof_tock(uint32_t start)
{
    return (DWT->CYCCNT - start);
};


#ifndef EL_STM32_LL_GPIO_H
#define EL_STM32_LL_GPIO_H

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

#define EL_GPIO_TO_PORT(n) \
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
#define EL_GPIO_TO_PIN(n) (1 << ((n) % 16))

/* Types */
/** @brief Type definition for global pin indexing. */
typedef uint16_t el_pin_t;

/** @brief GPIO Mode enumeration mapped to LL Driver. */
typedef enum {
    EL_GPIO_MODE_INPUT  = LL_GPIO_MODE_INPUT,
    EL_GPIO_MODE_OUTPUT = LL_GPIO_MODE_OUTPUT,
    EL_GPIO_MODE_ALT    = LL_GPIO_MODE_ALTERNATE,
    EL_GPIO_MODE_ANALOG = LL_GPIO_MODE_ANALOG
} el_gpio_mode_t;

/** @brief Logic state enumeration for GPIO pins. */
typedef enum {
    EL_GPIO_LOW  = 0,
    EL_GPIO_HIGH = 1
} el_gpio_state_t;

/* Static Inline Implementation */

/**
 * @brief Sets the pin mode for a specific global pin number.
 */
static inline void el_gpio_mode(el_pin_t pin_num, el_gpio_mode_t mode) {
    LL_GPIO_SetPinMode(EL_GPIO_TO_PORT(pin_num), 
                       EL_GPIO_TO_PIN(pin_num), 
                       (uint32_t)mode);
}

/**
 * @brief Writes to a pin using the BSRR register (Atomic).
 * If state is HIGH, it writes to the lower 16 bits; if LOW, the upper 16 bits.
 */
static inline void el_gpio_write(el_pin_t pin_num, el_gpio_state_t state) {
    if (state == EL_GPIO_HIGH) {
        LL_GPIO_SetOutputPin(EL_GPIO_TO_PORT(pin_num), 
                             EL_GPIO_TO_PIN(pin_num));
    } else {
        LL_GPIO_ResetOutputPin(EL_GPIO_TO_PORT(pin_num), 
                               EL_GPIO_TO_PIN(pin_num));
    }
}

/**
 * @brief Toggles the pin state.
 */
static inline void el_gpio_toggle(el_pin_t pin_num) {
    LL_GPIO_TogglePin(EL_GPIO_TO_PORT(pin_num), 
                      EL_GPIO_TO_PIN(pin_num));
}

/**
 * @brief Reads the current input state of the pin.
 */
static inline el_gpio_state_t el_gpio_read(el_pin_t pin_num) {
    return (el_gpio_state_t)LL_GPIO_IsInputPinSet(EL_GPIO_TO_PORT(pin_num), 
                                                       EL_GPIO_TO_PIN(pin_num));
}

/**
 * @brief  Disables interrupts to start an atomic section.
 * @retval uint32_t: Saved interrupt state (PRIMASK value).
 */
static inline uint32_t el_atomic_start(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief  Restores interrupts to end an atomic section.
 * @param  primask: The saved interrupt state from el_atomic_start().
 */
static inline void el_atomic_end(uint32_t primask) {
    __set_PRIMASK(primask);
}

#include "stm32f4xx_hal.h"
static inline void el_delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

#endif // EL_STM32_LL_GPIO_H



#ifdef __cplusplus
}
#endif