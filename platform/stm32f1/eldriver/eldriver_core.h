/**
 * @file    eldriver_core.h
 * @author  Carol Nasser (adapted for STM32F1)
 * @brief   Core Hardware Abstraction Layer for STM32F1.
 * @details This file contains core definitions, profiling utilities, and
 * GPIO low-level driver implementations for the EcoDrive platform.
 */

#pragma once

#include <stdint.h>

/* Minimal register definitions for STM32F1 profiling */
/* DWT (Data Watchpoint and Trace) registers for cycle counting */
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_TypeDef;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_TypeDef;

#define DWT                 ((DWT_TypeDef *)0xE0001000UL)
#define CoreDebug           ((CoreDebug_TypeDef *)0xE000EDF0UL)
#define DWT_CTRL_CYCCNTENA_Msk (1UL)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)


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

/**
 * @brief  Enables the DWT cycle counter for profiling.
 */
static inline void eldriver_core_prof_enable()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
};

#ifdef __cplusplus
}
#endif