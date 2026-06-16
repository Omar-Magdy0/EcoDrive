/**
 * @file    platform.c
 * @brief   Platform-specific initialization for STM32F1
 */

#include "platform.h"
#include <stdint.h>

void Error_Handler(void)
{
  __asm volatile("cpsid i");  /* Disable all interrupts */
  while (1) {}
}

void platform_init(void)
{
  /* Minimal platform init - user to provide clock config via HAL or direct register access */
  Error_Handler();  /* Placeholder - to be implemented by user */
}
