#pragma once

/* USER CODE BEGIN Header */
 /**
   ******************************************************************************
   * @file           : platform.h
   * @brief          : Header for sys_init.c file.
   *                   This file contains the common defines of the application.
   ******************************************************************************
   * @attention
   *
   * 
   * 
   */

#include "stm32f4xx.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void platform_init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif
