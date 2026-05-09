#pragma once

/**
 * @file    platform.h
 * @brief   STM32F1 platform abstraction layer interface
 */

#ifdef __cplusplus
extern "C" {
#endif

void platform_init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif
