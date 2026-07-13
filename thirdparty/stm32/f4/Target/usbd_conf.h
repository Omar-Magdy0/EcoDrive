/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_conf.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CONF
  * @brief USB device low level driver configuration file
  * @{
  */

/** @defgroup USBD_CONF_Exported_Defines
  * @{
  */
#define DEVICE_FS 		0
#define DEVICE_HS 		1
#define USBD_MAX_NUM_INTERFACES                     1U
#define USBD_MAX_NUM_CONFIGURATION                  1U
#define USBD_MAX_STR_DESC_SIZ                       0x80U
#define USBD_SELF_POWERED                           1U
#define USBD_DEBUG_LEVEL                            0U
/* #define USBD_USER_REGISTER_CALLBACK                 1U */

/** @defgroup USBD_Exported_Macros
  * @{
  */

/* Memory management macros make sure to use static memory allocation */
/** Alias for memory allocation. */
#define USBD_malloc         (void *)USBD_static_malloc

/** Alias for memory release. */
#define USBD_free           USBD_static_free

/** Alias for memory set. */
#define USBD_memset         memset

/** Alias for memory copy. */
#define USBD_memcpy         memcpy

/** Alias for delay. */
#define USBD_Delay          HAL_Delay

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0U)
#define  USBD_UsrLog(...)   do { \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_UsrLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 0U) */

#if (USBD_DEBUG_LEVEL > 1U)

#define  USBD_ErrLog(...) do { \
                               printf("ERROR: ") ; \
                               printf(__VA_ARGS__); \
                               printf("\n"); \
                             } while (0)
#else
#define USBD_ErrLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 1U) */

#if (USBD_DEBUG_LEVEL > 2U)
#define  USBD_DbgLog(...)   do { \
                                 printf("DEBUG : ") ; \
                                 printf(__VA_ARGS__); \
                                 printf("\n"); \
                               } while (0)
#else
#define USBD_DbgLog(...) do {} while (0)
#endif /* (USBD_DEBUG_LEVEL > 2U) */

/**
  * @}
  */

/**
  * @}
  */


/** @defgroup USBD_CONF_Exported_Types
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CONF_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CONF_Exported_FunctionsPrototype
  * @{
  */
/* Exported functions -------------------------------------------------------*/
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */


/**
  * @}
  */

/**
  * @}
  */
