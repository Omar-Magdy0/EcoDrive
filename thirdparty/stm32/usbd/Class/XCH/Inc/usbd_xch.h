/**
  ******************************************************************************
  * @file    usbd_xch_core.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_xch_core.c file.
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
#ifndef __USB_XCH_CORE_H
#define __USB_XCH_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_TEMPLATE
  * @brief This file is the header file for usbd_xch_core.c
  * @{
  */


/** @defgroup USBD_XCH_Exported_Defines
  * @{
  */
  #define XCH_OUT_EP                    0x01U
  #define XCH_IN_EP                     0x81U
  #define XCH_OUT_MPS                   64U
  #define XCH_IN_MPS                    64U




#define USB_XCH_CONFIG_DESC_SIZ       32U

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */
#include <stdint.h>

extern USBD_ClassTypeDef USBD_XCH;

typedef struct 
{
    uint8_t *rx_buf;
    uint16_t rx_size;
    void* ctx;
}USBD_XCH_HandleTypeDef;

void USBD_XCH_GetRxStorage(void *ctx, uint8_t **ptr, uint16_t *size);
void USBD_XCH_RxComplete(void *ctx, uint8_t *rx_buf, uint16_t size);
void USBD_XCH_TxComplete(void *ctx);
uint8_t USBD_XCH_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len);
/**
  * @}
  */


/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_XCH_CORE_H */
/**
  * @}
  */

/**
  * @}
  */
