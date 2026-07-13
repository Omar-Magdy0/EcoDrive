/**
  ******************************************************************************
  * @file    usbd_xch.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
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
  * @verbatim
  *
  *          ===================================================================
  *                                XCH Class  Description
  *          ===================================================================
  *
  *
  *
  *
  *
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_xch.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_XCH
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_XCH_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_XCH_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_XCH_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_XCH_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_XCH_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_XCH_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_XCH_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_XCH_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_XCH_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_XCH_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_XCH_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_XCH_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_XCH_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_XCH_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_XCH_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_XCH_GetDeviceQualifierDesc(uint16_t *length);
/**
  * @}
  */

/** @defgroup USBD_XCH_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_XCH =
{
  USBD_XCH_Init,
  USBD_XCH_DeInit,
  USBD_XCH_Setup,
  USBD_XCH_EP0_TxReady,
  USBD_XCH_EP0_RxReady,
  USBD_XCH_DataIn,
  USBD_XCH_DataOut,
  USBD_XCH_SOF,
  USBD_XCH_IsoINIncomplete,
  USBD_XCH_IsoOutIncomplete,
  USBD_XCH_GetCfgDesc,
  USBD_XCH_GetCfgDesc,
  USBD_XCH_GetCfgDesc,
  USBD_XCH_GetDeviceQualifierDesc,
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* __ICCARM__ */
/* USB XCH device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_XCH_CfgDesc[USB_XCH_CONFIG_DESC_SIZ] __ALIGN_END =
{
  USB_CONF_DESC_SIZE, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  LOBYTE(USB_XCH_CONFIG_DESC_SIZ),
  HIBYTE(USB_XCH_CONFIG_DESC_SIZ),
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */
  /* Interface 0 */
  USB_IF_DESC_SIZE,                       // bLength
  USB_DESC_TYPE_INTERFACE,    // bDescriptorType
  0x00,                       // bInterfaceNumber
  0x00,                       // bAlternateSetting
  0x02,                       // bNumEndpoints
  0xFF,                       // Vendor Specific
  0x00,                       // Subclass
  0x00,                       // Protocol
  0x00,                       // iInterface
  /* Endpoint 0x81*/
  USB_EP_DESC_SIZE,
  USB_DESC_TYPE_ENDPOINT,
  0x81,
  USBD_EP_TYPE_BULK,
  0x40,
  0x00,
  0x00,
  /* Endpoint 0x01*/
  USB_EP_DESC_SIZE,
  USB_DESC_TYPE_ENDPOINT,
  0x01,
  USBD_EP_TYPE_BULK,
  0x40,
  0x00,
  0x00
  /**********  Descriptor of XCH interface 0 Alternate setting 0 **************/

};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4
#endif /* __ICCARM__ */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_XCH_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_XCH_Private_Functions
  * @{
  */

/**
  * @brief  USBD_XCH_Init
  *         Initialize the XCH interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_XCH_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_XCH_HandleTypeDef *hxch = pdev->pClassData;
  USBD_LL_OpenEP(pdev, XCH_IN_EP, USBD_EP_TYPE_BULK, XCH_IN_MPS);
  USBD_LL_OpenEP(pdev, XCH_OUT_EP, USBD_EP_TYPE_BULK, XCH_OUT_MPS);
  uint8_t *ptr; uint16_t size;
  USBD_XCH_GetRxStorage(hxch->ctx, &ptr, &size);
  hxch->rx_buf = ptr; hxch->rx_size = size;
  USBD_LL_PrepareReceive(pdev, XCH_OUT_EP, ptr, size);
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_Init
  *         DeInitialize the XCH layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_XCH_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_LL_CloseEP(pdev, XCH_IN_EP);
  USBD_LL_CloseEP(pdev, XCH_OUT_EP);
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_Setup
  *         Handle the XCH specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_XCH_Setup(USBD_HandleTypeDef *pdev,
                                   USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_XCH_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_XCH_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_XCH_CfgDesc);
  return USBD_XCH_CfgDesc;
}

/**
  * @brief  USBD_XCH_GetDeviceQualifierDesc
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_XCH_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_XCH_DeviceQualifierDesc);
  return USBD_XCH_DeviceQualifierDesc;
}

/**
  * @brief  USBD_XCH_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_XCH_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_XCH_TxComplete(((USBD_XCH_HandleTypeDef*)pdev->pClassData)->ctx);
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_XCH_EP0_RxReady(USBD_HandleTypeDef *pdev)
{

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_XCH_EP0_TxReady(USBD_HandleTypeDef *pdev)
{

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_XCH_SOF(USBD_HandleTypeDef *pdev)
{

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_XCH_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_XCH_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_XCH_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_XCH_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_XCH_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  //Recieve data
  USBD_XCH_HandleTypeDef *h = pdev->pClassData;
  uint32_t received = USBD_LL_GetRxDataSize(pdev, XCH_OUT_EP);
  //Recieve callback
  USBD_XCH_RxComplete(h->ctx, h->rx_buf, received);
  //Rearm
  uint8_t *ptr; uint16_t size;
  USBD_XCH_GetRxStorage(h->ctx, &ptr, &size);
  h->rx_buf = ptr; h->rx_size = size;
  USBD_LL_PrepareReceive(
    pdev,
    XCH_OUT_EP,
    ptr,
    size);
  return (uint8_t)USBD_OK;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

uint8_t USBD_XCH_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len)
{
    return USBD_LL_Transmit(pdev, XCH_IN_EP, buf, len);
}
__weak void USBD_XCH_GetRxStorage(void *ctx,
                                  uint8_t **ptr,
                                  uint16_t *size)
{
    (void)ctx;
    *ptr = NULL;
    *size = 0;
}

__weak void USBD_XCH_RxComplete(void *ctx,
                                uint8_t *rx_buf,
                                uint16_t size)
{
    (void)ctx;
    (void)rx_buf;
    (void)size;
}

__weak void USBD_XCH_TxComplete(void *ctx)
{
    (void)ctx;
}