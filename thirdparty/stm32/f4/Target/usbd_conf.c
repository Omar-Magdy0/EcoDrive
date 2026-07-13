/**
 ******************************************************************************
 * @file    USB_Device/CDC_Standalone/Src/usbd_conf.c
 * @author  MCD Application Team
 * @brief   This file implements the USB Device library callbacks and MSP
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "usbd_xch.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
                       PCD BSP Routines
*******************************************************************************/

/**
 * @brief  Initializes the PCD MSP.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIOA clock */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure USB DM (PA11) and DP (PA12) */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10; // USB OTG FS on STM32F4

  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable USB FS Clock */
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  /* Enable SYSCFG Clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
 * @brief  De-Initializes the PCD MSP.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Disable USB FS Clock */
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

  /* Disable SYSCFG Clock */
  __HAL_RCC_SYSCFG_CLK_DISABLE();
}

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
 * @brief  SetupStage callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
 * @brief  DataOut Stage callback.
 * @param  hpcd: PCD handle
 * @param  epnum: Endpoint Number
 * @retval None
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
 * @brief  DataIn Stage callback.
 * @param  hpcd: PCD handle
 * @param  epnum: Endpoint Number
 * @retval None
 */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF(hpcd->pData);
}

/**
 * @brief  Reset callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
  /* Reset Device */
  USBD_LL_Reset(hpcd->pData);
}

/**
 * @brief  Suspend callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
}

/**
 * @brief  Resume callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
}

/**
 * @brief  ISOOUTIncomplete callback.
 * @param  hpcd: PCD handle
 * @param  epnum: Endpoint Number
 * @retval None
 */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

/**
 * @brief  ISOINIncomplete callback.
 * @param  hpcd: PCD handle
 * @param  epnum: Endpoint Number
 * @retval None
 */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

/**
 * @brief  ConnectCallback callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

/**
 * @brief  Disconnect callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
 * @brief  Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  /* Set LL Driver parameters */
  hpcd.Instance = USB_OTG_FS;
  hpcd.Init.dev_endpoints = 8;
  hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd.Init.speed = PCD_SPEED_FULL;
  hpcd.Init.Sof_enable = DISABLE;
  hpcd.Init.low_power_enable = DISABLE;
  hpcd.Init.lpm_enable = DISABLE;
  hpcd.Init.vbus_sensing_enable = DISABLE; 
  hpcd.Init.use_dedicated_ep1 = DISABLE;
  hpcd.Init.dma_enable = ENABLE;
  /* Link The driver to the stack */
  hpcd.pData = pdev;
  pdev->pData = &hpcd;
  /* Initialize LL Driver */
  HAL_PCD_Init(pdev->pData);
  HAL_PCDEx_SetRxFiFo(&hpcd, 128);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 32);   // EP0 IN
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 160);   // Bulk IN endpoint
  return USBD_OK;
}

/**
 * @brief  De-Initializes the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

/**
 * @brief  Starts the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

/**
 * @brief  Stops the Low Level portion of the Device driver.
 * @param  pdev: Device handle
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

/**
 * @brief  Opens an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  ep_type: Endpoint Type
 * @param  ep_mps: Endpoint Max Packet Size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type,
                                  uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData,
                  ep_addr,
                  ep_mps,
                  ep_type);

  return USBD_OK;
}

/**
 * @brief  Closes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
 * @brief  Flushes an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
 * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
 * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

/**
 * @brief  Returns Stall condition.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Stall (1: Yes, 0: No)
 */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if ((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

/**
 * @brief  Assigns a USB address to the device.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

/**
 * @brief  Transmits data over an endpoint.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be sent
 * @param  size: Data size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev,
                                    uint8_t ep_addr,
                                    uint8_t *pbuf,
                                    uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
 * @brief  Prepares an endpoint for reception.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @param  pbuf: Pointer to data to be received
 * @param  size: Data size
 * @retval USBD Status
 */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                          uint8_t ep_addr,
                                          uint8_t *pbuf,
                                          uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

/**
 * @brief  Returns the last transferred packet size.
 * @param  pdev: Device handle
 * @param  ep_addr: Endpoint Number
 * @retval Received Data Size
 */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
 * @brief  Delays routine for the USB Device Library.
 * @param  Delay: Delay in ms
 * @retval None
 */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
 * @brief  static single allocation.
 * @param  size: size of allocated memory
 * @retval None
 */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[sizeof(USBD_XCH_HandleTypeDef) / 4 + 1];
  return mem;
}

/**
 * @brief  Dummy memory free
 * @param  *p pointer to allocated  memory address
 * @retval None
 */
void USBD_static_free(void *p)
{
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}