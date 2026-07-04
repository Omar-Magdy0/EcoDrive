#include "eldriver_usbxch.h"
#include "tusb.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"

void eldriver_usbxch_init(eldriver_usbxch_handle_t *h)
{
   __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0); 
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};\
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;\
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;\
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;\
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;\
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;\
    GPIO_InitStruct.Alternate = LL_GPIO_AF_10;\
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);\
    tusb_init();
}

uint32_t eldriver_usbxch_write(eldriver_usbxch_handle_t *h, const uint8_t *data, uint32_t length)
{
    tud_vendor_write(data, length);
    return 0;
}

uint32_t eldriver_usbxch_read(eldriver_usbxch_handle_t *h, uint8_t *data, uint32_t length)
{
    return 0;
}

bool eldriver_usbxch_connected(eldriver_usbxch_handle_t *h)
{
    return false;
}

void eldriver_usbxch_update(eldriver_usbxch_handle_t *h)
{
    tud_task();
}

#include "tusb.h"
void OTG_FS_IRQHandler(void) {
  tud_int_handler(0);
}