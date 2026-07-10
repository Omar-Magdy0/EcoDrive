#include "eldriver_usbxch.h"
#include "tusb.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"

#ifdef ELDRIVER_USBXCH_ENABLED

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
    h->tx_blocks = 0;
    h->tx_min_freespace = UINT16_MAX;
    h->rx_high_watermark = 0;
    tusb_init();
}

uint32_t eldriver_usbxch_write(eldriver_usbxch_handle_t *h, const uint8_t *data, uint32_t length)
{
    uint32_t free = tud_vendor_write_available();
    //while(free < length)
    //{
        free = tud_vendor_write_available();
    //}
    if(length > free){h->tx_blocks++;return 0;}
    if(free < h->tx_min_freespace)h->tx_min_freespace = free;
    return tud_vendor_write(data, length);
}

uint32_t eldriver_usbxch_write_available(eldriver_usbxch_handle_t *h)
{
    return tud_vendor_write_available();
}

uint32_t eldriver_usbxch_flush(eldriver_usbxch_handle_t *h)
{
    return tud_vendor_flush();
}

uint32_t eldriver_usbxch_clear(eldriver_usbxch_handle_t *h)
{
    tud_vendor_write_clear();
    tud_vendor_read_flush();
    return 1;
}

uint32_t eldriver_usbxch_read(eldriver_usbxch_handle_t *h, uint8_t *data, uint32_t length)
{
    return tud_vendor_read(data, length);
}

uint32_t eldriver_usbxch_read_available(eldriver_usbxch_handle_t *h)
{
    uint16_t rx_available = tud_vendor_available();
    if(rx_available > h->rx_high_watermark)h->rx_high_watermark = rx_available;
    return rx_available;
}

bool eldriver_usbxch_connected(eldriver_usbxch_handle_t *h)
{
    return tud_vendor_mounted();
}

void eldriver_usbxch_update(eldriver_usbxch_handle_t *h)
{
    tud_task();
}


#include "tusb.h"
void OTG_FS_IRQHandler(void) {
  tud_int_handler(0);
}

#endif