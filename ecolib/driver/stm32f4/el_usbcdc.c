#include "el_usbcdc.h"

#ifdef EL_USBCDC_ENABLED
void el_usbcdc_init(el_uart_handle_t *handle)
{
}

uint8_t el_usbcdc_write(el_uart_handle_t *handle, uint8_t* data, uint8_t len)
{    
}

uint16_t el_usbcdc_read(el_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    return 0;
}

el_ring_stats_t el_usbcdc_rx_stats(el_uart_handle_t *handle)
{
}

el_ring_stats_t el_usbcdc_tx_stats(el_uart_handle_t *handle)
{
}

void el_usbcdc_resetStats(el_uart_handle_t *handle)
{
}

#endif