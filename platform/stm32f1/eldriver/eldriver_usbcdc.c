/**
 * @file    eldriver_usbcdc.c
 * @brief   STM32F1 USB CDC driver implementation
 */

#include "eldriver_usbcdc.h"

/* STM32F1 USB CDC driver stubs */
/* TODO: Implement actual USB CDC functionality */

void eldriver_usbcdc_init(eldriver_uart_handle_t *handle)
{
    /* Initialize USB CDC peripheral */
}

uint8_t eldriver_usbcdc_write(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    return 0; /* No bytes written */
}

uint16_t eldriver_usbcdc_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    return 0; /* No bytes read */
}

elcore_rstream_stats_t eldriver_usbcdc_rx_stats(eldriver_uart_handle_t *handle)
{
    elcore_rstream_stats_t stats = {0, 0, 0};
    return stats;
}

elcore_rstream_stats_t eldriver_usbcdc_tx_stats(eldriver_uart_handle_t *handle)
{
    elcore_rstream_stats_t stats = {0, 0, 0};
    return stats;
}

void eldriver_usbcdc_resetStats(eldriver_uart_handle_t *handle)
{
    /* Reset statistics */
}