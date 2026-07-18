#pragma once

#include "el_uart.h"
#include "eld_conf.h"


#ifdef __cplusplus
extern "C" {
#endif



void el_usbcdc_init(el_uart_handle_t *handle);
uint8_t el_usbcdc_write(el_uart_handle_t *handle, uint8_t* data, uint8_t len);
uint16_t el_usbcdc_read(el_uart_handle_t *handle, uint8_t* data, uint8_t len);
el_ring_stats_t el_usbcdc_rx_stats(el_uart_handle_t *handle);
el_ring_stats_t el_usbcdc_tx_stats(el_uart_handle_t *handle);
void el_usbcdc_resetStats(el_uart_handle_t *handle);

#ifdef __cplusplus
}
#endif
