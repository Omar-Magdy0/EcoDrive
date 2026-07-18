#pragma once
#include "el/el_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t baudrate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t flow_control;
}el_uart_config_t;

typedef struct{
    uint8_t tx_busy;
    uint8_t rx_busy;
    uint8_t rx_overflow;
    uint8_t tx_overflow;
}el_uart_status_t;

typedef struct{
    el_uart_config_t config;
    el_uart_status_t status;
    int fd;
}el_uart_handle_t;

void el_uart1_init(el_uart_handle_t *handle);
uint8_t el_uart1_write(el_uart_handle_t *handle, const uint8_t* data, uint8_t len);
uint16_t el_uart1_read(el_uart_handle_t *handle, uint8_t* data, uint8_t len);
el_ring_stats_t el_uart1_rx_stats(el_uart_handle_t *handle);
el_ring_stats_t el_uart1_tx_stats(el_uart_handle_t *handle);
void el_uart1_resetStats(el_uart_handle_t *handle);


#ifdef __cplusplus
}
#endif