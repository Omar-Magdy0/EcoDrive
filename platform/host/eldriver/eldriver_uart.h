#pragma once
#include "elcore_rstream.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t baudrate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t flow_control;
}eldriver_uart_config_t;

typedef struct{
    uint8_t tx_busy;
    uint8_t rx_busy;
    uint8_t rx_overflow;
    uint8_t tx_overflow;
}eldriver_uart_status_t;

typedef struct{
    eldriver_uart_config_t config;
    eldriver_uart_status_t status;
    int fd;
}eldriver_uart_handle_t;

void eldriver_uart1_init(eldriver_uart_handle_t *handle);
uint8_t eldriver_uart1_write(eldriver_uart_handle_t *handle, const uint8_t* data, uint8_t len);
uint16_t eldriver_uart1_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len);
elcore_rstream_stats_t eldriver_uart1_rx_stats(eldriver_uart_handle_t *handle);
elcore_rstream_stats_t eldriver_uart1_tx_stats(eldriver_uart_handle_t *handle);
void eldriver_uart1_resetStats(eldriver_uart_handle_t *handle);


#ifdef __cplusplus
}
#endif