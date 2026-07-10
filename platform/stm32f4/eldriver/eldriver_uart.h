/**
 * @file    eldriver_uart.h
 * @author  Carol Nasser
 * @brief   Header file for UART/Communication Peripheral Driver.
 * @details This driver manages telemetry data transmission to the vehicle dashboard
 * and handles incoming control commands.
 */

#pragma once
#include "elcore_rstream.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART Configuration structure.
 */
typedef struct {
    uint32_t baudrate;      /**< Baudrate for UART communication */
    uint8_t data_bits;     /**< Number of data bits */
    uint8_t stop_bits;     /**< Number of stop bits */
    uint8_t parity;        /**< Parity configuration */
    uint8_t flow_control;  /**< Hardware flow control setting */
}eldriver_uart_config_t;

/**
 * @brief UART Status structure to track transmission and errors.
 */
typedef struct{
    uint8_t tx_busy;       /**< Transmitter busy flag */
    uint8_t rx_busy;       /**< Receiver busy flag */
    uint8_t rx_overflow;   /**< Receive overflow error flag */
    uint8_t tx_overflow;   /**< Transmit overflow error flag */
}eldriver_uart_status_t;

/**
 * @brief UART Handle structure containing config and status.
 */
typedef struct{
    eldriver_uart_config_t config;
    eldriver_uart_status_t status;
}eldriver_uart_handle_t;

/**
 * @brief  Initializes UART1 instance.
 * @param  handle: Pointer to the UART handle.
 */
void eldriver_uart1_init(eldriver_uart_handle_t *handle);

/**
 * @brief  Writes data to UART1.
 * @param  handle: Pointer to the UART handle.
 * @param  data: Data buffer to write.
 * @param  len: Length of data.
 * @retval uint8_t: Status or bytes written.
 */
uint8_t eldriver_uart1_write(eldriver_uart_handle_t *handle, const uint8_t* data, uint8_t len);

/**
 * @brief  Reads data from UART1.
 * @param  handle: Pointer to the UART handle.
 * @param  data: Buffer to store read data.
 * @param  len: Length of data to read.
 * @retval uint16_t: Bytes read.
 */
uint16_t eldriver_uart1_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len);

/**
 * @brief  Returns RX stream statistics.
 * @param  handle: Pointer to the UART handle.
 * @retval elcore_rstream_stats_t: Statistics structure.
 */
elcore_rstream_stats_t eldriver_uart1_rx_stats(eldriver_uart_handle_t *handle);

/**
 * @brief  Returns TX stream statistics.
 * @param  handle: Pointer to the UART handle.
 * @retval elcore_rstream_stats_t: Statistics structure.
 */
elcore_rstream_stats_t eldriver_uart1_tx_stats(eldriver_uart_handle_t *handle);

/**
 * @brief  Resets UART1 statistics.
 * @param  handle: Pointer to the UART handle.
 */
void eldriver_uart1_resetStats(eldriver_uart_handle_t *handle);


#ifdef __cplusplus
}
#endif