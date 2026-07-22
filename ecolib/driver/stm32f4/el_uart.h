/**
 * @file    el_uart.h
 * @author  Carol Nasser
 * @brief   Header file for UART/Communication Peripheral Driver.
 * @details This driver manages telemetry data transmission to the vehicle dashboard
 * and handles incoming control commands.
 */

#pragma once
#include "el/el_ring.h"

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
}el_uart_config_t;

/**
 * @brief UART Status structure to track transmission and errors.
 */
typedef struct{
    uint8_t tx_busy;       /**< Transmitter busy flag */
    uint8_t rx_busy;       /**< Receiver busy flag */
    uint8_t rx_overflow;   /**< Receive overflow error flag */
    uint8_t tx_overflow;   /**< Transmit overflow error flag */
}el_uart_status_t;

/**
 * @brief UART Handle structure containing config and status.
 */
typedef struct{
    el_uart_config_t config;
    el_uart_status_t status;
}el_uart_handle_t;

/**
 * @brief  Initializes UART1 instance.
 * @param  handle: Pointer to the UART handle.
 */
void el_uart1_init(el_uart_handle_t *handle);

/**
 * @brief  Writes data to UART1.
 * @param  handle: Pointer to the UART handle.
 * @param  data: Data buffer to write.
 * @param  len: Length of data.
 * @retval uint8_t: Status or bytes written.
 */
uint8_t el_uart1_write(el_uart_handle_t *handle, const uint8_t* data, uint8_t len);

/**
 * @brief  Reads data from UART1.
 * @param  handle: Pointer to the UART handle.
 * @param  data: Buffer to store read data.
 * @param  len: Length of data to read.
 * @retval uint16_t: Bytes read.
 */
uint16_t el_uart1_read(el_uart_handle_t *handle, uint8_t* data, uint8_t len);

/**
 * @brief  Resets UART1 statistics.
 * @param  handle: Pointer to the UART handle.
 */
void el_uart1_resetStats(el_uart_handle_t *handle);


#ifdef __cplusplus
}
#endif