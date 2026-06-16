/**
 * @file    eldriver_uart.h
 * @author  Carol Nasser (adapted for STM32F1)
 * @brief   Header file for UART/Communication Peripheral Driver.
 * @details This driver manages telemetry data transmission to the vehicle dashboard
 * and handles incoming control commands.
 */

#pragma once

#include <stdint.h>

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
typedef struct eldriver_uart_handle {
    eldriver_uart_config_t config;
    eldriver_uart_status_t status;
}eldriver_uart_handle_t;

/**
 * @brief  Initializes UART1 instance.
 * @param  handle: Pointer to the UART handle.
 */
void eldriver_uart1_init(eldriver_uart_handle_t *handle);

#ifdef __cplusplus
}
#endif