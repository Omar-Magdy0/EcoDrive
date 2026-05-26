/**
 * @file    eldriver_usbcdc.h
 * @author  Carol Nasser (adapted for STM32F1)
 * @brief   USB CDC (Communication Device Class) Driver.
 * @details Implements a Virtual COM Port (VCP) interface for the STM32F1.
 */

#pragma once

#include <stdint.h>
#include "elcore.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration for UART handle */
typedef struct eldriver_uart_handle eldriver_uart_handle_t;

/**
 * @brief  Initializes the USB CDC peripheral.
 * @param  handle: UART handle instance to manage USB streams.
 */
void eldriver_usbcdc_init(eldriver_uart_handle_t *handle);

/**
 * @brief  Sends data over USB CDC.
 * @param  handle: USB CDC handle.
 * @param  data: Pointer to data buffer.
 * @param  len: Length of data to send.
 * @retval uint8_t: Status or bytes written.
 */
uint8_t eldriver_usbcdc_write(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len);

/**
 * @brief  Receives data from USB CDC.
 * @param  handle: USB CDC handle.
 * @param  data: Pointer to buffer for incoming data.
 * @param  len: Maximum bytes to read.
 * @retval uint16_t: Actual bytes read.
 */
uint16_t eldriver_usbcdc_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len);

/* Statistics structure */
/*
typedef struct {
    uint32_t bytes_received;
    uint32_t bytes_sent;
    uint32_t errors;
} elcore_rstream_stats_t;
*/
/** @brief Returns USB CDC receive stream statistics. */
elcore_rstream_stats_t eldriver_usbcdc_rx_stats(eldriver_uart_handle_t *handle);

/** @brief Returns USB CDC transmit stream statistics. */
elcore_rstream_stats_t eldriver_usbcdc_tx_stats(eldriver_uart_handle_t *handle);

/** @brief Resets all USB CDC traffic statistics. */
void eldriver_usbcdc_resetStats(eldriver_uart_handle_t *handle);

#ifdef __cplusplus
}
#endif
