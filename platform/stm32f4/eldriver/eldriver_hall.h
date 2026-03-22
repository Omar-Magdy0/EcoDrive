/**
 * @file    eldriver_hall.h
 * @author  Carol Nasser
 * @brief   Hall Sensor and Commutation Delay Driver.
 * @details This driver manages motor position feedback through Hall sensors 
 * and handles commutation timing for trapezoidal control.
 */

#ifndef TIM2_UTIL_H
#define TIM2_UTIL_H
#include <stdint.h>
#include "eldriver_conf.h"
#include <math.h>

/** @brief Initializes the Hall sensor interface. */
void eldriver_hall1_init();

/** @brief Sets the commutation delay in microseconds. */
void eldriver_hall1_setComDelay_uS(uint32_t COM_delay_uS);

/** @brief Sets the callback function for commutation events. */
void eldriver_hall1_setComCallback(void (*callback)(void));

/** @brief Calculates and returns the electrical speed of the motor. */
float eldriver_hall1_elec_speed();

/** @brief Reads the current digital state of the Hall sensors. */
uint8_t eldriver_hall1_read();

/** @brief Returns the estimated electrical angle in Q31 format. */
int32_t eldriver_hall1_elec_angle_q31();

/** @brief Initializes the commutation delay timer. */
void eldriver_comDelay_init();

/** @brief Configures the specific delay for commutation logic. */
void eldriver_comDelay_setComDelay_uS(uint32_t COM_delay_uS);

/** @brief Registers the callback for delay-based commutation. */
void eldriver_comDelay_setComCallback(void (*callback)(void));

#endif