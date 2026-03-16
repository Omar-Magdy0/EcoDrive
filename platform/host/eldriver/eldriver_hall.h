#ifndef TIM2_UTIL_H
#define TIM2_UTIL_H
#include <stdint.h>
#include "eldriver_conf.h"
#include <math.h>


void eldriver_hall1_init();
void eldriver_hall1_setComDelay_uS(uint32_t COM_delay_uS);
void eldriver_hall1_setComCallback(void (*callback)(void));
float eldriver_hall1_elec_speed();
int32_t eldriver_hall1_elec_angle_q31();
uint8_t eldriver_hall1_read();

void eldriver_comDelay_init();
void eldriver_comDelay_setComDelay_uS(uint32_t COM_delay_uS);
void eldriver_comDelay_setComCallback(void (*callback)(void));


#endif

