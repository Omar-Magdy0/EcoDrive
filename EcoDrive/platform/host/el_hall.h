#ifndef TIM2_UTIL_H
#define TIM2_UTIL_H
#include <stdint.h>
#include "eld_conf.h"
#include <math.h>
#include "sil.h"

#ifdef __cplusplus
extern "C"{
#endif

void el_hall1_init();
void el_hall1_setComDelay_uS(uint32_t COM_delay_uS);
void el_hall1_setComCallback(void (*callback)(void));
float el_hall1_elec_speed();
uint8_t el_hall1_read();

void el_comDelay_init();
void el_comDelay_setComDelay_uS(uint32_t COM_delay_uS);
void el_comDelay_setComCallback(void (*callback)(void));
void sil_hall_update();

#ifdef __cplusplus
}
#endif

#endif
