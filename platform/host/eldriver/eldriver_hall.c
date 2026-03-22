#include "eldriver_hall.h"
#include "eldriver_conf.h"


#define CLK_FREQ 1000000  // 1 MHz = 1us resolution
#define CLK_TO_US(clk) ((clk) * 1000000ULL / CLK_FREQ)
#define US_TO_CLK(us)(us * (CLK_FREQ / 1000000))

void(*commutation_callback)();
volatile static uint8_t hall1_value = 000;
volatile static uint32_t hall1_period_uS = 0;

#define HALL_GPIOInit(port, pin)do{\
\
}while(0)


uint8_t hall1_gpioRead()
{
    return 0;
}


void eldriver_hall1_init()
{   

}



uint32_t last_t = 0;
void eldriver_hall1_setComDelay_uS(uint32_t COM_delay_uS)
{

}

// Set your commutation callback function
void eldriver_hall1_setComCallback(void (*callback)(void))
{

}

float eldriver_hall1_elec_speed(){

}

int32_t eldriver_hall1_elec_angle_q31(){

}

uint8_t eldriver_hall1_read(){
    
}

void eldriver_comDelay_init()
{

}

void eldriver_comDelay_setComDelay_uS(uint32_t delay_uS)
{

}

void eldriver_comDelay_setComCallback(void (*callback)(void))
{

}

// Timer 2 Interrupt Handler - THIS IS YOUR COMMUTATION EVENT!
void TIM2_IRQHandler(void)
{

}

