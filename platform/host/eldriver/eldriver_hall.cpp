#include "eldriver_hall.h"
#include "eldriver_conf.h"


#define CLK_FREQ 1000000  // 1 MHz = 1us resolution
#define CLK_TO_US(clk) ((clk) * 1000000ULL / CLK_FREQ)
#define US_TO_CLK(us)(us * (CLK_FREQ / 1000000))

// shared between hall1 and comdelay modules
void(*commutation_callback)();
static bool comDelayReady = false;
static bool comTriggered = false;  
static double trigger_time = 0;
static double com_delay_us = 0;
bool usingHall = false;
uint8_t last_hall1_value = 0;
volatile static uint8_t hall1_value = 0;
volatile static uint32_t hall1_period_uS = 0;

#define HALL_GPIOInit(port, pin)do{\
\
}while(0)


uint8_t hall1_gpioRead()
{
    double theta_e = SIL_HALL_OFFSET + sil.state.theta * sil.param.motor_pp + sil.param.motor_rotorOffset;
    theta_e = fmod(theta_e, 2*M_PI);
    if (theta_e < 0) theta_e += 2*M_PI;

    return SIL_HALL_TABLE_PI_3[(int)(theta_e / (2*M_PI/6))];
}

void eldriver_hall1_init()
{   
    usingHall = true;
    comDelayReady = false;
    trigger_time = 0;
    comTriggered = false;
    last_hall1_value = hall1_gpioRead();
}

uint32_t last_t = 0;
void eldriver_hall1_setComDelay_uS(uint32_t delay_uS)
{
    com_delay_us = delay_uS;
}

// Set your commutation callback function
void eldriver_hall1_setComCallback(void (*callback)(void))
{
    commutation_callback = callback;
}

float eldriver_hall1_elec_speed(){
    return sil.state.omega * sil.param.motor_pp;
}

uint8_t eldriver_hall1_read(){
    return hall1_value;
}

void eldriver_comDelay_init()
{
    usingHall = false;
    comDelayReady = false;
    trigger_time = 0;
    comTriggered = false;
}

void eldriver_comDelay_setComDelay_uS(uint32_t delay_uS)
{
    com_delay_us = delay_uS;
    trigger_time = sil.state.time;
    comTriggered = true;
    comDelayReady = false;
}

void eldriver_comDelay_setComCallback(void (*callback)(void))
{
    commutation_callback = callback;
}

void sil_hall_update()
{
    if(usingHall){
        hall1_value = hall1_gpioRead();   
        if(hall1_value != last_hall1_value){
            last_hall1_value = hall1_value;
            comTriggered = true;
            trigger_time = sil.state.time;
        }
    }

    if(comTriggered){
        if((sil.state.time - trigger_time) > com_delay_us){
            comDelayReady = true;
            comTriggered = false;
        }
    }

    if(commutation_callback != NULL && comDelayReady){
        commutation_callback();
        comDelayReady = false;
    }
}

void eldriver_hall1_reset()
{

}
