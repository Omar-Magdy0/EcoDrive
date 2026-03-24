#include "sensor.h"
#include "eldriver/eldriver_mc3p.h"
#include "eldriver/eldriver_core.h"
#include "PmsmController.h"

void BemfZc::init(float threshold_low_V, float threshold_high_V, float phase_delay, uint16_t tick_freq)
{
    threshold_high = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_high_V);
    threshold_low = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_low_V);
    this->phase_delay_ = phase_delay;
    last_zc_tick = 0;
    this->tick_freq = tick_freq;
    eldriver_comDelay_init();
    state = false;
}

void BemfZc::update(int32_t bemf, uint32_t ticks, int8_t dir)
{
    bool rising  = (bemf > threshold_high && state == false);
    bool falling = (bemf < threshold_low && state == true);
    
    if(rising || falling){
        if(rising){state = true;}else{state = false;}
        elec_angle_q31_ += dir * INT32_MAX/6;
        elec_speed_ = ((tick_freq/(ticks - last_zc_tick))/6) * 2 * M_PI;
        uint32_t delay_us = ((phase_delay_ + M_PI/6)/ elec_speed_)*1e6;
        last_zc_tick = ticks;
        eldriver_comDelay_setComDelay_uS(delay_us);
    }
}

void BemfZc::takeover(void(*cb)(void))
{
    eldriver_comDelay_setComCallback(cb);
}

int32_t BemfZc::elec_angle_q31_value() const
{
    return elec_angle_q31_;
}

float BemfZc::elec_speed_value() const
{
    return elec_speed_ + 100;
}

void BemfZc::reset()
{
    state = false;
    elec_angle_q31_ = 0;   
}

void HallSensor::init(uint16_t tick_freq)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)tick_freq;
    eldriver_hall1_init();
#else
    (void)tick_freq;
#endif
}

void HallSensor::update(int32_t bemf_q31, uint32_t ticks, int8_t dir)
{
#ifdef ELDRIVER_HALL1_ENABLED
    (void)bemf_q31;
    (void)ticks;
    (void)dir;
    hall1_update();
#else
    (void)bemf_q31;
    (void)ticks;
    (void)dir;
#endif
}

float HallSensor::elec_speed() const
{
#ifdef ELDRIVER_HALL1_ENABLED
    return eldriver_hall1_elec_speed();
#else
    return 0.0f;
#endif
}

int32_t HallSensor::elec_angle_q31() const
{
#ifdef ELDRIVER_HALL1_ENABLED
    return eldriver_hall1_elec_angle_q31();
#else
    return 0;
#endif
}

void HallSensor::set_com_delay_us(uint32_t delay_uS)
{
#ifdef ELDRIVER_HALL1_ENABLED
    eldriver_hall1_setComDelay_uS(delay_uS);
#else
    (void)delay_uS;
#endif
}

void HallSensor::set_com_callback(void (*callback)(void))
{
#ifdef ELDRIVER_HALL1_ENABLED
    eldriver_hall1_setComCallback(callback);
#else
    (void)callback;
#endif
}

void hall1_update()
{
}
