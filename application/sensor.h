#pragma once
#include "eldriver/eldriver_conf.h"
#include "eldriver/eldriver_hall.h"
#include <cstdint>

#ifndef ELDRIVER_HALL1_ENABLED
#define ELDRIVER_BEMFZC_ENABLED
#endif

constexpr float BEMF_THRESHOLD_LOW = 0.1f;
constexpr float BEMF_THRESHOLD_HIGH = 0.3f;
constexpr uint32_t COMMUTATION_PHASE_DELAY = 0;

class BemfZc
{
public:
    int32_t threshold_low{};
    int32_t threshold_high{};
    int32_t elec_angle_q31_{};
    uint32_t last_zc_tick{};
    uint16_t tick_freq{};
    float elec_speed_{};
    float phase_delay_{};
    bool state{};

    void init(float threshold_low_V, float threshold_high_V, float phase_delay, uint16_t tick_freq)
    {
        threshold_high = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_high_V);
        threshold_low = ELDRIVER_MC3P_FLOAT_TO_VS(threshold_low_V);
        this->phase_delay_ = phase_delay;
        last_zc_tick = 0;
        this->tick_freq = tick_freq;
        eldriver_comDelay_init();
        state = false;
    }
    void update(int32_t bemf, uint32_t ticks, int8_t dir)
    {
        bool rising = (bemf > threshold_high && state == false);
        bool falling = (bemf < threshold_low && state == true);

        if (rising || falling)
        {
            if (rising)
            {
                state = true;
            }
            else
            {
                state = false;
            }
            elec_angle_q31_ += dir * INT32_MAX / 6;
            elec_speed_ = ((tick_freq / (ticks - last_zc_tick)) / 6) * 2 * M_PI;
            uint32_t delay_us = ((phase_delay_ + M_PI / 6) / elec_speed_) * 1e6;
            last_zc_tick = ticks;
            eldriver_comDelay_setComDelay_uS(delay_us);
        }
    }
    void takeover(void (*cb)(void))
    {
        eldriver_comDelay_setComCallback(cb);
    }

    int32_t elec_angle_q31_value() const
    {
        return elec_angle_q31_;
    }

    float elec_speed_value() const
    {
        return elec_speed_;
    }

    void reset()
    {
        state = false;
        elec_angle_q31_ = 0;
    }

    void init(uint16_t tick_freq) { init(BEMF_THRESHOLD_LOW, BEMF_THRESHOLD_HIGH, COMMUTATION_PHASE_DELAY, tick_freq); }
    float elec_speed() const { return elec_speed_value(); }
    int32_t elec_angle_q31() const { return elec_angle_q31_value(); }
    void set_com_delay_us(uint32_t delay_uS) { eldriver_comDelay_setComDelay_uS(delay_uS); }
    void set_com_callback(void (*callback)(void)) { eldriver_comDelay_setComCallback(callback); }
};

class HallSensor
{
public:
    void init(uint16_t tick_freq)
    {
#ifdef ELDRIVER_HALL1_ENABLED
        (void)tick_freq;
        eldriver_hall1_init();
#else
        (void)tick_freq;
#endif
    }

    void update(int32_t bemf_q31, uint32_t ticks, int8_t dir)
    {
#ifdef ELDRIVER_HALL1_ENABLED
        (void)bemf_q31;
        (void)ticks;
        (void)dir;
#else
        (void)bemf_q31;
        (void)ticks;
        (void)dir;
#endif
    }

    float elec_speed() const
    {
#ifdef ELDRIVER_HALL1_ENABLED
        return eldriver_hall1_elec_speed();
#else
        return 0.0f;
#endif
    }

    int32_t elec_angle_q31() const
    {
#ifdef ELDRIVER_HALL1_ENABLED
        return eldriver_hall1_elec_angle_q31();
#else
        return 0;
#endif
    }

    void set_com_delay_us(uint32_t delay_uS)
    {
#ifdef ELDRIVER_HALL1_ENABLED
        eldriver_hall1_setComDelay_uS(delay_uS);
#else
        (void)delay_uS;
#endif
    }

    void set_com_callback(void (*callback)(void))
    {
#ifdef ELDRIVER_HALL1_ENABLED
        eldriver_hall1_setComCallback(callback);
#else
        (void)callback;
#endif
    }
};

template <typename Impl>
class PosSensorT
{
public:
    void init(uint16_t tick_freq) { impl_.init(tick_freq); }
    void update(int32_t bemf_q31, uint32_t ticks, int8_t dir) { impl_.update(bemf_q31, ticks, dir); }
    float elec_speed() const { return impl_.elec_speed(); }
    int32_t elec_angle_q31() const { return impl_.elec_angle_q31(); }
    void set_com_delay_us(uint32_t delay_uS) { impl_.set_com_delay_us(delay_uS); }
    void set_com_callback(void (*callback)(void)) { impl_.set_com_callback(callback); }

    Impl &impl() { return impl_; }
    const Impl &impl() const { return impl_; }

private:
    Impl impl_{};
};

#ifdef ELDRIVER_HALL1_ENABLED
using PosSensor = PosSensorT<HallSensor>;
#else
using PosSensor = PosSensorT<BemfZc>;
#endif
