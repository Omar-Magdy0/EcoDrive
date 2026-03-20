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


class BemfZc {
public:
    int32_t threshold_low{};
    int32_t threshold_high{};
    int32_t elec_angle_q31_{};
    uint32_t last_zc_tick{};
    uint16_t tick_freq{};
    float elec_speed_{};
    float phase_delay_{};
    bool state{};

    void init(float threshold_low_V, float threshold_high_V, float phase_delay, uint16_t tick_freq);
    void update(int32_t bemf, uint32_t ticks, int8_t dir);
    void takeover(void(*cb)(void));
    int32_t elec_angle_q31_value() const;
    float elec_speed_value() const;
    void reset();

    void init(uint16_t tick_freq) { init(BEMF_THRESHOLD_LOW, BEMF_THRESHOLD_HIGH, COMMUTATION_PHASE_DELAY, tick_freq); }
    float elec_speed() const { return elec_speed_value(); }
    int32_t elec_angle_q31() const { return elec_angle_q31_value(); }
    void set_com_delay_us(uint32_t delay_uS) { eldriver_comDelay_setComDelay_uS(delay_uS); }
    void set_com_callback(void (*callback)(void)) { eldriver_comDelay_setComCallback(callback); }
};

class HallSensor {
public:
    void init(uint16_t tick_freq);
    void update(int32_t bemf_q31, uint32_t ticks, int8_t dir);
    float elec_speed() const;
    int32_t elec_angle_q31() const;
    void set_com_delay_us(uint32_t delay_uS);
    void set_com_callback(void (*callback)(void));
};
void    hall1_update();
int32_t hall1_elec_angle_q31();

template <typename Impl>
class PosSensorT {
public:
    void init(uint16_t tick_freq) { impl_.init(tick_freq); }
    void update(int32_t bemf_q31, uint32_t ticks, int8_t dir) { impl_.update(bemf_q31, ticks, dir); }
    float elec_speed() const { return impl_.elec_speed(); }
    int32_t elec_angle_q31() const { return impl_.elec_angle_q31(); }
    void set_com_delay_us(uint32_t delay_uS) { impl_.set_com_delay_us(delay_uS); }
    void set_com_callback(void (*callback)(void)) { impl_.set_com_callback(callback); }

    Impl& impl() { return impl_; }
    const Impl& impl() const { return impl_; }

private:
    Impl impl_{};
};

#ifdef ELDRIVER_HALL1_ENABLED
using PosSensor = PosSensorT<HallSensor>;
#else
using PosSensor = PosSensorT<BemfZc>;
#endif
