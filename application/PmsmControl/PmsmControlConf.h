#pragma once

// General Pmsm Config
namespace PmsmControlConf
{
//PmsmControl General
static constexpr float PWM_MAX_DUTY = 0.95;   // Hardware minimum
static constexpr uint32_t MIN_PWM_FREQUENCY = 8000;
static constexpr uint32_t MAX_PWM_FREQUENCY = 40000;
static constexpr uint32_t MIN_DEADTIME_NS = 1000;   // Hardware minimum
static constexpr uint32_t MAX_DEADTIME_NS = 2000; // Hardware maximum
static constexpr float MC3P_SYNC_SCALE[7][2] = 
{
    {30, 0},
    {15, 0},
    {20, 0},
    {15, 0},
    {12, 0},
    {50, 0},
    {12, 0}
};

//SCOMM
static constexpr uint8_t OVERSAMPLE_BITS = 15;
static constexpr uint16_t ESETTLE_MIN_TICKS = 20000;
};