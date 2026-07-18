#pragma once



#define el_SATURATE(x, min, max) (x < min ? min : (x > max ? max : x))
#define ELECFREQ_TO_MECHFREQ(freq, polepairs)(freq/polepairs)
#define MECHFREQ_TO_ELECFREQ(freq, polepairs)((freq*polepairs))
#define HZ_TO_RPM(hz)((hz*60))
#define RPM_TO_HZ(rpm)(rpm/60)


// SI Prefix Conversions
// Nano (10^-9)
#define UNIT_TO_MILLI(u)            ((u) * 1000)   // unit → m
#define UNIT_TO_MICRO(u)            ((u) * 1000000)
#define UNIT_TO_NANO(u)             ((u) * 1000000000)

#define MILLI_TO_UNIT(milli)        ((milli) / 1000)   // milli → unit
#define MICRO_TO_UNIT(micro)        ((micro) / 1000000)   // micro → unit
#define NANO_TO_UNIT(nano)          ((nano) / 1000000000)   // micro → unit

#define NANO_TO_MILLI(nano)         ((nano) / 1000000)

// Custom for your PWM context
#define TICKS_TO_NANOS(ticks, freq_Hz) (((ticks) * 1000000000) / (freq_Hz))
#define TICKS_TO_MICROS(ticks, freq_Hz) (((ticks) * 1000000) / (freq_Hz))
#define NANOS_TO_TICKS(ns, freq_Hz) (((ns) * (freq_Hz)) / 1000000000)
#define MICROS_TO_TICKS(us, freq_Hz) (((us) * (freq_Hz)) / 1000000)

#define IN_RANGE(c, min, max)((c<=max)&&(c>=min))
#define NEARLY_EQUAL(a, b, diff) (((a-b) <= diff) && ((a-b) >= -diff))

// Generic versions with configurable min/max
static inline int el_increment_roll(int x, int min, int max)
{
    return ((x) >= (max)) ? (min) : ((x) + 1);
}

static inline int el_decrement_roll(int x, int min, int max)
{   
    return ((x) <= (min)) ? (max) : ((x) - 1);
}

static inline float el_linearInterp(float *y_arr, float *x_arr, float x)
{
    return (y_arr[0] + ( (x - x_arr[0]) * ((y_arr[1] - y_arr[0])/(x_arr[1] - x_arr[0])) ) ); 
}


