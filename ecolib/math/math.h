#pragma once

// Generic versions with configurable min/max
namespace el
{
    template <typename T>
    bool constexpr in_range(T c, T min, T max) { return (c <= max) && (c >= min); }
    template <typename T>
    bool constexpr nearly_equal(T a, T b, T diff) { return ((a - b) <= diff) && ((a - b) >= -diff); }

    static inline int increment_roll(int x, int min, int max)
    {
        return ((x) >= (max)) ? (min) : ((x) + 1);
    }
    static inline int decrement_roll(int x, int min, int max)
    {
        return ((x) <= (min)) ? (max) : ((x)-1);
    }
};
