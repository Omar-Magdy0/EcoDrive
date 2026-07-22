#pragma once
#include <stdint.h>
#include <etl/array.h>

template <typename AxisT, typename ValueT>
class Lut1D
{
    etl::span<const AxisT> x;
    etl::span<const ValueT> a;
    Lut() = delete;
    size_t findInterval(AxisT xval) const
    {
        // Clamp below range
        if (xval <= x[0])
            return 0;

        // Clamp above range
        if (xval >= x[x.size() - 1])
            return x.size() - 2;

        // Linear search
        // (replace with binary search if LUTs become large)
        for (size_t i = 0; i < x.size() - 1; ++i)
        {
            if (xval < x[i + 1])
                return i;
        }

        return x.size() - 2;
    }

public:
    Lut(etl::span<const AxisT> x_, etl::span<const ValueT> a_) : x(x_), a(a_)
    {
        assert(a.size() >= 2);
        assert(a.size() == x.size());
    }
    ValueT lookupNearest(AxisT xval) const
    {
        size_t i = findInterval(xval);

        AxisT d0 = xval - x[i];
        AxisT d1 = x[i + 1] - xval;

        return (d0 <= d1) ? a[i] : a[i + 1];
    }

    ValueT lookupLinear(AxisT xval) const
    {
        size_t i = findInterval(xval);

        const AxisT x0 = x[i];
        const AxisT x1 = x[i + 1];

        const ValueT y0 = a[i];
        const ValueT y1 = a[i + 1];

        float t = float(xval - x0) / float(x1 - x0);

        return y0 + (y1 - y0) * t;
    }
};

template <typename AxisT, typename ValueT>
class UniformLut1D
{
    AxisT origin;
    AxisT step;
    etl::span<const ValueT> a;

    UniformLut1D() = delete;

public:
    UniformLut1D(AxisT origin_,
                 AxisT step_,
                 etl::span<const ValueT> a_)
        : origin(origin_), step(step_), a(a_)
    {
        assert(step > 0);
        assert(a.size() >= 2);
    }

    ValueT lookupNearest(AxisT xval) const
    {
        float f = float(xval - origin) / float(step);

        int32_t idx = int32_t(f + 0.5f);

        if (idx < 0)
            idx = 0;
        else if (idx >= int32_t(a.size()))
            idx = a.size() - 1;

        return a[idx];
    }

    ValueT lookupLinear(AxisT xval) const
    {
        float f = float(xval - origin) / float(step);

        int32_t idx = int32_t(f);

        if (idx < 0)
            idx = 0;
        else if (idx >= int32_t(a.size()) - 1)
            idx = a.size() - 2;

        float t = f - idx;

        const ValueT y0 = a[idx];
        const ValueT y1 = a[idx + 1];

        return y0 + (y1 - y0) * t;
    }
};