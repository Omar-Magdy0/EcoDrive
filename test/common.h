#pragma once

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cstring>
#include <string>
#include "el/ring.h"
#include <vector>

class UnitTest
{
public:
    template <typename T>
    static std::string toString(const std::vector<T> &vec,
                                const char *name = nullptr)
    {
        std::ostringstream ss;
        if (name)
            ss << name << ": ";

        ss << "[";

        for (size_t i = 0; i < vec.size(); i++)
        {
            ss << toString(vec[i]);

            if (i + 1 != vec.size())
                ss << ", ";
        }

        ss << "]\n";
        return ss.str();
    }

    static std::string toStringHex(const uint8_t* data, size_t length)
    {
        std::ostringstream oss;

        for (size_t i = 0; i < length; ++i)
        {
            if (i)
                oss << ' ';

            oss << std::hex
                << std::uppercase
                << std::setw(2)
                << std::setfill('0')
                << static_cast<unsigned>(data[i]);
        }

        return oss.str();
    }

    static std::string toStringHex(const std::vector<uint8_t> &vec,
                                   const char *name = nullptr)
    {
        std::ostringstream ss;
        if (name)
            ss << name << ": ";

        for (uint8_t b : vec)
        {
            ss
                << std::hex
                << std::uppercase
                << std::setw(2)
                << std::setfill('0')
                << unsigned(b)
                << ' ';
        }

        ss << std::dec << '\n';
        return ss.str();
    }

    static std::string toString(const daq::codec::ControlMark &mark)
    {
        std::ostringstream ss;
        ss
            << "{ idx=" << mark.idx
            << ", size=" << unsigned(mark.size)
            << " }";
        return ss.str();
    }

    static std::string toString(const daq::codec::Writer &codec)
    {
        std::ostringstream ss;

        ss << toStringHex(codec.stream, "codec_stream");
        ss << '\n';
        ss << toString(codec.marks, "codec_marks");

        return ss.str();
    }
    template <typename T>
    static std::string toString(const T &val)
    {
        std::ostringstream ss;
        ss << val;
        return ss.str();
    }

    template <typename T>
    static std::string toString(const el::ring_fast<T> &fifo,
                                const char *name = nullptr)
    {
        std::ostringstream ss;
        if (name)
            ss << name << '\n';

        ss
            << "Head : " << fifo.head() << '\n'
            << "Tail : " << fifo.tail() << '\n'
            << "Count: " << fifo.count() << '\n';

        ss << "[";

        uint16_t idx = fifo.tail();

        while (idx != fifo.head())
        {
            T value;
            fifo.peekRaw(value, idx);

            ss << toString(value);

            idx = (idx + 1) & fifo.mask_;

            if (idx != fifo.head())
                ss << ", ";
        }

        ss << "]\n";
        return ss.str();
    }

    template <typename T>
    static std::string toStringHex(const el::ring_fast<T> &fifo,
                                   const char *name = nullptr)
    {
        std::ostringstream ss;
        if (name)
            ss << name << '\n';

        uint16_t idx = fifo.tail_;

        while (idx != fifo.head_)
        {
            T value;
            fifo.peekRaw(value, idx);

            const uint8_t *bytes =
                reinterpret_cast<const uint8_t *>(&value);

            for (size_t i = 0; i < sizeof(T); i++)
            {
                ss
                    << std::hex
                    << std::uppercase
                    << std::setw(2)
                    << std::setfill('0')
                    << unsigned(bytes[i]);
            }

            ss << ", ";

            idx = (idx + 1) & fifo.mask_;
        }

        ss << std::dec << '\n';
        return ss.str();
    }

    template <typename T>
    static void Print(const T &val)
    {
        std::cout << toString(val);
    }

    template <typename T>
    static bool Compare(const std::vector<T> &vec1,
                        const std::vector<T> &vec2)
    {
        return vec1 == vec2;
    }

    template <typename T>
    static bool Compare(const T *buf1,
                        unsigned int size1,
                        const T *buf2,
                        unsigned int size2)
    {
        if (size1 != size2)
            return false;

        return std::equal(buf1, buf1 + size1, buf2);
    }
};
