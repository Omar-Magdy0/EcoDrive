#pragma once

#include <stdint.h>
#include <assert.h>
#include "../el/ring.h"

#define CODEC_READER

#ifdef CODEC_READER
#include <vector>
#endif

namespace daq::codec
{
    constexpr uint8_t CONTROL_RESERVE = 16; // Escape + Control Byte + 14 control bytes possible
    constexpr uint8_t CONTROL_FIRST = 248;
    constexpr uint8_t VIRTUAL_BITS_START = 240;
    constexpr uint8_t VIRTUAL_BITS_END = 247;

    enum class Escape : uint8_t
    {
        LARGE16 = 0xFE,
    };

    enum class Err : uint8_t
    {
        OK,
        RESYNCDOD_BAD,
    };

    enum class Flags : uint8_t
    {
        BACKPRESSURE = 1 << 0,
    };

    enum class ControlCode : uint8_t
    {
        RESYNCDOD = 0xFF,
    };

    struct ChannelState
    {
        int16_t dod_state[2]; // Sample, Delta
    };

    struct ControlMark
    {
        uint16_t idx;
        uint8_t size;
    };

    enum class StreamType : uint8_t
    {
        CONTROL = 0,
        SAMPLE = 1
    };

    class Writer
    {
        friend UnitTest;
        ChannelState *ccs;
        el::ring_fast<uint8_t> stream;
        el::ring_fast<ControlMark> marks;

        uint32_t &sample_counter;
        uint8_t samples_till_sync = 0;
        uint8_t channels;
        uint8_t sync_period = 64;
        uint8_t reserve;
        uint8_t flags;
        void internal_flush()
        {
            ControlMark info;
            info.idx = stream.head();
            stream.pushFast((uint8_t)ControlCode::RESYNCDOD); // We have reserve bytes in the tank at least, Push Control Bytes, handle escape, change compression to lossy
            info.size = sizeof(uint32_t) + sizeof(ccs->dod_state) * channels + 1;
            marks.push(info);
            stream.pushFast(sample_counter >> 24);
            stream.pushFast(sample_counter >> 16);
            stream.pushFast(sample_counter >> 8);
            stream.pushFast(sample_counter);
            for (int i = 0; i < channels; i++)
            {
                stream.pushFast(ccs[i].dod_state[0] >> 8);
                stream.pushFast(ccs[i].dod_state[0]);
                stream.pushFast(ccs[i].dod_state[1] >> 8);
                stream.pushFast(ccs[i].dod_state[1]);
            }
            samples_till_sync = sync_period;
        }

    public:
        Writer(uint8_t *stream_buf, uint16_t stream_buf_size, ControlMark *mark_buf, uint16_t mark_buf_size, ChannelState *ccs_, uint8_t channels_, uint32_t &_sample_counter, uint8_t _sync_period) : sample_counter(_sample_counter), stream(stream_buf, stream_buf_size), marks(mark_buf, mark_buf_size)
        {
            sync_period = _sync_period;
            ccs = ccs_;
            reserve = (uint8_t)CONTROL_RESERVE * channels;
            channels = channels_;
            reset();
            bool stream_valid = stream.isPowerOfTwo(stream_buf_size);
            bool mark_valid = stream.isPowerOfTwo(mark_buf_size);
            assert(stream_valid && mark_valid);
        }

        void reset()
        {
            stream.clear();

            samples_till_sync = 0;
            sample_counter = 0;
            flags = 0;
        }
        void reset_state(ChannelState *ccs_)
        {
            for (int i = 0; i < channels; i++)
                ccs[i] = ccs_[i];
        }
        inline bool pushSample(int16_t *samples)
        {
            bool write = true;
            bool success = true;
            if (flags & (uint8_t)Flags::BACKPRESSURE)
            {
                write = false;
                success = false;
            }
            if (success && stream.free() <= reserve)
            { // Minimum free allowable , keeping margin for control and similar (Put marker and similar set backpressure flag in stream, etc)
                flags |= (uint8_t)Flags::BACKPRESSURE;
                samples_till_sync = 0;
                success = false;
                write = false;
            }
            if (success && samples_till_sync == 0) // Resync
            {
                internal_flush();
                write = false;
            }
            for (int i = 0; i < channels; i++)
            {
                int16_t dod_val = dod_encode(i, samples[i]); // First of all get dod_encode residuals (decollerator)....
                uint8_t sign = (uint16_t)dod_val >> 15;
                uint16_t mag = sign ? -dod_val : dod_val;
                if (!write)
                    continue;
                if (mag < (VIRTUAL_BITS_START / 2))
                {
                    stream.pushFast(mag << 1 | sign); // zigzag
                }
                else
                {
                    uint16_t zz = mag << 1 | sign;
                    uint8_t high_bits = zz >> 8;
                    if (high_bits < 8)
                    {
                        stream.pushFast(VIRTUAL_BITS_START + high_bits); // high byte value
                        stream.pushFast(zz);                             // low byte
                    }
                    else
                    {
                        stream.pushFast((uint8_t)Escape::LARGE16);
                        stream.pushFast(zz >> 8); // high byte
                        stream.pushFast(zz);      // low byte
                    }
                }
            }
            if (write)
                samples_till_sync--;
            return success;
        }

        inline int16_t dod_encode(uint8_t channel_, int16_t sample)
        {
            int16_t delta = sample - ccs[channel_].dod_state[0];
            int16_t dod = delta - ccs[channel_].dod_state[1];
            ccs[channel_].dod_state[0] = sample;
            ccs[channel_].dod_state[1] = delta;
            return dod;
        }
        bool flush()
        {
            uint16_t _free = stream.free();
            if (_free <= (uint8_t)CONTROL_RESERVE)
                return false;
            samples_till_sync = 0;
            return true;
        }

        inline uint16_t readNext(uint8_t *buffer, uint16_t buffer_len, StreamType &type)
        {
            // No entropy compression for now, to be added later
            ControlMark next_mark;
            bool has_mark = marks.peek(next_mark);
            uint16_t dist_to_mark;
            uint16_t bytes_to_read;
            if (has_mark)
            {
                stream.distanceFromTail(next_mark.idx, dist_to_mark);
                if (dist_to_mark > 0)
                {
                    bytes_to_read = dist_to_mark;
                    type = StreamType::SAMPLE;
                }
                else
                {
                    bytes_to_read = next_mark.size;
                    type = StreamType::CONTROL;
                }
            }
            else
            {
                bytes_to_read = stream.count();
                type = StreamType::SAMPLE;
            }
            if (type == StreamType::CONTROL && bytes_to_read > buffer_len)
                return 0;
            if (bytes_to_read == 0)
                return 0;
            if (bytes_to_read > buffer_len)
                bytes_to_read = buffer_len;
            uint8_t *r1, *r2;
            uint16_t c1, c2;
            if (!stream.peekRead(&r1, &c1, &r2, &c2))
                return 0;
            uint16_t copied = 0;

            // First contiguous region
            uint16_t n = (c1 < bytes_to_read) ? c1 : bytes_to_read;
            memcpy(buffer, r1, n);
            copied += n;

            // Second contiguous region (if needed)
            if (copied < bytes_to_read && r2 != nullptr)
            {
                uint16_t rem = bytes_to_read - copied;
                uint16_t n2 = (c2 < rem) ? c2 : rem;

                memcpy(buffer + copied, r2, n2);
                copied += n2;
            }
            stream.releaseRead(copied);
            if (type == StreamType::CONTROL)
            {
                ControlMark dummy;
                marks.pop(dummy);
            }
            if (!stream.count())
                flags &= ~(uint8_t)Flags::BACKPRESSURE;
            return bytes_to_read;
        }
    };

#ifdef CODEC_READER

    enum class ReaderState
    {
        NONE,
        DOD,
        LARGEDOD,
        VIRTUALBITS
    };

    class Reader
    {
        uint8_t *buf;
        int16_t *temp_buf;
        int16_t *dod_buf;
        uint8_t current_channel;
        uint16_t buf_len;
        uint16_t buf_idx;
        ChannelState *ccs;
        uint8_t channels;
        ReaderState state;
        bool in_sync = false;

    public:
        Reader(ChannelState *ccs_, uint8_t channels_)
        {
            ccs = ccs_;
            channels = channels_;
            current_channel = 0;
            state = ReaderState::NONE;
        }
        inline Err decode(uint8_t *buffer, uint16_t buffer_len, StreamType type, el::ring_fast<int16_t> *sample_fifo)
        {
            buf_idx = 0;
            buf = buffer;
            if (buffer_len == 0)
                return Err::OK;
            uint8_t byte;
            if (type == StreamType::CONTROL)
            {
                switch (buf[buf_idx++])
                {
                case (uint8_t)ControlCode::RESYNCDOD:
                {
                    uint8_t expected_size = sizeof(uint32_t) + sizeof(ccs->dod_state) * channels + 1;
                    if (buffer_len != expected_size)
                        return Err::RESYNCDOD_BAD;
                    uint32_t counter = (buf[buf_idx] << 24) | (buf[buf_idx + 1] << 16) | (buf[buf_idx + 2] << 8) | (buf[buf_idx + 3]);
                    buf_idx += 4;
                    for(int i = 0; i < channels; i++)
                    {
                        ccs[i].dod_state[0] = (buf[buf_idx] << 8) | (buf[buf_idx + 1]);
                        ccs[i].dod_state[1] = (buf[buf_idx + 2] << 8) | (buf[buf_idx + 3]);
                        buf_idx += 4;
                    }
                    break;
                }

                default:
                    break;
                }
            }
            else
            {//A Sample stream full of DOD's
                

            }
        }
        inline int16_t dod_decode(uint8_t channel_, int16_t dod)
        {
            int16_t delta = dod + ccs[channel_].dod_state[1];
            int16_t sample = delta + ccs[channel_].dod_state[0];
            ccs[channel_].dod_state[0] = sample;
            ccs[channel_].dod_state[1] = delta;
            return sample;
        }
    };
};

#endif