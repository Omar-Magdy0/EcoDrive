#pragma once

#include <stdint.h>
#include <assert.h>
#include "../el/ring.h"
#include <etl/vector.h>
#include <etl/span.h>
class CodecTest;
namespace daq::codec
{
    class Reader;
    class Writer;

    constexpr uint8_t CONTROL_RESERVE = 16; // Escape + Control Byte + 14 control bytes possible
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
        NEED_SYNC,
        FAULT,
        BAD_BLOCK,
        BLOCK_FULL
    };

    enum class Flags : uint8_t
    {
        BACKPRESSURE = 1 << 0,
    };

    enum class ControlCode : uint8_t
    {
        RESYNCDOD = 0xFF,
        FLUSH = 0xFE,
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
        Control = 0,
        Sample = 1
    };
    
    class Writer
    {
        friend UnitTest;
        friend CodecTest;
        etl::span<ChannelState> ccs;
        uint32_t &sample_counter;
        el::ring_fast<uint8_t> stream;
        el::ring_fast<ControlMark> marks;
        uint8_t samples_till_sync = 0;
        uint8_t sync_period = 64;
        uint8_t reserve;
        uint8_t flags;
        Writer() = delete;
        void internal_resync()
        {
            ControlMark info;
            info.idx = stream.head();
            uint8_t channels = ccs.size();
            stream.pushFast((uint8_t)ControlCode::RESYNCDOD); // We have reserve bytes in the tank at least, Push Control Bytes, handle escape, change compression to lossy
            info.size = sizeof(uint32_t) + sizeof(int16_t) * 2 * channels + 1;
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
        inline int16_t dod_encode(uint8_t channel, int16_t sample)
        {
            int16_t delta = sample - ccs[channel].dod_state[0];
            int16_t dod = delta - ccs[channel].dod_state[1];
            ccs[channel].dod_state[0] = sample;
            ccs[channel].dod_state[1] = delta;
            return dod;
        }
    public:
        Writer(etl::span<uint8_t> stream_buf, etl::span<ControlMark> mark_buf, etl::span<ChannelState> ccs_, uint32_t &_sample_counter, uint8_t _sync_period) : sample_counter(_sample_counter), stream(stream_buf), marks(mark_buf)
        {
            sync_period = _sync_period;
            ccs = ccs_;
            reserve = (uint8_t)CONTROL_RESERVE * ccs.size();
            reset();
            bool stream_valid = stream.isPowerOfTwo(stream_buf.size());
            bool mark_valid = stream.isPowerOfTwo(mark_buf.size());
            assert(stream_valid && mark_valid);
        }
        void reset()
        {
            stream.clear();

            samples_till_sync = 0;
            sample_counter = 0;
            flags = 0;
        }
        bool reset_state(etl::span<ChannelState> ccs_)
        {
            if(ccs.size() != ccs_.size())return false;
            for (size_t i = 0; i < ccs.size(); i++)
                ccs[i] = ccs_[i];
            return true;
        }
        inline bool pushSample(int16_t *samples)
        {
            bool write = true;
            bool success = true;
            bool sync = false;
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
                write = false;
                sync = true;
            }
            for (size_t i = 0; i < ccs.size(); i++)
            {
                int16_t dod_val = dod_encode(i, samples[i]); // First of all get dod_encode residuals (decollerator)....
                uint8_t sign = (uint16_t)dod_val >> 15;
                uint16_t mag = sign ? -dod_val : dod_val;
                if (!write)
                    continue;
                if (mag < (VIRTUAL_BITS_START / 2))
                {
                    stream.pushFast((mag << 1) - sign); // zigzag
                }
                else
                {
                    uint16_t zz = (mag << 1) - sign;
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
            if (sync)
            {
                internal_resync();
            }
            if (write)
                samples_till_sync--;
            return success;
        }
        bool flush()
        {
            uint16_t _free = stream.free();
            if (_free <= (uint8_t)CONTROL_RESERVE)
                return false;
            ControlMark info;
            info.idx = stream.head();
            stream.pushFast((uint8_t)ControlCode::FLUSH); // We have reserve bytes in the tank at least, Push Control Bytes, handle escape, change compression to lossy
            info.size = 1;
            marks.push(info);
            samples_till_sync = 0;
            return true;
        }
        inline uint16_t readNext(etl::span<uint8_t> buf, StreamType &type)
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
                    type = StreamType::Sample;
                }
                else
                {
                    bytes_to_read = next_mark.size;
                    type = StreamType::Control;
                }
            }
            else
            {
                bytes_to_read = stream.size();
                type = StreamType::Sample;
            }
            if (type == StreamType::Control && (bytes_to_read > buf.size() || bytes_to_read == 0))
                return 0;
            if (bytes_to_read > buf.size())
                bytes_to_read = buf.size();
            uint8_t *r1, *r2;
            uint16_t c1, c2;
            if (!stream.read_reserve(&r1, &c1, &r2, &c2))
                return 0;
            uint16_t copied = 0;

            // First contiguous region
            uint16_t n = (c1 < bytes_to_read) ? c1 : bytes_to_read;
            memcpy(buf.data(), r1, n);
            copied += n;

            // Second contiguous region (if needed)
            if (copied < bytes_to_read && r2 != nullptr)
            {
                uint16_t rem = bytes_to_read - copied;
                uint16_t n2 = (c2 < rem) ? c2 : rem;

                memcpy(buf.data() + copied, r2, n2);
                copied += n2;
            }
            stream.read_commit(copied);
            if (type == StreamType::Control)
            {
                ControlMark dummy;
                marks.pop(dummy);
            }
            if (!stream.size())
                flags &= ~(uint8_t)Flags::BACKPRESSURE;
            return bytes_to_read;
        }
    };

    enum class ReaderState
    {
        Sample,
        DOD,
        LARGE16,
        VIRTUALBITS
    };

    class SampleBlock
    {
        friend Reader;

    private:
        bool ready = false;
        uint8_t channels;
        uint32_t counter = 0;
        etl::vector_ext<int16_t> samples;
        SampleBlock() = delete;
    public:
        bool isReady() { return ready; }
        uint8_t getChannels(){return channels;}
        SampleBlock(etl::span<int16_t> samples_buf, uint8_t channels_) : samples(samples_buf.data(), samples_buf.size())
        {
            channels = channels_;
        }
        etl::span<const int16_t> getSamples()
        {
            if(ready)
            {
                ready = false;
                return etl::span<const int16_t>(samples.data(), samples.size());
            }else
            {
                return {};
            }
        }
    };

    class Reader
    {
        etl::array<uint8_t, 64> scratch_buf;
        int16_t *dod_buf;
        ChannelState *ccs;
        uint8_t channels;
        uint8_t current_channel;
        ReaderState state;
        uint8_t state_rem = 0;
        bool in_sync = false;
        SampleBlock &block;
        etl::span<const uint8_t> buf;
        size_t buf_idx;

    public:
        Reader(etl::span<ChannelState> ccs_, etl::span<int16_t> dod_buf_, SampleBlock &block_) : block(block_)
        {
            ccs = ccs_.data();
            dod_buf = dod_buf_.data();
            channels = (ccs_.size()==dod_buf_.size())?ccs_.size():0;   
            current_channel = 0;
            state = ReaderState::DOD;
        }
        inline Err decode(etl::span<const uint8_t> buf_, StreamType type)
        {
            buf_idx = 0;
            buf = buf_;
            if (buf.size() == 0)
                return Err::OK;

            if(in_sync)
            {
                if (state == ReaderState::Sample)
                {
                    syncUpdate();
                }
            }

            if (type == StreamType::Control)
            {
                return decodeControl();
            }
            else if (in_sync)
            {
                return decodeSample();
            }
            else
            {
                return Err::NEED_SYNC;
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
        inline int16_t unzigzag(uint16_t val)
        {
            return static_cast<int16_t>(
                (val >> 1) ^ -(static_cast<int16_t>(val & 1)));
        }
        inline Err decodeControl()
        {
            switch (buf[buf_idx++])
            {
            case (uint8_t)ControlCode::RESYNCDOD:
                return decodeResyncDod();
                break;
            case (uint8_t)ControlCode::FLUSH:
                return decodeFlush();
                break;
            default:
                return Err::FAULT;
                break;
            }
        }
        inline Err decodeFlush()
        {
            if (in_sync)
            block.ready = true;
            return Err::OK;
        }
        inline Err decodeResyncDod()
        {
            uint8_t expected_size = sizeof(uint32_t) + sizeof(ccs->dod_state) * channels + 1;
            if (buf.size() != expected_size || buf.size() > scratch_buf.size())
                return Err::RESYNCDOD_BAD;
            for (size_t i = 0; i < buf.size(); i++)
            {
                scratch_buf[i] = buf[buf_idx++];
            }
            if (in_sync)
                block.ready = true;
            in_sync = true;
            state = ReaderState::Sample;
            return Err::OK;
        }
        inline Err decodeSample()
        {
            for (size_t i = 0; i < buf.size(); i++)
            {
                uint8_t byte = buf[buf_idx++];
                switch (state)
                {
                case ReaderState::DOD:
                    decodeDod(byte);
                    break;
                case ReaderState::LARGE16:
                    decodeLarge16(byte);
                    break;
                case ReaderState::VIRTUALBITS:
                    decodeVirtualBits(byte);
                    break;
                default:
                    break;
                }
                if (current_channel == channels)
                {
                    emitSample();
                }
            }

            return Err::OK;
        }
        inline Err decodeDod(uint8_t byte)
        {
            if (byte > VIRTUAL_BITS_END)
            { // Escape
                switch (byte)
                {
                case (uint8_t)Escape::LARGE16:
                    state = ReaderState::LARGE16;
                    state_rem = 2;
                    break;
                default:
                    break;
                }
            }
            else if (byte > VIRTUAL_BITS_START)
            { // VirtualBits
                state = ReaderState::VIRTUALBITS;
                state_rem = 1;
                scratch_buf[0] = byte - VIRTUAL_BITS_START;
            }
            else
            {
                // Normal DOD
                dod_buf[current_channel++] = unzigzag(byte);
            }
            return Err::OK;
        }
        inline Err decodeLarge16(uint8_t byte)
        {
            scratch_buf[2 - state_rem] = byte;
            state_rem--;
            if (state_rem == 0)
            {
                dod_buf[current_channel++] = unzigzag(scratch_buf[0] << 8 | scratch_buf[1]);
                state = ReaderState::DOD;
            }
            return Err::OK;
        }
        inline Err decodeVirtualBits(uint8_t byte)
        {
            scratch_buf[1] = byte;
            state_rem--;
            if (state_rem == 0)
            {
                dod_buf[current_channel++] = unzigzag(scratch_buf[0] << 8 | scratch_buf[1]);
                state = ReaderState::DOD;
            }
            return Err::OK;
        }
        inline Err syncUpdate()
        {
            uint8_t sch_idx = 0;
            block.counter = scratch_buf[sch_idx] << 24 | scratch_buf[sch_idx + 1] << 16 | scratch_buf[sch_idx + 2] << 8 | scratch_buf[sch_idx + 3];
            sch_idx += 4;
            block.samples.clear();
            if (block.samples.capacity() == 0)
                return Err::BAD_BLOCK;
            for (int i = 0; i < channels; i++)
            {
                int16_t sample = scratch_buf[sch_idx] << 8 | scratch_buf[sch_idx + 1];
                sch_idx += 2;
                int16_t delta = scratch_buf[sch_idx] << 8 | scratch_buf[sch_idx + 1];
                sch_idx += 2;
                block.samples.push_back(sample);
                ccs[i].dod_state[0] = sample;
                ccs[i].dod_state[1] = delta;
            }
            block.ready = false;
            state = ReaderState::DOD;
            return Err::OK;
        }
        inline Err emitSample()
        {
            if(block.samples.available() < channels){
                return Err::BLOCK_FULL;
            }
            for (int i = 0; i < channels; i++)
            {
                int16_t sample = dod_decode(i, dod_buf[i]);
                block.samples.push_back(sample);
            }
            current_channel = 0;
            return Err::OK;
        }
    };
};
