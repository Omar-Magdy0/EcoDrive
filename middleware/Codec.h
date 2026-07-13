#pragma once

#include <stdint.h>
#include <assert.h>

constexpr int8_t CODECCONTROL = -128;
constexpr uint16_t CODECCONTROL_RESERVE = 16; // Escape + Control Byte + 14 control bytes possible

struct CodecBank
{
    int8_t *buf;
    uint16_t buf_size;
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t mask;
    CodecBank(int8_t *_buf, uint16_t _buf_size)
    {
        buf = _buf;
        buf_size = _buf_size;
        head = 0;
        tail = 0;
        bool valid = isPowerOfTwo(_buf_size);
        assert(valid);
        if (valid)
        {
            mask = buf_size - 1;
        }
        else
        {
            mask = 0xFFFF;
        }
    }
    bool isPowerOfTwo(uint16_t x)
    {
        return x && ((x & (x - 1)) == 0);
    }
    inline bool push(int8_t b)
    {
        uint16_t next = (head + 1) & mask;
        if (next == tail)
            return false; // full
        buf[head] = b;
        head = next;
        return true;
    }
    inline bool pop(int8_t &b)
    {
        if (head == tail)
            return false;
        b = buf[tail];
        tail = (tail + 1) & mask;
        return true;
    }
    bool empty() const
    {
        return head == tail;
    }
    bool full() const
    {
        return ((head + 1) & mask) == tail;
    }
    void reset()
    {
        head = 0;
        tail = 0;
    }
    uint16_t available() const
    {
        return (head - tail) & mask;
    }

    uint16_t free() const
    {
        return (tail - head - 1) & mask;
    }
};
enum class CodecFlags : uint8_t
{
    OVERRUN = 1 << 0,
};

enum class CodecControlSchema : uint8_t
{
    Flags,
    ResyncEscape,
    LargeDod,
    compHeader,
};

enum class CodecControlSchemaSize : uint8_t
{
    Flags = 1,
    ResyncEscape = 5,
    LargeDod = 2,
    compHeader = 1,
};

class CodecWriter
{
    int16_t dod_state[2]; // Sample, Delta
    uint8_t *scratch_mem;
    uint16_t scratch_mem_size;
    uint16_t scratch_mem_idx;
    uint8_t scratch_mem_bit_idx;
    uint8_t rice_k = 3;

    CodecBank mem;
    uint8_t flags = 0;
    uint8_t sample_counter = 0;
    uint8_t samples_till_sync = 0;
    uint8_t sync_period = 64;

    inline bool pushSample(int16_t sample)
    {
        int16_t dod_val = dod_encode(sample); // First of all get dod_encode residuals (decollerator)....
        if (flags & (uint8_t)CodecFlags::OVERRUN)
            return false;
        uint16_t free = mem.free();

        if (free <= (uint8_t)CODECCONTROL_RESERVE)
        { // Minimum free allowable , keeping margin for control and similar (Put marker and similar set overun flag in stream, etc)
            flags |= (uint8_t)CodecFlags::OVERRUN;
            emitControl(CodecControlSchema::Flags, CodecControlSchemaSize::Flags, &flags);
            return false;
        }

        if (!samples_till_sync) // Resync
        {
            uint8_t controlPayload[(uint8_t)CodecControlSchemaSize::ResyncEscape];
            controlPayload[0] = sample_counter;
            controlPayload[1] = dod_state[0] >> 8;
            controlPayload[2] = dod_state[0];
            controlPayload[3] = dod_state[1] >> 8;
            controlPayload[4] = dod_state[1];
            samples_till_sync = sync_period;
            emitControl(CodecControlSchema::ResyncEscape, CodecControlSchemaSize::ResyncEscape, controlPayload);
        }

        if (dod_val < -127 || dod_val > 127)
        {
            uint8_t controlPayload[(uint8_t)CodecControlSchemaSize::LargeDod];
            controlPayload[0] = dod_val >> 8;
            controlPayload[1] = dod_val;
            emitControl(CodecControlSchema::LargeDod, CodecControlSchemaSize::LargeDod, controlPayload);
        }
        else
        {
            int8_t res = (int8_t)dod_val;
            mem.push(res); // no overrun and we prechecked free size earlier
        }
        sample_counter++;
        samples_till_sync--;
        return true;
    }

    inline uint8_t zigzag(int8_t value)
    {
        // Assumes value is in [-127, 127]
        return (value >= 0)
                   ? (uint8_t)(value << 1)
                   : (uint8_t)((-value << 1) - 1);
    }

    void writeBits(uint32_t value, uint8_t bits)
    {
        while (bits)
        {
            // Space remaining in current output byte
            uint8_t space = 8 - scratch_mem_bit_idx;

            // Number of bits to write this iteration
            uint8_t n = (bits < space) ? bits : space;

            // Extract the next n most-significant bits
            uint32_t chunk = (value >> (bits - n)) & ((1u << n) - 1);

            // Insert into current byte
            scratch_mem[scratch_mem_idx] |= chunk << (space - n);

            scratch_mem_bit_idx += n;
            bits -= n;

            // Current byte full?
            if (scratch_mem_bit_idx == 8)
            {
                scratch_mem_bit_idx = 0;
                scratch_mem_idx++;
                scratch_mem[scratch_mem_idx] = 0; // Prepare next byte
            }
        }
    }

    void flushBitWrite()
    {
        if (scratch_mem_bit_idx)
        {
            scratch_mem_bit_idx = 0;
            scratch_mem_idx++;
            scratch_mem[scratch_mem_idx] = 0;
        }
    }

    inline void rice_code(uint8_t val)
    {
        uint32_t q = val >> rice_k;
        writeBits(1, q + 1);
        writeBits(val, rice_k);
    }

    inline bool idle_compress()
    {
        scratch_mem_idx = 0;
        int8_t byte;
        while (mem.pop(byte))
        {
            if (byte == CODECCONTROL)
            {
                flushBitWrite(); // Reset entropy coder bit index
                scratch_mem[scratch_mem_idx++] = byte;
                mem.pop(byte); // Copy the control bytes
                uint8_t schema = byte & 0x0F;
                uint8_t size = byte >> 4;
                while (size--)
                {
                    mem.pop(byte);
                    scratch_mem[scratch_mem_idx++] = byte;
                }                                                                                             // Copy the control payload
                if(schema == (uint8_t)CodecControlSchema::ResyncEscape)
                {
                    uint8_t comp_header = rice_k;  // lower 4 bits, K
                    scratchEmitControl(CodecControlSchema::compHeader, CodecControlSchemaSize::compHeader, &comp_header); // Now update compression control (Rice K value, etc)
                }                                                                            
            }
            else
            {
                uint8_t _zigzag = zigzag(byte); // Binary data to be compressed(for now simple FLAC like pattern)
                rice_code(_zigzag);
            }
        }
    }

    inline void reset()
    {
        sample_counter = 0;
        samples_till_sync = 0;
    }

    inline int16_t dod_encode(int16_t sample)
    {
        int16_t delta = sample - dod_state[0];
        int16_t dod = delta - dod_state[1];
        dod_state[0] = sample;
        dod_state[1] = delta;
        return dod;
    }

    void emitControl(CodecControlSchema schema, CodecControlSchemaSize size, uint8_t *payload)
    {
        uint8_t control_byte = (uint8_t)size << 4 | (uint8_t)schema;
        mem.push(CODECCONTROL); // We have 4 bytes in the tank at least, Push Control Bytes, handle escape, change compression to lossy , etc
        mem.push(control_byte);
        for (uint8_t i = 0; i < (uint8_t)size; i++)
        {
            mem.push(payload[i]);
        }
    }

    void scratchEmitControl(CodecControlSchema schema, CodecControlSchemaSize size, uint8_t *payload)
    {
        uint8_t control_byte = (uint8_t)size << 4 | (uint8_t)schema;
        scratch_mem[scratch_mem_idx++] = CODECCONTROL;
        scratch_mem[scratch_mem_idx++] = control_byte;
        memcpy(scratch_mem + scratch_mem_idx, payload, (uint8_t)size);
        scratch_mem_idx += (uint8_t)size;
    }
};

class CodecReader
{
    int16_t dod_state[2]; // Sample, Delta

    inline int16_t dod_decode(int16_t dod)
    {
        int16_t delta = dod_state[1] + dod;
        int16_t sample = delta + dod_state[0];
        dod_state[0] = sample;
        dod_state[1] = delta;
        return sample;
    }

    inline int8_t zigzag_inv(uint8_t zigzag)
    {
        return (zigzag & 1)
                   ? -(int8_t)((zigzag + 1) >> 1)
                   : (int8_t)(zigzag >> 1);
    }

    inline void rice_decomp()
    {
    }
};