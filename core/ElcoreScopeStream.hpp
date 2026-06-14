#pragma once
#include "elcore.h"
#include <stdint.h>
#include <string.h>


template <typename T>
class ElcoreScopeStream {
public:
    T* buf;
    
    T trigger_level;
    T hysteresis_level;
    bool hyst_trig;
    
    uint8_t decimation;
    uint8_t decimation_counter;
    
    uint16_t write_idx;      // Simple linear index
    uint8_t channels_num;
    uint32_t sample_depth;
    
    bool triggered;
    volatile bool frozen;

    constexpr uint32_t buffer_size(uint8_t channels, uint32_t sample_depth) const {
        return sample_depth * channels * sizeof(T);
    }

    ElcoreScopeStream(T* storage, uint8_t channels, uint32_t sample_depth) {
        buf = storage;
        channels_num = channels;
        decimation = 1;
        decimation_counter = 1;
        this->sample_depth = sample_depth;
        trigger_level = 0;
        write_idx = 0;
        triggered = false;
        frozen = false;
        //last_sample = 0;
    }

    void write(const T *data)
    {
        if (!frozen)
        {
            // 1. Decimation
            if (--decimation_counter > 0)
                return;
            decimation_counter = decimation;

            // 2. Trigger Logic
            if (!triggered)
            {
                if (data[0] > trigger_level + hysteresis_level && !hyst_trig)
                {
                    hyst_trig = true;
                    triggered = true;
                    write_idx = 0; // Start filling from the beginning
                }else if (data[0] < trigger_level - hysteresis_level && hyst_trig)
                {
                    hyst_trig = false;
                }
            }

            // 3. Sequential Fill
            if (triggered)
            {
                // Copy channels for this specific sample
                memcpy(buf + write_idx * channels_num, data, sizeof(T) * channels_num);
                write_idx++;
                // 4. Check if Full
                if (write_idx >= sample_depth)
                {
                    frozen = true;
                }
            }
        }
       // last_sample = data[0];
    }

    /**
     * Since the buffer is now linear and aligned, we just 
     * copy the whole thing or even read it directly.
     */
    bool read(T* out_buffer, uint16_t buffer_size) {
        if (!frozen) return false;
        if (buffer_size < sample_depth * channels_num * sizeof(T)) return false;

        // Copy the captured window to your display buffer
        memcpy(out_buffer, buf, sample_depth * channels_num * sizeof(T));

        return true;
    }

    bool read(uint8_t channel, T* out_buffer, uint16_t buffer_size)
    {
        if (!frozen) return false;
        if (buffer_size < sample_depth * sizeof(T)) return false;

        // Copy the captured window to your display buffer
        for (uint16_t i = 0; i < sample_depth; i++)
        {
            out_buffer[i] = buf[i * channels_num + channel];
        }

        return true;
    }
    
    bool isFrozen() const {
        return frozen;
    }

    bool isTriggered() const {
        return triggered;
    }

    void reset() {
        write_idx = 0;
        triggered = false;
        frozen = false;
        decimation_counter = decimation;
    }

    const T* get_buffer() const {
        return buf; 
    }

    void set_trigger_level(T level) {
        trigger_level = level;
    }

    void update_decimation(uint8_t divisions, uint32_t time_per_div_us, uint32_t sample_rate)
    {
        uint32_t samples_per_div = (sample_depth/divisions);
        uint32_t base_time_per_div_us = (uint64_t(samples_per_div) * 1000000) / sample_rate;
        decimation = (time_per_div_us > base_time_per_div_us) ? (uint8_t)((float(time_per_div_us) / base_time_per_div_us) + 0.5) : 1;
        decimation_counter = decimation;
    }

    void update_decimation(uint8_t decimation)
    {
        this->decimation = decimation;
        decimation_counter = decimation;
    }

};