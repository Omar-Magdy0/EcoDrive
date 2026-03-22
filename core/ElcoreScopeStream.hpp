#pragma once
#include "elcore.h"
#include <stdint.h>
#include <string.h>


template <typename T>
class ElcoreScopeStream {
public:
    T* buf;
    uint16_t sample_depth;
    
    T trigger_level;
    T last_sample;
    
    unsigned int decimation;
    unsigned int decimation_counter;
    
    uint16_t write_idx;      // Simple linear index
    uint8_t channels_num;
    
    bool triggered;
    volatile bool frozen;

    ElcoreScopeStream(T* storage, uint8_t channels, uint8_t divs, uint8_t samples_per_div) {
        buf = storage;
        channels_num = channels;
        sample_depth = divs * samples_per_div;
        
        decimation = 1;
        decimation_counter = 1;
        
        trigger_level = 0;
        write_idx = 0;
        triggered = false;
        frozen = false;
        last_sample = 0;
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
                if (data[0] > trigger_level && last_sample <= trigger_level)
                {
                    triggered = true;
                    write_idx = 0; // Start filling from the beginning
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
        last_sample = data[0];
    }

    /**
     * Since the buffer is now linear and aligned, we just 
     * copy the whole thing or even read it directly.
     */
    bool read_aligned(T* out_buffer) {
        if (!frozen) return false;

        // Copy the captured window to your display buffer
        memcpy(out_buffer, buf, sample_depth * channels_num * sizeof(T));

        // Reset for the next capture
        triggered = false;
        write_idx = 0;
        frozen = false; 
    
        return true;
    }
};