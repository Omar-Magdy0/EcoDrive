#pragma once

#include "PmsmControl/PmsmControl.h"
#include "platform.h"
#include "ABFStream.h"
#include "eldriver_usbxch.h"
#include <cstdint>
#include <array>
#include <string.h>
#include <stdio.h>
//=======================================================
#include "eldriver_core.h"
#include "DAQSessionAPP.h"




class Sys
{
    static inline void onFrame(void* ctx, uint8_t id, uint8_t* payload, uint8_t payload_len)
    {
        daq.process(payload, payload_len);
    }
    static inline void onError(void* ctx, uint8_t id)
    {

    }

    inline static uint8_t abfStream_rx_buffer[255];
    inline static uint8_t daq_idv_buffer[255];
    inline static eldriver_core_t core;
    inline static eldriver_usbxch_handle_t usbxch;
    inline static ABFStream abfStream = ABFStream(abfStream_rx_buffer, sizeof(abfStream_rx_buffer), NULL, onFrame, onError);
    inline static DAQSessionAPP daq = DAQSessionAPP(abfStream, usbxch, daq_idv_buffer, sizeof(daq_idv_buffer));
    public:
    static void init(void);
};


