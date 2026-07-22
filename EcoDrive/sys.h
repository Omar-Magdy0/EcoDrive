#pragma once

#include "PmsmControl/PmsmControl.h"
#include "platform.h"
#include "ABFStream.h"
#include "el_usbxch.h"
#include <cstdint>
#include <array>
#include <string.h>
#include <stdio.h>
//=======================================================
#include "eld_core.h"
#include "DAQSessionAPP.h"




class Sys
{
    static inline void onFrame(void* ctx, uint8_t id, uint8_t* payload, uint8_t payload_len)
    {
        daq_session.process(payload, payload_len);
    }
    static inline void onError(void* ctx, uint8_t id)
    {

    }
    inline static uint8_t abfStream_rx_buffer[255];
    inline static uint8_t daq_idv_buffer[255];
    inline static el_core_t core;
    inline static el_usbxch_handle_t usbxch;
    inline static abf::Stream abfStream = abf::Stream(abfStream_rx_buffer, sizeof(abfStream_rx_buffer), NULL, onFrame, onError);
    inline static DAQSessionAPP daq_session = DAQSessionAPP(abfStream, usbxch, "SAD",daq_idv_buffer);
    public:
    static void init(void);
};


