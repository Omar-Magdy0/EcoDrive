#pragma once
#include "DAQStream.h"
#include "ABFStream.h"
#include "eldriver/eldriver_usbxch.h"

class DAQSessionAPP : public DAQSession
{
protected:
    ABFStream &abf;
    eldriver_usbxch_handle_t &usb;

public:
    DAQSessionAPP(ABFStream &abf_stream,
                  eldriver_usbxch_handle_t &usb_handle,
                  uint8_t *idv_buffer,
                  uint16_t idv_buffer_len) : DAQSession(idv_buffer, idv_buffer_len), abf(abf_stream), usb(usb_handle)
    {
    }
    uint8_t send() override
    {
        uint8_t abf_header[ABF_HEADER_SIZE];
        abf.encode(abf_header, writer.data(), 16, writer.size());
        eldriver_usbxch_write(&usb, abf_header, ABF_HEADER_SIZE);
        eldriver_usbxch_write(&usb, writer.data(), writer.size());
        eldriver_usbxch_flush(&usb);
        return 0;
    }
    uint8_t on_mark(DAQ_IDV::MARKER mark, bool entry)override
    {
        switch (mark)
        {
        case DAQ_IDV::MARKER::AUTO_DISCOVER_REQ:
            if(!entry)discovery_respond();
            break;
        
        default:
            break;
        }
        return 0;
    }
};