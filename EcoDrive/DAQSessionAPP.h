#pragma once
#include "DAQStream.h"
#include "ABFStream.h"
#include "el_usbxch.h"
#include <etl/span.h>
class DAQSessionAPP : public daq::Session
{
protected:
    abf::Stream &abf;
    el_usbxch_handle_t &usb;

public:
    DAQSessionAPP(abf::Stream &abf_stream,
                  el_usbxch_handle_t &usb_handle,
                  const char* Meta,
                  etl::span<uint8_t> idv_buf) : daq::Session(Meta, idv_buf), abf(abf_stream), usb(usb_handle)
    {
    }
    uint8_t send() override
    {
        uint8_t abf_header[abf::HEADER_SIZE];
        abf.encode(abf_header, writer.data(), 16, writer.size());
        el_usbxch_write(&usb, abf_header, abf::HEADER_SIZE);
        el_usbxch_write(&usb, writer.data(), writer.size());
        el_usbxch_flush(&usb);
        return 0;
    }
    uint8_t on_mark(daq::MARKER mark, bool entry) override
    {
        switch (mark)
        {
        case daq::MARKER::AUTO_DISCOVER_REQ:
            if (!entry)
                discovery_respond();
            break;

        default:
            break;
        }
        return 0;
    }
};