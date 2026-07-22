#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "Usbxch.h"
#include "ABFStream.h"
#include "DAQStream.h"
#include <string>
#include <cstdint>
#include "DAQSessionAPP.h"
#include <chrono>
#include <iomanip>
void print_buffer(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        std::cout << std::hex
                  << std::uppercase
                  << std::setw(2)
                  << std::setfill('0')
                  << static_cast<unsigned>(data[i]) << ' ';
    }

    std::cout << std::dec << '\n';
}

uint8_t abf_rx_buf[255];
uint8_t idv_rx_buf[255];
IUsbxch *usb;
DAQSessionAPP *idaq;
static inline void onFrame(void *ctx, uint8_t id, uint8_t *payload, uint8_t payload_len);
static inline void onError(void *ctx, uint8_t id);
abf::Stream astream = abf::Stream(abf_rx_buf, sizeof(abf_rx_buf), NULL, onFrame, onError);

static inline void onFrame(void *ctx, uint8_t id, uint8_t *payload, uint8_t payload_len)
{
    std::cout << "ID : " << id << std::endl;
    print_buffer(payload, payload_len);
    idaq->process(payload, payload_len);
}
static inline void onError(void *ctx, uint8_t id)
{
}

using Clock = std::chrono::steady_clock;

int main()
{
    auto nextEvent = Clock::now() + std::chrono::seconds(5);
    usb = new UsbxchLibusb;
    idaq = new DAQSessionAPP(astream, *usb, idv_rx_buf, sizeof(idv_rx_buf));
    while(!usb->connect(
        "",     // serial number (empty = first matching device)
        0x0000, // VID
        0x0000, // PID
        0,      // interface
        0x81,   // Bulk IN endpoint
        0x01,
        1000)
    );
        ; // Bulk OUT endpoint
    while (true)
    {
        uint8_t buffer[16555];
        int bytes = usb->read(buffer, sizeof(buffer));
        auto now = Clock::now();
        if (now >= nextEvent)
        {
            nextEvent += std::chrono::seconds(5);
            if (!usb->isConnected())
            {
                usb->connect(
                    "",     // serial number (empty = first matching device)
                    0x0000, // VID
                    0x0000, // PID
                    0,      // interface
                    0x81,   // Bulk IN endpoint
                    0x01,
                    1000);
            }
            idaq->discovery_req();
        }
        if (bytes > 0)
        {
            uint8_t service_id;
            uint8_t payload_length;
            abf::Err status = astream.process(buffer, bytes);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}