#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "Usbxch.h"

int main()
{
    IUsbxch *usb;
    usb = new UsbxchTcp;
    if (!usb->connect(
            "",     // serial number (empty = first matching device)
            0x0000, // VID
            0x0000, // PID
            0,      // interface
            0x81,   // Bulk IN endpoint
            0x01,
            1000))  // Bulk OUT endpoint
    {
        std::cerr << "Failed to connect\n";
    }

    std::vector<uint8_t> buffer(4096);
    while (true)
    {
        if (usb->available())
        {
            uint8_t buffer[512];

            size_t bytes = usb->read(buffer, sizeof(buffer));

            std::cout.write(reinterpret_cast<char *>(buffer), bytes);
            std::cout.flush();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    while (true)
    {
        if (usb->available())
        {
            uint8_t buffer[512];

            size_t bytes = usb->read(buffer, sizeof(buffer));

            std::cout.write(reinterpret_cast<char *>(buffer), bytes);
            std::cout.flush();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}