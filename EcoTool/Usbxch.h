#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>
#include <algorithm>
#include <libusb.h>
#include <array>
#include "RingBuffer.hpp"


class IUsbxch
{
    public:
    virtual ~IUsbxch() = default;
    virtual bool connect(const std::string &serial_number,
                         uint16_t vid,
                         uint16_t pid,
                         int interface_number,
                         unsigned char bulk_in_ep,
                         unsigned char bulk_out_ep,
                         int timeout_ms) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual bool write(const uint8_t *data, size_t length) = 0;
    virtual bool write(const std::vector<uint8_t> &data) = 0;
    virtual size_t read(uint8_t *data, size_t max_len) = 0;
    virtual size_t available() = 0;
};

class UsbxchLibusb : public IUsbxch
{
public:
    UsbxchLibusb() = default;
    ~UsbxchLibusb() override;

    bool connect(const std::string &serial_number,
                 uint16_t vid,
                 uint16_t pid,
                 int interface_number,
                 unsigned char bulk_in_ep,
                 unsigned char bulk_out_ep,
                 int timeout_ms) override;

    void disconnect() override;

    bool isConnected() const override;

    bool write(const uint8_t *data, size_t length) override;

    bool write(const std::vector<uint8_t> &data) override
    {
        return write(data.data(), data.size());
    }
    size_t read(uint8_t *data, size_t max_len) override;
    size_t available() override;

private:
    libusb_context *context_ = nullptr;
    libusb_device_handle *handle_ = nullptr;
    std::thread io_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    RingBuffer rx_buffer_{16384};

    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;

    int timeout_ms_ = 1000;
    int interface_number_ = 0;
    unsigned char bulk_in_ep_ = 0x81;
    unsigned char bulk_out_ep_ = 0x01;

    void ioLoop();
};

#include "TcpClient.hpp"
class UsbxchTcp : public IUsbxch
{
public:
    UsbxchTcp() = default;
    ~UsbxchTcp() override;

    bool connect(const std::string &serial_number,
                 uint16_t vid,
                 uint16_t pid,
                 int interface_number,
                 unsigned char bulk_in_ep,
                 unsigned char bulk_out_ep,
                 int timeout_ms) override;

    void disconnect() override;

    bool isConnected() const override;

    bool write(const uint8_t *data, size_t length) override;

    bool write(const std::vector<uint8_t> &data) override
    {
        return write(data.data(), data.size());
    }
    size_t read(uint8_t *data, size_t max_len) override;
    size_t available() override;

private:
    TcpClient client;
    std::thread io_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    RingBuffer rx_buffer_{16384};

    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;

    int timeout_ms_ = 1000;
    int interface_number_ = 0;
    void ioLoop();
};