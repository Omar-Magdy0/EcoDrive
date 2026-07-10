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

struct SpeedTracker
{
    std::chrono::steady_clock::time_point last{};
    size_t rx_bytes = 0;
    size_t tx_bytes = 0;
    double rx_speed = 0.0;
    double tx_speed = 0.0;
    mutable std::mutex mutex;

    void update(size_t rx, size_t tx)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = std::chrono::steady_clock::now();
        if (last != std::chrono::steady_clock::time_point{})
        {
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
            if (ms >= 1000)
            {
                const double s = ms / 1000.0;
                rx_speed = rx_bytes / s;
                tx_speed = tx_bytes / s;
                last = now;
                rx_bytes = 0;
                tx_bytes = 0;
            }
        }
        else
        {
            last = now;
        }
        rx_bytes += rx;
        tx_bytes += tx;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex);
        last = {};
        rx_bytes = 0;
        tx_bytes = 0;
        rx_speed = 0.0;
        tx_speed = 0.0;
    }

    double rxBps() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return rx_speed;
    }

    double txBps() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return tx_speed;
    }
};

class IUsbxch
{
    protected:
    SpeedTracker speed_tracker_;
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
    virtual int write(const uint8_t *data, size_t length) = 0;
    virtual int write(const std::vector<uint8_t> &data) = 0;
    virtual int read(uint8_t *data, size_t max_len) = 0;
    virtual size_t available() = 0;
    virtual double getRxSpeedBytesPerSecond() const = 0;
    virtual double getTxSpeedBytesPerSecond() const = 0;
    virtual void resetSpeeds() = 0;
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

    int write(const uint8_t *data, size_t length) override;

    int write(const std::vector<uint8_t> &data) override
    {
        return write(data.data(), data.size());
    }
    int read(uint8_t *data, size_t max_len) override;
    size_t available() override;
    double getRxSpeedBytesPerSecond() const override;
    double getTxSpeedBytesPerSecond() const override;
    void resetSpeeds() override;

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

    int write(const uint8_t *data, size_t length) override;

    int write(const std::vector<uint8_t> &data) override
    {
        return write(data.data(), data.size());
    }
    int read(uint8_t *data, size_t max_len) override;
    size_t available() override;
    double getRxSpeedBytesPerSecond() const override;
    double getTxSpeedBytesPerSecond() const override;
    void resetSpeeds() override;

private:
    TcpClient client;
    std::thread io_thread_;
    std::thread speed_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stats_running_{false};
    std::atomic<bool> connected_{false};

    std::mutex tx_mutex_;
    mutable std::mutex speed_mutex_;
    std::condition_variable tx_cv_;

    int timeout_ms_ = 1000;
    int interface_number_ = 0;
    void ioLoop();
};