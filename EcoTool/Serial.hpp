#pragma once

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <iostream>
#include <optional>
#include <condition_variable>
#include <algorithm>
#include "RingBuffer.hpp"

namespace asio = boost::asio;

class Serial {
public:
    using ReadCallback = std::function<void(const uint8_t* data, size_t length)>;

    Serial() : serial_port_(io_context_) {}

    ~Serial() {
        disconnect();
    }

    // ====================== Main API ======================

    bool connect(const std::string& port,
                 unsigned int baud_rate = 2000000,
                 asio::serial_port_base::parity::type parity = asio::serial_port_base::parity::none,
                 asio::serial_port_base::stop_bits::type stop_bits = asio::serial_port_base::stop_bits::one,
                 unsigned int char_size = 8)
    {
        try {
            disconnect();  // Clean previous connection
            io_context_.restart();
            work_guard_.emplace(asio::make_work_guard(io_context_));

            serial_port_.open(port);

            serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
            serial_port_.set_option(asio::serial_port_base::parity(parity));
            serial_port_.set_option(asio::serial_port_base::stop_bits(stop_bits));
            serial_port_.set_option(asio::serial_port_base::character_size(char_size));
            serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

            connected_ = true;
            running_ = true;

            startAsyncRead();

            // Start Asio in dedicated thread
            io_thread_ = std::thread([this]() { io_context_.run(); });

            // Start TX pump thread
            tx_thread_ = std::thread([this]() { txLoop(); });

            std::cout << "[Serial] Connected to " << port << " @ " << baud_rate << " baud\n";
            return true;

        } catch (const std::exception& e) {
            std::cerr << "[Serial] Connect error: " << e.what() << std::endl;
            return false;
        }
    }

    void disconnect() {
        if (!connected_) return;

        running_ = false;
        connected_ = false;

        work_guard_.reset();
        io_context_.stop();

        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        // Wake TX thread and join
        tx_cv_.notify_all();
        if (tx_thread_.joinable()) {
            tx_thread_.join();
        }

        if (serial_port_.is_open()) {
            serial_port_.close();
        }
    }

    bool isConnected() const {
        return connected_;
    }

    int write(const uint8_t* data, size_t length) {
        if (!connected_) {
            std::cerr << "[Serial] Write failed: Not connected\n";
            return false;
        }

        size_t pushed = tx_buffer_.push(data, length);
        if (pushed == 0) {
            std::cerr << "[Serial] TX buffer full, drop write\n";
            return false;
        }
        if (pushed < length) {
            std::cerr << "[Serial] TX buffer partial write, dropped " << (length - pushed) << " bytes\n";
        }
        tx_cv_.notify_one();
        return (pushed == length);
    }

    int write(const std::vector<uint8_t>& data) {
        return write(data.data(), data.size());
    }

    void setReadCallback(ReadCallback callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        read_callback_ = std::move(callback);
    }

    // Read from RX ring buffer (non-blocking)
    int read(uint8_t* data, size_t max_len) {
        return rx_buffer_.pop(data, max_len);
    }

private:
    asio::io_context io_context_;
    asio::serial_port serial_port_;
    std::optional<asio::executor_work_guard<asio::io_context::executor_type>> work_guard_;

    std::thread io_thread_;
    std::thread tx_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    std::array<uint8_t, 16384> read_buffer_{};

    RingBuffer rx_buffer_{16384};
    RingBuffer tx_buffer_{16384};

    ReadCallback read_callback_;
    std::mutex callback_mutex_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::atomic<bool> tx_thread_running_{false};

    void startAsyncRead() {
        serial_port_.async_read_some(
            asio::buffer(read_buffer_),
            [this](const boost::system::error_code& ec, std::size_t bytes) {
                handleRead(ec, bytes);
            });
    }

    void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (ec) {
            if (ec != asio::error::operation_aborted) {
                std::cerr << "[Serial] Read error: " << ec.message() << std::endl;
                connected_ = false;
            }
            return;
        }

        if (bytes_transferred > 0) {
            // Push data quickly into RX ring buffer
            size_t pushed = rx_buffer_.push(read_buffer_.data(), bytes_transferred);
            if (pushed < bytes_transferred) {
                std::cerr << "[Serial] RX buffer overflow, dropped " << (bytes_transferred - pushed) << " bytes\n";
            }

            // Also optionally notify callback with what was pushed
            ReadCallback callback_copy;
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback_copy = read_callback_;
            }
            if (callback_copy && pushed > 0) {
                callback_copy(read_buffer_.data(), pushed);
            }
        }

        // Restart async read for continuous high-speed reception
        if (connected_ && running_) {
            startAsyncRead();
        }
    }

    // TX pump thread: drains tx_buffer_ and performs synchronous writes
    void txLoop() {
        tx_thread_running_ = true;
        std::vector<uint8_t> tmp;
        tmp.reserve(4096);
        while (running_) {
            // wait for data
            {
                std::unique_lock<std::mutex> lk(tx_mutex_);
                tx_cv_.wait(lk, [this]() { return !running_ || !tx_buffer_.empty(); });
                if (!running_) break;
            }

            // Pop up to chunk size
            tmp.resize(4096);
            size_t got = tx_buffer_.pop(tmp.data(), tmp.size());
            if (got == 0) continue;
            tmp.resize(got);

            boost::system::error_code ec;
            asio::write(serial_port_, asio::buffer(tmp), ec);
            if (ec) {
                std::cerr << "[Serial] TX write error: " << ec.message() << std::endl;
            }
        }
        tx_thread_running_ = false;
    }
};
