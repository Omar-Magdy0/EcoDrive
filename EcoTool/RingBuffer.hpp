#pragma once

#include <vector>
#include <mutex>
#include <algorithm>
#include <cstddef>
#include <cstdint>

class RingBuffer {
public:
    explicit RingBuffer(size_t capacity = 16384)
        : buf_(capacity + 1), head_(0), tail_(0) {}

    size_t capacity() const { return buf_.size() - 1; }

    bool empty() const {
        std::lock_guard<std::mutex> l(m_);
        return head_ == tail_;
    }

    size_t available() const {
        std::lock_guard<std::mutex> l(m_);
        return (tail_ + buf_.size() - head_) % buf_.size();
    }

    size_t push(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> l(m_);
        size_t cap = capacity();
        size_t used = (tail_ + buf_.size() - head_) % buf_.size();
        size_t free = cap - used;
        size_t to_write = std::min(len, free);
        size_t first = std::min(to_write, buf_.size() - tail_);
        if (first)
            std::copy(data, data + first, buf_.begin() + tail_);
        size_t second = to_write - first;
        if (second)
            std::copy(data + first, data + first + second, buf_.begin());
        tail_ = (tail_ + to_write) % buf_.size();
        return to_write;
    }

    size_t pop(uint8_t* dst, size_t len) {
        std::lock_guard<std::mutex> l(m_);
        size_t used = (tail_ + buf_.size() - head_) % buf_.size();
        size_t to_read = std::min(len, used);
        size_t first = std::min(to_read, buf_.size() - head_);
        if (first)
            std::copy(buf_.begin() + head_, buf_.begin() + head_ + first, dst);
        size_t second = to_read - first;
        if (second)
            std::copy(buf_.begin(), buf_.begin() + second, dst + first);
        head_ = (head_ + to_read) % buf_.size();
        return to_read;
    }

private:
    std::vector<uint8_t> buf_;
    size_t head_;
    size_t tail_;
    mutable std::mutex m_;
};
