#pragma once
#include <stdint.h>
#include <type_traits>

template<typename T, uint16_t size>
class elcore_cfifo_fast_t
{
    static_assert(size > 0, "FIFO size must be greater than 0");
    static_assert((size & (size - 1)) == 0, "FIFO size must be a power of 2");
    static_assert(size <= 32768, "FIFO size limited to 32768 for practical use");

    static constexpr uint16_t mask_ = size - 1;

public:
    elcore_cfifo_fast_t() : head_(0), tail_(0) {}

    // Push element - ISR safe (single producer)
    // Returns false if full
    bool push(const T& item)
    {
        uint16_t next = (head_ + 1) & mask_;
        if (next == tail_) {
            return false;
        }
        buffer_[head_] = item;
        head_ = next;
        return true;
    }

    // Pop element - ISR safe (single consumer)
    // Returns false if empty
    bool pop(T& item)
    {
        if (head_ == tail_) {
            return false;
        }
        item = buffer_[tail_];
        tail_ = (tail_ + 1) & mask_;
        return true;
    }

    void clear()
    {
        head_ = 0;
        tail_ = 0;
    }

    bool empty() const { return head_ == tail_; }
    bool full() const  { return ((head_ + 1) & mask_) == tail_; }
    uint16_t count() const { return (head_ - tail_) & mask_; }
    static constexpr uint16_t capacity() { return size - 1; }
private:
    T buffer_[size];
    volatile uint16_t head_;
    volatile uint16_t tail_;
};

