#pragma once
#include <stdint.h>
#include <assert.h>

class UnitTest;
namespace el
{
    template <typename T>
    class ring_fast
    {
        friend UnitTest;
        uint16_t size;
        uint16_t mask_;

    public:
        ring_fast(T *buffer, uint16_t size_) : size(size_), head_(0), tail_(0), buffer_(buffer)
        {
            assert(size > 0);
            assert((size & (size - 1)) == 0);
            assert(size <= 32768);
            mask_ = size - 1;
        }

        bool isPowerOfTwo(uint16_t x)
        {
            return x && ((x & (x - 1)) == 0);
        }

        bool isPowerOfTwoAssert(uint16_t x)
        {
            bool s = x && ((x & (x - 1)) == 0);
            assert(s);
            return s;
        }

        // Push element - ISR safe (single producer)
        // Returns false if full
        bool push(const T &item)
        {
            uint16_t next = (head_ + 1) & mask_;
            if (next == tail_)
            {
                return false;
            }
            buffer_[head_] = item;
            head_ = next;
            return true;
        }
        void pushFast(const T &item)
        {
            buffer_[head_] = item;
            head_ = (head_ + 1) & mask_;
        }

        // Pop element - ISR safe (single consumer)
        // Returns false if empty
        bool pop(T &item)
        {
            if (head_ == tail_)
            {
                return false;
            }
            item = buffer_[tail_];
            tail_ = (tail_ + 1) & mask_;
            return true;
        }
        bool peek(T &item)
        {
            if (head_ == tail_)
            {
                return false;
            }
            item = buffer_[tail_];
            return true;
        }
        bool peekRaw(T &item, uint16_t idx) const
        {
            if (idx > size)
                return false;
            item = buffer_[idx];
            return true;
        }
        bool advance(uint16_t n)
        {
            if (n > count())
                return false;
            tail_ = (tail_ + n) & mask_;
            return true;
        }
        bool distanceFromTail(int16_t idx, uint16_t &distance)
        {
            if (idx >= size)
                return false;

            // Distance from tail to idx (modulo buffer size)
            uint32_t d = (idx - tail_) & mask_;

            // Is idx inside the valid FIFO contents?
            if (d >= count())
                return false;

            distance = d;
            return true;
        }

        void clear()
        {
            head_ = 0;
            tail_ = 0;
        }

        bool empty() const { return head_ == tail_; }
        bool full() const { return ((head_ + 1) & mask_) == tail_; }
        uint16_t free() const { return (capacity() - count()); }
        uint16_t count() const { return (head_ - tail_) & mask_; }
        uint16_t capacity() const { return size - 1; }

        // Multibyte Reserve/Commit pattern
        //  Reserve space for writing 'count' elements.
        // Returns false if insufficient space.
        bool reserveWrite(
            uint16_t count,
            T **writeptr1,
            uint16_t *cont1,
            T **writeptr2,
            uint16_t *cont2)
        {
            if (count > (capacity() - this->count()))
            {
                *writeptr1 = nullptr;
                *writeptr2 = nullptr;
                *cont1 = 0;
                *cont2 = 0;
                return false;
            }

            uint16_t cont_space;

            if (head_ >= tail_)
                cont_space = size - head_ - (tail_ == 0 ? 1 : 0);
            else
                cont_space = tail_ - head_ - 1;

            *writeptr1 = &buffer_[head_];

            if (cont_space >= count)
            {
                *writeptr2 = nullptr;
                *cont1 = count;
                *cont2 = 0;
            }
            else
            {
                *writeptr2 = &buffer_[0];
                *cont1 = cont_space;
                *cont2 = count - cont_space;
            }

            return true;
        }
        void commitWrite(uint16_t count)
        {
            head_ = (head_ + count) & mask_;
        }
        bool peekRead(
            T **readptr1,
            uint16_t *cont1,
            T **readptr2,
            uint16_t *cont2)
        {
            uint16_t cnt = this->count();

            if (cnt == 0)
            {
                *readptr1 = nullptr;
                *readptr2 = nullptr;
                *cont1 = 0;
                *cont2 = 0;
                return false;
            }

            uint16_t cont_space;

            if (tail_ >= head_)
                cont_space = size - tail_;
            else
                cont_space = head_ - tail_;

            *readptr1 = &buffer_[tail_];

            if (cont_space >= cnt)
            {
                *readptr2 = nullptr;
                *cont1 = cnt;
                *cont2 = 0;
            }
            else
            {
                *readptr2 = &buffer_[0];
                *cont1 = cont_space;
                *cont2 = cnt - cont_space;
            }

            return true;
        }
        void releaseRead(uint16_t count)
        {
            tail_ = (tail_ + count) & mask_;
        }
        uint16_t tail() const { return tail_; }
        uint16_t head() const { return head_; }

    private:
        T *buffer_;
        volatile uint16_t head_;
        volatile uint16_t tail_;
    };
}