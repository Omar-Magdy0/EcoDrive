#ifndef ELCORE_RSTREAM_H
#define ELCORE_RSTREAM_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t high_watermark;
    uint16_t failed_writes;
    uint32_t bytes_written;
    uint32_t bytes_read;
} el_ring_stats_t;

typedef struct {
    uint8_t *buffer; // pointer to the buffer memory
    uint16_t capacity; // total capacity of the buffer (number of elements)
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t item_size; // size of each element in the buffer
    el_ring_stats_t stats; // statistics for monitoring buffer usage
}el_ring_t;

static inline void el_ring_init(el_ring_t* cb,
                                      void* storage,
                                      uint16_t item_size,
                                      uint16_t capacity)
{
    cb->buffer = (uint8_t*)storage;
    cb->item_size = item_size;
    cb->capacity = capacity;
    cb->head = 0;
    cb->tail = 0;
    cb->stats.high_watermark = 0;
    cb->stats.failed_writes = 0;
    cb->stats.bytes_written = 0;
    cb->stats.bytes_read = 0;
}

static inline el_ring_stats_t el_ring_getStats(const el_ring_t* cb)
{
    return cb->stats;
}

static inline void el_ring_resetStats(el_ring_t* cb)
{
    cb->stats.high_watermark = 0;
    cb->stats.failed_writes = 0;
    cb->stats.bytes_written = 0;
    cb->stats.bytes_read = 0;
}

static inline uint16_t el_ring_occupied(const el_ring_t* cb) {
    uint16_t head = cb->head;  // atomic read
    uint16_t tail = cb->tail;  // atomic read
    if (head >= tail)
        return head - tail;
    return cb->capacity - tail + head;
}

static inline uint16_t el_ring_freeSpace(const el_ring_t* cb) {
    return cb->capacity - el_ring_occupied(cb) - 1;
}

static inline void el_ring_commitWrite(el_ring_t* cb, uint16_t count) {
    uint16_t occupied = el_ring_occupied(cb);
    cb->stats.bytes_written += count * cb->item_size;
    if (occupied > cb->stats.high_watermark) {
        cb->stats.high_watermark = occupied;
    }
    uint16_t head = cb->head;
    head = (head + count) - (head + count >= cb->capacity ? cb->capacity : 0);
    cb->head = head;
}

static inline void el_ring_releaseRead(el_ring_t* cb, uint16_t count) {
    uint16_t tail = cb->tail;
    cb->stats.bytes_read += count * cb->item_size;
    tail = (tail + count) - (tail + count >= cb->capacity ? cb->capacity : 0);
    cb->tail = tail;
}

static inline bool el_ring_reserveWrite(el_ring_t* cb,
                                       uint16_t count,
                                       void** writeptr1,
                                       uint16_t* cont1,
                                       void** writeptr2,
                                       uint16_t* cont2)
{
    if (count > el_ring_freeSpace(cb))
    {
        cb->stats.failed_writes++;
        return false;
    }
        
    uint16_t cont_space;

    if (cb->head >= cb->tail)
        cont_space = cb->capacity - cb->head - (cb->tail == 0 ? 1 : 0);
    else
        cont_space = cb->tail - cb->head - 1;

    *writeptr1 = cb->buffer + cb->head * cb->item_size;

    if (cont_space >= count) {
        *writeptr2 = NULL;
        *cont1 = count;
        *cont2 = 0;
    } else {
        *writeptr2 = cb->buffer;
        *cont1 = cont_space;
        *cont2 = count - cont_space;
    }

    return true;
}

static inline void el_ring_reserveWriteOverride(
                                       el_ring_t* cb,
                                       uint16_t count,
                                       void** writeptr1,
                                       uint16_t* cont1,
                                       void** writeptr2,
                                       uint16_t* cont2)
{
    uint16_t free = el_ring_freeSpace(cb);

    if (count > free)
    {
        uint16_t drop = count - free;

        cb->tail = (cb->tail + drop) % cb->capacity;
    }

    uint16_t cont_space;

    if (cb->head >= cb->tail)
        cont_space = cb->capacity - cb->head - (cb->tail == 0 ? 1 : 0);
    else
        cont_space = cb->tail - cb->head - 1;

    *writeptr1 = cb->buffer + cb->head * cb->item_size;

    if (cont_space >= count)
    {
        *writeptr2 = NULL;
        *cont1 = count;
        *cont2 = 0;
    }
    else
    {
        *writeptr2 = cb->buffer;
        *cont1 = cont_space;
        *cont2 = count - cont_space;
    }
}

static inline bool el_ring_peekRead(el_ring_t* cb,
                                      void** readptr1,
                                      uint16_t* cont1,
                                      void** readptr2,
                                      uint16_t* cont2)
{
    if (el_ring_occupied(cb) == 0)
    {
        *readptr1 = NULL;
        *cont1 = 0;
        *readptr2 = NULL;
        *cont2 = 0;
        return false;
    }

    uint16_t cont_space;
    uint16_t count = el_ring_occupied(cb);

    if (cb->tail >= cb->head)
        cont_space = cb->capacity - cb->tail;
    else
        cont_space = cb->head - cb->tail;

    *readptr1 = cb->buffer + cb->tail * cb->item_size;

    if (cont_space >= count) {
        *readptr2 = NULL;
        *cont1 = count;
        *cont2 = 0;
    } else {
        *readptr2 = cb->buffer;
        *cont1 = cont_space;
        *cont2 = count - cont_space;
    }

    return true;
}

#ifdef __cplusplus
}
#endif

#endif
