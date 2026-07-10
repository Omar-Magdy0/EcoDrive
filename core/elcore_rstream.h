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
} elcore_rstream_stats_t;

typedef struct {
    uint8_t *buffer; // pointer to the buffer memory
    uint16_t capacity; // total capacity of the buffer (number of elements)
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t item_size; // size of each element in the buffer
    elcore_rstream_stats_t stats; // statistics for monitoring buffer usage
}elcore_rstream_t;

static inline void elcore_rstream_init(elcore_rstream_t* cb,
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

static inline elcore_rstream_stats_t elcore_rstream_getStats(const elcore_rstream_t* cb)
{
    return cb->stats;
}

static inline void elcore_rstream_resetStats(elcore_rstream_t* cb)
{
    cb->stats.high_watermark = 0;
    cb->stats.failed_writes = 0;
    cb->stats.bytes_written = 0;
    cb->stats.bytes_read = 0;
}

static inline uint16_t elcore_rstream_occupied(const elcore_rstream_t* cb) {
    uint16_t head = cb->head;  // atomic read
    uint16_t tail = cb->tail;  // atomic read
    if (head >= tail)
        return head - tail;
    return cb->capacity - tail + head;
}

static inline uint16_t elcore_rstream_freeSpace(const elcore_rstream_t* cb) {
    return cb->capacity - elcore_rstream_occupied(cb) - 1;
}

static inline void elcore_rstream_commitWrite(elcore_rstream_t* cb, uint16_t count) {
    uint16_t occupied = elcore_rstream_occupied(cb);
    cb->stats.bytes_written += count * cb->item_size;
    if (occupied > cb->stats.high_watermark) {
        cb->stats.high_watermark = occupied;
    }
    uint16_t head = cb->head;
    head = (head + count) - (head + count >= cb->capacity ? cb->capacity : 0);
    cb->head = head;
}

static inline void elcore_rstream_releaseRead(elcore_rstream_t* cb, uint16_t count) {
    uint16_t tail = cb->tail;
    cb->stats.bytes_read += count * cb->item_size;
    tail = (tail + count) - (tail + count >= cb->capacity ? cb->capacity : 0);
    cb->tail = tail;
}

static inline bool elcore_rstream_reserveWrite(elcore_rstream_t* cb,
                                       uint16_t count,
                                       void** writeptr1,
                                       uint16_t* cont1,
                                       void** writeptr2,
                                       uint16_t* cont2)
{
    if (count > elcore_rstream_freeSpace(cb))
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

static inline void elcore_rstream_reserveWriteOverride(
                                       elcore_rstream_t* cb,
                                       uint16_t count,
                                       void** writeptr1,
                                       uint16_t* cont1,
                                       void** writeptr2,
                                       uint16_t* cont2)
{
    uint16_t free = elcore_rstream_freeSpace(cb);

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

static inline bool elcore_rstream_peekRead(elcore_rstream_t* cb,
                                      void** readptr1,
                                      uint16_t* cont1,
                                      void** readptr2,
                                      uint16_t* cont2)
{
    if (elcore_rstream_occupied(cb) == 0)
    {
        *readptr1 = NULL;
        *cont1 = 0;
        *readptr2 = NULL;
        *cont2 = 0;
        return false;
    }

    uint16_t cont_space;
    uint16_t count = elcore_rstream_occupied(cb);

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
