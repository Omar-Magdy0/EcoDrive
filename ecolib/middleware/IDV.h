#pragma once
#include <stdint.h>
#include <string.h>
#include <type_traits>



namespace idv
{
enum class Err : uint8_t
{
    OK = 0,
    BUFFER_FULL,
};

class Writer
{
    uint8_t *buffer;
    uint16_t buffer_capacity;
    uint16_t buffer_idx = 0;

    template <typename T>
    inline Err addPrimitive(uint8_t id, const T &value)
    {
        if (buffer_idx + sizeof(T) + 2 > buffer_capacity)
            return Err::BUFFER_FULL;
        buffer[buffer_idx++] = id;
        buffer[buffer_idx++] = sizeof(T);
        memcpy(buffer + buffer_idx, &value, sizeof(value));
        buffer_idx += sizeof(value);
        return Err::OK;
    }
public:
    Writer() = delete;
    Writer(uint8_t *_buffer, uint16_t _buffer_capacity) : buffer(_buffer), buffer_capacity(_buffer_capacity) {};
    Err add_UINT8(uint8_t id, uint8_t value) { return addPrimitive(id, value); }
    Err add_INT8(uint8_t id, int8_t value) { return addPrimitive(id, value); }
    Err add_UINT16(uint8_t id, uint16_t value) { return addPrimitive(id, value); }
    Err add_INT16(uint8_t id, int16_t value) { return addPrimitive(id, value); }
    Err add_UINT32(uint8_t id, uint32_t value) { return addPrimitive(id, value); }
    Err add_INT32(uint8_t id, int32_t value) { return addPrimitive(id, value); }
    Err add_UINT64(uint8_t id, uint64_t value) { return addPrimitive(id, value); }
    Err add_INT64(uint8_t id, int64_t value) { return addPrimitive(id, value); }
    Err add_FLOAT(uint8_t id, float value) { return addPrimitive(id, value); }
    Err add_DOUBLE(uint8_t id, double value) { return addPrimitive(id, value); }
    Err add_BINARY(uint8_t id, const uint8_t *str, uint8_t length)
    {
        if (buffer_idx + length + 2 > buffer_capacity)
            return Err::BUFFER_FULL;
        buffer[buffer_idx++] = id;
        buffer[buffer_idx++] = length;
        memcpy(buffer + buffer_idx, str, length);
        buffer_idx += length;
        return Err::OK;
    }
    template<typename T>
    Err add_BINARY(uint8_t id,
                      const T* data,
                      uint8_t count)
    {
        static_assert(std::is_trivially_copyable_v<T>, "IDV::add() requires a trivially copyable type");
        return add_BINARY(id,
                          reinterpret_cast<const uint8_t*>(data),
                          sizeof(T) * count);
    }
    template<typename T>
    Err add(uint8_t id, const T& value)
    {
        static_assert(std::is_trivially_copyable_v<T>);
        return add_BINARY(
            id,
            reinterpret_cast<const uint8_t*>(&value),
            sizeof(T));
    }
    template <typename T>
    static constexpr inline uint8_t PRIMITIVE_SIZE() { return sizeof(T) + 2; }
    template <typename T>
    static constexpr inline uint8_t SERIALIZED_SIZE(){return sizeof(T) + 2;}
    static constexpr inline uint8_t BINARY_SIZE(uint8_t length) { return length + 2; }
    uint16_t size() const { return buffer_idx; }
    void reset()
    {
        buffer_idx = 0;
    }
    uint8_t *data() const
    {
        return buffer;
    }
};

class Reader
{
    const uint8_t *data_mem;
    uint16_t length = 0;
    uint16_t data_idx = 0;

    uint8_t idv_id;
    uint8_t idv_len;
    const uint8_t *idv_value;
public:
    Reader() = default;
    Reader(const uint8_t* data, uint16_t length)
    {
        reset(data, length);
    }
    bool next()
    {
        if(data_idx == UINT16_MAX)return false;
        if(data_idx + 2 > length)return false;
        idv_id = data_mem[data_idx++];
        idv_len = data_mem[data_idx++];
        if (data_idx + idv_len > length)
        {
            data_idx = UINT16_MAX;
            return false;
        }
        idv_value = data_mem + data_idx;
        data_idx += idv_len;
        return true;
    }
    template <typename T>
    bool as(T &val) const
    {
        static_assert(std::is_trivially_copyable_v<T>,
              "IDV::as<T>() requires a trivially copyable type");
        if (idv_len != sizeof(T))
            return false;

        memcpy(&val, idv_value, sizeof(T));
        return true;
    }
    void reset(const uint8_t *p_data, uint16_t p_length)
    {
        data_mem = p_data;
        length = p_length;
        data_idx = 0;
    }
    inline const uint8_t *asBinary() const
    {
        return idv_value;
    }
    inline uint8_t valueLength() const
    {
        return idv_len;
    }
    inline uint8_t id() const
    {
        return idv_id;
    }
};

};