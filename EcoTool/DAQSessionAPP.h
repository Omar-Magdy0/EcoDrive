#pragma once
#include "Usbxch.h"
#include "ABFStream.h"
#include "DAQStream.h"
#include <unordered_map>
#include <span>
#include <vector>

struct Field
{
    std::string name;
    std::vector<uint8_t> value;

    template <typename T>
    void set(const T &v)
    {
        static_assert(std::is_trivially_copyable_v<T>);

        value.resize(sizeof(T));
        std::memcpy(value.data(), &v, sizeof(T));
    }

    void set(const void *data, size_t size)
    {
        const auto *p = static_cast<const uint8_t *>(data);
        value.assign(p, p + size);
    }

    template <typename T>
    T get() const
    {
        static_assert(std::is_trivially_copyable_v<T>);

        if (value.size() != sizeof(T))
            throw std::runtime_error("Bad field size");

        T v;
        std::memcpy(&v, value.data(), sizeof(T));
        return v;
    }

    std::string_view asString() const
    {
        return {
            reinterpret_cast<const char *>(value.data()),
            value.size()};
    }

    const std::vector<uint8_t>& asBytes() const
    {
        return value;
    }
};

struct Object
{
    uint8_t pid;
    DAQ_IDV::MARKER marker;
    std::unordered_map<DAQ_IDV::ID, Field> fields;
};

inline constexpr std::array<std::string_view, 10> ID_NAMES =
    {
        "ID",
        "PARENT_ID",
        "META",
        "STREAM_COUNT",
        "CHANNEL_COUNT",
        "SAMPLETIME",
        "TICKS",
        "SCALE",
        "OFFSET",
        "COMPRESSION",
};

class DAQSessionAPP : public DAQSession
{
protected:
    ABFStream &abf;
    IUsbxch &usb;
    Object c_obj;
    DAQ_IDV::MARKER c_mark;
    uint8_t c_id = 0;
    uint8_t c_pid = 0;
    bool has_id = false;
    bool has_pid = false;

public:
    std::unordered_map<uint8_t, Object> objects;
    DAQSessionAPP(ABFStream &abf_stream,
                  IUsbxch &usb,
                  uint8_t *idv_buffer,
                  uint16_t idv_buffer_len) : DAQSession(idv_buffer, idv_buffer_len), abf(abf_stream), usb(usb)
    {
    }
    uint8_t send() override
    {
        uint8_t abf_header[ABF_HEADER_SIZE];
        abf.encode(abf_header, writer.data(), 16, writer.size());
        usb.write(abf_header, ABF_HEADER_SIZE);
        usb.write(writer.data(), writer.size());
        return 0;
    }
    uint8_t on_mark(DAQ_IDV::MARKER marker, bool entry) override
    {
        switch (marker)
        {
        case DAQ_IDV::MARKER::AUTO_DISCOVER_REP:
            break;
        default: //
            if (entry)
            {
                c_mark = marker;
                has_id = false;
                has_pid = false;
            }
            else
            {
                has_id = false;
                has_pid = false;
            }
            break;
        }
        return 0;
    }
    uint8_t on_field(DAQ_IDV::ID id, IDVReader &reader) override
    {
        DAQ_IDV::ID field_id = id;
        switch (id)
        {
        case DAQ_IDV::ID::ID:
            has_id = true;
            reader.as<uint8_t>(c_id);
            objects[c_id].fields[field_id].set(c_id);
            objects[c_id].marker = c_mark;
            break;
        case DAQ_IDV::ID::PARENT_ID:
            has_pid = true;
            reader.as<uint8_t>(c_pid);
            if (has_id)
            {
                objects[c_id].fields[field_id].set(c_pid);
                objects[c_id].marker = c_mark;
                objects[c_id].pid = c_pid;
            }
            break;
        case DAQ_IDV::ID::META:
            if (has_id)
                objects[c_id].fields[field_id].set(reader.asBinary(), reader.valueLength());
            break;

        default:
            if (has_id)
                objects[c_id].fields[field_id].set(reader.asBinary(), reader.valueLength());
            break;
        }
        if (has_id)
        {
            objects[c_id].fields[field_id].name = ID_NAMES[(uint8_t)field_id];
        }
        return 0;
    }
};