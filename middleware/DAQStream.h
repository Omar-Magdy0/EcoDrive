#pragma once
#include "IDV.h"

namespace DAQ_IDV
{
    enum class ID : uint8_t
    {
        ID = 0,
        PARENT_ID,
        META,
        STREAM_COUNT,
        CHANNEL_COUNT,
        SAMPLETIME,
        TICKS,
        SCALE,
        OFFSET,
        COMPRESSION,
        MARKER = 0xFF,
    };

    enum class MARKER : uint8_t
    {
        AUTO_DISCOVER_REQ = 0,
        AUTO_DISCOVER_REP,
        SESSION,
        STREAM,
        CHANNEL,
    };

    constexpr uint8_t ENTRY(MARKER m) { return (uint8_t)m;}
    constexpr uint8_t EXIT(MARKER m) { return ((uint8_t)m|0x80);}
};

class DAQStream;
class DAQChannel;
class DAQSession;

class DAQChannel
{
    friend DAQStream;
    friend DAQSession;
    uint8_t *sample_buffer;
    uint8_t sample_buffer_size;
    uint8_t id;

public:
    const char *Meta;
    uint8_t meta_size = 0;
    float scale;
    float offset;
    IDVErr serialize(IDV &writer, uint8_t parent_id);
};

class DAQStream
{
    friend DAQSession;
    int32_t ticks;
    int32_t sample_time_ns;
    uint8_t id;

public:
    const char *Meta;
    uint8_t meta_size = 0;
    DAQChannel *channels;
    uint8_t channels_num = 0;

    IDVErr serialize(IDV &writer, uint8_t parent_id);
};

class DAQSession
{
protected:
    IDV writer;
    IDVReader reader;
    uint8_t id;
public:
    const char *Meta;
    uint8_t meta_size = 0;
    DAQStream *streams;
    uint8_t streams_num = 0;
    DAQSession(uint8_t *write_buffer, uint16_t buffer_capacity) : writer(write_buffer, buffer_capacity){}
    virtual uint8_t send() = 0;
    virtual uint8_t on_mark(DAQ_IDV::MARKER mark, bool entry){return 0;};
    virtual uint8_t on_field(DAQ_IDV::ID field, IDVReader &r){return 0;};
    void annonate();
    IDVErr discovery_req();
    IDVErr discovery_respond();
    IDVErr serialize(IDV &writer, uint8_t parent_id);
    uint8_t process(const uint8_t *data, uint16_t data_len);
};