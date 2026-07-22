#pragma once
#include "IDV.h"
#include "DAQCodec.h"
#include <etl/span.h>
namespace daq
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

    constexpr uint8_t ENTRY(MARKER m) { return (uint8_t)m; }
    constexpr uint8_t EXIT(MARKER m) { return ((uint8_t)m | 0x80); }
    class Stream;
    class Channel;
    class Session;


    class Channel
    {
        friend Stream;
        friend Session;
        uint8_t id;
    public:
        const char *Meta;
        float scale;
        float offset;
        idv::Err serialize(idv::Writer &writer, uint8_t parent_id);
    };

    class Stream
    {
        friend Session;
        int32_t ticks;
        int32_t sample_time_ns;
        uint8_t id;
        etl::span<Channel> channels;
        const char *Meta;
        Stream() = delete;
    public:
        Stream(etl::span<Channel> channels_, const char *Meta_): channels(channels_), Meta(Meta_){}
        idv::Err sample(int16_t *samples);
        idv::Err serialize(idv::Writer &writer, uint8_t parent_id);
    };

    class Session
    {
    protected:
        idv::Writer writer;
        idv::Reader reader;
        uint8_t id;
        const char *Meta;
        etl::span<Stream> streams;
        Session() = delete;
    public:
        Session(const char *Meta_, etl::span<uint8_t> writer_buffer) : writer(writer_buffer), Meta(Meta_){}
        void registerStreams(etl::span<Stream> streams_){streams = streams_;}
        virtual uint8_t send() = 0;
        virtual uint8_t on_mark(MARKER mark, bool entry) { return 0; };
        virtual uint8_t on_field(ID field, idv::Reader &r) { return 0; };
        void annonate();
        idv::Err discovery_req();
        idv::Err discovery_respond();
        idv::Err serialize(idv::Writer &writer, uint8_t parent_id);
        uint8_t process(const uint8_t *data, uint16_t data_len);
    };
};


