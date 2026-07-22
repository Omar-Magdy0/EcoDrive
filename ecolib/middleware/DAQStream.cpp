#include "DAQStream.h"
#include <string.h>
using namespace daq;
void Session::annonate()
{
    uint8_t cid = 0;
    id = cid++;
    for(size_t i = 0; i < streams.size(); i++)
    {
        streams[i].id = cid++;
        for(size_t x = 0; x < streams[i].channels.size(); x++)
        {
            streams[i].channels[x].id = cid++;
        }
    }
}

idv::Err Session::discovery_req()
{
    idv::Err err = idv::Err::OK;
    writer.reset();
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::AUTO_DISCOVER_REQ));
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::AUTO_DISCOVER_REQ));
    send();writer.reset();
    return err;
}

idv::Err Session::discovery_respond()
{
    idv::Err err = idv::Err::OK; // SEND session information first
    annonate();
    writer.reset();
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::AUTO_DISCOVER_REP));
    send();
    writer.reset();
    serialize(writer, id);
    send();
    writer.reset();
    for (size_t i = 0; i < streams.size(); i++) // Serialize streams
    {
        err = streams[i].serialize(writer, id);
        send();
        writer.reset();
        for (size_t z = 0; z < streams[i].channels.size(); z++) // Serialize channels
        {
            err = streams[i].channels[z].serialize(writer, streams[i].id);
            send();
            writer.reset();
        }
    }
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::AUTO_DISCOVER_REP));
    send();
    writer.reset();
    return err;
}

uint8_t Session::process(const uint8_t *data, uint16_t data_len)
{
    reader.reset();
    while (reader.next())
    {
        ID id = (ID)reader.id();
        if (id == ID::MARKER)
        {
            uint8_t marker = 0xFF;
            bool entry;
            reader.as<uint8_t>(marker);
            entry = !(marker & 0x80);
            marker = marker & ~0x80;
            on_mark((MARKER)marker, entry);
        }
        else
        {
            on_field((ID)id, reader);
        }
    }
    return 0;
}

idv::Err Stream::serialize(idv::Writer &writer, uint8_t parent_id)
{
    idv::Err err = idv::Err::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::STREAM));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, strlen(Meta));
    err = writer.add_UINT8((uint8_t)ID::CHANNEL_COUNT, channels.size());
    err = writer.add_UINT32((uint8_t)ID::SAMPLETIME, sample_time_ns);
    err = writer.add_UINT32((uint8_t)ID::TICKS, ticks);
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::STREAM));
    return err;
}

idv::Err Channel::serialize(idv::Writer &writer, uint8_t parent_id)
{
    idv::Err err = idv::Err::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::CHANNEL));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, strlen(Meta));
    err = writer.add_FLOAT((uint8_t)ID::SCALE, scale);
    err = writer.add_FLOAT((uint8_t)ID::OFFSET, offset);
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::CHANNEL));
    return err;
}

idv::Err Session::serialize(idv::Writer &writer, uint8_t parent_id)
{
    idv::Err err = idv::Err::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::SESSION));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, strlen(Meta));
    err = writer.add_UINT8((uint8_t)ID::STREAM_COUNT, streams.size());
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::SESSION));
    return err;
}
