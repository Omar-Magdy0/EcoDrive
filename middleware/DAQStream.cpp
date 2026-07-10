#include "DAQStream.h"

using namespace DAQ_IDV;
void DAQSession::annonate()
{
    uint8_t cid = 0;
    id = cid++;
    for(int i = 0; i < streams_num; i++)
    {
        streams[i].id = cid++;
        for(int x = 0; x < streams[i].channels_num; x++)
        {
            streams[i].channels[x].id = cid++;
        }
    }
}

IDVErr DAQSession::discovery_req()
{
    IDVErr err = IDVErr::OK;
    writer.reset();
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::AUTO_DISCOVER_REQ));
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::AUTO_DISCOVER_REQ));
    send();writer.reset();
    return err;
}

IDVErr DAQSession::discovery_respond()
{
    IDVErr err = IDVErr::OK; // SEND session information first
    annonate();
    writer.reset();
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::AUTO_DISCOVER_REP));
    send();
    writer.reset();
    serialize(writer, id);
    send();
    writer.reset();
    for (int i = 0; i < streams_num; i++) // Serialize streams
    {
        err = streams[i].serialize(writer, id);
        send();
        writer.reset();
        for (int z = 0; z < streams[i].channels_num; z++) // Serialize channels
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


uint8_t DAQSession::process(const uint8_t *data, uint16_t data_len)
{
    reader.reset(data, data_len);
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

IDVErr DAQStream::serialize(IDV &writer, uint8_t parent_id)
{
    IDVErr err = IDVErr::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::STREAM));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, meta_size);
    err = writer.add_UINT8((uint8_t)ID::CHANNEL_COUNT, channels_num);
    err = writer.add_UINT32((uint8_t)ID::SAMPLETIME, sample_time_ns);
    err = writer.add_UINT32((uint8_t)ID::TICKS, ticks);
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::STREAM));
    return err;
}

IDVErr DAQChannel::serialize(IDV &writer, uint8_t parent_id)
{
    IDVErr err = IDVErr::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::CHANNEL));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, meta_size);
    err = writer.add_FLOAT((uint8_t)ID::SCALE, scale);
    err = writer.add_FLOAT((uint8_t)ID::OFFSET, offset);
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::CHANNEL));
    return err;
}

IDVErr DAQSession::serialize(IDV &writer, uint8_t parent_id)
{
    IDVErr err = IDVErr::OK;
    err = writer.add_UINT8((uint8_t)ID::MARKER, ENTRY(MARKER::SESSION));
    err = writer.add_UINT8((uint8_t)ID::ID, id);
    err = writer.add_UINT8((uint8_t)ID::PARENT_ID, parent_id);
    err = writer.add_BINARY((uint8_t)ID::META, Meta, meta_size);
    err = writer.add_UINT8((uint8_t)ID::STREAM_COUNT, streams_num);
    err = writer.add_UINT8((uint8_t)ID::MARKER, EXIT(MARKER::SESSION));
    return err;
}
