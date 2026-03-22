#include "aebfStream.h"
#include <string.h>

//   service_id: 0x00 ---> 0x0F  reserved for system use , with CRC
//   service_id: 0x10 ---> 0xC0  reserved for application parameterization and similar , with CRC
//   service_id: 0xC1 ---> 0xFF  reserved for application specific high speed telemetry, no CRC

__attribute__((weak)) uint16_t aebf_crc16_modbus(uint8_t* buf, uint8_t len)
{
    const uint16_t crc16_modbus_LUT[] = 
    {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ buf[i]) & 0xFF;
        crc = (crc >> 8) ^ crc16_modbus_LUT[index];
    }
    
    return crc;
}


// ================== COBS ENCODING ==================
void mcobs_encode(uint8_t* buf, uint8_t len){
    if (len == 0) {
        buf[0] = 0x01;  // Special case for empty data
        return;
    }
    
    uint8_t read_pos = 1;
    uint8_t code_pos = 0;  // Position to write next code byte
    uint8_t distance = 1;        // Distance to next zero
    
    while (read_pos < len) {
        if (buf[read_pos] == 0) {
            // Found zero: write distance at code_pos
            buf[code_pos] = distance;
            // Start new block
            code_pos = read_pos;
            distance = 1;
        } else {
            distance++;
        }
        read_pos++;
    }
    // Write last distance
    buf[code_pos] = distance;
}

// ================== COBS DECODE ==================
void mcobs_decode(uint8_t* buf, uint8_t len){
    uint8_t distance = buf[0] - 1;
    uint8_t read_pos = 1;
    
    while (read_pos < len) {
        if (distance == 0) {
            // Found zero: write distance at code_pos
            distance = buf[read_pos] - 1;
            buf[read_pos] = 0;
            // Start new block
        } else {
            distance--;
        }
        read_pos++;
    }
    // Write last distance
    buf[read_pos] = 0;
    
}


bool aebf_is_frame_cplt(uint8_t* frame_buffer, uint16_t frame_len)
{
    if(frame_buffer[0] != 0x00) return false;
    if(frame_buffer[1] != (frame_len - AEBF_FRAMING_BYTESNUM) ) return false;
    return true;
}


uint8_t aebf_encode_frame(uint8_t* output_buffer,
                        uint8_t device_id, uint16_t service_id, 
                        uint8_t payload_len)
{
    bool has_crc = false;
    if (payload_len > AEBF_MAX_PAYLOAD_SIZE) return false;
    if (device_id > AEBF_MAX_DEVICES || service_id > AEBF_MAX_SERVICES) return false;

    output_buffer[0] = 0x00;
    output_buffer[1] = payload_len;
    
    //SHOULD BE PROPER CRC
    output_buffer[3] = (device_id << 2) | (service_id >> 8);
    output_buffer[4] = service_id & 0xFF;
    if(service_id >= 0x00 && service_id <= 0xC0) has_crc = true;
    uint16_t crc16_modbus;
    if(has_crc){
        crc16_modbus = aebf_crc16_modbus(output_buffer + 3, payload_len + 2);
    }else{
        crc16_modbus = 0xFFFF;
    }
    output_buffer[AEBF_FRAMED_SIZE(payload_len) - 2] = crc16_modbus >> 8;
    output_buffer[AEBF_FRAMED_SIZE(payload_len) - 1] = crc16_modbus & 0xFF;
    mcobs_encode(output_buffer+2, AEBF_FRAMED_SIZE(payload_len) - 2);
    return AEBF_ERROR_FRAME_OK;
}



uint8_t aebf_decode_frame(uint8_t* frame_buffer, uint16_t frame_len,
                      uint8_t* device_id, uint16_t* service_id, 
                      uint8_t* payload_len)
{
    bool has_crc = false;
    if(!aebf_is_frame_cplt(frame_buffer, frame_len)){
        return AEBF_ERROR_FRAME_LENGTH_MISMATCH;
    }else
    {
        *payload_len = frame_buffer[1];
        mcobs_decode(frame_buffer+2, AEBF_FRAMED_SIZE(*payload_len) - 2);

        *device_id = (frame_buffer[3] >> 2) & 0x3F;
        *service_id = ((frame_buffer[3] & 0x03) << 8) | frame_buffer[4];
        if(*service_id >= 0x00 && *service_id <= 0xC0) has_crc = true;
        if(has_crc){
            uint16_t crc16_modbus_calc = aebf_crc16_modbus(frame_buffer + 3, *payload_len + 2);
            uint16_t crc16_modbus_recv = (frame_buffer[AEBF_FRAMED_SIZE(*payload_len) - 2] << 8) | frame_buffer[AEBF_FRAMED_SIZE(*payload_len) - 1];
            if (crc16_modbus_calc != crc16_modbus_recv) return AEBF_ERROR_CRC_MISMATCH;
        }
        return AEBF_ERROR_FRAME_OK;
    }
}

/**
 * Initialize stream with frame accumulation buffer
 */
void aebfStream_init(aebfStream_t* stream,
                        uint8_t* buffer,
                        uint16_t buffer_size,
                        void (*on_frame)(const uint8_t deviceID, const uint16_t serviceID, const uint8_t* payload, const uint16_t payload_len, void* user_data),
                        void (*on_error)(char error_code, void*),
                        void* user_data) {
    if (!stream) return;
    
    memset(stream, 0xFF, sizeof(aebfStream_t));
    
    stream->frame_buf = buffer;
    stream->frame_buf_size = buffer_size;
    stream->frame_pos = 0;
    stream->state = STATE_HUNT_SYNC;
    
    stream->on_frame = on_frame;
    stream->on_error = on_error;
    stream->user_data = user_data;
}

/**
 * Main stream processing - byte-by-byte accumulation
 * 
 * Byte-by-byte state machine with simple linear buffer:
 *  - HUNT_SYNC: Skip bytes until we find 0x00, start accumulating
 *  - READ_LENGTH: Got sync, next byte is payload length
 *  - READ_HEADER: Got length, read device/service (2 bytes)
 *  - READ_PAYLOAD: Read payload bytes
 *  - READ_CRC: Read CRC bytes (2 bytes)
 *  - VALIDATE: Frame complete → call on_frame → reset
 */

void aebfStream_process(aebfStream_t* stream, const uint8_t* data, uint16_t length) {
    if (!stream || !data || length == 0) {
        return;
    }
    
    // Process each incoming byte
    for (uint16_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        
        switch (stream->state) {
            
            case STATE_HUNT_SYNC:
                // Looking for sync byte (0x00)
                if (byte == AEBF_SYNC_BYTE) {
                    // Found sync - start accumulating
                    stream->frame_buf[0] = byte;
                    stream->frame_pos = 1;
                    stream->state = STATE_READ_LENGTH;
                    stream->frames_received++;
                } else {
                    // Not sync, skip (don't accumulate)
                }
                break;
                
            // Got sync, next byte is payload length
            case STATE_READ_LENGTH: {
                stream->frame_buf[stream->frame_pos++] = byte;
                stream->frame_length = byte;
                
                // Expected Remaining Bytes: Cobs Code (1) + device/service(2) + payload(N) + crc(2)
                stream->bytes_expected = AEBF_FRAMED_SIZE(stream->frame_length) - 2;
                
                // Sanity check payload length
                if (stream->bytes_expected > AEBF_FRAMED_SIZE(AEBF_MAX_PAYLOAD_SIZE) - 2 || stream->bytes_expected <= (AEBF_FRAMED_SIZE(0) - 2) ) {
                    // Invalid length, resync
                    if (stream->on_error) {
                        stream->on_error(AEBF_ERROR_PAYLOAD_SIZE, stream->user_data);
                    }
                    stream->frames_corrupted++;
                    stream->frame_pos = 0;
                    stream->state = STATE_HUNT_SYNC;
                } else {
                    stream->state = STATE_READ_RESTFRAME;
                }
                break;
            }
            
            case STATE_READ_RESTFRAME: {                
                stream->frame_buf[stream->frame_pos++] = byte;
                
                if(byte == AEBF_SYNC_BYTE) {
                    // Found sync byte in payload - resync
                    if (stream->on_error) {
                        stream->on_error(AEBF_ERROR_FRAME_LENGTH_MISMATCH, stream->user_data);
                    }
                    stream->frames_corrupted++;
                    stream->frame_pos = 0;
                    stream->state = STATE_HUNT_SYNC;
                    break;
                }

                stream->bytes_expected--;
                if (stream->bytes_expected == 0) {
                    // Frame complete
                    stream->state = STATE_VALIDATE;
                }else{
                    break;
                }
                
            }
            
            
            case STATE_VALIDATE: {
                // Frame is complete in frame_buf
                uint16_t frame_len = stream->frame_pos;
                uint8_t deviceID;
                uint16_t serviceID;
                uint8_t payload_len;

                uint8_t error_code = aebf_decode_frame(stream->frame_buf, frame_len, &deviceID, &serviceID, &payload_len);
                // Call user callback with complete frame
                if (error_code != AEBF_ERROR_FRAME_OK) 
                {
                    if (stream->on_error) {
                        stream->on_error(error_code, stream->user_data);
                    }
                    stream->frames_corrupted++;
                } else
                {
                    if (stream->on_frame) 
                    {
                        stream->on_frame(deviceID, serviceID, stream->frame_buf + 5, payload_len, stream->user_data);
                    }
                    stream->frames_valid++;
                }
                
                stream->frame_pos = 0;
                stream->bytes_expected = 0;
                stream->state = STATE_HUNT_SYNC;
                break;
            }
            
            default:
                stream->state = STATE_HUNT_SYNC;
                break;
        }
    }
}

/**
 * Reset stream state (for error recovery)
 */
void aebfStream_reset(aebfStream_t* stream) {
    if (!stream) return;
    
    stream->frame_pos = 0;
    stream->state = STATE_HUNT_SYNC;
    stream->resync_events++;
}

/**
 * Clear the frame buffer
 */
void aebfStream_clear(aebfStream_t* stream) {
    if (!stream) return;
    
    memset(stream->frame_buf, 0xFF, stream->frame_buf_size);
    stream->frame_pos = 0;
    stream->state = STATE_HUNT_SYNC;
}

/**
 * Get diagnostic statistics
 */
void aebfStream_get_stats(aebfStream_t* stream,
                           uint32_t* frames_rx,
                           uint32_t* frames_valid,
                           uint32_t* frames_corrupted,
                           uint32_t* resyncs) {
    if (!stream) return;
    
    if (frames_rx)        *frames_rx = stream->frames_received;
    if (frames_valid)     *frames_valid = stream->frames_valid;
    if (frames_corrupted) *frames_corrupted = stream->frames_corrupted;
    if (resyncs)          *resyncs = stream->resync_events;
}

