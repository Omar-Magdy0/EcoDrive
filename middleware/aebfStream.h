// aebfStream.h
#pragma once

#include <stdint.h>
#include <stdbool.h>


//==================DESCRIPTION============================
/*
    
    AEBFV0 (Asynchronous Expandable Binary frame version 0)

    Frame format:
    [0x00 SYNC_BYTE][1Byte Payload Size][1Byte ServiceID][1Byte MCOBS_CODE][CRC8][MCOBS_ENCODED(Payload)]
    1(0x00) + 1(Payload Size) + 1(ServiceID) + 1(MCOBS firstCodeByte) + 1(CRC8) = 5
*/

// ================== FUNCTION PROTOTYPES ==================
#define AEBF_HEADER_SIZE 5
// ================== CONSTANTS ==================
#define AEBF_SYNC_BYTE          0x00
#define AEBF_MAX_PAYLOAD_SIZE   255

typedef enum {
    AEBF_ERROR_OK = 0,
    AEBF_ERROR_PAYLOAD_INCOMPLETE = 1,
    AEBF_ERROR_PAYLOAD_LARGER_THAN_EXPECTED = 2,
    AEBF_ERROR_CRC_MISMATCH = 3,
    AEBF_ERROR_UNKNOWN = 255
} aebf_error_code_t;

typedef enum {
    STATE_HUNT_SYNC,       // Looking for sync byte (0x00)
    STATE_READ_HEADER,     // Got sync, expecting length byte
    STATE_READ_PAYLOAD,    // Reading payload data
    STATE_VALIDATE         // Frame complete, ready to validate
} aebfStream_state_t;

typedef struct {
    uint32_t frames_received;
    uint32_t frames_valid;
    uint32_t frames_corrupted;
    uint32_t resync_events;
} aebfStream_stats_t;

typedef struct {
    // ===== Frame Accumulation Buffer =====
    uint16_t frame_buf_size;
    uint16_t frame_pos;                        // Current position in frame_buf
    uint16_t frame_buf_end;
    
    // ===== Frame Parsing State =====
    aebfStream_state_t state;
    
    // Current frame metadata (populated as bytes arrive)
    uint16_t bytes_expected;   // Total bytes expected (sync + len + hdr + payload + crc)
    
    // ===== Statistics for Error Detection =====
    aebfStream_stats_t stats;
    
    // ===== Callbacks (called from ISR context) =====
    void (*on_frame)(const uint16_t serviceID, const uint8_t* payload, const uint16_t payload_len, void* user_data);
    void (*on_error)(char error_code, void* user_data);    
} aebfStream_t;

void aebfStream_init(aebfStream_t* stream,
                      uint8_t* buffer, 
                      uint16_t buffer_size,
                      void (*on_frame)(const uint8_t serviceID, const uint8_t* payload, const uint16_t payload_len, void* user_data),
                      void (*on_error)(char, void* user_data)
                    );

       //             
void aebfStream_process(aebfStream_t* stream, const uint8_t* data, uint16_t length);

void aebfStream_reset(aebfStream_t* stream);

uint8_t aebfStream_encode_frame(uint8_t* output_buffer,
                            uint8_t service_id,
                            uint8_t payload_len);

uint8_t aebfStream_decode_frame(uint8_t* frame_buffer, uint16_t frame_len,
                            uint8_t *service_id,
                            uint8_t *payload_len);

aebfStream_stats_t aebfStream_get_stats(aebfStream_t* stream);
