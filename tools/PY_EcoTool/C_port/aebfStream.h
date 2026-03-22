// aebfStream.h
#pragma once

#include <stdint.h>
#include <stdbool.h>


//==================DESCRIPTION============================
/*
    
    AEBFV0 (Asynchronous Expandable Binary frame version 0)

    Frame format:
    [0x00 SYNC_BYTE][1Byte Payload Size][1Byte MCOBS_CODE][MCOBS_ENCODED([6Bit DeviceID][10Bit ServiceID][Payload][CRC16])]
    
    CRC if for entire frame including all bytes except the SYNC_BYTE
    total Frame encoding overhead
    1(0x00) + 1(Payload Size) + 2(DeviceID+ServiceID) + 1(MCOBS firstCodeByte) + 2(CRC16) = 7
*/

// ================== FUNCTION PROTOTYPES ==================
#define AEBF_FRAMING_BYTESNUM 7
#define AEBF_FRAMED_SIZE(s)(s + AEBF_FRAMING_BYTESNUM)
// ================== CONSTANTS ==================
#define AEBF_SYNC_BYTE          0x00
#define AEBF_MAX_PAYLOAD_SIZE   245
#define AEBF_MAX_DEVICES        63      // 6 bits = 63
#define AEBF_MAX_SERVICES       1023    // 10 bits = 1023

#define AEBF_LOG_SERVICEID 0x0F
#define AEBF_DEBUG_SERVICEID 0x0E

typedef enum {
    AEBF_ERROR_FRAME_OK = 0,
    AEBF_ERROR_PAYLOAD_SIZE = 1,
    AEBF_ERROR_CRC_MISMATCH = 2,
    AEBF_ERROR_FRAME_TOO_LARGE = 3,
    AEBF_ERROR_FRAME_LENGTH_MISMATCH = 4,
    AEBF_ERROR_UNKNOWN = 255
} aebf_error_code_t;


#ifdef __cplusplus
extern "C" {
#endif

bool aebf_is_frame_cplt(uint8_t* frame_buffer, uint16_t frame_len);

uint8_t aebf_encode_frame(uint8_t* output_buffer,
                            uint8_t device_id, uint16_t service_id,
                            uint8_t payload_len);

uint8_t aebf_decode_frame(uint8_t* frame_buffer, uint16_t frame_len,
                            uint8_t *device_id, uint16_t *service_id,
                           uint8_t *payload_len);


#ifdef __cplusplus
}
#endif

/**
 * Command Metadata - extracted from frame header
 * Use this for ISR-safe RTOS queue enqueuing (small, fixed-size)
 * 
 * USAGE PATTERN:
 *   1. In on_frame callback (ISR context, must be fast):
 *      - Decode header to extract service/device
 *      - Create cmd_msg_t with metadata
 *      - xQueueSendFromISR() to RTOS queue
 *      - Return immediately
 *   
 *   2. In worker task (can block, can be slow):
 *      - xQueueReceive() to get cmd_msg
 *      - Call aebf_decode(msg.frame_ptr, msg.frame_len)
 *      - Process payload (take time, do I/O, etc.)
 *      - Loop back
 */

typedef enum {
    STATE_HUNT_SYNC,       // Looking for sync byte (0x00)
    STATE_READ_LENGTH,     // Got sync, expecting length byte
    STATE_READ_RESTFRAME,    // Reading payload data
    STATE_VALIDATE         // Frame complete, ready to validate
} aebfStream_state_t;

/**
 * AEBF Stream Parser - Simple Linear Buffer + State Machine
 * 
 * Design:
 *  - Single frame accumulation buffer (260 bytes max)
 *  - Byte-by-byte state machine (accumulate into buffer)
 *  - When frame complete: on_frame callback fires
 *  - Then reset for next frame
 *  - No wraparound, no complex logic
 * 
 * ISR-Safety:
 *  - aebfStream_process() is safe to call from ISR/serial thread
 *  - on_frame callback fires in same ISR context (must be fast <1ms)
 *  - Use callback to enqueue metadata to RTOS queue only
 *  - Heavy processing in separate worker task
 */

typedef struct {
    // ===== Frame Accumulation Buffer =====
    uint8_t *frame_buf;   // Single frame (sync + len + hdr + payload + crc)
    uint16_t frame_buf_size;
    uint16_t frame_pos;                        // Current position in frame_buf
    uint16_t frame_buf_end;
    
    // ===== Frame Parsing State =====
    aebfStream_state_t state;
    
    // Current frame metadata (populated as bytes arrive)
    uint8_t  frame_length;     // Payload length from frame's length byte
    uint16_t bytes_expected;   // Total bytes expected (sync + len + hdr + payload + crc)
    
    // ===== Statistics for Error Detection =====
    uint32_t frames_received;
    uint32_t frames_valid;
    uint32_t frames_corrupted;
    uint32_t resync_events;
    uint32_t consecutive_valid;
    uint8_t  consecutive_errors;
    
    // ===== Callbacks (called from ISR context) =====
    void (*on_frame)(const uint8_t deviceID, const uint16_t serviceID, const uint8_t* payload, const uint16_t payload_len, void* user_data);
    void (*on_error)(char error_code, void* user_data);
    void* user_data;
    
} aebfStream_t;


/**
 * Initialize stream parser with circular buffer
 * 
 * @param stream      Pointer to stream structure
 * @param on_frame    Callback when frame complete (ISR context, must be fast!)
 * @param on_error    Callback for frame errors
 * @param user_data   User context passed to callbacks
 * 
 * CALLBACK RECOMMENDATION:
 * In on_frame, extract device/service from frame header and enqueue cmd_msg_t to
 * RTOS queue instead of processing directly. Processing should happen in worker task.
 */
void aebfStream_init(aebfStream_t* stream,
                      uint8_t* buffer, 
                      uint16_t buffer_size,
                      void (*on_frame)(const uint8_t deviceID, const uint16_t serviceID, const uint8_t* payload, const uint16_t payload_len, void* user_data),
                      void (*on_error)(char, void*),
                      void* user_data);

/**
 * Process incoming serial data bytes
 * 
 * Byte-by-byte state machine:
 *  1. Adds bytes to circular buffer (head pointer increments)
 *  2. Runs state machine until no more complete frames
 *  3. When frame complete: calls on_frame(frame_ptr, frame_len)
 *  4. Continues hunting for next sync byte
 * 
 * Frame detection is BYTE-BY-BYTE as data arrives:
 *  - Byte 0: Hunt for sync (0x00)
 *  - Byte 1: Read payload length
 *  - Bytes 2-3: Read device/service IDs
 *  - Bytes 4..N: Read payload
 *  - Bytes N+1-N+2: Read CRC
 *  - Then validate and call on_frame
 * 
 * @param stream     Stream context
 * @param data       Incoming bytes from serial (can be 1 byte or many)
 * @param length     Number of bytes to process
 * 
 * @note ISR-SAFE: No blocking, no malloc, O(1) per byte
 * @note Callbacks fire in ISR context: keep callback fast (<1ms)
 */
void aebfStream_process(aebfStream_t* stream, const uint8_t* data, uint16_t length);

/**
 * Reset stream state (for error recovery)
 * Clears all state, does NOT clear buffer (allows re-reading)
 */
void aebfStream_reset(aebfStream_t* stream);

/**
 * Clear the buffer completely
 * Use when you need to discard all pending data
 */
void aebfStream_clear(aebfStream_t* stream);

/**
 * Get stats for diagnostics
 */
void aebfStream_get_stats(aebfStream_t* stream,
                           uint32_t* frames_rx,
                           uint32_t* frames_valid,
                           uint32_t* frames_corrupted,
                           uint32_t* resyncs);
