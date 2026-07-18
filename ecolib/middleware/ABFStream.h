#pragma once

#include <stdint.h>
#include <cstring>

/**
 * @brief ABFV0 (Asynchronous Binary Frame version 0)
 *
 * Frame format:
 * [0x00 SYNC_BYTE][1Byte ServiceID][1Byte Payload Size][1Byte MCOBS_CODE][CRC8][MCOBS_ENCODED(Payload)]
 * Total Header: 5 bytes (0x00 + Size + ServiceID + MCOBS_Code + CRC8)
 */

/**
 * @brief ABF (Asynchronous Binary Frame) Protocol Handler
 *
 * Handles encoding and decoding of ABF protocol frames with MCOBS encoding
 * and CRC-8 AUTOSAR error detection.
 */
namespace abf
{
    // ================== CONSTANTS ==================
    constexpr uint8_t HEADER_SIZE = 5;
    constexpr uint8_t SYNC_BYTE = 0x00;
    constexpr uint8_t MAX_PAYLOAD_SIZE = 255;

    // ================== ENUMS ==================
    enum class Err : uint8_t
    {
        OK = 0,
        FRAME_COMPLETE = 1,
        PAYLOAD_INCOMPLETE = 2,
        PAYLOAD_LARGER_THAN_EXPECTED = 3,
        HEADER_CRC_MISMATCH = 4,
        RX_BUFFER_SMALL = 5,
        UNKNOWN = 255
    };

    enum class State : uint8_t
    {
        HUNT_SYNC = 0,    // Looking for sync byte (0x00)
        READ_HEADER = 1,  // Got sync, expecting length byte
        READ_PAYLOAD = 2, // Reading payload data
        VALIDATE = 3      // Frame complete, ready to validate
    };

    // ================== STRUCTURES ==================
    struct Stats
    {
        uint32_t framesReceived = 0;
        uint32_t framesValid = 0;
        uint32_t framesCorrupted = 0;
        uint32_t resyncEvents = 0;
    };
    
    class Stream
    {

    public:
        /**
         * @brief Construct Stream with a pre-allocated buffer
         *
         * @param buffer Pointer to receive payload buffer
         * @param bufferSize Maximum payload buffer size
         */
        Stream(uint8_t *buffer, uint8_t bufferSize, void *_context, void (*onFrame)(void *context, uint8_t, uint8_t *, uint8_t), void (*onError)(void *context, uint8_t));

        /**
         * @brief Destructor
         */
        ~Stream() = default;

        /**
         * @brief Process incoming data stream
         *
         * @param data Input data bytes
         * @param length Number of bytes in input
         * @param serviceId [out] Service ID of decoded frame
         * @param payloadLength [out] Length of decoded payload
         * @return Err status
         */
        Err process(const uint8_t *data, uint16_t length);

        /**
         * @brief Encode a frame for transmission
         *
         * @param header [out] Header buffer (must be at least 5 bytes)
         * @param payload Payload data to encode
         * @param serviceId Service identifier
         * @param payloadLen Payload length
         * @return Err status
         */
        Err encode(uint8_t *header, uint8_t *payload,
                   uint8_t serviceId, uint8_t payloadLen);

        /**
         * @brief Reset parser state machine
         */
        void reset();

        /**
         * @brief Get protocol statistics
         *
         * @return Stats structure
         */
        Stats getStats() const;

    private:
        // ===== Frame Accumulation Buffer =====
        uint8_t *m_rxPayload;
        uint16_t m_rxCapacity;
        uint16_t m_rxPos;
        uint8_t m_rxNextZero;
        uint8_t m_header[HEADER_SIZE];
        uint8_t m_headerPos;

        // ===== Frame Parsing State =====
        State m_state;
        Stats m_stats;
        void *context = nullptr;
        void (*onFrame)(void *context, uint8_t serviceId, uint8_t *payload, uint8_t payloadLength);
        void (*onError)(void *context, uint8_t error);

        /**
         * @brief Calculate CRC-8 AUTOSAR
         *
         * @param data Input data
         * @param len Data length
         * @return CRC-8 value
         */
        static uint8_t crc8Calculate(const uint8_t *data, uint8_t len);
    };
};