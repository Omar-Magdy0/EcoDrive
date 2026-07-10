#include "ABFStream.h"

// ============================================
// CRC-8/AUTOSAR Lookup Table
// ============================================
static const uint8_t CRC8_AUTOSAR_LUT[] = 
{
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
    0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
    0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
    0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
    0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
    0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
    0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
    0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
    0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
    0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
    0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
    0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
    0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
    0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
    0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42
};

// ============================================
// CRC-8/AUTOSAR Calculator
// ============================================
uint8_t ABFStream::crc8Calculate(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0xFF;  // AUTOSAR init = 0xFF
    
    for (uint8_t i = 0; i < len; i++) {
        crc = CRC8_AUTOSAR_LUT[crc ^ data[i]];
    }
    
    return crc ^ 0xFF;  // AUTOSAR xorout = 0xFF
}

// ============================================
// Constructor & Destructor
// ============================================
ABFStream::ABFStream(uint8_t* buffer, uint8_t bufferSize, void *_context, void(*_onFrame)(void*, uint8_t, uint8_t*, uint8_t), void(*_onError)(void*, uint8_t))
    : m_rxPayload(buffer),
      m_rxCapacity(bufferSize),
      m_rxPos(0),
      m_rxNextZero(0),
      m_headerPos(0),
      m_state(ABFState::HUNT_SYNC),
      context(_context),
      onFrame(_onFrame),
      onError(_onError)

{  
    std::memset(m_header, 0, sizeof(m_header));
}

// ============================================
// Public Methods
// ============================================
ABFErrorCode ABFStream::encode(uint8_t* header, uint8_t* payload, 
                               uint8_t serviceId, uint8_t payloadLen)
{
    if (payloadLen == 0)
        return ABFErrorCode::UNKNOWN;
    if (serviceId == 0)
        return ABFErrorCode::UNKNOWN;
    
    header[0] = ABF_SYNC_BYTE;
    header[1] = serviceId;
    header[2] = payloadLen;
    
    uint8_t* nextZeroPos = header + 3;
    uint8_t zeroDistance = 0;
    
    for (uint8_t i = 0; i < payloadLen; i++) {
        zeroDistance++;
        if (payload[i] == 0) {
            *nextZeroPos = zeroDistance;
            nextZeroPos = payload + i;
            zeroDistance = 0;
        }
    }
    
    *nextZeroPos = zeroDistance + 1;
    
    uint8_t headerCrc = crc8Calculate(&header[1], 3);
    if (headerCrc == 0x00)
        headerCrc = 0xFF;
    
    header[4] = headerCrc;
    return ABFErrorCode::OK;
}

ABFErrorCode ABFStream::process(const uint8_t* data, uint16_t length)
{
    for (int i = 0; i < length; i++) {
        switch (m_state) {
            case ABFState::HUNT_SYNC:
                if (data[i] == ABF_SYNC_BYTE) {
                    m_state = ABFState::READ_HEADER;
                    m_headerPos = 0;
                    m_header[m_headerPos++] = ABF_SYNC_BYTE;
                }
                break;
                
            case ABFState::READ_HEADER:
                m_header[m_headerPos] = data[i];
                if (m_headerPos == 4) {
                    uint8_t crc8 = crc8Calculate(&m_header[1], 3);
                    if (crc8 == 0x00)
                        crc8 = 0xFF;
                    
                    if (crc8 != m_header[4]) {
                        reset();
                        onError(context, (uint8_t)ABFErrorCode::HEADER_CRC_MISMATCH);
                    }
                    
                    m_rxNextZero = m_header[3];
                    m_state = ABFState::READ_PAYLOAD;
                    m_rxPos = 0;
                }
                m_headerPos++;
                break;
                
            case ABFState::READ_PAYLOAD:
                if (m_rxPos < m_header[2]) {
                    if (data[i] == 0x00) {
                        reset();
                        onError(context, (uint8_t)ABFErrorCode::PAYLOAD_INCOMPLETE);
                    }
                    
                    if (m_rxPos < m_rxCapacity) {
                        if (m_rxNextZero == 1) {
                            m_rxPayload[m_rxPos++] = 0x00;
                            m_rxNextZero = data[i];
                        } else {
                            m_rxPayload[m_rxPos++] = data[i];
                            m_rxNextZero--;
                        }
                    } else {
                        reset();
                        onError(context, (uint8_t)ABFErrorCode::RX_BUFFER_SMALL);
                    }
                }
                
                if (m_rxPos == m_header[2]) {
                    reset();
                    uint8_t serviceId = m_header[1];
                    uint8_t payloadLength = m_header[2];
                    onFrame(context, serviceId, m_rxPayload, payloadLength);
                }
                break;
                
            default:
                break;
        }
    }
    return ABFErrorCode::OK;
}

void ABFStream::reset()
{
    m_state = ABFState::HUNT_SYNC;
    m_rxPos = 0;
    m_headerPos = 0;
}

ABFStats ABFStream::getStats() const
{
    return m_stats;
}
