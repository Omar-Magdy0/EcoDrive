#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ABFStream.h"

// ============================================
// TEST HELPER FUNCTIONS
// ============================================

void printHex(const uint8_t* data, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        printf("0x%02X ", data[i]);
    }
}

// ============================================
// TEST CASES
// ============================================

struct TestCase {
    uint8_t serviceId;
    uint8_t payload[64];
    uint8_t payloadLen;
    bool shouldPass;
    const char* description;
};

bool runTest(const TestCase* tc)
{
    ABFStream encoder(nullptr, 256);
    ABFStream decoder(nullptr, 256);
    uint8_t header[5];
    uint8_t receivedService = 0;
    uint8_t receivedLen = 0;
    uint8_t frame[5 + 256];
    uint8_t decodeBuffer[256];
    
    printf("\n--- %s ---\n", tc->description);
    printf("Service: 0x%02X, Len: %d, Payload: ", tc->serviceId, tc->payloadLen);
    printHex(tc->payload, tc->payloadLen);
    printf("\n");
    
    // Encode
    ABFErrorCode encResult = encoder.encode(header, (uint8_t*)tc->payload, 
                                            tc->serviceId, tc->payloadLen);
    
    if (encResult != ABFErrorCode::OK) {
        if (!tc->shouldPass) {
            printf("✓ Correctly rejected (encode error: %d)\n", (int)encResult);
            return true;
        }
        printf("✗ Encode failed: %d\n", (int)encResult);
        return false;
    }
    
    printf("Header: ");
    printHex(header, 5);
    printf("\n");
    printf("  CRC: 0x%02X %s\n", header[4], (header[4] == 0x00) ? "✗ ZERO!" : "✓");
    
    // Build frame
    uint16_t frameLen = 5 + tc->payloadLen;
    if (frameLen > sizeof(frame)) {
        printf("✗ Frame too large for buffer\n");
        return false;
    }
    
    memcpy(frame, header, 5);
    memcpy(frame + 5, tc->payload, tc->payloadLen);
    
    // Decode
    decoder = ABFStream(decodeBuffer, 256);
    ABFErrorCode decResult = decoder.process(frame, frameLen, receivedService, receivedLen);
    
    if (tc->shouldPass) {
        if (decResult == ABFErrorCode::FRAME_COMPLETE) {
            printf("✓ Decode success\n");
            printf("  Received: Service 0x%02X, Len %d\n", receivedService, receivedLen);
            if (receivedService == tc->serviceId && receivedLen == tc->payloadLen) {
                printf("✓ Payload matches\n");
                return true;
            }
            printf("✗ Payload mismatch\n");
            return false;
        }
        printf("✗ Decode failed: %d\n", (int)decResult);
        return false;
    } else {
        if (decResult != ABFErrorCode::FRAME_COMPLETE) {
            printf("✓ Correctly rejected (decode error: %d)\n", (int)decResult);
            return true;
        }
        printf("✗ Should have rejected but passed\n");
        return false;
    }
}

// ============================================
// MAIN
// ============================================

int main()
{
    printf("=== ABF UNIT TESTS ===\n");
    
    int passed = 0;
    int total = 0;
    
    // Test 1: No zeros
    TestCase t1 = {
        .serviceId = 0x05,
        .payload = {0x12, 0x34, 0x56, 0x78},
        .payloadLen = 4,
        .shouldPass = true,
        .description = "No zeros"
    };
    total++; if (runTest(&t1)) passed++;
    
    // Test 2: Zero at start
    TestCase t2 = {
        .serviceId = 0x0A,
        .payload = {0x00, 0x12, 0x34, 0x56},
        .payloadLen = 4,
        .shouldPass = true,
        .description = "Zero at pos 0"
    };
    total++; if (runTest(&t2)) passed++;
    
    // Test 3: Zero in middle
    TestCase t3 = {
        .serviceId = 0x03,
        .payload = {0x12, 0x00, 0x34, 0x56},
        .payloadLen = 4,
        .shouldPass = true,
        .description = "Zero at pos 1"
    };
    total++; if (runTest(&t3)) passed++;
    
    // Test 4: Multiple zeros
    TestCase t4 = {
        .serviceId = 0x07,
        .payload = {0x00, 0x12, 0x00, 0x34, 0x00, 0x56},
        .payloadLen = 6,
        .shouldPass = true,
        .description = "Multiple zeros"
    };
    total++; if (runTest(&t4)) passed++;
    
    // Test 5: Zero at end
    TestCase t5 = {
        .serviceId = 0x0F,
        .payload = {0x12, 0x34, 0x56, 0x00},
        .payloadLen = 4,
        .shouldPass = true,
        .description = "Zero at end"
    };
    total++; if (runTest(&t5)) passed++;
    
    // Test 6: All zeros
    TestCase t6 = {
        .serviceId = 0x01,
        .payload = {0x00, 0x00, 0x00, 0x00},
        .payloadLen = 4,
        .shouldPass = true,
        .description = "All zeros"
    };
    total++; if (runTest(&t6)) passed++;
    
    // Test 7: Service ID = 0 (invalid)
    TestCase t7 = {
        .serviceId = 0x00,
        .payload = {0x12, 0x34, 0x56, 0x78},
        .payloadLen = 4,
        .shouldPass = false,
        .description = "Service ID=0 (should fail)"
    };
    total++; if (runTest(&t7)) passed++;
    
    // Test 8: Empty payload (invalid)
    TestCase t8 = {
        .serviceId = 0x05,
        .payload = {0},
        .payloadLen = 0,
        .shouldPass = false,
        .description = "Empty payload (should fail)"
    };
    total++; if (runTest(&t8)) passed++;
    
    // Test 9: CRC never 0x00 (100 random)
    printf("\n--- CRC never 0x00 (100 random) ---\n");
    ABFStream encoder(nullptr, 256);
    uint8_t header[5];
    int zeroCount = 0;
    
    for (int i = 0; i < 100; i++) {
        uint8_t randPayload[16];
        for (int j = 0; j < 16; j++) {
            randPayload[j] = (uint8_t)(i * 37 + j * 13);
        }
        encoder.encode(header, randPayload, 0x05, 16);
        if (header[4] == 0x00)
            zeroCount++;
    }
    
    if (zeroCount == 0) {
        printf("✓ CRC never 0x00\n");
        passed++;
    } else {
        printf("✗ CRC was 0x00 %d times\n", zeroCount);
    }
    total++;
    
    // Results
    printf("\n=== RESULTS: %d/%d passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}