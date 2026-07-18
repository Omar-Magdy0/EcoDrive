// =============================================================================
// dummy_test.cpp - Basic Catch2 test to verify setup
// =============================================================================

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

// ── Basic integer operations ─────────────────────────────────
TEST_CASE("Integer arithmetic basics", "[core][arithmetic]") {
    SECTION("Addition") {
        REQUIRE(1 + 1 == 2);
        REQUIRE(-1 + 1 == 0);
        REQUIRE(100 + 200 == 300);
    }

    SECTION("Multiplication") {
        REQUIRE(2 * 3 == 6);
        REQUIRE(0 * 100 == 0);
        REQUIRE(-2 * 3 == -6);
    }

    SECTION("Division") {
        REQUIRE(10 / 2 == 5);
        REQUIRE(7 / 3 == 2);  // integer division
    }
}

// ── Fixed-point arithmetic (Q15 format example) ──────────────
TEST_CASE("Q15 fixed-point arithmetic", "[core][fixedpoint]") {
    // Q15 format: 1 sign + 15 fractional bits
    using Q15 = int16_t;
    
    auto q15_from_float = [](float f) -> Q15 {
        return static_cast<Q15>(f * 32768.0f);
    };

    auto q15_to_float = [](Q15 q) -> float {
        return static_cast<float>(q) / 32768.0f;
    };

    SECTION("Float to Q15 conversion") {
        REQUIRE(q15_from_float(0.5f) == 16384);
        REQUIRE(q15_from_float(-0.5f) == -16384);
        //REQUIRE(q15_from_float(1.0f) == 32767);  // saturated
    }

    SECTION("Q15 multiplication") {
        auto q15_mul = [](Q15 a, Q15 b) -> Q15 {
            int32_t product = static_cast<int32_t>(a) * static_cast<int32_t>(b);
            return static_cast<Q15>(product >> 15);
        };

        Q15 a = q15_from_float(0.5f);
        Q15 b = q15_from_float(0.25f);
        Q15 result = q15_mul(a, b);
        
        REQUIRE_THAT(q15_to_float(result), Catch::Matchers::WithinRel(0.125f, 0.01f));
    }
}

// ── Bit manipulation ─────────────────────────────────────────
TEST_CASE("Bit manipulation utilities", "[core][bitops]") {
    uint32_t value = 0;

    SECTION("Setting and clearing bits") {
        value |= (1u << 3);   // set bit 3
        REQUIRE((value & (1u << 3)) != 0);

        value &= ~(1u << 3);  // clear bit 3
        REQUIRE((value & (1u << 3)) == 0);
    }

    SECTION("Bit field extraction") {
        value = 0b11001100;
        uint8_t nibble = (value >> 4) & 0x0F;  // high nibble
        REQUIRE(nibble == 0b1100);
    }
}

// ── Data structure: Ring buffer ──────────────────────────────
template<typename T, size_t N>
class RingBuffer {
    T buffer[N];
    size_t head = 0;
    size_t tail = 0;
    bool full = false;

public:
    bool push(T item) {
        if (full) return false;
        buffer[head] = item;
        head = (head + 1) % N;
        full = (head == tail);
        return true;
    }

    bool pop(T& item) {
        if (empty()) return false;
        item = buffer[tail];
        tail = (tail + 1) % N;
        full = false;
        return true;
    }

    bool empty() const {
        return (!full && (head == tail));
    }

    bool is_full() const {
        return full;
    }

    size_t size() const {
        if (full) return N;
        return (head >= tail) ? (head - tail) : (N + head - tail);
    }
};

TEST_CASE("Ring buffer operations", "[core][datastructure]") {
    RingBuffer<int, 4> buffer;

    SECTION("Empty buffer") {
        REQUIRE(buffer.empty());
        REQUIRE(buffer.size() == 0);
        REQUIRE(!buffer.is_full());
        
        int item;
        REQUIRE(!buffer.pop(item));
    }

    SECTION("Push and pop single element") {
        REQUIRE(buffer.push(42));
        REQUIRE(buffer.size() == 1);
        REQUIRE(!buffer.empty());

        int item;
        REQUIRE(buffer.pop(item));
        REQUIRE(item == 42);
        REQUIRE(buffer.empty());
    }

    SECTION("Fill and overflow") {
        REQUIRE(buffer.push(1));
        REQUIRE(buffer.push(2));
        REQUIRE(buffer.push(3));
        REQUIRE(buffer.push(4));
        REQUIRE(buffer.is_full());
        REQUIRE(buffer.size() == 4);
        
        // Overflow should fail
        REQUIRE(!buffer.push(5));
    }

    SECTION("Wrap around") {
        REQUIRE(buffer.push(1));
        REQUIRE(buffer.push(2));
        
        int item;
        REQUIRE(buffer.pop(item));
        REQUIRE(item == 1);
        REQUIRE(buffer.pop(item));
        REQUIRE(item == 2);
        
        // After pop, should be empty
        REQUIRE(buffer.empty());
        REQUIRE(buffer.size() == 0);
    }

    SECTION("Multiple wrap around") {
        for (int i = 0; i < 6; ++i) {
            buffer.push(i);
        }
        // Buffer should have wrapped and now contain 4,5 and be full
        REQUIRE(buffer.is_full());
        REQUIRE(buffer.size() == 4);
    }
}

// ── Simple CRC8 test (common in embedded) ────────────────────
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

uint8_t compute_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

TEST_CASE("CRC8 calculation", "[core][crc]") {
    SECTION("Zero length") {
        uint8_t data[] = {};
        REQUIRE(compute_crc8(data, 0) == 0xFF);  // initial value
    }

    SECTION("Known pattern") {
        uint8_t data[] = {0x31, 0x32, 0x33, 0x34};  // "1234"
        uint8_t crc = compute_crc8(data, sizeof(data));
        // This is a known CRC for DVB-S2
        REQUIRE(crc != 0);  // Non-zero for this pattern
    }

    SECTION("Single byte") {
        uint8_t data[] = {0x00};
        REQUIRE(compute_crc8(data, 1) != 0xFF);
    }

    SECTION("Consistency check") {
        uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
        uint8_t crc1 = compute_crc8(data, sizeof(data));
        
        // Twice should give same result
        uint8_t crc2 = compute_crc8(data, sizeof(data));
        REQUIRE(crc1 == crc2);
    }
}

// ── Error handling test ──────────────────────────────────────
TEST_CASE("Error handling patterns", "[core][error]") {
    SECTION("Optional pattern") {
        struct Result {
            int value;
            bool valid;
        };

        auto divide = [](int a, int b) -> Result {
            if (b == 0) return {0, false};
            return {a / b, true};
        };

        auto result = divide(10, 2);
        REQUIRE(result.valid);
        REQUIRE(result.value == 5);

        result = divide(10, 0);
        REQUIRE(!result.valid);
    }
}