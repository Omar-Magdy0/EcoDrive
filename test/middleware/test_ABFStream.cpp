#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <vector>

#include "ABFStream.h"
using namespace abf;
namespace {
constexpr uint8_t kBufferCapacity = 255;

struct FrameCapture {
    uint8_t serviceId = 0;
    std::vector<uint8_t> payload;
    uint8_t lastError = 0;
    uint8_t frameCount = 0;
    uint8_t errorCount = 0;
};

void captureFrame(void* context, uint8_t serviceId, uint8_t* payload, uint8_t payloadLength)
{
    auto* capture = static_cast<FrameCapture*>(context);
    capture->serviceId = serviceId;
    capture->payload.assign(payload, payload + payloadLength);
    capture->frameCount++;
}

void captureError(void* context, uint8_t error)
{
    auto* capture = static_cast<FrameCapture*>(context);
    capture->lastError = error;
    capture->errorCount++;
}

std::vector<uint8_t> buildFrame(const std::vector<uint8_t>& payload, uint8_t serviceId)
{
    std::array<uint8_t, abf::HEADER_SIZE> header{};
    std::array<uint8_t, kBufferCapacity> rxBuffer{};
    FrameCapture capture{};
    abf::Stream stream(rxBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);

    std::vector<uint8_t> encodedPayload(payload.begin(), payload.end());
    REQUIRE(stream.encode(header.data(), encodedPayload.data(), serviceId, static_cast<uint8_t>(payload.size())) == abf::Err::OK);

    std::vector<uint8_t> frame;
    frame.insert(frame.end(), header.begin(), header.end());
    frame.insert(frame.end(), encodedPayload.begin(), encodedPayload.end());
    return frame;
}
} // namespace

TEST_CASE("abf::Stream encodes and decodes a known payload", "[abf][encode]")
{
    const std::vector<uint8_t> payload = {0x41, 0x42, 0x43, 0x44};
    const auto frame = buildFrame(payload, 0x21);

    FrameCapture capture{};
    std::array<uint8_t, kBufferCapacity> decoderBuffer{};
    abf::Stream decoder(decoderBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);

    REQUIRE(decoder.process(frame.data(), static_cast<uint16_t>(frame.size())) == abf::Err::OK);
    REQUIRE(capture.frameCount == 1);
    REQUIRE(capture.serviceId == 0x21);
    REQUIRE(capture.payload == payload);
    REQUIRE(capture.errorCount == 0);
}

TEST_CASE("abf::Stream decodes a frame when the bytes arrive one at a time", "[abf][streaming]")
{
    const std::vector<uint8_t> payload = {0x10, 0x20, 0x30, 0x40, 0x50};
    const auto frame = buildFrame(payload, 0x33);

    FrameCapture capture{};
    std::array<uint8_t, kBufferCapacity> decoderBuffer{};
    abf::Stream decoder(decoderBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);

    for (size_t i = 0; i < frame.size(); ++i) {
        REQUIRE(decoder.process(frame.data() + i, 1) == abf::Err::OK);
    }

    REQUIRE(capture.frameCount == 1);
    REQUIRE(capture.serviceId == 0x33);
    REQUIRE(capture.payload == payload);
    REQUIRE(capture.errorCount == 0);
}

TEST_CASE("abf::Stream reports CRC errors and small RX buffers", "[abf][errors]")
{
    const std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    auto frame = buildFrame(payload, 0x44);
    frame[4] ^= 0x01;

    FrameCapture capture{};
    std::array<uint8_t, kBufferCapacity> decoderBuffer{};
    abf::Stream decoder(decoderBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);
    REQUIRE(decoder.process(frame.data(), static_cast<uint16_t>(frame.size())) == abf::Err::OK);
    REQUIRE(capture.frameCount == 1);
    REQUIRE(capture.errorCount == 1);
    REQUIRE(capture.lastError == static_cast<uint8_t>(abf::Err::HEADER_CRC_MISMATCH));

    std::array<uint8_t, 2> smallBuffer{};
    FrameCapture smallCapture{};
    abf::Stream smallDecoder(smallBuffer.data(), static_cast<uint8_t>(smallBuffer.size()), &smallCapture, captureFrame, captureError);
    REQUIRE(smallDecoder.process(frame.data(), static_cast<uint16_t>(frame.size())) == abf::Err::OK);
    REQUIRE(smallCapture.frameCount == 0);
    REQUIRE(smallCapture.errorCount == 2);
    REQUIRE(smallCapture.lastError == static_cast<uint8_t>(abf::Err::RX_BUFFER_SMALL));
}

TEST_CASE("abf::Stream accepts deterministic fuzz payloads", "[abf][fuzz]")
{
    for (int iter = 0; iter < 500; ++iter) {
        std::vector<uint8_t> payload;
        payload.reserve(static_cast<size_t>(1 + (iter % 12)));
        for (int byte = 0; byte < 1 + (iter % 12); ++byte) {
            payload.push_back(static_cast<uint8_t>(0x20 + ((iter * 17 + byte * 13) % 95)));
        }

        const auto frame = buildFrame(payload, static_cast<uint8_t>(0x50 + (iter % 16)));

        FrameCapture capture{};
        std::array<uint8_t, kBufferCapacity> decoderBuffer{};
        abf::Stream decoder(decoderBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);
        REQUIRE(decoder.process(frame.data(), static_cast<uint16_t>(frame.size())) == abf::Err::OK);
        REQUIRE(capture.frameCount == 1);
        REQUIRE(capture.payload == payload);
        REQUIRE(capture.errorCount == 0);
    }
}

TEST_CASE("abf::Stream rejects invalid arguments", "[abf][validation]")
{
    std::array<uint8_t, 8> header{};
    std::array<uint8_t, 8> payload{};
    std::array<uint8_t, kBufferCapacity> rxBuffer{};
    FrameCapture capture{};
    abf::Stream stream(rxBuffer.data(), kBufferCapacity, &capture, captureFrame, captureError);

    REQUIRE(stream.encode(header.data(), payload.data(), 0x00, 4) == abf::Err::UNKNOWN);
    REQUIRE(stream.encode(header.data(), payload.data(), 0x55, 0) == abf::Err::UNKNOWN);
}
