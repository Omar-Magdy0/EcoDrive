#include <catch2/catch_test_macros.hpp>

#include "DAQCodec.h"
#include "el/ring.h"

#include <vector>
#include <cmath>
#include "common.h"
#include <cstdint>

using namespace daq::codec;
class SignalGenerator
{
public:
    static constexpr float PI = 3.14159265358979323846f;

    static std::vector<int16_t> Sine(
        uint32_t samples,
        float sampleRate,
        float frequency,
        float amplitude,
        float offset = 0.0f,
        float phase = 0.0f)
    {
        std::vector<int16_t> out(samples);

        const float w = 2.0f * PI * frequency;

        for (uint32_t i = 0; i < samples; i++)
        {
            float t = static_cast<float>(i) / sampleRate;

            float value =
                offset +
                amplitude * std::sin(w * t + phase);

            out[i] = static_cast<int16_t>(std::lround(value));
        }

        return out;
    }

    static std::vector<int16_t> Triangle(
        uint32_t samples,
        float amplitude,
        uint32_t period,
        float offset = 0.0f)
    {
        assert(period >= 2);

        std::vector<int16_t> out(samples);

        for (uint32_t i = 0; i < samples; ++i)
        {
            float phase = static_cast<float>(i % period) / period;

            float value;
            if (phase < 0.5f)
                value = 4.0f * amplitude * phase - amplitude;
            else
                value = -4.0f * amplitude * (phase - 0.5f) + amplitude;

            out[i] = static_cast<int16_t>(
                std::lround(offset + value));
        }

        return out;
    }

    static std::vector<int16_t> Constant(
        uint32_t samples,
        int16_t value)
    {
        return std::vector<int16_t>(samples, value);
    }

    static std::vector<int16_t> Step(
        uint32_t samples,
        uint32_t stepIndex,
        int16_t low,
        int16_t high)
    {
        std::vector<int16_t> out(samples);

        for (uint32_t i = 0; i < samples; i++)
        {
            out[i] = (i < stepIndex) ? low : high;
        }

        return out;
    }
};

class CodecTestHelper
{
public:
    explicit CodecTestHelper(Writer &codec, uint8_t channels)
        : codec_(codec), channelCount_(channels)
    {
    }

    void feedSamples(const std::vector<std::vector<int16_t>> &samples)
    {
        assert(!samples.empty());

        const size_t channels = samples.size();
        const size_t sampleCount = samples[0].size();

        for (size_t ch = 1; ch < channels; ++ch)
            assert(samples[ch].size() == sampleCount);

        std::vector<int16_t> frame(channels);

        for (size_t i = 0; i < sampleCount; ++i)
        {
            for (size_t ch = 0; ch < channels; ++ch)
                frame[ch] = samples[ch][i];

            codec_.pushSample(frame.data());
        }
    }

    std::vector<uint8_t> readAll()
    {
        std::vector<uint8_t> output;
        StreamType type;

        uint8_t temp[256];

        while (true)
        {
            uint16_t n = codec_.readNext(temp, sizeof(temp), type);

            if (n == 0)
                break;

            output.insert(output.end(), temp, temp + n);
        }

        return output;
    }

    void flush()
    {
        codec_.flush();
    }

    Writer &codec()
    {
        return codec_;
    }

    Writer &codec_;
    uint8_t channelCount_ = 2; // or obtain from codec
};

static void RunGoldenTest(
    const std::vector<std::vector<int16_t>>& signals)
{
    constexpr int STREAM_BUF_SIZE = 2048;
    constexpr int MARK_BUF_SIZE = 64;

    const size_t channelCount = signals.size();

    uint8_t stream_buf[STREAM_BUF_SIZE];
    ControlMark mark_buf[MARK_BUF_SIZE];
    std::vector<ChannelState> channel_states(channelCount);

    uint32_t counter = 0;

    Writer stream(
        stream_buf,
        STREAM_BUF_SIZE,
        mark_buf,
        MARK_BUF_SIZE,
        channel_states.data(),
        channelCount,
        counter,
        16);

    CodecTestHelper test(stream, channelCount);
    test.feedSamples(signals);

    UnitTest::Print("Codec:\n");
    UnitTest::Print(stream);

    uint8_t output_buf[240];
    StreamType msg;

    while (uint16_t len = stream.readNext(output_buf, sizeof(output_buf), msg))
    {
        UnitTest::Print(
            std::string("MESSAGE TYPE: ")
            + (msg == StreamType::SAMPLE ? "SAMPLE" : "CONTROL")
            + "\n");

        UnitTest::Print(UnitTest::toStringHex(output_buf, len));
        UnitTest::Print("\n");
    }
}

TEST_CASE("CODEC_GOLDEN0")
{
    RunGoldenTest({
        SignalGenerator::Sine(41, 20, 1, 500, 2048, 0)
    });
}

TEST_CASE("CODEC_GOLDEN1")
{
    RunGoldenTest({
        SignalGenerator::Sine(41, 20, 1, 500, 2048, 0),
        SignalGenerator::Triangle(41, 200, 20, 2048)
    });
}

TEST_CASE("CODEC_GOLDEN2")
{
    RunGoldenTest({
        SignalGenerator::Sine(41, 20, 1, 500, 2048, 0),
        SignalGenerator::Triangle(41, 200, 20, 2048),
        SignalGenerator::Sine(41, 20, 1, 500, 2048, 0),
        SignalGenerator::Triangle(41, 200, 20, 2048)
    });
}