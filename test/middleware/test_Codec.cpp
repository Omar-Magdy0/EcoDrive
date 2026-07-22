#include <catch2/catch_test_macros.hpp>

#include "DAQCodec.h"
#include "el/ring.h"

#include <vector>
#include <cmath>
#include "common.h"
#include <cstdint>
#include <random>

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

struct CodecTestConfig
{
    std::vector<std::vector<int16_t>> samples;
    uint16_t stream_buffer_size = 1024;
    uint16_t mark_buffer_size = 64;
    uint16_t sample_buffer_size = 256;
    uint8_t resync_period = 16;
    std::vector<unsigned int> feed_vector;
};
constexpr size_t MaxChannels = 32;

class CodecTest
{
public:
    CodecTest(CodecTestConfig &cfg_) : stream_buf(cfg_.stream_buffer_size),
                                       mark_buf(cfg_.mark_buffer_size),
                                       channels(cfg_.samples.size()),
                                       w_ccs_buf(channels),
                                       r_ccs_buf(channels),
                                       dod_buf(channels),
                                       block_buf(channels*cfg.sample_buffer_size),
                                       cfg(cfg_),
                                       block(etl::span(block_buf.data(), block_buf.size()), cfg_.samples.size()),
                                       writer(etl::span(stream_buf.data(), stream_buf.size()), etl::span(mark_buf.data(), mark_buf.size()), etl::span(w_ccs_buf.data(), w_ccs_buf.size()), sample_counter, cfg_.resync_period),
                                       reader(etl::span(r_ccs_buf.data(), r_ccs_buf.size()), etl::span(dod_buf.data(), dod_buf.size()), block)
    {
        reconstructed.resize(cfg.samples.size());
        channels = cfg_.samples.size();
        sample_counter = 0;
    }

    bool feed_writer(size_t n)
    {
        size_t rem = cfg.samples[0].size() - sample_counter;
        if (n > rem)
            n = rem;
        for (int i = 0; i < n; i++)
        {
            int16_t temp[MaxChannels];
            for (int j = 0; j < channels; j++)
            {
                temp[j] = cfg.samples.at(j).at(sample_counter);
            }
            writer.pushSample(temp);
            sample_counter++;
        }
        return n;
    }

    CodecTestConfig &cfg;
    uint8_t channels;
    std::vector<std::vector<int16_t>> reconstructed;
    uint32_t sample_counter = 0;
    std::vector<uint8_t> stream_buf;
    std::vector<ControlMark> mark_buf;
    std::vector<ChannelState> w_ccs_buf;
    std::vector<ChannelState> r_ccs_buf;
    std::vector<int16_t> dod_buf;
    std::vector<int16_t> block_buf;

    Reader reader;
    Writer writer;
    SampleBlock block;
    unsigned int feed_index = 0;
    bool test_fullfeed()
    {
        uint8_t read_buf[255];
        StreamType type;
        uint8_t len = 0;
        uint8_t feed = cfg.samples.at(0).size();
        feed_writer(feed);
        writer.flush();
        while (len = writer.readNext(etl::span(read_buf), type))
        {
            reader.decode(etl::span(read_buf, len), type);
            if (block.isReady())
            {
                auto samples = block.getSamples();
                auto channels = block.getChannels();
                for (int i = 0; i < samples.size()/channels; i++)
                    for (int j = 0; j < channels; j++)
                    {
                        reconstructed.at(j).push_back(samples.at(j + i * channels));
                    }
            }
        }
        return UnitTest::Compare(reconstructed, cfg.samples);
    }

    bool test_streamfeed()
    {
        uint8_t read_buf[255];
        StreamType type;
        uint8_t len = 0;
        uint8_t feed = cfg.feed_vector.at(feed_index);
        while(feed_writer(feed))
        {
            while (len = writer.readNext(etl::span(read_buf), type))
            {
                reader.decode(etl::span(read_buf, len), type);
                if (block.isReady())
                {
                    auto samples = block.getSamples();
                    auto channels = block.getChannels();
                    for (int i = 0; i < samples.size()/channels; i++)
                        for (int j = 0; j < channels; j++)
                        {
                            reconstructed.at(j).push_back(samples.at(j + i * channels));
                        }
                }
            }
            feed = cfg.feed_vector.at(feed_index);
            feed_index = (feed_index + 1)%cfg.feed_vector.size();
        }
        writer.flush();
        while (len = writer.readNext(etl::span(read_buf), type))
        {
            reader.decode(etl::span(read_buf, len), type);
            if (block.isReady())
            {
                auto samples = block.getSamples();
                auto channels = block.getChannels();
                for (int i = 0; i < samples.size()/channels; i++)
                    for (int j = 0; j < channels; j++)
                    {
                        reconstructed.at(j).push_back(samples.at(j + i * channels));
                    }
            }
        }
        return UnitTest::Compare(reconstructed, cfg.samples);
    }
};

TEST_CASE("SINGLE_SIGNAL_FULLFEED")
{
    constexpr unsigned int N_START = 1;
    constexpr unsigned int N_END = 256;
    constexpr unsigned int R_START = 1;
    constexpr unsigned int R_END = N_END;
    for (int N = N_START; N < N_END; N++)
    {
        for (int R = R_START; R < R_END; R++)
        {
            CodecTestConfig cfg;
            cfg.samples =
                {
                    SignalGenerator::Sine(N, 20, 1, 512, 2048, 0),
                };
            cfg.resync_period = R;
            size_t x = (cfg.samples.at(0).size() / cfg.resync_period) + 64;
            cfg.mark_buffer_size = 2048;
            INFO(cfg.mark_buffer_size);
            cfg.sample_buffer_size = 1024;
            cfg.stream_buffer_size = 2048;
            CodecTest test(cfg);
            INFO("T : " << N << " R : " << R);
            bool suc = test.test_fullfeed();
            INFO("ORIGINAL : " << UnitTest::toString(cfg.samples) << "\n");
            INFO("RECONSTRUCTED : " << UnitTest::toString(test.reconstructed) << "\n");
            REQUIRE(suc);
        }
    }
}

TEST_CASE("MULTI_SIGNAL_FULLFEED")
{
    constexpr unsigned int N_START = 1;
    constexpr unsigned int N_END = 256;
    constexpr unsigned int R_START = 1;
    constexpr unsigned int R_END = N_END;
    for (int N = N_START; N < N_END; N++)
    {
        for (int R = R_START; R < R_END; R++)
        {
            CodecTestConfig cfg;
            cfg.samples =
                {
                    SignalGenerator::Sine(N, 20, 1, 512, 2048, 0),
                    SignalGenerator::Sine(N, 28, 1, 120, 2048, 0),
                    SignalGenerator::Sine(N, 16, 1, 1200, 2048, 0),
                    SignalGenerator::Sine(N, 32, 1, 1700, 2048, 0)
                };
            size_t x = (cfg.samples.at(0).size() / cfg.resync_period) + 64;
            cfg.mark_buffer_size = 2048;
            cfg.sample_buffer_size = 2048;
            cfg.stream_buffer_size = 8192;
            cfg.resync_period = R;
            CodecTest test(cfg);
            INFO("T : " << N << " R : " << R);
            bool suc = test.test_fullfeed();
            INFO("ORIGINAL : " << UnitTest::toString(cfg.samples) << "\n");
            INFO("RECONSTRUCTED : " << UnitTest::toString(test.reconstructed) << "\n");
            REQUIRE(suc);
        }
    }
}

TEST_CASE("SINGLE_SIGNAL_STREAMFEED")
{
    constexpr unsigned int N_START = 2;
    constexpr unsigned int N_END = 256;
    constexpr unsigned int R_START = 1;
    constexpr unsigned int R_END = N_END;
    for (int N = N_START; N < N_END; N++)
    {
        for (int R = R_START; R < R_END; R++)
        {
            CodecTestConfig cfg;
            cfg.samples =
                {
                    SignalGenerator::Sine(N, 20, 1, 512, 2048, 0)
                };
            size_t x = (cfg.samples.at(0).size() / cfg.resync_period) + 64;
            cfg.mark_buffer_size = 2048;
            cfg.sample_buffer_size = 2048;
            cfg.stream_buffer_size = 8192;
            cfg.resync_period = R;
            cfg.feed_vector = {1, 2, 3, 4, 5, 6, 7, 8 , 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
            CodecTest test(cfg);
            INFO("T : " << N << " R : " << R);
            bool suc = test.test_streamfeed();
            INFO("ORIGINAL : " << UnitTest::toString(cfg.samples) << "\n");
            INFO("RECONSTRUCTED : " << UnitTest::toString(test.reconstructed) << "\n");
            REQUIRE(suc);
        }
    }
}
    
TEST_CASE("MULTI_SIGNAL_STREAMFEED")
{
    constexpr unsigned int N_START = 2;
    constexpr unsigned int N_END = 256;
    constexpr unsigned int R_START = 1;
    constexpr unsigned int R_END = N_END;
    for (int N = N_START; N < N_END; N++)
    {
        for (int R = R_START; R < R_END; R++)
        {
            CodecTestConfig cfg;
            cfg.samples =
                {
                    SignalGenerator::Sine(N, 20, 1, 512, 2048, 0),
                    SignalGenerator::Sine(N, 28, 1, 120, 2048, 0),
                    SignalGenerator::Sine(N, 16, 1, 1200, 2048, 0),
                    SignalGenerator::Sine(N, 32, 1, 1700, 2048, 0)
                };
            size_t x = (cfg.samples.at(0).size() / cfg.resync_period) + 64;
            cfg.mark_buffer_size = 2048;
            cfg.sample_buffer_size = 2048;
            cfg.stream_buffer_size = 8192;
            cfg.resync_period = R;
            cfg.feed_vector = {1, 2, 3, 4, 5, 6, 7, 8 , 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
            CodecTest test(cfg);
            INFO("T : " << N << " R : " << R);
            bool suc = test.test_streamfeed();
            INFO("ORIGINAL : " << UnitTest::toString(cfg.samples) << "\n");
            INFO("RECONSTRUCTED : " << UnitTest::toString(test.reconstructed) << "\n");
            REQUIRE(suc);
        }
    }
}
    