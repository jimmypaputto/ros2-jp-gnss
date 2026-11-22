/*
 * Jimmy Paputto 2026
 */

// NtripRoverLink against a gnsshat NtripCaster on localhost: real sockets,
// no receiver needed.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>
#include <jimmypaputto/ntrip/NtripCaster.hpp>
#include <rclcpp/logger.hpp>

#include "../src/Ntrip.hpp"


namespace JimmyPaputto::test
{

using namespace std::chrono_literals;
using Frames = std::vector<std::vector<uint8_t>>;

uint32_t crc24q(const std::vector<uint8_t>& data)
{
    uint32_t crc = 0;
    for (const uint8_t byte : data)
    {
        crc ^= static_cast<uint32_t>(byte) << 16;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc <<= 1;
            if (crc & 0x1000000)
                crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

std::vector<uint8_t> rtcmFrame(const uint16_t messageType,
    const uint16_t payloadSize)
{
    std::vector<uint8_t> frame{0xD3,
        static_cast<uint8_t>((payloadSize >> 8) & 0x03),
        static_cast<uint8_t>(payloadSize & 0xFF),
        static_cast<uint8_t>(messageType >> 4),
        static_cast<uint8_t>((messageType & 0x0F) << 4)};
    for (uint16_t i = 2; i < payloadSize; ++i)
        frame.push_back(static_cast<uint8_t>(i));

    const uint32_t crc = crc24q(frame);
    frame.push_back(static_cast<uint8_t>(crc >> 16));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    frame.push_back(static_cast<uint8_t>(crc));
    return frame;
}

const Frames kEpoch{rtcmFrame(1005, 19), rtcmFrame(1077, 400),
    rtcmFrame(1087, 300), rtcmFrame(1230, 8)};

// Distinct per test process, so parallel ctest runs do not collide.
uint16_t testPort(const uint16_t offset)
{
    return static_cast<uint16_t>(20000 + (::getpid() % 10000) * 4 + offset);
}

bool waitUntil(const std::function<bool()>& condition,
    const std::chrono::milliseconds timeout = 5s)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition())
    {
        if (std::chrono::steady_clock::now() > deadline)
            return false;
        std::this_thread::sleep_for(10ms);
    }
    return true;
}

class FakeRover : public IRover
{
public:
    void applyCorrections(const Frames& corrections) override
    {
        std::lock_guard lock(mutex_);
        batches_.push_back(corrections);
    }

    std::vector<Frames> batches() const
    {
        std::lock_guard lock(mutex_);
        return batches_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<Frames> batches_;
};

NtripConfig localCaster(const uint16_t port)
{
    NtripConfig cfg;
    cfg.host = "127.0.0.1";
    cfg.port = port;
    cfg.mountpoint = "TEST";
    cfg.ggaInterval_s = 0;
    return cfg;
}

TEST(NtripRoverLink, AppliesOneBatchPerBurst)
{
    const uint16_t port = testPort(0);
    NtripCaster caster("127.0.0.1", port, "TEST");
    ASSERT_TRUE(caster.start());

    FakeRover rover;
    NtripRoverLink link(localCaster(port), rover,
        rclcpp::get_logger("test_ntrip"));
    ASSERT_TRUE(waitUntil([&] { return caster.clientCount() == 1; }));

    caster.feed(kEpoch);
    ASSERT_TRUE(waitUntil([&] { return rover.batches().size() == 1; }));

    // Next epoch after the stream went quiet: a batch of its own.
    std::this_thread::sleep_for(500ms);
    caster.feed(kEpoch);
    ASSERT_TRUE(waitUntil([&] { return rover.batches().size() == 2; }));

    for (const auto& batch : rover.batches())
        EXPECT_EQ(batch, kEpoch);
}

TEST(NtripRoverLink, StopsWhileCasterIsUnreachable)
{
    FakeRover rover;
    auto link = std::make_unique<NtripRoverLink>(localCaster(testPort(1)),
        rover, rclcpp::get_logger("test_ntrip"));
    std::this_thread::sleep_for(300ms);

    const auto start = std::chrono::steady_clock::now();
    link.reset();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
    EXPECT_TRUE(rover.batches().empty());
}

}  // JimmyPaputto::test
