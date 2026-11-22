/*
 * Jimmy Paputto 2026
 */

#include "Ntrip.hpp"
#include "YamlUtils.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iterator>
#include <limits>
#include <mutex>
#include <vector>

#include <rclcpp/logging.hpp>


namespace JimmyPaputto
{

namespace
{

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr auto kPollPeriod = 100ms;
constexpr auto kMaxBatchAge = 1s;
constexpr std::chrono::seconds kFirstRetryDelay = 1s;
constexpr std::chrono::seconds kMaxRetryDelay = 30s;

// Returns false when a stop was requested before the period ended.
bool sleepFor(std::stop_token stopToken, const Clock::duration period)
{
    std::mutex mutex;
    std::condition_variable_any cv;
    std::unique_lock lock(mutex);
    cv.wait_for(lock, stopToken, period, [] { return false; });
    return !stopToken.stop_requested();
}

NtripLogCallback toRosLog(const rclcpp::Logger& logger)
{
    return [logger](const ENtripLogLevel level, const std::string& message)
    {
        switch (level)
        {
            case ENtripLogLevel::Error:
                RCLCPP_ERROR(logger, "%s", message.c_str());
                break;
            case ENtripLogLevel::Warning:
                RCLCPP_WARN(logger, "%s", message.c_str());
                break;
            case ENtripLogLevel::Info:
                RCLCPP_INFO(logger, "%s", message.c_str());
                break;
            case ENtripLogLevel::Debug:
                RCLCPP_DEBUG(logger, "%s", message.c_str());
                break;
        }
    };
}

uint32_t ggaIntervalMs(const uint32_t interval_s)
{
    const uint64_t ms = uint64_t{interval_s} * 1000;
    return static_cast<uint32_t>(
        std::min<uint64_t>(ms, std::numeric_limits<uint32_t>::max()));
}

}  // namespace

std::optional<NtripConfig> NtripConfig::fromYaml(const YAML::Node& rtkNode)
{
    const auto node = yaml::optionalMap(rtkNode, "ntrip", "rtk.ntrip");
    if (!node)
        return std::nullopt;

    NtripConfig cfg;
    cfg.host = yaml::requiredValue<std::string>(
        *node, "host", "rtk.ntrip.host");
    cfg.port = yaml::value<uint16_t>(
        *node, "port", "rtk.ntrip.port", cfg.port);
    cfg.mountpoint = yaml::requiredValue<std::string>(
        *node, "mountpoint", "rtk.ntrip.mountpoint");
    cfg.username = yaml::value<std::string>(
        *node, "username", "rtk.ntrip.username", "");
    cfg.password = yaml::value<std::string>(
        *node, "password", "rtk.ntrip.password", "");
    cfg.ggaInterval_s = yaml::value<uint32_t>(
        *node, "gga_interval_s", "rtk.ntrip.gga_interval_s",
        cfg.ggaInterval_s);

    if (cfg.host.empty())
        yaml::fail("rtk.ntrip.host", "must not be empty");
    if (cfg.port == 0)
        yaml::fail("rtk.ntrip.port", "expected 1-65535, got 0");
    if (cfg.mountpoint.empty())
        yaml::fail("rtk.ntrip.mountpoint", "must not be empty");

    return cfg;
}

void NtripConfig::toYaml(YAML::Emitter& out) const
{
    out << YAML::Key << "ntrip" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "host" << YAML::Value << host;
    out << YAML::Key << "port" << YAML::Value << port;
    out << YAML::Key << "mountpoint" << YAML::Value << mountpoint;
    if (!username.empty())
        out << YAML::Key << "username" << YAML::Value << username;
    if (!password.empty())
        out << YAML::Key << "password" << YAML::Value << password;
    out << YAML::Key << "gga_interval_s" << YAML::Value << ggaInterval_s;
    out << YAML::EndMap;
}

NtripRoverLink::NtripRoverLink(const NtripConfig& config, IRover& rover,
    const rclcpp::Logger& logger)
:   rover_(rover),
    logger_(logger),
    client_(config.host, config.port, config.mountpoint,
        config.username, config.password)
{
    client_.setLogCallback(toRosLog(logger_));
    client_.setLogLevel(ENtripLogLevel::Debug);
    client_.setAutoReconnect(true);
    client_.setAutoGGA(ggaIntervalMs(config.ggaInterval_s));

    RCLCPP_INFO(logger_, "NTRIP client - corrections from %s:%u/%s",
        config.host.c_str(), static_cast<unsigned>(config.port),
        config.mountpoint.c_str());

    worker_ = std::jthread([this](std::stop_token stopToken) {
        run(stopToken);
    });
}

void NtripRoverLink::updatePosition(
    const double latitude, const double longitude, const double altitude)
{
    client_.updatePosition(latitude, longitude, altitude);
}

void NtripRoverLink::run(std::stop_token stopToken)
{
    // NtripClient reconnects by itself, but only after a first successful
    // connect(). One attempt cannot be interrupted and may block for ~15 s
    // (resolve, connect, response), so stopping the link waits for it.
    auto delay = kFirstRetryDelay;
    while (!client_.connect())
    {
        if (!sleepFor(stopToken, delay))
            return;
        delay = std::min(delay * 2, kMaxRetryDelay);
    }

    // gnsshat keeps only the newest batch until the UART takes it, so the
    // frames of one burst from the caster (one epoch) go to the rover
    // together, once the stream has paused for a poll period.
    std::vector<std::vector<uint8_t>> batch;
    Clock::time_point batchStart;
    while (sleepFor(stopToken, kPollPeriod))
    {
        auto frames = client_.receiveFrames();
        if (!frames.empty())
        {
            if (batch.empty())
                batchStart = Clock::now();
            batch.insert(batch.end(), std::make_move_iterator(frames.begin()),
                std::make_move_iterator(frames.end()));
            if (Clock::now() - batchStart < kMaxBatchAge)
                continue;
        }

        if (!batch.empty())
        {
            rover_.applyCorrections(batch);
            batch.clear();
        }
    }
}

}  // JimmyPaputto
