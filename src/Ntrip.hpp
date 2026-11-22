/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_NTRIP_HPP_
#define JP_GNSS_HAT_NTRIP_HPP_

#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

#include <jimmypaputto/GnssHat.hpp>
#include <jimmypaputto/ntrip/NtripClient.hpp>
#include <rclcpp/logger.hpp>
#include <yaml-cpp/yaml.h>


namespace JimmyPaputto
{

// NTRIP caster from the `rtk.ntrip` section. The RTK mode picks the role:
// a rover receives corrections from the caster.
struct NtripConfig
{
    std::string host;
    uint16_t port{2101};
    std::string mountpoint;
    std::string username;
    std::string password;
    uint32_t ggaInterval_s{10};  // GGA sent by a rover, 0 = none

    static std::optional<NtripConfig> fromYaml(const YAML::Node& rtkNode);
    void toYaml(YAML::Emitter& out) const;

    bool operator==(const NtripConfig&) const = default;
};

// Applies the corrections of an NTRIP caster to the rover.
class NtripRoverLink
{
public:
    NtripRoverLink(const NtripConfig& config, IRover& rover,
        const rclcpp::Logger& logger);

    // Position reported to the caster in GGA (VRS, nearest base).
    void updatePosition(double latitude, double longitude, double altitude);

private:
    void run(std::stop_token stopToken);

    IRover& rover_;
    rclcpp::Logger logger_;
    NtripClient client_;
    std::jthread worker_;  // last, so it stops before the client goes
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_NTRIP_HPP_
