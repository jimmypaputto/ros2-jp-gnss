/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_CONFIG_HPP_
#define JP_GNSS_HAT_CONFIG_HPP_

#include <optional>
#include <string>

#include <jimmypaputto/GnssHat.hpp>

#include "Ntrip.hpp"
#include "Rtk.hpp"

#include <jp_gnss_hat/msg/gnss_config.hpp>
#include <jp_gnss_hat/srv/get_geofencing_cfg.hpp>


namespace JimmyPaputto
{

class Config
{
public:
    Config() = default;

    void loadFromYaml(const std::string& path);
    void saveToYaml(const std::string& path) const;

    jp_gnss_hat::msg::GnssConfig toMsg() const;
    void fromMsg(const jp_gnss_hat::msg::GnssConfig& msg);

    jp_gnss_hat::srv::GetGeofencingCfg::Response toGeofencingResponse() const;

    static std::string getDynamicModelName(EDynamicModel model);
    static std::string readNodeId(const std::string& path);

    GnssConfig& gnssConfig() { return gnssConfig_; }
    void gnssConfig(const GnssConfig& config) { gnssConfig_ = config; }

    bool publishStandardTopics() const { return publishStandardTopics_; }
    void publishStandardTopics(const bool value) { publishStandardTopics_ = value; }

    ERtkCorrections rtkCorrections() const { return rtkCorrections_; }
    void rtkCorrections(const ERtkCorrections value) { rtkCorrections_ = value; }

    // From YAML only: GnssConfig has no NTRIP fields, so get_config does not
    // hand out the caster password.
    const std::optional<NtripConfig>& ntrip() const { return ntrip_; }
    void ntrip(const std::optional<NtripConfig>& value) { ntrip_ = value; }

    const std::string& nodeId() const { return nodeId_; }
    void nodeId(const std::string& value) { nodeId_ = value; }

private:
    GnssConfig gnssConfig_{};
    bool publishStandardTopics_{true};
    ERtkCorrections rtkCorrections_{ERtkCorrections::Full};
    std::optional<NtripConfig> ntrip_;
    std::string nodeId_;
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_CONFIG_HPP_
