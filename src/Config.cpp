/*
 * Jimmy Paputto 2026
 */

#include "Config.hpp"
#include "Rtk.hpp"
#include "YamlUtils.hpp"

#include <fstream>
#include <optional>
#include <stdexcept>

#include <yaml-cpp/yaml.h>


namespace JimmyPaputto
{

namespace
{

constexpr size_t kMaxGeofences = 4;

TimepulsePinConfig::Pulse loadPulse(
    const YAML::Node& parent, const std::string& key, const std::string& path)
{
    TimepulsePinConfig::Pulse pulse{.frequency = 1, .pulseWidth = 0.1f};
    const auto node = yaml::optionalMap(parent, key, path);
    if (!node)
        return pulse;

    pulse.frequency = yaml::value<uint32_t>(
        *node, "frequency", path + ".frequency", pulse.frequency);
    pulse.pulseWidth = yaml::value<float>(
        *node, "pulse_width", path + ".pulse_width", pulse.pulseWidth);
    return pulse;
}

// A missing section disables the timepulse; a present one is active unless
// it says otherwise.
TimepulsePinConfig loadTimepulse(const YAML::Node& root)
{
    TimepulsePinConfig tp{};
    tp.fixedPulse = {.frequency = 1, .pulseWidth = 0.1f};
    tp.polarity = ETimepulsePinPolarity::RisingEdgeAtTopOfSecond;

    const auto node = yaml::optionalMap(root, "timepulse", "timepulse");
    if (!node)
    {
        tp.active = false;
        return tp;
    }

    tp.active = yaml::value<bool>(*node, "active", "timepulse.active", true);
    tp.fixedPulse =
        loadPulse(*node, "fixed_pulse", "timepulse.fixed_pulse");
    if (yaml::optionalMap(*node, "pulse_when_no_fix",
            "timepulse.pulse_when_no_fix"))
    {
        tp.pulseWhenNoFix = loadPulse(
            *node, "pulse_when_no_fix", "timepulse.pulse_when_no_fix");
    }

    const auto polarity = yaml::value<std::string>(
        *node, "polarity", "timepulse.polarity", "rising");
    if (polarity == "rising")
        tp.polarity = ETimepulsePinPolarity::RisingEdgeAtTopOfSecond;
    else if (polarity == "falling")
        tp.polarity = ETimepulsePinPolarity::FallingEdgeAtTopOfSecond;
    else
        yaml::fail("timepulse.polarity",
            "expected \"rising\" or \"falling\", got \"" + polarity + "\"");

    return tp;
}

std::optional<GnssConfig::Geofencing> loadGeofencing(const YAML::Node& root)
{
    const auto node = yaml::optionalMap(root, "geofencing", "geofencing");
    if (!node)
        return std::nullopt;

    GnssConfig::Geofencing geo{};
    geo.confidenceLevel = yaml::value<uint8_t>(
        *node, "confidence_level", "geofencing.confidence_level", 0);

    const auto polarity = yaml::optionalValue<std::string>(
        *node, "pio_pin_polarity", "geofencing.pio_pin_polarity");
    if (polarity)
    {
        if (*polarity == "low_means_inside")
            geo.pioPinPolarity = EPioPinPolarity::LowMeansInside;
        else if (*polarity == "low_means_outside")
            geo.pioPinPolarity = EPioPinPolarity::LowMeansOutside;
        else
            yaml::fail("geofencing.pio_pin_polarity",
                "expected \"low_means_inside\" or \"low_means_outside\", "
                "got \"" + *polarity + "\"");
    }

    const YAML::Node fences = (*node)["geofences"];
    if (!fences.IsDefined())
        return geo;
    if (!fences.IsSequence())
        yaml::fail("geofencing.geofences", "expected a list");
    if (fences.size() > kMaxGeofences)
    {
        yaml::fail("geofencing.geofences",
            "at most " + std::to_string(kMaxGeofences) + " geofences are "
            "supported, got " + std::to_string(fences.size()));
    }

    for (size_t i = 0; i < fences.size(); ++i)
    {
        const std::string path =
            "geofencing.geofences[" + std::to_string(i) + "]";
        if (!fences[i].IsMap())
            yaml::fail(path, "expected a mapping with lat, lon and radius");

        Geofence gf;
        gf.lat    = yaml::requiredValue<float>(fences[i], "lat", path + ".lat");
        gf.lon    = yaml::requiredValue<float>(fences[i], "lon", path + ".lon");
        gf.radius = yaml::requiredValue<float>(
            fences[i], "radius", path + ".radius");
        geo.geofences.push_back(gf);
    }

    return geo;
}

}  // namespace

void Config::loadFromYaml(const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap())
    {
        throw std::runtime_error(
            "Invalid config file " + path + ": expected a YAML mapping");
    }

    const auto nodeId =
        yaml::value<std::string>(root, "node_id", "node_id", "");
    const auto publishStandardTopics = yaml::value<bool>(
        root, "publish_standard_topics", "publish_standard_topics", true);
    const auto useNtripRtcm =
        yaml::value<bool>(root, "use_ntrip_rtcm", "use_ntrip_rtcm", false);

    GnssConfig config{};

    config.measurementRate_Hz = yaml::value<uint16_t>(
        root, "measurement_rate_hz", "measurement_rate_hz", 1);
    config.saveToFlash =
        yaml::value<bool>(root, "save_to_flash", "save_to_flash", false);

    config.dynamicModel = static_cast<EDynamicModel>(
        yaml::value<uint8_t>(root, "dynamic_model", "dynamic_model", 0));
    if (getDynamicModelName(config.dynamicModel) == "Unknown")
    {
        throw std::runtime_error(
            "Invalid dynamic_model: " +
            std::to_string(static_cast<int>(config.dynamicModel)));
    }

    config.timepulsePinConfig = loadTimepulse(root);
    config.geofencing = loadGeofencing(root);

    if (const auto rtkNode = yaml::optionalMap(root, "rtk", "rtk"))
        config.rtk = RtkBridge::fromYaml(*rtkNode);

    nodeId_ = nodeId;
    publishStandardTopics_ = publishStandardTopics;
    useNtripRtcm_ = useNtripRtcm;
    gnssConfig_ = config;
}

void Config::saveToYaml(const std::string& path) const
{
    YAML::Emitter out;
    out << YAML::BeginMap;

    if (!nodeId_.empty())
        out << YAML::Key << "node_id" << YAML::Value << nodeId_;

    out << YAML::Key << "measurement_rate_hz"
        << YAML::Value << gnssConfig_.measurementRate_Hz;
    out << YAML::Key << "publish_standard_topics"
        << YAML::Value << publishStandardTopics_;
    out << YAML::Key << "use_ntrip_rtcm"
        << YAML::Value << useNtripRtcm_;
    out << YAML::Key << "save_to_flash"
        << YAML::Value << gnssConfig_.saveToFlash;
    out << YAML::Key << "dynamic_model"
        << YAML::Value << static_cast<int>(gnssConfig_.dynamicModel);

    out << YAML::Key << "timepulse" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "active"
        << YAML::Value << gnssConfig_.timepulsePinConfig.active;
    out << YAML::Key << "fixed_pulse" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "frequency"
        << YAML::Value << gnssConfig_.timepulsePinConfig.fixedPulse.frequency;
    out << YAML::Key << "pulse_width"
        << YAML::Value << gnssConfig_.timepulsePinConfig.fixedPulse.pulseWidth;
    out << YAML::EndMap;
    if (gnssConfig_.timepulsePinConfig.pulseWhenNoFix)
    {
        out << YAML::Key << "pulse_when_no_fix" << YAML::Value
            << YAML::BeginMap;
        out << YAML::Key << "frequency"
            << YAML::Value
            << gnssConfig_.timepulsePinConfig.pulseWhenNoFix->frequency;
        out << YAML::Key << "pulse_width"
            << YAML::Value
            << gnssConfig_.timepulsePinConfig.pulseWhenNoFix->pulseWidth;
        out << YAML::EndMap;
    }
    out << YAML::Key << "polarity" << YAML::Value
        << (gnssConfig_.timepulsePinConfig.polarity ==
                ETimepulsePinPolarity::FallingEdgeAtTopOfSecond
            ? "falling" : "rising");
    out << YAML::EndMap;

    if (gnssConfig_.geofencing)
    {
        out << YAML::Key << "geofencing" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "confidence_level"
            << YAML::Value
            << static_cast<int>(gnssConfig_.geofencing->confidenceLevel);
        if (gnssConfig_.geofencing->pioPinPolarity)
        {
            out << YAML::Key << "pio_pin_polarity" << YAML::Value
                << (*gnssConfig_.geofencing->pioPinPolarity ==
                        EPioPinPolarity::LowMeansOutside
                    ? "low_means_outside" : "low_means_inside");
        }
        out << YAML::Key << "geofences" << YAML::Value << YAML::BeginSeq;
        for (const auto& gf : gnssConfig_.geofencing->geofences)
        {
            out << YAML::BeginMap;
            out << YAML::Key << "lat" << YAML::Value << gf.lat;
            out << YAML::Key << "lon" << YAML::Value << gf.lon;
            out << YAML::Key << "radius" << YAML::Value << gf.radius;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
    }

    if (gnssConfig_.rtk)
        RtkBridge::toYaml(out, *gnssConfig_.rtk);

    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open())
    {
        throw std::runtime_error(
            "Cannot open config file for writing: " + path);
    }
    fout << out.c_str();
}

jp_gnss_hat::msg::GnssConfig Config::toMsg() const
{
    jp_gnss_hat::msg::GnssConfig msg;
    msg.measurement_rate_hz = gnssConfig_.measurementRate_Hz;
    msg.dynamic_model = static_cast<uint8_t>(gnssConfig_.dynamicModel);
    msg.publish_standard_topics = publishStandardTopics_;
    msg.use_ntrip_rtcm = useNtripRtcm_;
    msg.save_to_flash = gnssConfig_.saveToFlash;

    const auto& tp = gnssConfig_.timepulsePinConfig;
    msg.timepulse_active = tp.active;
    msg.timepulse_frequency = tp.fixedPulse.frequency;
    msg.timepulse_pulse_width = tp.fixedPulse.pulseWidth;
    msg.timepulse_has_no_fix_pulse = tp.pulseWhenNoFix.has_value();
    if (tp.pulseWhenNoFix)
    {
        msg.timepulse_no_fix_frequency = tp.pulseWhenNoFix->frequency;
        msg.timepulse_no_fix_pulse_width = tp.pulseWhenNoFix->pulseWidth;
    }
    msg.timepulse_polarity = static_cast<uint8_t>(tp.polarity);

    msg.geofencing_enabled = gnssConfig_.geofencing.has_value();
    if (gnssConfig_.geofencing)
    {
        msg.geofencing_confidence_level =
            gnssConfig_.geofencing->confidenceLevel;
        msg.geofencing_has_pio_pin_polarity =
            gnssConfig_.geofencing->pioPinPolarity.has_value();
        if (gnssConfig_.geofencing->pioPinPolarity)
            msg.geofencing_pio_pin_polarity =
                static_cast<uint8_t>(*gnssConfig_.geofencing->pioPinPolarity);
        for (const auto& gf : gnssConfig_.geofencing->geofences)
        {
            jp_gnss_hat::msg::Geofence fence;
            fence.lat = gf.lat;
            fence.lon = gf.lon;
            fence.radius = gf.radius;
            msg.geofences.push_back(fence);
        }
    }

    msg.rtk_enabled = gnssConfig_.rtk.has_value();
    if (gnssConfig_.rtk)
        RtkBridge::toMsg(*gnssConfig_.rtk, msg);

    return msg;
}

void Config::fromMsg(const jp_gnss_hat::msg::GnssConfig& msg)
{
    GnssConfig config{};
    config.measurementRate_Hz = msg.measurement_rate_hz;
    config.dynamicModel = static_cast<EDynamicModel>(msg.dynamic_model);
    config.saveToFlash = msg.save_to_flash;

    config.timepulsePinConfig.active = msg.timepulse_active;
    config.timepulsePinConfig.fixedPulse.frequency = msg.timepulse_frequency;
    config.timepulsePinConfig.fixedPulse.pulseWidth = msg.timepulse_pulse_width;
    if (msg.timepulse_has_no_fix_pulse)
    {
        TimepulsePinConfig::Pulse noFix;
        noFix.frequency = msg.timepulse_no_fix_frequency;
        noFix.pulseWidth = msg.timepulse_no_fix_pulse_width;
        config.timepulsePinConfig.pulseWhenNoFix = noFix;
    }
    if (msg.timepulse_polarity > static_cast<uint8_t>(
            ETimepulsePinPolarity::RisingEdgeAtTopOfSecond))
    {
        throw std::runtime_error("Invalid timepulse_polarity: " +
            std::to_string(msg.timepulse_polarity));
    }
    config.timepulsePinConfig.polarity =
        static_cast<ETimepulsePinPolarity>(msg.timepulse_polarity);

    if (msg.geofencing_enabled)
    {
        if (msg.geofences.size() > kMaxGeofences)
        {
            throw std::runtime_error(
                "At most " + std::to_string(kMaxGeofences) +
                " geofences are supported, got " +
                std::to_string(msg.geofences.size()));
        }

        GnssConfig::Geofencing geo{};
        geo.confidenceLevel = msg.geofencing_confidence_level;
        if (msg.geofencing_has_pio_pin_polarity)
        {
            if (msg.geofencing_pio_pin_polarity > static_cast<uint8_t>(
                    EPioPinPolarity::LowMeansOutside))
            {
                throw std::runtime_error(
                    "Invalid geofencing_pio_pin_polarity: " +
                    std::to_string(msg.geofencing_pio_pin_polarity));
            }
            geo.pioPinPolarity =
                static_cast<EPioPinPolarity>(msg.geofencing_pio_pin_polarity);
        }
        for (const auto& f : msg.geofences)
        {
            Geofence gf;
            gf.lat = f.lat;
            gf.lon = f.lon;
            gf.radius = f.radius;
            geo.geofences.push_back(gf);
        }
        config.geofencing = geo;
    }

    if (msg.rtk_enabled)
        config.rtk = RtkBridge::fromMsg(msg);

    publishStandardTopics_ = msg.publish_standard_topics;
    useNtripRtcm_ = msg.use_ntrip_rtcm;
    gnssConfig_ = config;
}

jp_gnss_hat::srv::GetGeofencingCfg::Response Config::toGeofencingResponse() const
{
    jp_gnss_hat::srv::GetGeofencingCfg::Response response{};

    const auto& geo = gnssConfig_.geofencing;
    if (!geo)
    {
        response.enabled = false;
        return response;
    }

    response.enabled = true;
    response.pio_enabled = geo->pioPinPolarity.has_value();
    if (geo->pioPinPolarity)
        response.pin_polarity = static_cast<uint8_t>(*geo->pioPinPolarity);
    response.confidence_level = geo->confidenceLevel;
    for (const auto& gf : geo->geofences)
    {
        jp_gnss_hat::msg::Geofence fence;
        fence.lat = gf.lat;
        fence.lon = gf.lon;
        fence.radius = gf.radius;
        response.geofences.push_back(fence);
    }

    return response;
}

std::string Config::getDynamicModelName(const EDynamicModel model)
{
    switch (model)
    {
        case EDynamicModel::Portable:    return "Portable";
        case EDynamicModel::Stationary:  return "Stationary";
        case EDynamicModel::Pedestrian:  return "Pedestrian";
        case EDynamicModel::Automotive:  return "Automotive";
        case EDynamicModel::Sea:         return "Sea";
        case EDynamicModel::Airborne1G:  return "Airborne1G";
        case EDynamicModel::Airborne2G:  return "Airborne2G";
        case EDynamicModel::Airborne4G:  return "Airborne4G";
        case EDynamicModel::Wrist:       return "Wrist";
        case EDynamicModel::Bike:        return "Bike";
        case EDynamicModel::Mower:       return "Mower";
        case EDynamicModel::Escooter:    return "Escooter";
        default:                         return "Unknown";
    }
}

std::string Config::readNodeId(const std::string& path)
{
    try
    {
        auto yaml = YAML::LoadFile(path);
        return yaml["node_id"].as<std::string>("");
    }
    catch (...)
    {
        return "";
    }
}

}  // JimmyPaputto
