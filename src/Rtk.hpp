/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_RTK_HPP_
#define JP_GNSS_HAT_RTK_HPP_

#include <string>
#include <vector>

#include <jimmypaputto/GnssHat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <yaml-cpp/yaml.h>

#include <jp_gnss_hat/msg/gnss_config.hpp>
#include <jp_gnss_hat/msg/rtk_corrections.hpp>


namespace JimmyPaputto
{

// RTCM3 set published by a base: Full = MSM7 (1005, 1077, 1087, 1097, 1127,
// 1230), Tiny = MSM4 (1005, 1074, 1084, 1094, 1124, 1230).
enum class ERtkCorrections : uint8_t
{
    Full = 0,
    Tiny = 1
};

class RtkBridge
{
public:
    RtkBridge(rclcpp_lifecycle::LifecycleNode& node, const std::string& frameId);
    ~RtkBridge();

    bool initialize(const GnssConfig& config, IRtk* rtk,
        const std::string& correctionsTopic,
        ERtkCorrections corrections);
    void pollAndPublish();
    void onCorrections(const jp_gnss_hat::msg::RtkCorrections::SharedPtr msg);

    static RtkConfig fromYaml(const YAML::Node& rtkNode);
    static ERtkCorrections correctionsFromYaml(const YAML::Node& rtkNode);
    static void toYaml(YAML::Emitter& out, const RtkConfig& cfg,
        ERtkCorrections corrections = ERtkCorrections::Full);

    static void toMsg(const RtkConfig& cfg, jp_gnss_hat::msg::GnssConfig& msg);
    static RtkConfig fromMsg(const jp_gnss_hat::msg::GnssConfig& msg);
    static ERtkCorrections correctionsFromMsg(
        const jp_gnss_hat::msg::GnssConfig& msg);
    static const char* correctionsName(ERtkCorrections corrections);

private:
    rclcpp_lifecycle::LifecycleNode& node_;
    std::string frameId_;
    IBase* base_{nullptr};
    IRover* rover_{nullptr};
    ERtkCorrections corrections_{ERtkCorrections::Full};
    rclcpp_lifecycle::LifecyclePublisher<
        jp_gnss_hat::msg::RtkCorrections>::SharedPtr pub_;
    rclcpp::Subscription<jp_gnss_hat::msg::RtkCorrections>::SharedPtr sub_;
    std::vector<std::vector<uint8_t>> bufor_;
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_RTK_HPP_
