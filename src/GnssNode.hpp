/*
 * Jimmy Paputto 2026
 */

#ifndef JP_GNSS_HAT_NODE_HPP_
#define JP_GNSS_HAT_NODE_HPP_

#include <optional>

#include <jimmypaputto/GnssHat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <jp_gnss_hat/msg/navigation.hpp>
#include <jp_gnss_hat/msg/rtk_corrections.hpp>
#include <jp_gnss_hat/srv/get_geofencing_cfg.hpp>
#include <jp_gnss_hat/srv/get_gnss_config.hpp>
#include <jp_gnss_hat/srv/set_gnss_config.hpp>

#include "Config.hpp"
#include "Converter.hpp"
#include "Rtk.hpp"
#include "Topics.hpp"


namespace JimmyPaputto
{

class GnssNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit GnssNode(const std::string& nodeId = "");
    ~GnssNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void sendNavigation(const Navigation& navigation);
    void navigationLoop(std::stop_token stopToken);
    void stopNavigationThread();
    void releaseResources();
    void onGetGeofencingCfg(
        const jp_gnss_hat::srv::GetGeofencingCfg::Request::SharedPtr request,
        jp_gnss_hat::srv::GetGeofencingCfg::Response::SharedPtr response);
    void onGetGnssConfig(
        const jp_gnss_hat::srv::GetGnssConfig::Request::SharedPtr request,
        jp_gnss_hat::srv::GetGnssConfig::Response::SharedPtr response);
    void onSetGnssConfig(
        const jp_gnss_hat::srv::SetGnssConfig::Request::SharedPtr request,
        jp_gnss_hat::srv::SetGnssConfig::Response::SharedPtr response);
    std::string rollbackToActive();

    std::unique_ptr<IGnssHat, void(*)(IGnssHat*)> hat_;
    Topics topics_;
    std::string frameId_;
    std::unique_ptr<Converter> converter_;

    rclcpp_lifecycle::LifecyclePublisher<jp_gnss_hat::msg::Navigation>::SharedPtr
        navPublisher_;
    std::jthread navigationThread_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
        navSatFixPub_;
    rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velPub_;
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::TimeReference>::SharedPtr timeRefPub_;

    std::unique_ptr<RtkBridge> rtk_;

    rclcpp::Service<jp_gnss_hat::srv::GetGeofencingCfg>::SharedPtr
        geofencingCfgService_;

    rclcpp::Service<jp_gnss_hat::srv::GetGnssConfig>::SharedPtr
        gnssConfigGetService_;
    rclcpp::Service<jp_gnss_hat::srv::SetGnssConfig>::SharedPtr
        gnssConfigSetService_;
    Config config_;
    std::string configPath_;
    std::optional<Config> pendingConfig_;
};

}  // JimmyPaputto

#endif  // JP_GNSS_HAT_NODE_HPP_
