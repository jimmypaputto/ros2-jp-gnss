/*
 * Jimmy Paputto 2026
 */

#include <memory>
#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>

#include "Config.hpp"
#include "GnssNode.hpp"


static std::string resolveConfigPath(int argc, char** argv)
{
    // Scan argv for ROS parameter override: -p config_file:=<path>
    for (int i = 1; i < argc; ++i)
    {
        std::string_view arg(argv[i]);
        constexpr std::string_view prefix = "config_file:=";
        if (auto pos = arg.find(prefix); pos != std::string_view::npos)
            return std::string(arg.substr(pos + prefix.size()));
    }
    return ament_index_cpp::get_package_share_directory("jp_gnss_hat")
           + "/config/gnss_config.yaml";
}


auto main(int argc, char** argv) -> int
{
    rclcpp::init(argc, argv);

    const auto configPath = resolveConfigPath(argc, argv);
    const auto nodeId = JimmyPaputto::Config::readNodeId(configPath);

    auto node = std::make_shared<JimmyPaputto::GnssNode>(nodeId);

    using CallbackReturn = JimmyPaputto::GnssNode::CallbackReturn;
    // trigger_transition() leaves ret untouched if the transition cannot start.
    auto ret = CallbackReturn::ERROR;

    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(),
            "Failed to configure JP GNSS HAT node");
        rclcpp::shutdown();
        return 1;
    }

    ret = CallbackReturn::ERROR;
    node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(node->get_logger(),
            "Failed to activate JP GNSS HAT node");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    // spin() returns once SIGINT/SIGTERM has shut the context down. Lifecycle
    // transitions cannot run from then on, so the node is not deactivated
    // here: its destructor stops the navigation thread and releases the HAT.
    rclcpp::shutdown();
    return 0;
}
