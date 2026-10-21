/*
 * Jimmy Paputto 2026
 */

#include "GnssNode.hpp"
#include "Topics.hpp"

#include <exception>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>


namespace JimmyPaputto
{

GnssNode::GnssNode(const std::string& nodeId)
:   LifecycleNode(Topics(nodeId).nodeName),
    hat_(nullptr, [](IGnssHat* p){ delete p; }),
    topics_(nodeId)
{
    const auto pkgShare =
        ament_index_cpp::get_package_share_directory("jp_gnss_hat");
    declare_parameter<std::string>(
        "config_file", pkgShare + "/config/gnss_config.yaml");

    geofencingCfgService_ = create_service<jp_gnss_hat::srv::GetGeofencingCfg>(
        topics_.geofencingCfg,
        std::bind(&GnssNode::onGetGeofencingCfg, this,
            std::placeholders::_1, std::placeholders::_2)
    );

    gnssConfigGetService_ = create_service<jp_gnss_hat::srv::GetGnssConfig>(
        topics_.getConfig,
        std::bind(&GnssNode::onGetGnssConfig, this,
            std::placeholders::_1, std::placeholders::_2)
    );

    gnssConfigSetService_ = create_service<jp_gnss_hat::srv::SetGnssConfig>(
        topics_.setConfig,
        std::bind(&GnssNode::onSetGnssConfig, this,
            std::placeholders::_1, std::placeholders::_2)
    );
}

GnssNode::~GnssNode() = default;

GnssNode::CallbackReturn GnssNode::on_configure(
    const rclcpp_lifecycle::State&)
{
    hat_.reset(IGnssHat::create());
    frameId_ = std::string(hat_->name());

    RCLCPP_INFO(get_logger(), "Detected HAT: %s", frameId_.c_str());

    hat_->softResetUbloxSom_HotStart();

    if (pendingConfig_)
    {
        config_ = std::move(*pendingConfig_);
        pendingConfig_.reset();
    }
    else
    {
        configPath_ = get_parameter("config_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading config from: %s",
            configPath_.c_str());
        try
        {
            config_.loadFromYaml(configPath_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config %s: %s",
                configPath_.c_str(), e.what());
            return CallbackReturn::FAILURE;
        }
    }

    const auto& cfg = config_.gnssConfig();
    RCLCPP_INFO(get_logger(),
        "Config loaded:\n"
        "  HAT:              %s\n"
        "  Measurement Rate: %d Hz\n"
        "  Dynamic Model:    %s (%d)\n"
        "  Timepulse:        %s\n"
        "  Geofencing:       %s\n"
        "  RTK:              %s\n"
        "  Save to Flash:    %s",
        std::string(hat_->name()).c_str(),
        cfg.measurementRate_Hz,
        Config::getDynamicModelName(cfg.dynamicModel).c_str(),
        static_cast<int>(cfg.dynamicModel),
        cfg.timepulsePinConfig.active ? "Enabled" : "Disabled",
        cfg.geofencing.has_value() ? "Enabled" : "Disabled",
        cfg.rtk.has_value()
            ? (cfg.rtk->mode == ERtkMode::Base ? "Base" : "Rover")
            : "Disabled",
        cfg.saveToFlash ? "Yes" : "No"
    );

    if (!hat_->start(config_.gnssConfig()))
    {
        RCLCPP_ERROR(get_logger(), "Failed to start GNSS HAT");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "GNSS HAT configured");

    converter_ = std::make_unique<Converter>(frameId_, get_clock());

    navPublisher_ = create_publisher<jp_gnss_hat::msg::Navigation>(
        topics_.navigation, 10);

    if (config_.publishStandardTopics())
    {
        navSatFixPub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
            topics_.navSatFix, 10);
        velPub_ = create_publisher<
            geometry_msgs::msg::TwistWithCovarianceStamped>(
            topics_.velocity, 10);
        timeRefPub_ = create_publisher<sensor_msgs::msg::TimeReference>(
            topics_.timeReference, 10);
    }

    if (config_.gnssConfig().rtk.has_value())
    {
        rtk_ = std::make_unique<RtkBridge>(*this, frameId_);
        const bool rtkInitialized = rtk_->initialize(
            config_.gnssConfig(),
            hat_->rtk(),
            config_.useNtripRtcm(),
            topics_.rtkCorrections,
            topics_.ntrip
        );
        if (!rtkInitialized)
            return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_activate(
    const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_activate(state);

    navigationThread_ = std::jthread([this](std::stop_token token) {
        navigationLoop(token);
    });

    RCLCPP_INFO(get_logger(), "GNSS node activated");
    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_deactivate(
    const rclcpp_lifecycle::State& state)
{
    if (navigationThread_.joinable())
    {
        navigationThread_.request_stop();
        navigationThread_.join();
    }

    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(get_logger(), "GNSS node deactivated");
    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting rtk_");
    rtk_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting navPublisher_");
    navPublisher_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting navSatFixPub_");
    navSatFixPub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting velPub_");
    velPub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting timeRefPub_");
    timeRefPub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resetting converter_");
    converter_.reset();

    RCLCPP_INFO(get_logger(), "on_cleanup: resetting hat_ (ptr=%p)",
        static_cast<void*>(hat_.get()));
    hat_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: hat_ reset complete");

    RCLCPP_INFO(get_logger(), "GNSS node cleaned up");
    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_shutdown(
    const rclcpp_lifecycle::State&)
{
    if (navigationThread_.joinable())
    {
        navigationThread_.request_stop();
        navigationThread_.join();
    }

    rtk_.reset();
    navPublisher_.reset();
    navSatFixPub_.reset();
    velPub_.reset();
    timeRefPub_.reset();
    converter_.reset();

    hat_.reset();

    RCLCPP_INFO(get_logger(), "GNSS node shut down");
    return CallbackReturn::SUCCESS;
}

void GnssNode::sendNavigation(const Navigation& navigation)
{
    navPublisher_->publish(converter_->toNavigation(navigation));

    if (config_.publishStandardTopics())
    {
        navSatFixPub_->publish(converter_->toNavSatFix(navigation));
        velPub_->publish(converter_->toTwist(navigation.pvt));

        if (const auto timeRef = converter_->toTimeReference(navigation.pvt))
            timeRefPub_->publish(*timeRef);
    }
}

void GnssNode::navigationLoop(std::stop_token stopToken)
{
    while (!stopToken.stop_requested() && rclcpp::ok())
    {
        const auto navigation = hat_->waitAndGetFreshNavigation();

        if (stopToken.stop_requested())
            break;

        sendNavigation(navigation);

        if (rtk_)
            rtk_->pollAndPublish();
    }
}

void GnssNode::onGetGeofencingCfg(
    const jp_gnss_hat::srv::GetGeofencingCfg::Request::SharedPtr,
    jp_gnss_hat::srv::GetGeofencingCfg::Response::SharedPtr response)
{
    *response = config_.toGeofencingResponse();
}

void GnssNode::onGetGnssConfig(
    const jp_gnss_hat::srv::GetGnssConfig::Request::SharedPtr,
    jp_gnss_hat::srv::GetGnssConfig::Response::SharedPtr response)
{
    response->config = config_.toMsg();
}

void GnssNode::onSetGnssConfig(
    const jp_gnss_hat::srv::SetGnssConfig::Request::SharedPtr request,
    jp_gnss_hat::srv::SetGnssConfig::Response::SharedPtr response)
{
    Config newConfig;
    try
    {
        newConfig.fromMsg(request->config);
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = e.what();
        return;
    }
    newConfig.nodeId(config_.nodeId());

    if (Config::getDynamicModelName(newConfig.gnssConfig().dynamicModel)
            == "Unknown")
    {
        response->success = false;
        response->message = "Invalid dynamic_model: " +
            std::to_string(
                static_cast<int>(newConfig.gnssConfig().dynamicModel));
        return;
    }

    const Config oldConfig = config_;
    pendingConfig_ = std::move(newConfig);

    CallbackReturn ret;
    const auto currentStateId = get_current_state().id();

    if (currentStateId ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, ret);
        if (ret != CallbackReturn::SUCCESS)
        {
            pendingConfig_.reset();
            response->success = false;
            response->message = "Deactivation failed";
            return;
        }
    }

    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, ret);
        if (ret != CallbackReturn::SUCCESS)
        {
            pendingConfig_.reset();
            RCLCPP_WARN(get_logger(),
                "Cleanup failed, attempting to reactivate with old config");
            trigger_transition(
                lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ret);
            response->success = false;
            response->message = (ret == CallbackReturn::SUCCESS)
                ? "Cleanup failed; restored old config"
                : "Cleanup failed; reactivation also failed — node is inactive";
            return;
        }
    }

    trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_WARN(get_logger(),
            "Configuration with new config failed, rolling back");
        pendingConfig_ = oldConfig;
        const auto rollbackMsg = rollbackToActive();
        response->success = false;
        response->message = "Configuration failed; " + rollbackMsg;
        return;
    }

    trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_WARN(get_logger(),
            "Activation with new config failed, rolling back");
        trigger_transition(
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, ret);
        pendingConfig_ = oldConfig;
        const auto rollbackMsg = rollbackToActive();
        response->success = false;
        response->message = "Activation failed; " + rollbackMsg;
        return;
    }

    if (request->save_to_yaml)
    {
        config_.saveToYaml(configPath_);
        RCLCPP_INFO(get_logger(), "Config saved to: %s",
            configPath_.c_str());
    }

    RCLCPP_INFO(get_logger(), "GNSS HAT reconfigured via service");
    response->success = true;
    response->message = "OK";
}

std::string GnssNode::rollbackToActive()
{
    CallbackReturn ret;

    trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(get_logger(),
            "Rollback configuration failed - node is unconfigured");
        return "rollback failed - node is unconfigured";
    }

    trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, ret);
    if (ret != CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(get_logger(),
            "Rollback activation failed - node is inactive");
        return "rollback configured but activation failed - node is inactive";
    }

    RCLCPP_INFO(get_logger(), "Rolled back to previous config");
    return "rolled back to previous config";
}

}  // JimmyPaputto
