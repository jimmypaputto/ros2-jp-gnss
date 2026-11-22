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

// After SIGINT/SIGTERM the context is already shut down, so no lifecycle
// transition can run any more: each one fails to publish its transition event
// before reaching the callback. Release everything here instead, before the
// members used by the navigation thread are destroyed.
GnssNode::~GnssNode()
{
    releaseResources();
}

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
        "  Nav Filters:      %s\n"
        "  RTK:              %s\n"
        "  Save to Flash:    %s",
        std::string(hat_->name()).c_str(),
        cfg.measurementRate_Hz,
        Config::getDynamicModelName(cfg.dynamicModel).c_str(),
        static_cast<int>(cfg.dynamicModel),
        cfg.timepulsePinConfig.active ? "Enabled" : "Disabled",
        cfg.geofencing.has_value() ? "Enabled" : "Disabled",
        cfg.navigationFilters.has_value() ? "Enabled" : "Receiver defaults",
        cfg.rtk.has_value()
            ? (cfg.rtk->mode == ERtkMode::Base ? "Base" : "Rover")
            : "Disabled",
        cfg.saveToFlash ? "Yes" : "No"
    );

    // gnsshat reports why start() failed only on stderr. Since 2.0.0 it
    // returns false instead of throwing, but an exception must still not
    // escape the lifecycle callback.
    bool started = false;
    try
    {
        started = hat_->start(config_.gnssConfig());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to start GNSS HAT: %s", e.what());
        return CallbackReturn::FAILURE;
    }
    if (!started)
    {
        RCLCPP_ERROR(get_logger(),
            "Failed to start GNSS HAT: gnsshat rejected the configuration or "
            "the receiver did not accept it. The reason is printed by gnsshat "
            "on stderr just above (e.g. \"[GnssConfig] Invalid navigation "
            "filter ...\").");
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
            topics_.ntrip,
            config_.rtkCorrections()
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
    stopNavigationThread();

    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(get_logger(), "GNSS node deactivated");
    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    releaseResources();

    RCLCPP_INFO(get_logger(), "GNSS node cleaned up");
    return CallbackReturn::SUCCESS;
}

GnssNode::CallbackReturn GnssNode::on_shutdown(
    const rclcpp_lifecycle::State&)
{
    releaseResources();

    RCLCPP_INFO(get_logger(), "GNSS node shut down");
    return CallbackReturn::SUCCESS;
}

void GnssNode::stopNavigationThread()
{
    if (!navigationThread_.joinable())
        return;

    navigationThread_.request_stop();
    navigationThread_.join();
}

// The navigation thread uses everything released below, so it goes first.
// Safe to call again: everything is already released then.
void GnssNode::releaseResources()
{
    stopNavigationThread();

    RCLCPP_DEBUG(get_logger(), "Releasing RTK bridge and publishers");
    rtk_.reset();
    navPublisher_.reset();
    navSatFixPub_.reset();
    velPub_.reset();
    timeRefPub_.reset();
    converter_.reset();

    RCLCPP_DEBUG(get_logger(), "Releasing GNSS HAT");
    hat_.reset();
    RCLCPP_DEBUG(get_logger(), "GNSS HAT released");
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

// The stop token ends the wait as soon as deactivate/shutdown asks for it,
// so join() does not wait for the next epoch of a silent receiver.
void GnssNode::navigationLoop(std::stop_token stopToken)
{
    while (rclcpp::ok())
    {
        const auto navigation = hat_->waitAndGetFreshNavigation(stopToken);
        if (!navigation)
            break;

        sendNavigation(*navigation);

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

    const auto currentStateId = get_current_state().id();

    if (currentStateId ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        if (!runTransition(
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
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
        if (!runTransition(
                lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        {
            pendingConfig_.reset();
            RCLCPP_WARN(get_logger(),
                "Cleanup failed, attempting to reactivate with old config");
            const bool reactivated = runTransition(
                lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            response->success = false;
            response->message = reactivated
                ? "Cleanup failed; restored old config"
                : "Cleanup failed; reactivation also failed — node is inactive";
            return;
        }
    }

    if (!runTransition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
        RCLCPP_WARN(get_logger(),
            "Configuration with new config failed, rolling back");
        pendingConfig_ = oldConfig;
        const auto rollbackMsg = rollbackToActive();
        response->success = false;
        response->message = "Configuration failed; " + rollbackMsg;
        return;
    }

    if (!runTransition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
        RCLCPP_WARN(get_logger(),
            "Activation with new config failed, rolling back");
        runTransition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        pendingConfig_ = oldConfig;
        const auto rollbackMsg = rollbackToActive();
        response->success = false;
        response->message = "Activation failed; " + rollbackMsg;
        return;
    }

    RCLCPP_INFO(get_logger(), "GNSS HAT reconfigured via service");

    if (request->save_to_yaml)
    {
        // An exception escaping a service callback would abort the process.
        try
        {
            config_.saveToYaml(configPath_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to save config to %s: %s",
                configPath_.c_str(), e.what());
            response->success = false;
            response->message =
                std::string("Config applied but not saved: ") + e.what();
            return;
        }
        RCLCPP_INFO(get_logger(), "Config saved to: %s",
            configPath_.c_str());
    }

    response->success = true;
    response->message = "OK";
}

std::string GnssNode::rollbackToActive()
{
    if (!runTransition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
        RCLCPP_FATAL(get_logger(),
            "Rollback configuration failed - node is unconfigured");
        return "rollback failed - node is unconfigured";
    }

    if (!runTransition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
        RCLCPP_FATAL(get_logger(),
            "Rollback activation failed - node is inactive");
        return "rollback configured but activation failed - node is inactive";
    }

    RCLCPP_INFO(get_logger(), "Rolled back to previous config");
    return "rolled back to previous config";
}

// trigger_transition() leaves the return code untouched when the transition
// cannot even start, e.g. when it is not valid from the current state.
bool GnssNode::runTransition(const std::uint8_t transitionId)
{
    auto ret = CallbackReturn::ERROR;
    trigger_transition(transitionId, ret);
    return ret == CallbackReturn::SUCCESS;
}

}  // JimmyPaputto
