/*
 * Jimmy Paputto 2026
 */

#ifndef JIMMY_PAPUTTO_CONVERTERS_HPP_
#define JIMMY_PAPUTTO_CONVERTERS_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <jimmypaputto/GnssHat.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <jp_gnss_hat/msg/navigation.hpp>
#include <jp_gnss_hat/msg/rf_block_spectrum.hpp>
#include <jp_gnss_hat/msg/rtk_corrections.hpp>


namespace JimmyPaputto
{

class Converter
{
public:
    // hatName goes into the custom messages (the visualization tells the HAT
    // variant by it), frameId into the standard ones (a TF frame).
    Converter(std::string hatName, std::string frameId,
        rclcpp::Clock::SharedPtr clock);

    // Not const: remembers the last published spectrum (see lastSpectrum_).
    jp_gnss_hat::msg::Navigation toNavigation(const Navigation& navigation);
    sensor_msgs::msg::NavSatFix toNavSatFix(
        const Navigation& navigation) const;
    geometry_msgs::msg::TwistWithCovarianceStamped toTwist(
        const PositionVelocityTime& pvt) const;
    std::optional<sensor_msgs::msg::TimeReference> toTimeReference(
        const PositionVelocityTime& pvt) const;

private:
    std_msgs::msg::Header makeHeader(const std::string& frameId) const;

    static jp_gnss_hat::msg::PositionVelocityTime toPvt(
        const PositionVelocityTime& pvt);
    static jp_gnss_hat::msg::DilutionOverPrecision toDop(
        const DilutionOverPrecision& dop);
    static jp_gnss_hat::msg::GeofencingNav toGeofencingNav(
        const Geofencing& geofencing);
    static jp_gnss_hat::msg::RfBlock toRfBlock(
        const RfBlock& rfBlock);
    static jp_gnss_hat::msg::SatelliteInfo toSatelliteInfo(
        const SatelliteInfo& sat);
    static jp_gnss_hat::msg::RfBlockSpectrum toRfBlockSpectrum(
        const RfBlockSpectrumData& spectrum);

    std::string hatName_;
    std::string frameId_;
    rclcpp::Clock::SharedPtr clock_;
    // gnsshat hands out the last MON-SPAN with every epoch, so the spectrum
    // goes out only when it differs from the one published before.
    std::vector<RfBlockSpectrumData> lastSpectrum_;
};

}  // JimmyPaputto

#endif  // JIMMY_PAPUTTO_CONVERTERS_HPP_
