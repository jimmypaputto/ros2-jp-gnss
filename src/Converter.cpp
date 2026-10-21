/*
 * Jimmy Paputto 2026
 */

#include "Converter.hpp"

#include <cmath>
#include <utility>


namespace JimmyPaputto
{

Converter::Converter(std::string frameId, rclcpp::Clock::SharedPtr clock)
:   frameId_(std::move(frameId)),
    clock_(std::move(clock))
{
}

std_msgs::msg::Header Converter::makeHeader() const
{
    std_msgs::msg::Header header;
    header.stamp = clock_->now();
    header.frame_id = frameId_;
    return header;
}

jp_gnss_hat::msg::PositionVelocityTime Converter::toPvt(
    const PositionVelocityTime& pvt)
{
    jp_gnss_hat::msg::PositionVelocityTime msg;

    msg.fix_quality = static_cast<uint8_t>(pvt.fixQuality);
    msg.fix_status  = static_cast<uint8_t>(pvt.fixStatus);
    msg.fix_type    = static_cast<uint8_t>(pvt.fixType);

    msg.latitude  = pvt.latitude;
    msg.longitude = pvt.longitude;
    msg.altitude  = pvt.altitude;
    msg.altitude_msl = pvt.altitudeMSL;

    msg.speed_over_ground = pvt.speedOverGround;
    msg.speed_accuracy    = pvt.speedAccuracy;
    msg.heading           = pvt.heading;
    msg.heading_accuracy  = pvt.headingAccuracy;

    msg.horizontal_accuracy = pvt.horizontalAccuracy;
    msg.vertical_accuracy   = pvt.verticalAccuracy;

    msg.visible_satellites = pvt.visibleSatellites;

    msg.utc_hours    = pvt.utc.hh;
    msg.utc_minutes  = pvt.utc.mm;
    msg.utc_seconds  = pvt.utc.ss;
    msg.utc_valid    = pvt.utc.valid;
    msg.utc_accuracy = pvt.utc.accuracy;

    msg.date_day   = pvt.date.day;
    msg.date_month = pvt.date.month;
    msg.date_year  = pvt.date.year;
    msg.date_valid = pvt.date.valid;

    return msg;
}

jp_gnss_hat::msg::DilutionOverPrecision Converter::toDop(
    const DilutionOverPrecision& dop)
{
    jp_gnss_hat::msg::DilutionOverPrecision msg;

    msg.geometric  = dop.geometric;
    msg.position   = dop.position;
    msg.time       = dop.time;
    msg.vertical   = dop.vertical;
    msg.horizontal = dop.horizontal;
    msg.northing   = dop.northing;
    msg.easting    = dop.easting;

    return msg;
}

jp_gnss_hat::msg::GeofencingNav Converter::toGeofencingNav(
    const Geofencing& geofencing)
{
    jp_gnss_hat::msg::GeofencingNav msg;

    msg.itow = geofencing.nav.iTOW;
    msg.geofencing_status = static_cast<uint8_t>(
        geofencing.nav.geofencingStatus);
    msg.number_of_geofences = geofencing.nav.numberOfGeofences;
    msg.combined_state = static_cast<uint8_t>(geofencing.nav.combinedState);
    for (size_t i = 0; i < 4; ++i)
        msg.geofences_status[i] = static_cast<uint8_t>(
            geofencing.nav.geofencesStatus[i]);

    return msg;
}

jp_gnss_hat::msg::RfBlock Converter::toRfBlock(const RfBlock& rfBlock)
{
    jp_gnss_hat::msg::RfBlock msg;

    msg.band           = static_cast<uint8_t>(rfBlock.id);
    msg.jamming_state  = static_cast<uint8_t>(rfBlock.jammingState);
    msg.antenna_status = static_cast<uint8_t>(rfBlock.antennaStatus);
    msg.antenna_power  = static_cast<uint8_t>(rfBlock.antennaPower);
    msg.post_status    = rfBlock.postStatus;
    msg.noise_per_ms   = rfBlock.noisePerMS;
    msg.agc_monitor    = rfBlock.agcMonitor;
    msg.cw_interference_suppression_level =
        rfBlock.cwInterferenceSuppressionLevel;
    msg.ofs_i = rfBlock.ofsI;
    msg.mag_i = rfBlock.magI;
    msg.ofs_q = rfBlock.ofsQ;
    msg.mag_q = rfBlock.magQ;

    return msg;
}

jp_gnss_hat::msg::SatelliteInfo Converter::toSatelliteInfo(
    const SatelliteInfo& sat)
{
    jp_gnss_hat::msg::SatelliteInfo msg;

    msg.gnss_id     = static_cast<uint8_t>(sat.gnssId);
    msg.sv_id       = sat.svId;
    msg.cno         = sat.cno;
    msg.elevation   = sat.elevation;
    msg.azimuth     = sat.azimuth;
    msg.quality     = static_cast<uint8_t>(sat.quality);
    msg.used_in_fix = sat.usedInFix;
    msg.healthy     = sat.healthy;
    msg.diff_corr   = sat.diffCorr;
    msg.eph_avail   = sat.ephAvail;
    msg.alm_avail   = sat.almAvail;

    return msg;
}

jp_gnss_hat::msg::Navigation Converter::toNavigation(
    const Navigation& navigation) const
{
    jp_gnss_hat::msg::Navigation msg;

    msg.header = makeHeader();
    msg.pvt = toPvt(navigation.pvt);
    msg.dop = toDop(navigation.dop);
    msg.geofencing = toGeofencingNav(navigation.geofencing);
    msg.rf_blocks.reserve(navigation.rfBlocks.size());
    for (const auto& rfBlock : navigation.rfBlocks)
        msg.rf_blocks.push_back(toRfBlock(rfBlock));
    msg.satellites.reserve(navigation.satellites.size());
    for (const auto& sat : navigation.satellites)
        msg.satellites.push_back(toSatelliteInfo(sat));

    return msg;
}

sensor_msgs::msg::NavSatFix Converter::toNavSatFix(
    const Navigation& navigation) const
{
    sensor_msgs::msg::NavSatFix fix;
    fix.header = makeHeader();
    const auto& pvt = navigation.pvt;

    // Map fix_quality to NavSatStatus
    switch (pvt.fixQuality)
    {
        case EFixQuality::Invalid:
            fix.status.status =
                sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            break;
        case EFixQuality::GpsFix2D3D:
        case EFixQuality::PpsFix:
        case EFixQuality::DeadReckoning:
            fix.status.status =
                sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            break;
        case EFixQuality::DGNSS:
            fix.status.status =
                sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case EFixQuality::FixedRTK:
        case EFixQuality::FloatRtk:
            fix.status.status =
                sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
        default:
            fix.status.status =
                sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            break;
    }

    // Determine which GNSS services are in use from satellite info
    fix.status.service = 0;
    for (const auto& sat : navigation.satellites)
    {
        if (!sat.usedInFix)
            continue;
        switch (sat.gnssId)
        {
            case EGnssId::GPS:
                fix.status.service |=
                    sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                break;
            case EGnssId::Galileo:
                fix.status.service |=
                    sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
                break;
            case EGnssId::GLONASS:
                fix.status.service |=
                    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
                break;
            case EGnssId::BeiDou:
                fix.status.service |=
                    sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
                break;
            default:
                break;
        }
    }

    fix.latitude  = pvt.latitude;
    fix.longitude = pvt.longitude;
    fix.altitude  = static_cast<double>(pvt.altitude);

    // gnsshat already reports accuracies in metres
    const double hAcc_m = static_cast<double>(pvt.horizontalAccuracy);
    const double vAcc_m = static_cast<double>(pvt.verticalAccuracy);
    const double hVar = hAcc_m * hAcc_m;
    const double vVar = vAcc_m * vAcc_m;

    // position_covariance is row-major [East, North, Up]
    fix.position_covariance[0] = hVar;  // East-East
    fix.position_covariance[1] = 0.0;
    fix.position_covariance[2] = 0.0;
    fix.position_covariance[3] = 0.0;
    fix.position_covariance[4] = hVar;  // North-North
    fix.position_covariance[5] = 0.0;
    fix.position_covariance[6] = 0.0;
    fix.position_covariance[7] = 0.0;
    fix.position_covariance[8] = vVar;  // Up-Up

    fix.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    return fix;
}

geometry_msgs::msg::TwistWithCovarianceStamped Converter::toTwist(
    const PositionVelocityTime& pvt) const
{
    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header = makeHeader();

    // Decompose speed + heading into ENU linear velocity
    const double heading_rad = pvt.heading * M_PI / 180.0;
    const double speed = static_cast<double>(pvt.speedOverGround);
    twist.twist.twist.linear.x = speed * std::sin(heading_rad); // East
    twist.twist.twist.linear.y = speed * std::cos(heading_rad); // North
    twist.twist.twist.linear.z = 0.0; // not available from PVT

    // Angular velocity not available from GNSS
    twist.twist.twist.angular.x = 0.0;
    twist.twist.twist.angular.y = 0.0;
    twist.twist.twist.angular.z = 0.0;

    // Speed accuracy → diagonal covariance
    const double sVar =
        static_cast<double>(pvt.speedAccuracy * pvt.speedAccuracy);
    // 6×6 row-major, only diagonal for linear x/y
    twist.twist.covariance.fill(0.0);
    twist.twist.covariance[0]  = sVar;  // vx-vx
    twist.twist.covariance[7]  = sVar;  // vy-vy
    twist.twist.covariance[14] = 99.0;  // vz-vz unknown → large

    return twist;
}

std::optional<sensor_msgs::msg::TimeReference> Converter::toTimeReference(
    const PositionVelocityTime& pvt) const
{
    // A UTC time without a valid date cannot form an epoch
    if (!pvt.date.valid || !pvt.utc.valid)
        return std::nullopt;

    sensor_msgs::msg::TimeReference timeRef;
    timeRef.header = makeHeader();

    struct tm t{};
    t.tm_year = pvt.date.year - 1900;
    t.tm_mon  = pvt.date.month - 1;
    t.tm_mday = pvt.date.day;
    t.tm_hour = pvt.utc.hh;
    t.tm_min  = pvt.utc.mm;
    t.tm_sec  = pvt.utc.ss;
    const time_t epoch = timegm(&t);

    timeRef.time_ref.sec = static_cast<int32_t>(epoch);
    timeRef.time_ref.nanosec = 0u;
    timeRef.source = "gnss";

    return timeRef;
}

}  // JimmyPaputto
