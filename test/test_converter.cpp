/*
 * Jimmy Paputto 2026
 */

#include <cmath>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "../src/Converter.hpp"


namespace JimmyPaputto::test
{

class ConverterTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
    }

    void SetUp() override
    {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        converter_ = std::make_unique<Converter>("gnss_frame", clock_);
    }

    Navigation makeNavigation() const
    {
        Navigation nav;

        nav.pvt.fixQuality = EFixQuality::GpsFix2D3D;
        nav.pvt.fixStatus = EFixStatus::Active;
        nav.pvt.fixType = EFixType::Fix3D;
        nav.pvt.latitude = 52.2297;
        nav.pvt.longitude = 21.0122;
        nav.pvt.altitude = 130.0f;
        nav.pvt.altitudeMSL = 125.0f;
        nav.pvt.speedOverGround = 1.5f;
        nav.pvt.speedAccuracy = 0.1f;
        nav.pvt.heading = 90.0f;
        nav.pvt.headingAccuracy = 1.0f;
        nav.pvt.horizontalAccuracy = 1.5f;  // m
        nav.pvt.verticalAccuracy = 2.0f;    // m
        nav.pvt.visibleSatellites = 12;
        nav.pvt.utc = {.hh = 14, .mm = 30, .ss = 45,
                       .valid = true, .accuracy = 20};
        nav.pvt.date = {.day = 15, .month = 3, .year = 2026, .valid = true};

        nav.dop.geometric = 1.2f;
        nav.dop.position = 1.1f;
        nav.dop.time = 0.9f;
        nav.dop.vertical = 0.8f;
        nav.dop.horizontal = 0.7f;
        nav.dop.northing = 0.6f;
        nav.dop.easting = 0.5f;

        nav.geofencing.nav.iTOW = 12345;
        nav.geofencing.nav.geofencingStatus =
            EGeofencingStatus::Active;
        nav.geofencing.nav.numberOfGeofences = 1;
        nav.geofencing.nav.combinedState = EGeofenceStatus::Inside;
        nav.geofencing.nav.geofencesStatus.fill(EGeofenceStatus::Unknown);
        nav.geofencing.nav.geofencesStatus[0] = EGeofenceStatus::Inside;

        RfBlock rf{};
        rf.id = static_cast<uint8_t>(EGnssBand::L1);
        rf.jammingState = EJammingState::Ok_NoSignificantJamming;
        rf.antennaStatus = EAntennaStatus::Ok;
        rf.antennaPower = EAntennaPower::On;
        rf.postStatus = 0;
        rf.noisePerMS = 100;
        rf.agcMonitor = 50.0f;
        rf.cwInterferenceSuppressionLevel = 0.0f;
        rf.ofsI = 1; rf.magI = 2; rf.ofsQ = 3; rf.magQ = 4;
        nav.rfBlocks.push_back(rf);

        SatelliteInfo sat{};
        sat.gnssId = EGnssId::GPS;
        sat.svId = 5;
        sat.cno = 40;
        sat.elevation = 45;
        sat.azimuth = 180;
        sat.quality = ESvQuality::CodeAndCarrierLocked3;
        sat.usedInFix = true;
        sat.healthy = true;
        sat.diffCorr = false;
        sat.ephAvail = true;
        sat.almAvail = true;
        nav.satellites.push_back(sat);

        SatelliteInfo satGal{};
        satGal.gnssId = EGnssId::Galileo;
        satGal.svId = 12;
        satGal.cno = 38;
        satGal.elevation = 60;
        satGal.azimuth = 90;
        satGal.quality = ESvQuality::CodeAndCarrierLocked2;
        satGal.usedInFix = true;
        satGal.healthy = true;
        nav.satellites.push_back(satGal);

        return nav;
    }

    rclcpp::Clock::SharedPtr clock_;
    std::unique_ptr<Converter> converter_;
};


// --- toNavigation ---

TEST_F(ConverterTest, NavigationPvtFields)
{
    const auto nav = makeNavigation();
    const auto msg = converter_->toNavigation(nav);

    EXPECT_EQ(msg.header.frame_id, "gnss_frame");

    EXPECT_EQ(msg.pvt.fix_quality,
              static_cast<uint8_t>(EFixQuality::GpsFix2D3D));
    EXPECT_EQ(msg.pvt.fix_status,
              static_cast<uint8_t>(EFixStatus::Active));
    EXPECT_EQ(msg.pvt.fix_type,
              static_cast<uint8_t>(EFixType::Fix3D));
    EXPECT_DOUBLE_EQ(msg.pvt.latitude, 52.2297);
    EXPECT_DOUBLE_EQ(msg.pvt.longitude, 21.0122);
    EXPECT_FLOAT_EQ(msg.pvt.altitude, 130.0f);
    EXPECT_FLOAT_EQ(msg.pvt.altitude_msl, 125.0f);
    EXPECT_FLOAT_EQ(msg.pvt.speed_over_ground, 1.5f);
    EXPECT_FLOAT_EQ(msg.pvt.heading, 90.0f);
    EXPECT_EQ(msg.pvt.visible_satellites, 12);
    EXPECT_EQ(msg.pvt.utc_hours, 14);
    EXPECT_EQ(msg.pvt.utc_minutes, 30);
    EXPECT_EQ(msg.pvt.utc_seconds, 45);
    EXPECT_TRUE(msg.pvt.utc_valid);
}

TEST_F(ConverterTest, NavigationDopFields)
{
    const auto nav = makeNavigation();
    const auto msg = converter_->toNavigation(nav);

    EXPECT_FLOAT_EQ(msg.dop.geometric, 1.2f);
    EXPECT_FLOAT_EQ(msg.dop.position, 1.1f);
    EXPECT_FLOAT_EQ(msg.dop.time, 0.9f);
    EXPECT_FLOAT_EQ(msg.dop.vertical, 0.8f);
    EXPECT_FLOAT_EQ(msg.dop.horizontal, 0.7f);
    EXPECT_FLOAT_EQ(msg.dop.northing, 0.6f);
    EXPECT_FLOAT_EQ(msg.dop.easting, 0.5f);
}

TEST_F(ConverterTest, NavigationGeofencingFields)
{
    const auto nav = makeNavigation();
    const auto msg = converter_->toNavigation(nav);

    EXPECT_EQ(msg.geofencing.itow, 12345u);
    EXPECT_EQ(msg.geofencing.geofencing_status,
              static_cast<uint8_t>(EGeofencingStatus::Active));
    EXPECT_EQ(msg.geofencing.number_of_geofences, 1);
    EXPECT_EQ(msg.geofencing.combined_state,
              static_cast<uint8_t>(EGeofenceStatus::Inside));
}

TEST_F(ConverterTest, NavigationRfBlockFields)
{
    const auto nav = makeNavigation();
    const auto msg = converter_->toNavigation(nav);

    ASSERT_EQ(msg.rf_blocks.size(), 1u);
    EXPECT_EQ(msg.rf_blocks[0].band,
              static_cast<uint8_t>(EGnssBand::L1));
    EXPECT_EQ(msg.rf_blocks[0].jamming_state,
              static_cast<uint8_t>(EJammingState::Ok_NoSignificantJamming));
    EXPECT_EQ(msg.rf_blocks[0].noise_per_ms, 100);
    EXPECT_FLOAT_EQ(msg.rf_blocks[0].agc_monitor, 50.0f);
}

TEST_F(ConverterTest, NavigationSatelliteFields)
{
    const auto nav = makeNavigation();
    const auto msg = converter_->toNavigation(nav);

    ASSERT_EQ(msg.satellites.size(), 2u);
    EXPECT_EQ(msg.satellites[0].gnss_id,
              static_cast<uint8_t>(EGnssId::GPS));
    EXPECT_EQ(msg.satellites[0].sv_id, 5);
    EXPECT_EQ(msg.satellites[0].cno, 40);
    EXPECT_TRUE(msg.satellites[0].used_in_fix);
    EXPECT_EQ(msg.satellites[1].gnss_id,
              static_cast<uint8_t>(EGnssId::Galileo));
}


// --- toNavSatFix ---

TEST_F(ConverterTest, NavSatFixPosition)
{
    const auto nav = makeNavigation();
    const auto fix = converter_->toNavSatFix(nav);

    EXPECT_DOUBLE_EQ(fix.latitude, 52.2297);
    EXPECT_DOUBLE_EQ(fix.longitude, 21.0122);
    EXPECT_DOUBLE_EQ(fix.altitude, 130.0);
}

TEST_F(ConverterTest, NavSatFixCovariance)
{
    const auto nav = makeNavigation();
    const auto fix = converter_->toNavSatFix(nav);

    // gnsshat reports accuracy in metres: variance = accuracy²
    const double hVar = 1.5 * 1.5;
    const double vVar = 2.0 * 2.0;

    EXPECT_DOUBLE_EQ(fix.position_covariance[0], hVar);
    EXPECT_DOUBLE_EQ(fix.position_covariance[4], hVar);
    EXPECT_DOUBLE_EQ(fix.position_covariance[8], vVar);
    EXPECT_EQ(fix.position_covariance_type,
              sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
}

TEST_F(ConverterTest, NavSatFixCovarianceIsSquareOfMetres)
{
    auto nav = makeNavigation();
    nav.pvt.horizontalAccuracy = 0.014f;
    nav.pvt.verticalAccuracy = 25.0f;
    const auto fix = converter_->toNavSatFix(nav);

    const double h = static_cast<double>(0.014f);
    EXPECT_DOUBLE_EQ(fix.position_covariance[0], h * h);
    EXPECT_DOUBLE_EQ(fix.position_covariance[4], h * h);
    EXPECT_DOUBLE_EQ(fix.position_covariance[8], 625.0);
    for (const auto i : {1, 2, 3, 5, 6, 7})
        EXPECT_DOUBLE_EQ(fix.position_covariance[i], 0.0);
}

TEST_F(ConverterTest, NavSatFixStatusMapping)
{
    auto nav = makeNavigation();

    // GpsFix2D3D → STATUS_FIX
    nav.pvt.fixQuality = EFixQuality::GpsFix2D3D;
    EXPECT_EQ(converter_->toNavSatFix(nav).status.status,
              sensor_msgs::msg::NavSatStatus::STATUS_FIX);

    // DGNSS → STATUS_SBAS_FIX
    nav.pvt.fixQuality = EFixQuality::DGNSS;
    EXPECT_EQ(converter_->toNavSatFix(nav).status.status,
              sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX);

    // FixedRTK → STATUS_GBAS_FIX
    nav.pvt.fixQuality = EFixQuality::FixedRTK;
    EXPECT_EQ(converter_->toNavSatFix(nav).status.status,
              sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);

    // Invalid → STATUS_NO_FIX
    nav.pvt.fixQuality = EFixQuality::Invalid;
    EXPECT_EQ(converter_->toNavSatFix(nav).status.status,
              sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST_F(ConverterTest, NavSatFixServiceDetection)
{
    const auto nav = makeNavigation();
    const auto fix = converter_->toNavSatFix(nav);

    EXPECT_TRUE(fix.status.service &
                sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    EXPECT_TRUE(fix.status.service &
                sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO);
}


// --- toTwist ---

TEST_F(ConverterTest, TwistSpeedDecomposition)
{
    const auto nav = makeNavigation();
    const auto twist = converter_->toTwist(nav.pvt);

    // heading = 90° → sin(90°) = 1, cos(90°) ≈ 0
    // speed = 1.5 m/s
    const double heading_rad = 90.0 * M_PI / 180.0;
    const double expectedX = 1.5 * std::sin(heading_rad);  // East ~1.5
    const double expectedY = 1.5 * std::cos(heading_rad);  // North ~0

    EXPECT_NEAR(twist.twist.twist.linear.x, expectedX, 1e-6);
    EXPECT_NEAR(twist.twist.twist.linear.y, expectedY, 1e-6);
    EXPECT_DOUBLE_EQ(twist.twist.twist.linear.z, 0.0);
}

TEST_F(ConverterTest, TwistCovariance)
{
    const auto nav = makeNavigation();
    const auto twist = converter_->toTwist(nav.pvt);

    const double sVar = 0.1 * 0.1;  // speedAccuracy = 0.1
    EXPECT_NEAR(twist.twist.covariance[0], sVar, 1e-6);   // vx-vx
    EXPECT_NEAR(twist.twist.covariance[7], sVar, 1e-6);   // vy-vy
    EXPECT_DOUBLE_EQ(twist.twist.covariance[14], 99.0);    // vz-vz
}


// --- toTimeReference ---

TEST_F(ConverterTest, TimeReferenceValid)
{
    const auto nav = makeNavigation();
    const auto timeRef = converter_->toTimeReference(nav.pvt);

    ASSERT_TRUE(timeRef.has_value());
    EXPECT_EQ(timeRef->header.frame_id, "gnss_frame");
    // 2026-03-15 14:30:45 UTC
    EXPECT_EQ(timeRef->time_ref.sec, 1773585045);
    EXPECT_EQ(timeRef->time_ref.nanosec, 0u);
    EXPECT_EQ(timeRef->source, "gnss");
}

TEST_F(ConverterTest, TimeReferenceSkippedWithoutValidDate)
{
    auto nav = makeNavigation();
    nav.pvt.date.valid = false;

    EXPECT_FALSE(converter_->toTimeReference(nav.pvt).has_value());
}

TEST_F(ConverterTest, TimeReferenceSkippedWithoutValidUtc)
{
    auto nav = makeNavigation();
    nav.pvt.utc.valid = false;

    EXPECT_FALSE(converter_->toTimeReference(nav.pvt).has_value());
}


}  // JimmyPaputto::test
