/*
 * Jimmy Paputto 2026
 */

#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "../src/Rtk.hpp"


namespace JimmyPaputto::test
{

// --- fromMsg / toMsg round-trip ---

TEST(RtkMsgRoundTrip, RoverMode)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.rtk_mode = static_cast<uint8_t>(ERtkMode::Rover);

    const auto cfg = RtkBridge::fromMsg(msg);
    EXPECT_EQ(cfg.mode, ERtkMode::Rover);
    EXPECT_FALSE(cfg.base.has_value());

    jp_gnss_hat::msg::GnssConfig out{};
    RtkBridge::toMsg(cfg, out);
    EXPECT_EQ(out.rtk_mode, msg.rtk_mode);
}

TEST(RtkMsgRoundTrip, BaseSurveyIn)
{
    jp_gnss_hat::msg::GnssConfig original{};
    original.rtk_mode = static_cast<uint8_t>(ERtkMode::Base);
    original.rtk_base_type = 0;
    original.rtk_min_observation_time_s = 120;
    original.rtk_required_accuracy_m = 1.5;

    const auto cfg = RtkBridge::fromMsg(original);
    ASSERT_EQ(cfg.mode, ERtkMode::Base);
    ASSERT_TRUE(cfg.base.has_value());
    ASSERT_TRUE(std::holds_alternative<BaseConfig::SurveyIn>(
        cfg.base->mode));

    const auto& si = std::get<BaseConfig::SurveyIn>(cfg.base->mode);
    EXPECT_EQ(si.minimumObservationTime_s, 120u);
    EXPECT_DOUBLE_EQ(si.requiredPositionAccuracy_m, 1.5);

    jp_gnss_hat::msg::GnssConfig restored{};
    RtkBridge::toMsg(cfg, restored);
    EXPECT_EQ(restored.rtk_mode, original.rtk_mode);
    EXPECT_EQ(restored.rtk_base_type, original.rtk_base_type);
    EXPECT_EQ(restored.rtk_min_observation_time_s,
              original.rtk_min_observation_time_s);
    EXPECT_DOUBLE_EQ(restored.rtk_required_accuracy_m,
                     original.rtk_required_accuracy_m);
}

TEST(RtkMsgRoundTrip, BaseFixedEcef)
{
    jp_gnss_hat::msg::GnssConfig original{};
    original.rtk_mode = static_cast<uint8_t>(ERtkMode::Base);
    original.rtk_base_type = 1;
    original.rtk_ecef_x_m = 3856242.0;
    original.rtk_ecef_y_m = 1456789.0;
    original.rtk_ecef_z_m = 5021456.0;
    original.rtk_position_accuracy_m = 0.01;

    const auto cfg = RtkBridge::fromMsg(original);
    ASSERT_EQ(cfg.mode, ERtkMode::Base);
    ASSERT_TRUE(cfg.base.has_value());
    ASSERT_TRUE(std::holds_alternative<BaseConfig::FixedPosition>(
        cfg.base->mode));

    const auto& fp =
        std::get<BaseConfig::FixedPosition>(cfg.base->mode);
    ASSERT_TRUE(std::holds_alternative<BaseConfig::FixedPosition::Ecef>(
        fp.position));

    const auto& ecef =
        std::get<BaseConfig::FixedPosition::Ecef>(fp.position);
    EXPECT_DOUBLE_EQ(ecef.x_m, 3856242.0);
    EXPECT_DOUBLE_EQ(ecef.y_m, 1456789.0);
    EXPECT_DOUBLE_EQ(ecef.z_m, 5021456.0);
    EXPECT_DOUBLE_EQ(fp.positionAccuracy_m, 0.01);

    jp_gnss_hat::msg::GnssConfig restored{};
    RtkBridge::toMsg(cfg, restored);
    EXPECT_EQ(restored.rtk_base_type, 1);
    EXPECT_DOUBLE_EQ(restored.rtk_ecef_x_m, original.rtk_ecef_x_m);
    EXPECT_DOUBLE_EQ(restored.rtk_ecef_y_m, original.rtk_ecef_y_m);
    EXPECT_DOUBLE_EQ(restored.rtk_ecef_z_m, original.rtk_ecef_z_m);
    EXPECT_DOUBLE_EQ(restored.rtk_position_accuracy_m,
                     original.rtk_position_accuracy_m);
}

TEST(RtkMsgRoundTrip, BaseFixedLla)
{
    jp_gnss_hat::msg::GnssConfig original{};
    original.rtk_mode = static_cast<uint8_t>(ERtkMode::Base);
    original.rtk_base_type = 2;
    original.rtk_lla_latitude_deg = 52.2297;
    original.rtk_lla_longitude_deg = 21.0122;
    original.rtk_lla_height_m = 120.0;
    original.rtk_position_accuracy_m = 0.01;

    const auto cfg = RtkBridge::fromMsg(original);
    ASSERT_EQ(cfg.mode, ERtkMode::Base);
    ASSERT_TRUE(cfg.base.has_value());

    const auto& fp =
        std::get<BaseConfig::FixedPosition>(cfg.base->mode);
    ASSERT_TRUE(std::holds_alternative<BaseConfig::FixedPosition::Lla>(
        fp.position));

    const auto& lla =
        std::get<BaseConfig::FixedPosition::Lla>(fp.position);
    EXPECT_DOUBLE_EQ(lla.latitude_deg, 52.2297);
    EXPECT_DOUBLE_EQ(lla.longitude_deg, 21.0122);
    EXPECT_DOUBLE_EQ(lla.height_m, 120.0);

    jp_gnss_hat::msg::GnssConfig restored{};
    RtkBridge::toMsg(cfg, restored);
    EXPECT_EQ(restored.rtk_base_type, 2);
    EXPECT_DOUBLE_EQ(restored.rtk_lla_latitude_deg,
                     original.rtk_lla_latitude_deg);
    EXPECT_DOUBLE_EQ(restored.rtk_lla_longitude_deg,
                     original.rtk_lla_longitude_deg);
    EXPECT_DOUBLE_EQ(restored.rtk_lla_height_m,
                     original.rtk_lla_height_m);
}


// --- fromYaml / toYaml round-trip ---

TEST(RtkYamlRoundTrip, BaseSurveyIn)
{
    const std::string yaml = R"(
mode: base
base:
  type: survey_in
  min_observation_time_s: 60
  required_accuracy_m: 2.0
)";

    const YAML::Node node = YAML::Load(yaml);
    const auto cfg = RtkBridge::fromYaml(node);

    EXPECT_EQ(cfg.mode, ERtkMode::Base);
    ASSERT_TRUE(cfg.base.has_value());
    const auto& si = std::get<BaseConfig::SurveyIn>(cfg.base->mode);
    EXPECT_EQ(si.minimumObservationTime_s, 60u);
    EXPECT_DOUBLE_EQ(si.requiredPositionAccuracy_m, 2.0);

    // Round-trip through YAML emitter
    YAML::Emitter out;
    out << YAML::BeginMap;
    RtkBridge::toYaml(out, cfg);
    out << YAML::EndMap;

    const YAML::Node reloaded = YAML::Load(out.c_str());
    const auto restored = RtkBridge::fromYaml(reloaded["rtk"]);

    EXPECT_EQ(restored.mode, cfg.mode);
    ASSERT_TRUE(restored.base.has_value());
    const auto& rSi = std::get<BaseConfig::SurveyIn>(restored.base->mode);
    EXPECT_EQ(rSi.minimumObservationTime_s, si.minimumObservationTime_s);
    EXPECT_DOUBLE_EQ(rSi.requiredPositionAccuracy_m,
                     si.requiredPositionAccuracy_m);
}

TEST(RtkYamlRoundTrip, BaseFixedEcef)
{
    const std::string yaml = R"(
mode: base
base:
  type: fixed_ecef
  x_m: 3856242.0
  y_m: 1456789.0
  z_m: 5021456.0
  position_accuracy_m: 0.01
)";

    const YAML::Node node = YAML::Load(yaml);
    const auto cfg = RtkBridge::fromYaml(node);

    ASSERT_TRUE(cfg.base.has_value());
    const auto& fp =
        std::get<BaseConfig::FixedPosition>(cfg.base->mode);
    const auto& ecef =
        std::get<BaseConfig::FixedPosition::Ecef>(fp.position);
    EXPECT_DOUBLE_EQ(ecef.x_m, 3856242.0);

    YAML::Emitter out;
    out << YAML::BeginMap;
    RtkBridge::toYaml(out, cfg);
    out << YAML::EndMap;

    const YAML::Node reloaded = YAML::Load(out.c_str());
    const auto restored = RtkBridge::fromYaml(reloaded["rtk"]);

    const auto& rFp =
        std::get<BaseConfig::FixedPosition>(restored.base->mode);
    const auto& rEcef =
        std::get<BaseConfig::FixedPosition::Ecef>(rFp.position);
    EXPECT_DOUBLE_EQ(rEcef.x_m, ecef.x_m);
    EXPECT_DOUBLE_EQ(rEcef.y_m, ecef.y_m);
    EXPECT_DOUBLE_EQ(rEcef.z_m, ecef.z_m);
    EXPECT_DOUBLE_EQ(rFp.positionAccuracy_m, fp.positionAccuracy_m);
}

TEST(RtkYamlRoundTrip, Rover)
{
    const std::string yaml = "mode: rover\n";

    const YAML::Node node = YAML::Load(yaml);
    const auto cfg = RtkBridge::fromYaml(node);

    EXPECT_EQ(cfg.mode, ERtkMode::Rover);
    EXPECT_FALSE(cfg.base.has_value());

    YAML::Emitter out;
    out << YAML::BeginMap;
    RtkBridge::toYaml(out, cfg);
    out << YAML::EndMap;

    const YAML::Node reloaded = YAML::Load(out.c_str());
    const auto restored = RtkBridge::fromYaml(reloaded["rtk"]);
    EXPECT_EQ(restored.mode, ERtkMode::Rover);
}


// --- validation ---

TEST(RtkYamlValidation, MissingModeThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load("base: {type: survey_in}")),
        std::runtime_error);
}

TEST(RtkYamlValidation, UnknownModeThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load("mode: Base")),
        std::runtime_error);
}

TEST(RtkYamlValidation, BaseWithoutBaseSectionThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load("mode: base")),
        std::runtime_error);
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load("mode: base\nbase:\n")),
        std::runtime_error);
}

TEST(RtkYamlValidation, BaseWithoutTypeThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load(
        "mode: base\nbase: {min_observation_time_s: 60}")),
        std::runtime_error);
}

TEST(RtkYamlValidation, UnknownBaseTypeThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load(
        "mode: base\nbase: {type: fixed_utm}")),
        std::runtime_error);
}

TEST(RtkYamlValidation, FixedEcefMissingCoordinateThrows)
{
    try
    {
        RtkBridge::fromYaml(YAML::Load(
            "mode: base\nbase: {type: fixed_ecef, x_m: 1.0, y_m: 2.0}"));
        FAIL() << "expected an error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find("rtk.base.z_m"),
            std::string::npos) << e.what();
    }
}

TEST(RtkYamlValidation, FixedLlaMissingHeightThrows)
{
    EXPECT_THROW(RtkBridge::fromYaml(YAML::Load(
        "mode: base\n"
        "base: {type: fixed_lla, latitude_deg: 52.0, longitude_deg: 21.0}")),
        std::runtime_error);
}

TEST(RtkYamlValidation, SurveyInDefaults)
{
    const auto cfg = RtkBridge::fromYaml(YAML::Load(
        "mode: base\nbase: {type: survey_in}"));
    ASSERT_TRUE(cfg.base.has_value());
    const auto& si = std::get<BaseConfig::SurveyIn>(cfg.base->mode);
    EXPECT_EQ(si.minimumObservationTime_s, 60u);
    EXPECT_DOUBLE_EQ(si.requiredPositionAccuracy_m, 2.0);
}

TEST(RtkYamlValidation, RoverIgnoresBaseSection)
{
    const auto cfg = RtkBridge::fromYaml(YAML::Load(
        "mode: rover\nbase: {type: survey_in}"));
    EXPECT_EQ(cfg.mode, ERtkMode::Rover);
    EXPECT_FALSE(cfg.base.has_value());
}

TEST(RtkMsgValidation, UnknownModeThrows)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.rtk_mode = 2;
    EXPECT_THROW(RtkBridge::fromMsg(msg), std::runtime_error);
}

TEST(RtkMsgValidation, UnknownBaseTypeThrows)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.rtk_mode = static_cast<uint8_t>(ERtkMode::Base);
    msg.rtk_base_type = 3;
    EXPECT_THROW(RtkBridge::fromMsg(msg), std::runtime_error);
}

}  // JimmyPaputto::test
