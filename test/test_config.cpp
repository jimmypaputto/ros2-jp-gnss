/*
 * Jimmy Paputto 2026
 */

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "../src/Config.hpp"
#include "../src/Topics.hpp"


namespace JimmyPaputto::test
{

namespace fs = std::filesystem;

static const std::string kMinimalYaml = R"(
measurement_rate_hz: 5
publish_standard_topics: false
use_ntrip_rtcm: false
dynamic_model: 4
timepulse:
  active: true
  fixed_pulse:
    frequency: 10
    pulse_width: 0.25
  polarity: rising
)";

static const std::string kFullYaml = R"(
measurement_rate_hz: 10
publish_standard_topics: true
use_ntrip_rtcm: true
save_to_flash: true
dynamic_model: 2
timepulse:
  active: true
  fixed_pulse:
    frequency: 10
    pulse_width: 0.1
  pulse_when_no_fix:
    frequency: 1
    pulse_width: 0.5
  polarity: falling
geofencing:
  confidence_level: 4
  pio_pin_polarity: low_means_outside
  geofences:
    - lat: 52.2297
      lon: 21.0122
      radius: 50.0
    - lat: 51.1079
      lon: 17.0385
      radius: 100.0
rtk:
  mode: base
  base:
    type: survey_in
    min_observation_time_s: 120
    required_accuracy_m: 1.5
)";

class TmpFile
{
public:
    explicit TmpFile(const std::string& content)
    :   path_(fs::temp_directory_path() /
              ("jp_gnss_test_" + std::to_string(counter_++) + ".yaml"))
    {
        std::ofstream out(path_);
        out << content;
    }

    ~TmpFile() { std::remove(path_.c_str()); }

    const std::string& path() const { return path_; }

private:
    std::string path_;
    static inline int counter_{0};
};


// --- loadFromYaml ---

TEST(ConfigLoad, MinimalYaml)
{
    TmpFile file(kMinimalYaml);
    Config cfg;
    cfg.loadFromYaml(file.path());

    EXPECT_EQ(cfg.gnssConfig().measurementRate_Hz, 5);
    EXPECT_FALSE(cfg.publishStandardTopics());
    EXPECT_FALSE(cfg.useNtripRtcm());
    EXPECT_FALSE(cfg.gnssConfig().saveToFlash);
    EXPECT_EQ(cfg.gnssConfig().dynamicModel, EDynamicModel::Automotive);
    EXPECT_TRUE(cfg.gnssConfig().timepulsePinConfig.active);
    EXPECT_EQ(cfg.gnssConfig().timepulsePinConfig.fixedPulse.frequency, 10u);
    EXPECT_FLOAT_EQ(cfg.gnssConfig().timepulsePinConfig.fixedPulse.pulseWidth,
                    0.25f);
    EXPECT_FALSE(cfg.gnssConfig().timepulsePinConfig.pulseWhenNoFix.has_value());
    EXPECT_EQ(cfg.gnssConfig().timepulsePinConfig.polarity,
              ETimepulsePinPolarity::RisingEdgeAtTopOfSecond);
    EXPECT_FALSE(cfg.gnssConfig().geofencing.has_value());
    EXPECT_FALSE(cfg.gnssConfig().rtk.has_value());
}

TEST(ConfigLoad, FullYaml)
{
    TmpFile file(kFullYaml);
    Config cfg;
    cfg.loadFromYaml(file.path());

    EXPECT_EQ(cfg.gnssConfig().measurementRate_Hz, 10);
    EXPECT_TRUE(cfg.publishStandardTopics());
    EXPECT_TRUE(cfg.useNtripRtcm());
    EXPECT_TRUE(cfg.gnssConfig().saveToFlash);
    EXPECT_EQ(cfg.gnssConfig().dynamicModel, EDynamicModel::Stationary);

    // Timepulse
    const auto& tp = cfg.gnssConfig().timepulsePinConfig;
    EXPECT_TRUE(tp.active);
    EXPECT_EQ(tp.fixedPulse.frequency, 10u);
    EXPECT_FLOAT_EQ(tp.fixedPulse.pulseWidth, 0.1f);
    ASSERT_TRUE(tp.pulseWhenNoFix.has_value());
    EXPECT_EQ(tp.pulseWhenNoFix->frequency, 1u);
    EXPECT_FLOAT_EQ(tp.pulseWhenNoFix->pulseWidth, 0.5f);
    EXPECT_EQ(tp.polarity,
              ETimepulsePinPolarity::FallingEdgeAtTopOfSecond);

    // Geofencing
    ASSERT_TRUE(cfg.gnssConfig().geofencing.has_value());
    const auto& geo = *cfg.gnssConfig().geofencing;
    EXPECT_EQ(geo.confidenceLevel, 4);
    ASSERT_TRUE(geo.pioPinPolarity.has_value());
    EXPECT_EQ(*geo.pioPinPolarity, EPioPinPolarity::LowMeansOutside);
    ASSERT_EQ(geo.geofences.size(), 2u);
    EXPECT_FLOAT_EQ(geo.geofences[0].lat, 52.2297f);
    EXPECT_FLOAT_EQ(geo.geofences[0].radius, 50.0f);
    EXPECT_FLOAT_EQ(geo.geofences[1].lon, 17.0385f);

    // RTK
    ASSERT_TRUE(cfg.gnssConfig().rtk.has_value());
    EXPECT_EQ(cfg.gnssConfig().rtk->mode, ERtkMode::Base);
    ASSERT_TRUE(cfg.gnssConfig().rtk->base.has_value());
    ASSERT_TRUE(std::holds_alternative<BaseConfig::SurveyIn>(
        cfg.gnssConfig().rtk->base->mode));
    const auto& si =
        std::get<BaseConfig::SurveyIn>(cfg.gnssConfig().rtk->base->mode);
    EXPECT_EQ(si.minimumObservationTime_s, 120u);
    EXPECT_DOUBLE_EQ(si.requiredPositionAccuracy_m, 1.5);
}

TEST(ConfigLoad, InvalidDynamicModelThrows)
{
    const std::string yaml = R"(
measurement_rate_hz: 1
dynamic_model: 99
timepulse:
  active: false
  fixed_pulse:
    frequency: 1
    pulse_width: 0.1
  polarity: rising
)";
    TmpFile file(yaml);
    Config cfg;
    EXPECT_THROW(cfg.loadFromYaml(file.path()), std::runtime_error);
}


// --- validation ---

static void expectLoadThrows(const std::string& yaml, const std::string& key)
{
    TmpFile file(yaml);
    Config cfg;
    try
    {
        cfg.loadFromYaml(file.path());
        ADD_FAILURE() << "expected an error mentioning '" << key << "'";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find(key), std::string::npos)
            << "error does not mention '" << key << "': " << e.what();
    }
}

TEST(ConfigValidation, MissingTimepulseDisablesPin)
{
    TmpFile file("measurement_rate_hz: 5\n");
    Config cfg;
    ASSERT_NO_THROW(cfg.loadFromYaml(file.path()));
    EXPECT_FALSE(cfg.gnssConfig().timepulsePinConfig.active);
}

TEST(ConfigValidation, TimepulseSectionWithoutFixedPulseUsesDefaults)
{
    TmpFile file("timepulse:\n  polarity: falling\n");
    Config cfg;
    ASSERT_NO_THROW(cfg.loadFromYaml(file.path()));
    const auto& tp = cfg.gnssConfig().timepulsePinConfig;
    EXPECT_TRUE(tp.active);
    EXPECT_EQ(tp.fixedPulse.frequency, 1u);
    EXPECT_FLOAT_EQ(tp.fixedPulse.pulseWidth, 0.1f);
    EXPECT_FALSE(tp.pulseWhenNoFix.has_value());
    EXPECT_EQ(tp.polarity, ETimepulsePinPolarity::FallingEdgeAtTopOfSecond);
}

TEST(ConfigValidation, NullTimepulseSectionThrows)
{
    expectLoadThrows("timepulse:\n", "timepulse");
}

TEST(ConfigValidation, NonMapTopLevelThrows)
{
    TmpFile file("- just\n- a list\n");
    Config cfg;
    EXPECT_THROW(cfg.loadFromYaml(file.path()), std::runtime_error);
}

TEST(ConfigValidation, BadScalarNamesKey)
{
    expectLoadThrows("measurement_rate_hz: fast\n", "measurement_rate_hz");
    expectLoadThrows(
        "timepulse:\n  fixed_pulse:\n    frequency: often\n",
        "timepulse.fixed_pulse.frequency");
}

TEST(ConfigValidation, UnknownTimepulsePolarityThrows)
{
    expectLoadThrows("timepulse:\n  polarity: up\n", "timepulse.polarity");
}

TEST(ConfigValidation, UnknownPioPinPolarityThrows)
{
    expectLoadThrows(R"(
geofencing:
  pio_pin_polarity: high_means_inside
  geofences:
    - {lat: 52.0, lon: 21.0, radius: 10.0}
)", "geofencing.pio_pin_polarity");
}

TEST(ConfigValidation, FiveGeofencesThrow)
{
    expectLoadThrows(R"(
geofencing:
  confidence_level: 3
  geofences:
    - {lat: 52.0, lon: 21.0, radius: 10.0}
    - {lat: 52.1, lon: 21.0, radius: 10.0}
    - {lat: 52.2, lon: 21.0, radius: 10.0}
    - {lat: 52.3, lon: 21.0, radius: 10.0}
    - {lat: 52.4, lon: 21.0, radius: 10.0}
)", "geofencing.geofences");
}

TEST(ConfigValidation, FourGeofencesLoad)
{
    TmpFile file(R"(
geofencing:
  geofences:
    - {lat: 52.0, lon: 21.0, radius: 10.0}
    - {lat: 52.1, lon: 21.0, radius: 10.0}
    - {lat: 52.2, lon: 21.0, radius: 10.0}
    - {lat: 52.3, lon: 21.0, radius: 10.0}
)");
    Config cfg;
    ASSERT_NO_THROW(cfg.loadFromYaml(file.path()));
    ASSERT_TRUE(cfg.gnssConfig().geofencing.has_value());
    EXPECT_EQ(cfg.gnssConfig().geofencing->geofences.size(), 4u);
}

TEST(ConfigValidation, GeofenceMissingRadiusThrows)
{
    expectLoadThrows(R"(
geofencing:
  geofences:
    - {lat: 52.0, lon: 21.0}
)", "geofencing.geofences[0].radius");
}

TEST(ConfigValidation, RtkBaseWithoutBaseSectionThrows)
{
    expectLoadThrows("rtk:\n  mode: base\n", "rtk.base");
}

TEST(ConfigValidation, RtkUnknownBaseTypeThrows)
{
    expectLoadThrows(
        "rtk:\n  mode: base\n  base:\n    type: fixed_utm\n",
        "rtk.base.type");
}

TEST(ConfigValidation, RtkUnknownModeThrows)
{
    expectLoadThrows("rtk:\n  mode: moving_base\n", "rtk.mode");
}

TEST(ConfigValidation, FailedLoadKeepsPreviousConfig)
{
    TmpFile good("node_id: rover\n");
    Config cfg;
    cfg.loadFromYaml(good.path());

    TmpFile bad("node_id: other\nrtk:\n  mode: base\n");
    EXPECT_THROW(cfg.loadFromYaml(bad.path()), std::runtime_error);
    EXPECT_EQ(cfg.nodeId(), "rover");
    EXPECT_FALSE(cfg.gnssConfig().rtk.has_value());
}

TEST(ConfigValidation, MsgWithFiveGeofencesThrows)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.geofencing_enabled = true;
    msg.geofences.resize(5);
    Config cfg;
    EXPECT_THROW(cfg.fromMsg(msg), std::runtime_error);
}

TEST(ConfigValidation, MsgWithInvalidPolarityThrows)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.timepulse_polarity = 2;
    Config cfg;
    EXPECT_THROW(cfg.fromMsg(msg), std::runtime_error);

    msg.timepulse_polarity = 1;
    msg.geofencing_enabled = true;
    msg.geofences.resize(1);
    msg.geofencing_has_pio_pin_polarity = true;
    msg.geofencing_pio_pin_polarity = 7;
    EXPECT_THROW(cfg.fromMsg(msg), std::runtime_error);
}

TEST(ConfigValidation, MsgWithInvalidRtkThrows)
{
    jp_gnss_hat::msg::GnssConfig msg{};
    msg.timepulse_polarity = 1;
    msg.rtk_enabled = true;
    msg.rtk_mode = static_cast<uint8_t>(ERtkMode::Base);
    msg.rtk_base_type = 3;
    Config cfg;
    EXPECT_THROW(cfg.fromMsg(msg), std::runtime_error);
}


// --- shipped configs ---

TEST(ConfigShipped, NoTimepulseExampleDisablesPin)
{
    const fs::path path = fs::path(JP_GNSS_HAT_SOURCE_DIR) /
        "config" / "examples" / "config_no_timepulse.yaml";
    Config cfg;
    ASSERT_NO_THROW(cfg.loadFromYaml(path.string()));
    EXPECT_FALSE(cfg.gnssConfig().timepulsePinConfig.active);
}

TEST(ConfigShipped, AllConfigsLoad)
{
    const fs::path root = fs::path(JP_GNSS_HAT_SOURCE_DIR) / "config";
    size_t loaded = 0;
    for (const auto& dir : {root, root / "examples"})
    {
        for (const auto& entry : fs::directory_iterator(dir))
        {
            if (entry.path().extension() != ".yaml")
                continue;
            SCOPED_TRACE(entry.path().string());
            Config cfg;
            EXPECT_NO_THROW(cfg.loadFromYaml(entry.path().string()));
            ++loaded;
        }
    }
    EXPECT_GE(loaded, 6u);
}


// --- saveToYaml / loadFromYaml round-trip ---

TEST(ConfigRoundTrip, SaveLoadMinimal)
{
    TmpFile src(kMinimalYaml);
    Config original;
    original.loadFromYaml(src.path());

    TmpFile dst("");
    original.saveToYaml(dst.path());

    Config reloaded;
    reloaded.loadFromYaml(dst.path());

    EXPECT_EQ(reloaded.gnssConfig().measurementRate_Hz,
              original.gnssConfig().measurementRate_Hz);
    EXPECT_EQ(reloaded.publishStandardTopics(),
              original.publishStandardTopics());
    EXPECT_EQ(reloaded.useNtripRtcm(), original.useNtripRtcm());
    EXPECT_EQ(reloaded.gnssConfig().dynamicModel,
              original.gnssConfig().dynamicModel);
    EXPECT_EQ(reloaded.gnssConfig().saveToFlash,
              original.gnssConfig().saveToFlash);
    EXPECT_EQ(reloaded.gnssConfig().timepulsePinConfig.fixedPulse.frequency,
              original.gnssConfig().timepulsePinConfig.fixedPulse.frequency);
}

TEST(ConfigRoundTrip, SaveLoadFull)
{
    TmpFile src(kFullYaml);
    Config original;
    original.loadFromYaml(src.path());

    TmpFile dst("");
    original.saveToYaml(dst.path());

    Config reloaded;
    reloaded.loadFromYaml(dst.path());

    const auto& o = original.gnssConfig();
    const auto& r = reloaded.gnssConfig();

    EXPECT_EQ(r.measurementRate_Hz, o.measurementRate_Hz);
    EXPECT_EQ(r.dynamicModel, o.dynamicModel);
    EXPECT_EQ(reloaded.publishStandardTopics(),
              original.publishStandardTopics());
    EXPECT_EQ(reloaded.useNtripRtcm(), original.useNtripRtcm());
    EXPECT_EQ(r.saveToFlash, o.saveToFlash);

    // Timepulse
    EXPECT_EQ(r.timepulsePinConfig.active, o.timepulsePinConfig.active);
    EXPECT_EQ(r.timepulsePinConfig.fixedPulse.frequency,
              o.timepulsePinConfig.fixedPulse.frequency);
    EXPECT_FLOAT_EQ(r.timepulsePinConfig.fixedPulse.pulseWidth,
                    o.timepulsePinConfig.fixedPulse.pulseWidth);
    ASSERT_EQ(r.timepulsePinConfig.pulseWhenNoFix.has_value(),
              o.timepulsePinConfig.pulseWhenNoFix.has_value());
    if (o.timepulsePinConfig.pulseWhenNoFix)
    {
        EXPECT_EQ(r.timepulsePinConfig.pulseWhenNoFix->frequency,
                  o.timepulsePinConfig.pulseWhenNoFix->frequency);
        EXPECT_FLOAT_EQ(r.timepulsePinConfig.pulseWhenNoFix->pulseWidth,
                        o.timepulsePinConfig.pulseWhenNoFix->pulseWidth);
    }
    EXPECT_EQ(r.timepulsePinConfig.polarity, o.timepulsePinConfig.polarity);

    // Geofencing
    ASSERT_EQ(r.geofencing.has_value(), o.geofencing.has_value());
    EXPECT_EQ(r.geofencing->confidenceLevel, o.geofencing->confidenceLevel);
    ASSERT_EQ(r.geofencing->pioPinPolarity.has_value(),
              o.geofencing->pioPinPolarity.has_value());
    if (o.geofencing->pioPinPolarity)
    {
        EXPECT_EQ(*r.geofencing->pioPinPolarity,
                  *o.geofencing->pioPinPolarity);
    }
    ASSERT_EQ(r.geofencing->geofences.size(), o.geofencing->geofences.size());
    for (size_t i = 0; i < o.geofencing->geofences.size(); ++i)
    {
        EXPECT_FLOAT_EQ(r.geofencing->geofences[i].lat,
                        o.geofencing->geofences[i].lat);
        EXPECT_FLOAT_EQ(r.geofencing->geofences[i].lon,
                        o.geofencing->geofences[i].lon);
        EXPECT_FLOAT_EQ(r.geofencing->geofences[i].radius,
                        o.geofencing->geofences[i].radius);
    }

    // RTK
    ASSERT_EQ(r.rtk.has_value(), o.rtk.has_value());
    EXPECT_EQ(r.rtk->mode, o.rtk->mode);
    ASSERT_EQ(r.rtk->base.has_value(), o.rtk->base.has_value());
    ASSERT_TRUE(std::holds_alternative<BaseConfig::SurveyIn>(
        r.rtk->base->mode));
    const auto& rSi =
        std::get<BaseConfig::SurveyIn>(r.rtk->base->mode);
    const auto& oSi =
        std::get<BaseConfig::SurveyIn>(o.rtk->base->mode);
    EXPECT_EQ(rSi.minimumObservationTime_s, oSi.minimumObservationTime_s);
    EXPECT_DOUBLE_EQ(rSi.requiredPositionAccuracy_m,
                     oSi.requiredPositionAccuracy_m);
}


// --- toMsg / fromMsg round-trip ---

TEST(ConfigMsgRoundTrip, FullConfig)
{
    TmpFile src(kFullYaml);
    Config original;
    original.loadFromYaml(src.path());

    auto msg = original.toMsg();

    Config restored;
    restored.fromMsg(msg);

    const auto& o = original.gnssConfig();
    const auto& r = restored.gnssConfig();

    EXPECT_EQ(r.measurementRate_Hz, o.measurementRate_Hz);
    EXPECT_EQ(r.dynamicModel, o.dynamicModel);
    EXPECT_EQ(restored.publishStandardTopics(),
              original.publishStandardTopics());
    EXPECT_EQ(restored.useNtripRtcm(), original.useNtripRtcm());
    EXPECT_EQ(r.saveToFlash, o.saveToFlash);

    EXPECT_EQ(r.timepulsePinConfig.active, o.timepulsePinConfig.active);
    EXPECT_EQ(r.timepulsePinConfig.fixedPulse.frequency,
              o.timepulsePinConfig.fixedPulse.frequency);
    EXPECT_EQ(r.timepulsePinConfig.polarity, o.timepulsePinConfig.polarity);

    ASSERT_EQ(r.timepulsePinConfig.pulseWhenNoFix.has_value(),
              o.timepulsePinConfig.pulseWhenNoFix.has_value());

    ASSERT_EQ(r.geofencing.has_value(), o.geofencing.has_value());
    EXPECT_EQ(r.geofencing->confidenceLevel, o.geofencing->confidenceLevel);
    ASSERT_EQ(r.geofencing->geofences.size(), o.geofencing->geofences.size());

    ASSERT_EQ(r.rtk.has_value(), o.rtk.has_value());
    EXPECT_EQ(r.rtk->mode, o.rtk->mode);
}

TEST(ConfigMsgRoundTrip, MinimalConfig)
{
    TmpFile src(kMinimalYaml);
    Config original;
    original.loadFromYaml(src.path());

    auto msg = original.toMsg();

    Config restored;
    restored.fromMsg(msg);

    EXPECT_EQ(restored.gnssConfig().measurementRate_Hz,
              original.gnssConfig().measurementRate_Hz);
    EXPECT_FALSE(restored.gnssConfig().geofencing.has_value());
    EXPECT_FALSE(restored.gnssConfig().rtk.has_value());
}


// --- getDynamicModelName ---

TEST(ConfigDynamicModel, KnownModels)
{
    EXPECT_EQ(Config::getDynamicModelName(EDynamicModel::Portable),
              "Portable");
    EXPECT_EQ(Config::getDynamicModelName(EDynamicModel::Stationary),
              "Stationary");
    EXPECT_EQ(Config::getDynamicModelName(EDynamicModel::Automotive),
              "Automotive");
    EXPECT_EQ(Config::getDynamicModelName(EDynamicModel::Escooter),
              "Escooter");
}

TEST(ConfigDynamicModel, UnknownModel)
{
    EXPECT_EQ(Config::getDynamicModelName(static_cast<EDynamicModel>(255)),
              "Unknown");
}


// --- node_id ---

static const std::string kYamlWithNodeId = R"(
node_id: "rover"
measurement_rate_hz: 5
publish_standard_topics: false
use_ntrip_rtcm: false
dynamic_model: 4
timepulse:
  active: true
  fixed_pulse:
    frequency: 10
    pulse_width: 0.25
  polarity: rising
)";

TEST(ConfigNodeId, LoadsNodeIdFromYaml)
{
    TmpFile file(kYamlWithNodeId);
    Config cfg;
    cfg.loadFromYaml(file.path());
    EXPECT_EQ(cfg.nodeId(), "rover");
}

TEST(ConfigNodeId, MissingNodeIdDefaultsToEmpty)
{
    TmpFile file(kMinimalYaml);
    Config cfg;
    cfg.loadFromYaml(file.path());
    EXPECT_TRUE(cfg.nodeId().empty());
}

TEST(ConfigNodeId, ReadNodeIdStaticHelper)
{
    TmpFile file(kYamlWithNodeId);
    EXPECT_EQ(Config::readNodeId(file.path()), "rover");
}

TEST(ConfigNodeId, ReadNodeIdMissingField)
{
    TmpFile file(kMinimalYaml);
    EXPECT_TRUE(Config::readNodeId(file.path()).empty());
}

TEST(ConfigNodeId, ReadNodeIdBadPath)
{
    EXPECT_TRUE(Config::readNodeId("/nonexistent/path.yaml").empty());
}

TEST(ConfigNodeId, SaveLoadRoundTrip)
{
    TmpFile src(kYamlWithNodeId);
    Config original;
    original.loadFromYaml(src.path());

    TmpFile dst("");
    original.saveToYaml(dst.path());

    Config reloaded;
    reloaded.loadFromYaml(dst.path());
    EXPECT_EQ(reloaded.nodeId(), "rover");
}

TEST(ConfigNodeId, EmptyNodeIdNotSaved)
{
    TmpFile src(kMinimalYaml);
    Config cfg;
    cfg.loadFromYaml(src.path());

    TmpFile dst("");
    cfg.saveToYaml(dst.path());

    // Verify node_id key not present in the output
    std::ifstream in(dst.path());
    std::string content((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
    EXPECT_EQ(content.find("node_id"), std::string::npos);
}


// --- Topics struct ---

TEST(Topics, DefaultEmpty)
{
    Topics t;
    EXPECT_EQ(t.nodeName,      "jp_gnss_hat");
    EXPECT_EQ(t.navigation,    "/gnss/navigation");
    EXPECT_EQ(t.navSatFix,     "/gps/fix");
    EXPECT_EQ(t.velocity,      "/gps/vel");
    EXPECT_EQ(t.timeReference, "/gps/time_reference");
    EXPECT_EQ(t.geofencingCfg, "/gnss/geofencing_cfg");
    EXPECT_EQ(t.getConfig,     "/gnss/get_config");
    EXPECT_EQ(t.setConfig,     "/gnss/set_config");
    EXPECT_EQ(t.rtkCorrections, "/gnss/rtk/corrections");
    EXPECT_EQ(t.ntrip,          "/rtcm");
}

TEST(Topics, WithNodeId)
{
    Topics t("rover");
    EXPECT_EQ(t.nodeName,      "jp_gnss_hat_rover");
    EXPECT_EQ(t.navigation,    "/gnss/rover/navigation");
    EXPECT_EQ(t.geofencingCfg, "/gnss/rover/geofencing_cfg");
    EXPECT_EQ(t.getConfig,     "/gnss/rover/get_config");
    EXPECT_EQ(t.setConfig,     "/gnss/rover/set_config");
    // Standard ROS topics unchanged regardless of node_id
    EXPECT_EQ(t.navSatFix,     "/gps/fix");
    EXPECT_EQ(t.velocity,      "/gps/vel");
    EXPECT_EQ(t.timeReference, "/gps/time_reference");
    // Shared RTK topics unchanged
    EXPECT_EQ(t.rtkCorrections, "/gnss/rtk/corrections");
    EXPECT_EQ(t.ntrip,          "/rtcm");
}

TEST(Topics, BaseNodeId)
{
    Topics t("base");
    EXPECT_EQ(t.nodeName,   "jp_gnss_hat_base");
    EXPECT_EQ(t.navigation, "/gnss/base/navigation");
    EXPECT_EQ(t.navSatFix,  "/gps/fix");
}

}  // JimmyPaputto::test
