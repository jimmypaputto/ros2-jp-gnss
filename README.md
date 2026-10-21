# ros2-jp-gnss

ROS 2 package `jp_gnss_hat` — a Lifecycle Node wrapping the [jimmypaputto/GnssHat](https://github.com/jimmypaputto/GnssHat) driver library. Publishes full navigation data from Jimmy Paputto GNSS HATs on Raspberry Pi. Buy the HATs at [jimmypaputto.com](https://jimmypaputto.com).

## Supported Hardware

The HAT variant is auto-detected at startup via `/proc/device-tree/hat/product`.

| HAT | u-blox Module | RTK | Geofencing |
|-----|---------------|-----|------------|
| L1 GNSS HAT | NEO-M9N | -- | Up to 4 zones |
| L1/L5 GNSS TIME HAT | NEO-F10T | -- | -- |
| L1/L5 GNSS RTK HAT | NEO-F9P | Base & Rover | Up to 4 zones |

## Installation

### Prerequisites — ROS 2 from Source

Raspberry Pi OS does not ship ROS 2 packages via `apt`. Build ROS 2 Jazzy from source following the official guide:
[Building ROS 2 on Linux](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)

Once built, source the workspace before building this package:

```sh
source ~/ros2_jazzy/install/setup.bash
```

### Dependencies

```sh
sudo apt-get install libgpiod-dev
```

Optional - NTRIP bridge (`rtcm_msgs/Message` on `/rtcm`). Since there are no apt packages for ROS 2 on Raspberry Pi, build `rtcm_msgs` package from source, for example: https://github.com/tilk/rtcm_msgs.git.

### Build

```sh
source ~/ros2_jazzy/install/setup.bash
mkdir -p ~/ros2-ws/src
cd ~/ros2-ws/src
git clone https://github.com/jimmypaputto/ros2-jp-gnss.git
cd ~/ros2-ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select jp_gnss_hat
source install/setup.bash
```

The [GnssHat](https://github.com/jimmypaputto/GnssHat) library is fetched automatically via CMake `FetchContent` if not found system-wide.

## Quick Start

```sh
ros2 launch jp_gnss_hat gnss.launch.py
```

`Main.cpp` auto-transitions the lifecycle node through `configure` → `activate`. Navigation data is published immediately.

```sh
ros2 topic echo /gnss/navigation          # custom — full data
ros2 topic echo /gps/fix                  # sensor_msgs/NavSatFix
```

Custom config file:

```sh
ros2 launch jp_gnss_hat gnss.launch.py config_file:=/path/to/config.yaml
```

## Configuration

Single YAML file. Default installed at `<pkg_share>/config/gnss_config.yaml`. See `config/examples/` for RTK base, RTK rover, geofencing configs.

```yaml
node_id: ""                    # multi-instance: changes node name and topic prefix
measurement_rate_hz: 1         # 1–25 Hz
save_to_flash: false           # persist config to u-blox receiver flash
publish_standard_topics: true  # publish /gps/fix, /gps/vel, /gps/time_reference
use_ntrip_rtcm: false          # bridge to/from rtcm_msgs (requires rtcm_msgs)
dynamic_model: 2               # see table below

timepulse:
  active: true
  fixed_pulse:
    frequency: 1               # Hz
    pulse_width: 0.1           # duty cycle 0.0–1.0
  # pulse_when_no_fix:         # optional — omit to disable pulse before fix
  #   frequency: 1
  #   pulse_width: 0.5
  polarity: rising             # "rising" or "falling"

geofencing:                    # optional — omit entire section to disable
  confidence_level: 3          # 0=none 1=68% 2=95% 3=99.7% 4=99.99% 5=99.9999%
  pio_pin_polarity: low_means_inside   # optional — "low_means_inside" or "low_means_outside"
  geofences:                   # max 4 
    - lat: 41.902205071091224
      lon: 12.4539203390548
      radius: 2005.0
    - lat: 52.257211745024186
      lon: 20.311759615806704
      radius: 1810.0

rtk:                           # optional — omit to disable
  mode: base                   # "base" or "rover"
  base:
    type: survey_in            # "survey_in", "fixed_ecef", or "fixed_lla"
    min_observation_time_s: 120
    required_accuracy_m: 50.0
    # fixed_ecef: x_m, y_m, z_m, position_accuracy_m
    # fixed_lla: latitude_deg, longitude_deg, height_m, position_accuracy_m
```

### Dynamic Model

| Value | Model | Value | Model |
|-------|-------|-------|-------|
| 0 | Portable | 7 | Airborne 2G |
| 2 | Stationary | 8 | Airborne 4G |
| 3 | Pedestrian | 9 | Wrist |
| 4 | Automotive | 10 | Bike |
| 5 | Sea | 11 | Mower |
| 6 | Airborne 1G | 12 | E-scooter |

## Topics

All publishers use QoS depth 10. All messages carry `std_msgs/Header` with timestamp from the ROS clock.

### Published

| Topic | Type | Condition |
|-------|------|-----------|
| `/gnss/navigation` | `jp_gnss_hat/Navigation` | Always |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `publish_standard_topics: true` |
| `/gps/vel` | `geometry_msgs/TwistWithCovarianceStamped` | `publish_standard_topics: true` |
| `/gps/time_reference` | `sensor_msgs/TimeReference` | `publish_standard_topics: true` AND valid UTC time and date |
| `/gnss/rtk/corrections` | `jp_gnss_hat/RtkCorrections` | RTK base |
| `/rtcm` | `rtcm_msgs/Message` | `use_ntrip_rtcm: true` + RTK base |

### Subscribed

| Topic | Type | Condition |
|-------|------|-----------|
| `/gnss/rtk/corrections` | `jp_gnss_hat/RtkCorrections` | RTK rover |
| `/rtcm` | `rtcm_msgs/Message` | `use_ntrip_rtcm: true` + RTK rover |

### Multi-Instance

Set `node_id` in YAML (e.g. `"rover"`). Custom topics and services get the ID inserted: `/gnss/rover/navigation`, `/gnss/rover/get_config`, etc. Standard topics (`/gps/*`) and RTK topics (`/gnss/rtk/corrections`, `/rtcm`) stay global — intentionally shared between instances.

| `node_id` | Node name | Navigation topic | Services prefix |
|-----------|-----------|-----------------|-----------------|
| `""` (default) | `jp_gnss_hat` | `/gnss/navigation` | `/gnss/` |
| `"rover"` | `jp_gnss_hat_rover` | `/gnss/rover/navigation` | `/gnss/rover/` |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/gnss/get_config` | `GetGnssConfig` | Returns current `GnssConfig` |
| `/gnss/set_config` | `SetGnssConfig` | Applies new config at runtime; `save_to_yaml: true` persists to YAML file |
| `/gnss/geofencing_cfg` | `GetGeofencingCfg` | Returns active geofencing configuration |

### Runtime Reconfiguration

`SetGnssConfig` triggers a full lifecycle cycle: deactivate → cleanup → configure (new config) → activate. On failure the node automatically rolls back to the previous working config.

```sh
ros2 service call /gnss/get_config jp_gnss_hat/srv/GetGnssConfig

ros2 service call /gnss/set_config jp_gnss_hat/srv/SetGnssConfig \
  "{config: {measurement_rate_hz: 5, dynamic_model: 4, publish_standard_topics: true, \
  timepulse_active: true, timepulse_frequency: 10, timepulse_pulse_width: 0.1, \
  timepulse_polarity: 0}, save_to_yaml: true}"
```

## Messages

### jp_gnss_hat/Navigation

```
std_msgs/Header header
PositionVelocityTime pvt
DilutionOverPrecision dop
GeofencingNav geofencing
RfBlock[] rf_blocks
SatelliteInfo[] satellites
```

Single message per GNSS update. Contains PVT (UBX-NAV-PVT), DOP (UBX-NAV-DOP), per-satellite data (UBX-NAV-SAT), RF band diagnostics (UBX-MON-RF), and geofencing state (UBX-NAV-GEOFENCE).

### jp_gnss_hat/PositionVelocityTime

| Field | Type | Unit |
|-------|------|------|
| `fix_quality` | `uint8` | 0=Invalid, 1=GPS, 2=DGNSS, 3=PPS, 4=FixedRTK, 5=FloatRTK, 6=DR |
| `fix_status` | `uint8` | 0=Void, 1=Active |
| `fix_type` | `uint8` | 0=NoFix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=TimeOnly |
| `latitude` / `longitude` | `float64` | degrees |
| `altitude` / `altitude_msl` | `float32` | meters (ellipsoid / MSL) |
| `speed_over_ground` / `speed_accuracy` | `float32` | m/s |
| `heading` / `heading_accuracy` | `float32` | degrees |
| `horizontal_accuracy` / `vertical_accuracy` | `float32` | m |
| `visible_satellites` | `uint8` | count |
| `utc_hours`, `utc_minutes`, `utc_seconds` | `uint8` | |
| `utc_valid` | `bool` | |
| `utc_accuracy` | `int32` | ns |
| `date_day`, `date_month` | `uint8` | |
| `date_year` | `uint16` | |
| `date_valid` | `bool` | |

### jp_gnss_hat/DilutionOverPrecision

All `float32`: `geometric`, `position`, `time`, `vertical`, `horizontal`, `northing`, `easting`.

### jp_gnss_hat/SatelliteInfo

| Field | Type | Description |
|-------|------|-------------|
| `gnss_id` | `uint8` | 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 4=IMES, 5=QZSS, 6=GLONASS |
| `sv_id` | `uint8` | Satellite vehicle ID |
| `cno` | `uint8` | dBHz |
| `elevation` | `int8` | degrees |
| `azimuth` | `int16` | degrees |
| `quality` | `uint8` | 0–7 |
| `used_in_fix` | `bool` | |
| `healthy`, `diff_corr`, `eph_avail`, `alm_avail` | `bool` | |

### jp_gnss_hat/RfBlock

| Field | Type | Description |
|-------|------|-------------|
| `band` | `uint8` | 0=L1, 1=L2/L5 |
| `jamming_state` | `uint8` | 0=Unknown, 1=OK, 2=Warning, 3=Critical |
| `antenna_status` | `uint8` | 0=Init, 1=DontKnow, 2=Ok, 3=Short, 4=Open |
| `antenna_power` | `uint8` | 0=Off, 1=On, 2=DontKnow |
| `post_status` | `uint32` | |
| `noise_per_ms` | `uint16` | |
| `agc_monitor` | `float32` | |
| `cw_interference_suppression_level` | `float32` | |
| `ofs_i`, `ofs_q` | `int8` | I/Q offset |
| `mag_i`, `mag_q` | `uint8` | I/Q magnitude |

### jp_gnss_hat/GeofencingNav

| Field | Type | Description |
|-------|------|-------------|
| `itow` | `uint32` | GPS time of week (ms) |
| `geofencing_status` | `uint8` | 0=NotAvailable, 1=Active |
| `number_of_geofences` | `uint8` | |
| `combined_state` | `uint8` | 0=Unknown, 1=Inside, 2=Outside |
| `geofences_status` | `uint8[4]` | per-fence: 0=Unknown, 1=Inside, 2=Outside |

### jp_gnss_hat/RtkCorrections

```
std_msgs/Header header
Rtcm3Frame[] frames
```

### jp_gnss_hat/Rtcm3Frame

```
uint8[] data
```

### jp_gnss_hat/GnssConfig

Flat message used by `GetGnssConfig` / `SetGnssConfig` services. Contains all config fields including timepulse, geofencing (with `Geofence[]` array), and RTK parameters. See `msg/GnssConfig.msg` for the full definition.

## Standard Topic Mapping

When `publish_standard_topics: true`:

| Source | Target | Notes |
|--------|--------|-------|
| PVT lat/lon/alt | `NavSatFix.latitude/longitude/altitude` | Altitude = height above the WGS84 ellipsoid (not MSL) |
| PVT horizontal/vertical accuracy (m) | `NavSatFix.position_covariance` | Diagonal, σ² = acc_m² |
| PVT fix_quality | `NavSatFix.status` | Invalid→NO_FIX, GPS/PPS/DR→FIX, DGNSS→SBAS_FIX, RTK→GBAS_FIX |
| Satellites with `used_in_fix` | `NavSatFix.status.service` | GPS/Galileo/GLONASS/BeiDou flags |
| PVT speed + heading | `TwistWithCovarianceStamped` | Decomposed to ENU: vx=speed×sin(heading), vy=speed×cos(heading) |
| PVT UTC time/date | `TimeReference` | Published only when both `utc_valid` and `date_valid` are true |

## Lifecycle

| Transition | Action |
|------------|--------|
| `configure` | `IGnssHat::create()`, soft reset, load YAML (or pending config), `hat->start()`, create publishers, init RTK bridge |
| `activate` | Start `std::jthread` running `navigationLoop()` — blocks on `hat->waitAndGetFreshNavigation()` |
| `deactivate` | `request_stop()` + `join()` on navigation thread |
| `cleanup` | Reset all publishers, converter, RTK bridge, HAT pointer |
| `shutdown` | Defensive cleanup from any state |

Auto-transition in `Main.cpp` uses `MultiThreadedExecutor`. On `SIGINT` the executor exits, then the node is cleanly deactivated and cleaned up.

Manual control:

```sh
ros2 lifecycle set /jp_gnss_hat configure
ros2 lifecycle set /jp_gnss_hat activate
ros2 lifecycle set /jp_gnss_hat deactivate
ros2 lifecycle set /jp_gnss_hat cleanup
```

## RTK

NEO-F9P only. Corrections flow over `/gnss/rtk/corrections` (`RtkCorrections` message — vector of raw RTCM3 frames). DDS handles discovery and transport between base and rover nodes.

**Base** — calls `hat->rtk()->base()->getFullCorrections()` every navigation cycle, publishes the frames. Three position modes:

| Mode | Fields |
|------|--------|
| `survey_in` | `min_observation_time_s`, `required_accuracy_m` |
| `fixed_ecef` | `x_m`, `y_m`, `z_m`, `position_accuracy_m` |
| `fixed_lla` | `latitude_deg`, `longitude_deg`, `height_m`, `position_accuracy_m` |

**Rover** — subscribes to corrections topic, calls `hat->rtk()->rover()->applyCorrections()`.

**NTRIP bridge** — compiled conditionally (`-DJP_GNSS_NTRIP`, auto-detected via `find_package(rtcm_msgs QUIET)`). When enabled and `use_ntrip_rtcm: true`, the base additionally publishes each RTCM3 frame as `rtcm_msgs/Message` on `/rtcm`, and the rover additionally subscribes to `/rtcm`. Both native and NTRIP corrections can work simultaneously.

## Build Details

- C++20, CMake ≥ 3.11
- System dependency: `libgpiod-dev`
- GnssHat: `find_library(GnssHat)` → fallback to `FetchContent` from GitHub
- `rtcm_msgs`: `find_package(QUIET)` → if found, adds `-DJP_GNSS_NTRIP` and links

```sh
# Build
source ~/ros2_jazzy/install/setup.bash
colcon build --packages-select jp_gnss_hat

# Test
colcon test --packages-select jp_gnss_hat && colcon test-result --verbose
```

## License

MIT — see [LICENSE](LICENSE).
