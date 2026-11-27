# ros2-jp-gnss

ROS 2 package `jp_gnss_hat` — a Lifecycle Node wrapping the [jimmypaputto/gnsshat](https://github.com/jimmypaputto/gnsshat) driver library. Publishes full navigation data from Jimmy Paputto GNSS HATs on Raspberry Pi. Buy the HATs at [jimmypaputto.com](https://jimmypaputto.com).

## Supported Hardware

The HAT variant is auto-detected at startup via `/proc/device-tree/hat/product`.

| HAT | u-blox Module | RTK | Geofencing |
|-----|---------------|-----|------------|
| L1 GNSS HAT | NEO-M9N | -- | Up to 4 zones |
| L1/L5 GNSS TIME HAT | NEO-F10T | -- | -- |
| L1/L5 GNSS RTK HAT | NEO-F9P | Base & Rover | Up to 4 zones |

## Installation

Raspberry Pi OS does not ship ROS 2 packages via `apt`. The quickest way to run the node there is Docker: the image contains ROS 2 Jazzy, gnsshat, this package and the gnsshat web visualization. To build everything natively instead, see [Build from Source](#build-from-source).

### Docker (recommended on Raspberry Pi OS)

#### 1. Prepare the host

Docker cannot configure the kernel, so these steps stay on the Pi:

- Install Docker: `curl -fsSL https://get.docker.com | sh`
- Enable the interfaces your HAT uses in `sudo raspi-config` → *Interface Options*: **SPI** (L1 GNSS HAT, RTK HAT) and **Serial Port** — login shell *No*, hardware *Yes* (RTK HAT, TIME HAT)
- RTK and TIME HAT on Pi 3 / 4 / Zero 2 W: free the UART from Bluetooth with `dtoverlay=disable-bt` in `/boot/firmware/config.txt`
- Do not load the kernel PPS overlay on GPIO 5 (`dtoverlay=pps-gpio,gpiopin=5`) — the timepulse needs that pin
- Stop gpsd if it is installed, it holds the receiver: `sudo systemctl disable --now gpsd gpsd.socket`
- Reboot

#### 2. Build and start

```sh
git clone https://github.com/jimmypaputto/ros2-jp-gnss.git
cd ros2-jp-gnss/docker
mkdir config
cp ../config/gnss_config.yaml config/          # or a file from ../config/examples/
sudo docker compose up -d --build
```

The first build takes a while (on a Pi with 2 GB RAM, up to half an hour, and it needs swap). The same image runs on Pi 4 and Pi 5 with a current Raspberry Pi OS kernel — keep the system updated (`sudo apt full-upgrade`).

This starts two containers from the same image, both again after a reboot (`restart: unless-stopped`):

- `gnss` — the ROS 2 node
- `visualization` — the [gnsshat web visualization](https://github.com/jimmypaputto/gnsshat/tree/master/examples/visualization) in ROS 2 mode at **http://\<pi-address\>:5000**, fed by the node's topics. It reads `node_id` from the same config, so it follows `/gnss/<node_id>/navigation` on its own. The port is fixed at 5000.

To run the node without the visualization: `sudo docker compose up -d gnss`.

#### Everyday commands

Run them in `ros2-jp-gnss/docker`:

| Task | Command |
|------|---------|
| Follow the logs (both / node only) | `sudo docker compose logs -f` / `sudo docker compose logs -f gnss` |
| Restart after editing `config/gnss_config.yaml` | `sudo docker compose restart` |
| Stop / start | `sudo docker compose down` / `sudo docker compose up -d` |
| ROS 2 shell inside the image | `sudo docker compose exec gnss /entrypoint.sh bash` |
| Update | `git pull && sudo docker compose up -d --build` |

Inside the shell all `ros2` commands work: `ros2 topic list`, `ros2 topic echo /gps/fix`, `ros2 service call ...`.

#### Config file

`docker/config/` is mounted into the container as `/config`, so the node reads `config/gnss_config.yaml` from the host and `set_config` with `save_to_yaml` writes back to it. The whole [Configuration](#configuration) section applies; just edit the file and restart.

#### Network and other machines

The container uses the host network, so the node is visible to ROS 2 on the LAN like a native process. Requirements on the other machine:

- the same `ROS_DOMAIN_ID` — the container uses `0` unless started with `sudo ROS_DOMAIN_ID=<id> docker compose up -d`
- the standard topics (`/gps/fix`, `/gps/vel`, `/gps/time_reference`) work with any ROS 2 installation
- the custom topics and services (`/gnss/navigation`, `get_config`, ...) need the `jp_gnss_hat` message definitions, i.e. this package built on that machine

The image runs Fast DDS over UDP only. Shared-memory transport between a container and the host fails silently (topics are listed but no data arrives), and at GNSS data rates UDP costs nothing.

RTK base and rover on two Pis: run the container on both with the same `ROS_DOMAIN_ID`; corrections travel over `/gnss/rtk/corrections` between them.

#### Hardware access

`compose.yaml` runs the container `privileged` with the host `/dev` mounted. Which devices a HAT uses depends on the variant and the Pi model, and Docker refuses to start a container when a listed device is missing, so a single explicit list would not fit every setup. If you prefer not to use `privileged`, replace it with the devices of your HAT and allow HAT detection through `/proc/device-tree`:

```yaml
    # instead of privileged: true and - /dev:/dev
    devices:
      - /dev/spidev0.0     # L1 GNSS HAT, RTK HAT
      - /dev/ttyAMA0       # RTK HAT, TIME HAT
      - /dev/gpiochip0
    security_opt:
      - systempaths=unconfined
```

Do not list `/dev/gpiochip4` (on Pi 5 it is a symlink to `gpiochip0`; the node uses `gpiochip0`).

#### Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `Failed to open gpiochip` | `/dev/gpiochip0` not reachable — check `privileged` and the `/dev` mount; on Pi 5 `ls -l /dev/gpiochip4` on the host must show a symlink to `gpiochip0`, otherwise update the kernel (`sudo apt full-upgrade`) |
| Node exits right after start, HAT not recognized | HAT EEPROM not read by the firmware — `cat /proc/device-tree/hat/product` on the host must print the HAT name |
| `Failed to open UART device` | serial port not enabled or used by Bluetooth / a login console — see step 1 |
| Topics visible on another machine, but no data | different `ROS_DOMAIN_ID`, or a firewall blocking UDP between the machines |
| `bind source path does not exist: .../config` | create `docker/config/` and copy a config file into it (step 2) |
| Visualization page loads, but shows no data | node not running or `node_id` changed without a restart — `sudo docker compose restart`; `sudo docker compose logs visualization` shows the topic it listens on |
| `visualization` exits with `Address already in use` | something else on the Pi uses port 5000 — stop it, or run the node only (`up -d gnss`) |

### Build from Source

#### Prerequisites — ROS 2 from Source

Build ROS 2 Jazzy from source following the official guide:
[Building ROS 2 on Linux](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)

Once built, source the workspace before building this package:

```sh
source ~/ros2_jazzy/install/setup.bash
```

#### Dependencies

```sh
sudo apt-get install libgpiod-dev
```

#### Build

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

The [gnsshat](https://github.com/jimmypaputto/gnsshat) library is taken from the system if installed (e.g. `sudo make install` in gnsshat → `/usr/local`). Otherwise CMake `FetchContent` fetches it from GitHub. Supported gnsshat release: `v2.0.0`.

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

The file is validated on load: a malformed section, an unknown value (polarity, RTK mode, base type, ...) or a missing required key fails `configure` with an error naming the offending key. Sections the connected HAT does not support (`geofencing` on the TIME HAT, `rtk` on anything but the RTK HAT) fail `configure` too — they are not ignored.

```yaml
node_id: ""                    # multi-instance: changes node name and topic prefix
measurement_rate_hz: 1         # 1–25 Hz
save_to_flash: false           # persist config to u-blox receiver flash
publish_standard_topics: true  # publish /gps/fix, /gps/vel, /gps/time_reference
dynamic_model: 2               # see table below

timepulse:                     # optional — omit entire section to disable the pin
  active: true
  fixed_pulse:
    frequency: 1               # Hz
    pulse_width: 0.1           # duty cycle, 0.0 <= x < 1.0
  # pulse_when_no_fix:         # optional — omit to disable pulse before fix
  #   frequency: 1
  #   pulse_width: 0.5
  polarity: rising             # "rising" or "falling"

geofencing:                    # optional — omit entire section to disable
  confidence_level: 3          # 0=none 1=68% 2=95% 3=99.7% 4=99.99% 5=99.9999%
  pio_pin_polarity: low_means_inside   # optional — "low_means_inside" or "low_means_outside"
  geofences:                   # 1–4 entries
    - lat: 41.902205071091224
      lon: 12.4539203390548
      radius: 2005.0
    - lat: 52.257211745024186
      lon: 20.311759615806704
      radius: 1810.0

navigation_filters:            # optional — every key optional; omitted keys keep the receiver default
  min_svs: 4                   # 3–32 satellites
  max_svs: 32                  # 3–32, >= min_svs
  min_cno_dbhz: 15             # 0–63 dBHz, ignore weaker satellites
  min_elev_deg: 10             # -90–90 deg, ignore satellites below this elevation
  n_cno_thrs: 0                # satellites required above cno_thrs_dbhz for a fix
  cno_thrs_dbhz: 0             # 0–63 dBHz
  fix_mode: auto               # "2d", "3d" or "auto"
  pdop_mask_x10: 250           # PDOP mask in 0.1 units (25.0)
  tdop_mask_x10: 250           # TDOP mask in 0.1 units
  p_acc_mask_m: 100            # position accuracy mask [m]
  t_acc_mask_m: 350            # time accuracy mask [m]

rtk:                           # optional — omit to disable
  mode: base                   # required: "base" or "rover"
  base:                        # required for "base", ignored for "rover"
    type: survey_in            # required: "survey_in", "fixed_ecef", or "fixed_lla"
    min_observation_time_s: 120
    required_accuracy_m: 50.0
    corrections: full          # "full" (MSM7, default) or "tiny" (MSM4)
    # fixed_ecef: x_m, y_m, z_m (required), position_accuracy_m (default 0.01)
    # fixed_lla: latitude_deg, longitude_deg, height_m (required), position_accuracy_m (default 0.01)
  ntrip:                       # optional — NTRIP caster, see RTK below
    host: caster.example.com   # required
    port: 2101
    mountpoint: MOUNT          # required
    username: user
    password: pass
    gga_interval_s: 10         # rover: send the position to the caster (VRS), 0 = off
```

### Navigation Filters

`navigation_filters` maps to `GnssConfig::navigationFilters` in gnsshat (UBX CFG-NAVSPG) and works on every HAT. The keys match the `navigation_filters` dict of the gnsshat Python API. The YAML loader checks only types (e.g. `min_svs: 300` does not fit `uint8`); value ranges are validated by gnsshat when the HAT starts, so an out-of-range filter fails `configure` with `Failed to start GNSS HAT` in the node log and the exact reason (`[GnssConfig] Invalid navigation filter ...`) on stderr.

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

### Subscribed

| Topic | Type | Condition |
|-------|------|-----------|
| `/gnss/rtk/corrections` | `jp_gnss_hat/RtkCorrections` | RTK rover |

### Multi-Instance

Set `node_id` in YAML (e.g. `"rover"`). Custom topics and services get the ID inserted: `/gnss/rover/navigation`, `/gnss/rover/get_config`, etc. Standard topics (`/gps/*`) and the RTK corrections topic (`/gnss/rtk/corrections`) stay global — intentionally shared between instances.

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

`SetGnssConfig` triggers a full lifecycle cycle: deactivate → cleanup → configure (new config) → activate. On failure the node automatically rolls back to the previous working config. A request with invalid enum values (`dynamic_model`, polarities, `rtk_mode`, `rtk_base_type`, `rtk_corrections`) or more than 4 geofences is rejected up front with `success: false` and the node keeps running unchanged.

`save_to_yaml: true` rewrites the config file the node was started with (comments are not kept). The file is replaced atomically and keeps its owner and mode; a symlink is followed. If saving fails, the new config stays applied and the response is `success: false` with the reason.

`SetGnssConfig` replaces the **whole** configuration: every field left out of the request takes its message default (`0` / `false`). That disables geofencing, RTK and navigation filters, and `timepulse_polarity: 0` means the falling edge. Only the NTRIP settings (`rtk.ntrip`) are not in the message; they stay as loaded from YAML. Start from `get_config` and send the full config back with your changes:

```python
import rclpy
from rclpy.node import Node
from jp_gnss_hat.srv import GetGnssConfig, SetGnssConfig

rclpy.init()
node = Node('gnss_reconfigure')
get_cfg = node.create_client(GetGnssConfig, '/gnss/get_config')
set_cfg = node.create_client(SetGnssConfig, '/gnss/set_config')
get_cfg.wait_for_service()
set_cfg.wait_for_service()

future = get_cfg.call_async(GetGnssConfig.Request())
rclpy.spin_until_future_complete(node, future)
config = future.result().config
config.measurement_rate_hz = 5          # change only what you need

future = set_cfg.call_async(SetGnssConfig.Request(config=config, save_to_yaml=False))
rclpy.spin_until_future_complete(node, future)
print(future.result())
```

From the shell, list every field you rely on. Here: a HAT without geofencing and RTK, rising-edge timepulse as in the shipped configs:

```sh
ros2 service call /gnss/get_config jp_gnss_hat/srv/GetGnssConfig

ros2 service call /gnss/set_config jp_gnss_hat/srv/SetGnssConfig \
  "{config: {measurement_rate_hz: 5, dynamic_model: 4, publish_standard_topics: true, \
  timepulse_active: true, timepulse_frequency: 10, timepulse_pulse_width: 0.1, \
  timepulse_polarity: 1}, save_to_yaml: false}"
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
RfBlockSpectrum[] rf_spectrum
```

Single message per GNSS update. Contains PVT (UBX-NAV-PVT), DOP (UBX-NAV-DOP), per-satellite data (UBX-NAV-SAT), RF band diagnostics (UBX-MON-RF), geofencing state (UBX-NAV-GEOFENCE) and the RF spectrum (UBX-MON-SPAN).

The receiver refreshes the spectrum about once per second, so `rf_spectrum` is set only in the first message after a new spectrum and is empty in the others. Keep the last non-empty one if you need it between updates.

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
| `band` | `uint8` | RF block index: 0=first, 1=second (kept for compatibility — use `gnss_band` for the band) |
| `gnss_band` | `uint8` | 0=Unknown, 1=L1, 2=L2, 3=L3, 4=L5, 241=L2orL5 (`EGnssBand`) |
| `jamming_state` | `uint8` | 0=Unknown, 1=OK, 2=Warning, 3=Critical |
| `antenna_status` | `uint8` | 0=Init, 1=DontKnow, 2=Ok, 3=Short, 4=Open |
| `antenna_power` | `uint8` | 0=Off, 1=On, 2=DontKnow |
| `post_status` | `uint32` | |
| `noise_per_ms` | `uint16` | |
| `agc_monitor` | `float32` | |
| `cw_interference_suppression_level` | `float32` | |
| `ofs_i`, `ofs_q` | `int8` | I/Q offset |
| `mag_i`, `mag_q` | `uint8` | I/Q magnitude |

### jp_gnss_hat/RfBlockSpectrum

| Field | Type | Description |
|-------|------|-------------|
| `id` | `uint8` | RF block index, matches `RfBlock.band` |
| `data` | `uint8[]` | Amplitude bins as sent by the receiver |
| `span` / `resolution` / `center_freq` | `uint32` | Hz |
| `gain` | `uint8` | Receiver programmable gain, dB |

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

Flat message used by `GetGnssConfig` / `SetGnssConfig` services. Contains all config fields including timepulse, geofencing (with `Geofence[]` array), RTK parameters and `navigation_filters` (`NavigationFilters`). See `msg/GnssConfig.msg` for the full definition.

`NavigationFilters` pairs every filter with a `has_<field>` flag (e.g. `has_min_svs` + `min_svs`); only fields with the flag set are applied, `fix_mode` is 1=2D, 2=3D, 3=Auto. A `SetGnssConfig` request with no flag set — including one from a client that does not know this field — runs with the receiver defaults, i.e. it clears filters loaded from YAML.

## Standard Topic Mapping

When `publish_standard_topics: true`:

| Source | Target | Notes |
|--------|--------|-------|
| PVT lat/lon/alt | `NavSatFix.latitude/longitude/altitude` | Altitude = height above the WGS84 ellipsoid (not MSL) |
| PVT horizontal/vertical accuracy (m) | `NavSatFix.position_covariance` | Diagonal, σ² = acc_m² |
| PVT fix_quality, fix_status | `NavSatFix.status` | Invalid→NO_FIX, GPS/PPS/DR→FIX, DGNSS→SBAS_FIX, RTK→GBAS_FIX; `fix_status` Void (gnssFixOK clear, e.g. outside the `navigation_filters` masks)→NO_FIX |
| Satellites with `used_in_fix` | `NavSatFix.status.service` | GPS/Galileo/GLONASS/BeiDou flags |
| PVT speed + heading | `TwistWithCovarianceStamped` | Decomposed to ENU: vx=speed×sin(heading), vy=speed×cos(heading); variance speed_accuracy². vz and angular velocity are not measured: variance 99 |
| PVT UTC time/date | `TimeReference` | Published only when both `utc_valid` and `date_valid` are true |

## Lifecycle

| Transition | Action |
|------------|--------|
| `configure` | `IGnssHat::create()`, soft reset, load YAML (or pending config), `hat->start()`, create publishers, init RTK bridge and NTRIP link |
| `activate` | Start `std::jthread` running `navigationLoop()` — blocks on `hat->waitAndGetFreshNavigation(stop_token)` |
| `deactivate` | `request_stop()` + `join()` on navigation thread — the stop token ends the wait at once, without waiting for the next epoch |
| `cleanup` | Reset all publishers, converter, RTK bridge (closes the NTRIP connection), HAT pointer |
| `shutdown` | Defensive cleanup from any state |

Auto-transition in `Main.cpp` uses `MultiThreadedExecutor`. On `SIGINT`/`SIGTERM` rclcpp shuts the context down and the executor exits. No lifecycle transition can run after that, so `on_deactivate`/`on_cleanup` are not called: the node destructor stops the navigation thread and releases the HAT instead.

Manual control:

```sh
ros2 lifecycle set /jp_gnss_hat configure
ros2 lifecycle set /jp_gnss_hat activate
ros2 lifecycle set /jp_gnss_hat deactivate
ros2 lifecycle set /jp_gnss_hat cleanup
```

## RTK

NEO-F9P only. Corrections flow over `/gnss/rtk/corrections` (`RtkCorrections` message — vector of raw RTCM3 frames). DDS handles discovery and transport between base and rover nodes.

**Base** — every navigation cycle fetches one RTCM3 set from `hat->rtk()->base()` and publishes the frames. `rtk.base.corrections` (GnssConfig `rtk_corrections`) picks the set:

| Value | gnsshat call | RTCM3 messages |
|-------|--------------|----------------|
| `full` (default, `0`) | `getFullCorrections()` | MSM7: 1005, 1077, 1087, 1097, 1127, 1230 |
| `tiny` (`1`) | `getTinyCorrections()` | MSM4: 1005, 1074, 1084, 1094, 1124, 1230 — less bandwidth, lower observation resolution |

Three position modes:

| Mode | Fields |
|------|--------|
| `survey_in` | `min_observation_time_s`, `required_accuracy_m` |
| `fixed_ecef` | `x_m`, `y_m`, `z_m`, `position_accuracy_m` |
| `fixed_lla` | `latitude_deg`, `longitude_deg`, `height_m`, `position_accuracy_m` |

**Rover** — subscribes to corrections topic, calls `hat->rtk()->rover()->applyCorrections()`.

### NTRIP

`rtk.ntrip` connects the node to an NTRIP caster through the NTRIP 2.0 client and server built into gnsshat — no other ROS package or node is needed. The RTK mode picks the direction:

- **Rover** (`NtripClient`) — receives corrections from the caster. Until the first connection succeeds it retries every 1 s, doubling up to 30 s; after that the client reconnects by itself. The RTCM3 frames of one burst (one epoch) go to the receiver together, once the stream pauses for 100 ms. Every `gga_interval_s` seconds the current fix is sent to the caster as GGA, which VRS and nearest-base mountpoints need; nothing is sent before the first fix.
- **Base** (`NtripServer`, HTTP POST) — sends each RTCM3 set it publishes on `/gnss/rtk/corrections` (`full` or `tiny`) to the mountpoint and reconnects with the same backoff. A set the caster has not taken yet is replaced by the newer one. `gga_interval_s` is ignored.

Both connect in the background, so `configure` does not wait for the network, and `/gnss/rtk/corrections` keeps working alongside. A base's mountpoint serves any NTRIP client, including ROS packages such as `ntrip_client`.

The NTRIP settings come from the YAML file only. They are not part of `GnssConfig`: `get_config` does not return them (so the password stays off the network), `set_config` keeps them and `save_to_yaml` writes them back. The password is stored in the YAML file in plain text.

## Build Details

- C++20, CMake ≥ 3.11
- System dependency: `libgpiod-dev`
- gnsshat: `find_library(gnsshat)` + `find_path(jimmypaputto/GnssHat.hpp)` → fallback to `FetchContent` of the tag in `JP_GNSSHAT_GIT_TAG` from GitHub
  - the system install's version is read from `jimmypaputto/Version.hpp`; below `JP_GNSSHAT_GIT_TAG` (or with no `Version.hpp`) CMake prints a warning
  - to move to a newer gnsshat release, change the single line `set(JP_GNSSHAT_GIT_TAG vX.Y.Z)` at the top of `CMakeLists.txt`. The same value sets the minimum version for the system install.

```sh
# Build
source ~/ros2_jazzy/install/setup.bash
colcon build --packages-select jp_gnss_hat

# Test
colcon test --packages-select jp_gnss_hat && colcon test-result --verbose
```

## License

MIT — see [LICENSE](LICENSE).
