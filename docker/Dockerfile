# jp_gnss_hat runtime image for Raspberry Pi (arm64): the node and the gnsshat web visualization.
# Build from the repository root:  docker build -f docker/Dockerfile .
# or from this directory:          docker compose build
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Dependencies from package.xml (separate layer: code changes do not reinstall them).
# tini -g forwards Ctrl+C / docker stop to the whole process group, so the node
# gets SIGINT together with ros2 launch instead of a SIGTERM 5 s later.
COPY package.xml /tmp/deps/jp_gnss_hat/package.xml
RUN apt-get update \
 && apt-get install -y --no-install-recommends tini python3-venv \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/deps --ignore-src -y \
      -t buildtool -t build -t build_export -t exec \
 && rm -rf /var/lib/apt/lists/* /tmp/deps

# gnsshat is fetched from GitHub by CMake (tag JP_GNSSHAT_GIT_TAG in CMakeLists.txt).
# GNSSHAT_PLATFORM=rpi4 compiles in gpiochip0 — the GPIO header on Pi 4 and on Pi 5 with a
# current kernel (/dev/gpiochip4 there is only a symlink to gpiochip0). One image for both.
# The web visualization (examples/visualization) is taken from the same gnsshat checkout,
# so its version always matches the library.
COPY . /tmp/src/jp_gnss_hat
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon --log-base /tmp/log build \
      --base-paths /tmp/src \
      --build-base /tmp/build \
      --install-base /opt/jp_gnss_hat \
      --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DGNSSHAT_PLATFORM=rpi4 \
 && cp -r /tmp/build/jp_gnss_hat/_deps/gnsshat-src/examples/visualization /opt/jp_gnss_hat/visualization \
 && rm -rf /tmp/src /tmp/build /tmp/log

# Visualization dependencies in a venv (pip: apt in Ubuntu 24.04 has no simple-websocket and an
# incompatible Flask / Flask-SocketIO pair). --system-site-packages keeps rclpy reachable.
RUN python3 -m venv --system-site-packages /opt/jp_gnss_hat/visualization/venv \
 && /opt/jp_gnss_hat/visualization/venv/bin/pip install --no-cache-dir \
      -r /opt/jp_gnss_hat/visualization/requirements.txt

# Fast DDS over UDP only: shared-memory transport fails silently between a container
# and the host (or another container) unless both share IPC and the same user.
COPY docker/fastdds-udp.xml /opt/jp_gnss_hat/fastdds-udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/jp_gnss_hat/fastdds-udp.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/jp_gnss_hat/fastdds-udp.xml

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/visualization.sh /usr/local/bin/jp-gnss-visualization
ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/entrypoint.sh"]
CMD ["ros2", "launch", "jp_gnss_hat", "gnss.launch.py"]
