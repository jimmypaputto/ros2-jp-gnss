# jp_gnss_hat runtime image for Raspberry Pi (arm64).
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
 && apt-get install -y --no-install-recommends tini \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/deps --ignore-src -y \
      -t buildtool -t build -t build_export -t exec \
 && rm -rf /var/lib/apt/lists/* /tmp/deps

# gnsshat is fetched from GitHub by CMake (tag JP_GNSSHAT_GIT_TAG in CMakeLists.txt).
# GNSSHAT_PLATFORM=rpi4 compiles in gpiochip0 — the GPIO header on Pi 4 and on Pi 5 with a
# current kernel (/dev/gpiochip4 there is only a symlink to gpiochip0). One image for both.
COPY . /tmp/src/jp_gnss_hat
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon --log-base /tmp/log build \
      --base-paths /tmp/src \
      --build-base /tmp/build \
      --install-base /opt/jp_gnss_hat \
      --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DGNSSHAT_PLATFORM=rpi4 \
 && rm -rf /tmp/src /tmp/build /tmp/log

# Fast DDS over UDP only: shared-memory transport fails silently between a container
# and the host (or another container) unless both share IPC and the same user.
COPY docker/fastdds-udp.xml /opt/jp_gnss_hat/fastdds-udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/jp_gnss_hat/fastdds-udp.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/jp_gnss_hat/fastdds-udp.xml

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/usr/bin/tini", "-g", "--", "/entrypoint.sh"]
CMD ["ros2", "launch", "jp_gnss_hat", "gnss.launch.py"]
