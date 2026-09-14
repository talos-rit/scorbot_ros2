# Talos-RIT Scorbot ROS 2 workspace image.
#
# One image serves three purposes, selected by the container command and the
# SCORBOT_DISPLAY_MODE environment variable (see docker/entrypoint.sh):
#   - virtual station in a browser (noVNC on port 6080)   -- works on macOS, Windows, Linux
#   - native X11 windows on a Linux host                  -- lowest latency
#   - a development shell with the source tree mounted
#
# Build context is the scorbot_ros2 folder (see docker/compose.yaml).

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    WS=/ws

# ROS packages the stack needs beyond the desktop image, plus a minimal VNC desktop
# so RViz can run inside the container and be viewed from any browser.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-rqt-joint-trajectory-controller \
        ros-jazzy-moveit \
        ros-jazzy-moveit-ros-control-interface \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-cmake-gtest \
        libeigen3-dev \
        python3-pytest \
        tigervnc-standalone-server \
        novnc \
        websockify \
        openbox \
        xterm \
        dbus-x11 \
        mesa-utils \
        && rm -rf /var/lib/apt/lists/*

# Optional: pick_ik, the IK solver that handles the 5-DOF arm well (ik:=pick_ik).
# Tolerant install so the image still builds if the binary is missing for this distro.
RUN apt-get update \
    && (apt-get install -y --no-install-recommends ros-jazzy-pick-ik \
        || echo "WARNING: ros-jazzy-pick-ik not available; ik:=pick_ik will not work") \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace. The source is copied in so the image is self-contained;
# the dev service in compose.yaml bind-mounts the live tree over it.
COPY . ${WS}/src/scorbot_ros2
WORKDIR ${WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/ws /usr/local/bin/ws
RUN chmod +x /entrypoint.sh /usr/local/bin/ws \
    # Interactive shells (docker compose exec ... bash) get the workspace sourced too.
    && printf '\nsource /opt/ros/%s/setup.bash\n[ -f %s/install/setup.bash ] && source %s/install/setup.bash\n' \
       "${ROS_DISTRO}" "${WS}" "${WS}" >> /root/.bashrc

# RViz inside Xvnc has no GPU; force Mesa software rendering.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    SCORBOT_DISPLAY_MODE=vnc \
    VNC_GEOMETRY=1600x900

EXPOSE 6080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "scorbot_description", "view_robot.launch.py"]
