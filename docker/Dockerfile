# Talos-RIT Scorbot ROS 2 workspace image.
#
# One image serves three purposes, selected by the container command and the
# SCORBOT_DISPLAY_MODE environment variable:
#   - virtual station in a browser, works on any computer
#   - native X11
#   - a development shell with the source tree
# Build context is the scorbot_ros2 folder

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    WS=/ws

# ROS packages the stack needs beyond the desktop image, plus a minimal VNC desktop
# so RViz can run inside the container and be viewed from any browser.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-xacro \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-cmake-gtest \
        python3-pytest \
        xterm \
        && rm -rf /var/lib/apt/lists/*

# Build the workspace. The source is copied in so the image is self contained
# the dev service in componse.yaml bind-mounts the live tree
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
    # Interactive shells get workspace sourced
    && printf '\nsource /opt/ros/%s/setup.bash\n[ -f %s/install/setup.bash ] && source %s/install/setup.bash\n' \
        "${ROS_DISTRO}" "${WS}" "${WS}" >> /root/.bashrc

# RViz inside Xvnc has no GPU; force Mesa software rendering
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    SCORBOT_DISPLAY_MODE=vnc \
    VNC_GEOMETRY=1600X900

EXPOSE 6080

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "ros2", "launch", "scorbot_description", "view_robot.launch.py" ]