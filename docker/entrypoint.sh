#!/usr/bin/env bash
# Container entrypoint for the scorbot_ros2 image.
#
# SCORBOT_DISPLAY_MODE=vnc     start an X server + window manager + noVNC on :6080,
#                              then run the command with DISPLAY pointing at it.
# SCORBOT_DISPLAY_MODE=native  run the command as-is (the host's DISPLAY is used).
# SCORBOT_DISPLAY_MODE=none    run the command with no display at all (tests, CI).
# No `-u`: the ROS setup scripts reference variables that are legitimately unset.
set -eo pipefail

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi

mode="${SCORBOT_DISPLAY_MODE:-vnc}"

if [ "$mode" = "vnc" ]; then
  export DISPLAY=:1
  geometry="${VNC_GEOMETRY:-1600x900}"

  # X server with an RFB port only reachable inside the container; noVNC fronts it.
  Xtigervnc "$DISPLAY" -geometry "$geometry" -depth 24 -rfbport 5901 \
    -SecurityTypes None -localhost yes -AlwaysShared >/tmp/xvnc.log 2>&1 &
  for _ in $(seq 1 50); do
    [ -e "/tmp/.X11-unix/X1" ] && break
    sleep 0.1
  done
  openbox >/tmp/openbox.log 2>&1 &
  websockify --web=/usr/share/novnc 6080 localhost:5901 >/tmp/websockify.log 2>&1 &

  echo "----------------------------------------------------------------"
  echo " Virtual station display ready."
  echo " Open  http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
  echo "----------------------------------------------------------------"
fi

exec "$@"
