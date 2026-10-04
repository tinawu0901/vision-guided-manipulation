#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/vision_guided_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${GZ_PARTITION:-vision_guided}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
# Reuse the logged-in desktop's Xwayland display when started through SSH.
if [[ -z "${DISPLAY:-}" ]]; then
  desktop_pid=$(pgrep -u "$USER" -n gnome-shell || true)
  if [[ -n "$desktop_pid" ]]; then
    while IFS= read -r desktop_var; do
      case "$desktop_var" in DISPLAY=*|XAUTHORITY=*) export "$desktop_var";; esac
    done < <(tr '\0' '\n' < "/proc/$desktop_pid/environ")
  fi
fi
exec ros2 launch vision_guided_simulation gazebo.launch.py "$@"
