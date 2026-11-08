#!/bin/bash
# Wrapper around component_container so apt depthai_ros_driver finds
# libdiagnostic_updater.so (header-only in current Humble; shim provided).
set -e
SHIM_LIB="/home/rabeb/Downloads/enimia_black/install/diagnostic_updater_shim/lib"
if [ ! -f "$SHIM_LIB/libdiagnostic_updater.so" ]; then
  # fallback relative to this script -> workspace
  WS="$(cd "$(dirname "$0")/../../.." && pwd)"
  SHIM_LIB="$WS/install/diagnostic_updater_shim/lib"
fi
export LD_LIBRARY_PATH="${SHIM_LIB}:/opt/ros/humble/lib:${LD_LIBRARY_PATH}"
exec /opt/ros/humble/lib/rclcpp_components/component_container "$@"
