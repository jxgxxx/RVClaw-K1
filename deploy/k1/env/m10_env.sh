#!/usr/bin/env bash
# Load native K1 ROS dependencies; no K3 sysroot or binaries are used.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Source this environment script.' >&2
  exit 1
fi
export ROS_LOCALHOST_ONLY=1
export RVCLAW_LAB_ROOT="${RVCLAW_LAB_ROOT:-/data/RVClaw/runtime/k1/navigation}"
export RVCLAW_ROS_WS="${RVCLAW_ROS_WS:-${RVCLAW_LAB_ROOT}/ros2_ws}"
for required_path in /opt/ros/humble/setup.bash "${RVCLAW_ROS_WS}/install/setup.bash"; do
  if [[ ! -r "$required_path" ]]; then
    echo "Missing required K1 runtime path: $required_path" >&2
    return 1
  fi
done
_rvclaw_restore_nounset=0
case "$-" in *u*) _rvclaw_restore_nounset=1; set +u ;; esac
source /opt/ros/humble/setup.bash
source "${RVCLAW_ROS_WS}/install/setup.bash"
if [[ $_rvclaw_restore_nounset -eq 1 ]]; then set -u; fi
unset _rvclaw_restore_nounset
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///data/RVClaw/deploy/k1/env/cyclonedds.xml"
export LD_LIBRARY_PATH="/data/RVClaw/runtime/k1/ros-middleware/opt/ros/humble/lib:/data/RVClaw/runtime/k1/ros-middleware/opt/ros/humble/lib/riscv64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
