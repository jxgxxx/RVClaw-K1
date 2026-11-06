#!/usr/bin/env bash
# Voice supplies the mission root; mapping needs its own unchanged profile.
set -eo pipefail
ENGINEERING_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ${1:-} == --voice-run && -z ${RVCLAW_RUNTIME_LEASE:-} ]]; then
  exec python3 "$ENGINEERING_ROOT/deploy/k1/runtime_demand.py" run --kind mapping -- bash "$0" "$@"
fi
export RVCLAW_LAB_ROOT="$ENGINEERING_ROOT/runtime/k1/mapping"
export RVCLAW_BASE_ROOT="$ENGINEERING_ROOT/runtime/k1/base"
export RVCLAW_ROS_WS="$ENGINEERING_ROOT/robot/profiles/mapping/ros2_ws"
export RVCLAW_LIDAR_SYSROOT="/"
export RVCLAW_LIDAR_LIB_DIR="/usr/lib/riscv64-linux-gnu"
# Native K1 ROS CLI initialization takes longer than the K3 CLI budget.
export RVCLAW_MAPPING_GRAPH_QUERY_TIMEOUT_S="${RVCLAW_MAPPING_GRAPH_QUERY_TIMEOUT_S:-15}"
export RVCLAW_MAPPING_MOTOR_CONTROL_TIMEOUT_S="${RVCLAW_MAPPING_MOTOR_CONTROL_TIMEOUT_S:-20}"
exec bash "$RVCLAW_LAB_ROOT/scripts/slam_mapping_session.sh" "$@"
