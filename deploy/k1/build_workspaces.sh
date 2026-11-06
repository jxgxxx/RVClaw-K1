#!/usr/bin/env bash
set -eo pipefail
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
test "$(uname -m)" = riscv64
test -d "$ROBOT_ROOT/runtime/k1"
source /opt/ros/humble/setup.bash
export CPATH="/opt/ros/humble/include/pcl_conversions:/opt/ros/humble/include/pcl_msgs:/opt/ros/humble/include/message_filters${CPATH:+:$CPATH}"
export MAKEFLAGS="${MAKEFLAGS:--j3}"
cd "$ROBOT_ROOT/robot/ros2_ws"
colcon build --base-paths src \
  --packages-select rvclaw_stm32_base lslidar_msgs lslidar_driver rvclaw_nav2_bringup \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$ROBOT_ROOT/robot/ros2_ws/install/setup.bash"
for profile in navigation mapping; do
  cd "$ROBOT_ROOT/robot/profiles/$profile/ros2_ws"
  colcon build --base-paths src --packages-select rvclaw_nav2_bringup --executor sequential
done
