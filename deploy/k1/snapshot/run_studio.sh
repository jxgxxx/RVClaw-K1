#!/usr/bin/env bash
set -eo pipefail
ROOT=@ROBOT_ROOT@
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=1
export PYTHONPATH="$ROOT/services/studio_gateway:/opt/opencv-spacemit/lib/python3.12/dist-packages"
export LD_LIBRARY_PATH="@ROBOT_ROOT@/runtime/k1/ros-middleware/opt/ros/humble/lib:@ROBOT_ROOT@/runtime/k1/ros-middleware/opt/ros/humble/lib/riscv64-linux-gnu:/opt/opencv-spacemit/lib:/data/spacemit_robot/output/staging/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://@ROBOT_ROOT@/deploy/k1/env/cyclonedds.xml
export RVCLAW_ROBOT_ROOT=$ROOT
export RVCLAW_BOARD_LABEL='Firefly K1'
export RVCLAW_STUDIO_BIND=@BOARD_ADDRESS@
export RVCLAW_STUDIO_ORIGINS=http://@BOARD_ADDRESS@:8017
export RVCLAW_STUDIO_ROOT=$ROOT/studio/web
export RVCLAW_VISION_REPO_ROOT=$ROOT/runtime/k1/vision
export RVCLAW_VISION_UPSTREAM=http://127.0.0.1:8017
export RVCLAW_INSPECTION_PROGRESS_OUT=$ROOT/runtime/inspection-progress/inspection-progress.json
export RVCLAW_INSPECTION_PROFILE_ROOT=$ROOT/state
export RVCLAW_INSPECTION_MAP_ROOTS=$ROOT/configs/scenes:$ROOT/configs/studio-maps
export RVCLAW_STUDIO_MAP_REGISTRY=$ROOT/configs/studio/k1-maps.json
export RVCLAW_NAV2_WAYPOINTS_FILE=$ROOT/configs/scenes/inspection-k1-reviewed/waypoints.yaml
export RVCLAW_INSPECTION_CONFIG=$ROOT/configs/scenes/inspection-k1-reviewed/inspection_route_tasks.yaml
export RVCLAW_TASK_DB=$ROOT/runtime/voice_tasks.sqlite3
exec /usr/bin/python3 -m rvclaw_studio.server "$@"
