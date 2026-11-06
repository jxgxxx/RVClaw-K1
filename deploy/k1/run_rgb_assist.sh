#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
set -a
source "$project_root/runtime/k1/environment/rvclaw-vendor-command.service.env"
set +a
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$project_root/robot/ros2_ws/src/rvclaw_perception:/opt/opencv-spacemit/lib/python3.12/dist-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 -m rgb_assist.worker --config "$project_root/runtime/k1/vision/configs/meter/meters.json" --landmarks "$project_root/configs/perception/rgb_landmarks/landmarks.json" --log-dir "$project_root/runtime/k1/perception/rgb-assist" "$@"
