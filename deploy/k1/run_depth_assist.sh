#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source "$project_root/configs/perception/astra_depth.env"
: "${XDG_RUNTIME_DIR:?XDG_RUNTIME_DIR is required}"
state_dir="$XDG_RUNTIME_DIR/rvclaw-depth"
mkdir -p -- "$state_dir"
chmod 700 "$state_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$project_root/build/depth-assist/rvclaw_depth_reader" \
    "$state_dir/observation.json" "$RVCLAW_DEPTH_SAMPLE_HZ"
