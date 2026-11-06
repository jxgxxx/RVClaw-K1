#!/usr/bin/env bash
# Build only AMCL's node library against the board's matching 1.1.18 ABI.
set -eo pipefail
test "$(dpkg-query -W -f='${Version}' ros-humble-nav2-amcl)" = 1.1.18-1jammy
source /opt/ros/humble/setup.bash
src=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
out=${1:?Specify an isolated build directory}
cmake -S "$src" -B "$out" -DCMAKE_BUILD_TYPE=Release
cmake --build "$out" --parallel 2
