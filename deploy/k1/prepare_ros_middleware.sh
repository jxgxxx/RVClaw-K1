#!/usr/bin/env bash
# Extract board-repository ROS middleware privately; never replace system ROS.
set -euo pipefail
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
MIDDLEWARE_ROOT="$ROBOT_ROOT/runtime/k1/ros-middleware"
[[ $(uname -m) == riscv64 ]] || { echo 'K1 board execution required.' >&2; exit 2; }
[[ -d "$ROBOT_ROOT/runtime/k1" ]] || { echo 'Prepare the K1 runtime first.' >&2; exit 2; }
[[ ! -e "$MIDDLEWARE_ROOT" ]] || { echo 'Middleware already exists; preserve it.' >&2; exit 2; }
PACKAGE_DIR=$(mktemp -d)
trap 'rm -f -- "$PACKAGE_DIR"/*.deb; rmdir -- "$PACKAGE_DIR"' EXIT
cd -- "$PACKAGE_DIR"
apt-get download ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp \
  ros-humble-iceoryx-binding-c ros-humble-iceoryx-hoofs ros-humble-iceoryx-posh
for package in *.deb; do
  [[ $(dpkg-deb -f "$package" Architecture) == riscv64 ]]
done
mkdir -p -- "$MIDDLEWARE_ROOT"
for package in *.deb; do
  dpkg-deb -x "$package" "$MIDDLEWARE_ROOT"
done
echo 'K1 private ROS middleware ready; no robot service was started.'
