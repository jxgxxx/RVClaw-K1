#!/usr/bin/env bash
# Extract the official native compiler locally; leave the system compiler intact.
set -euo pipefail
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
compiler_root="$ROBOT_ROOT/build/toolchain/gcc14"
package_root="$ROBOT_ROOT/build/toolchain/packages"
version='14.2.0-4ubuntu2~24.04bb2'
mkdir -p "$compiler_root" "$package_root"
cd "$package_root"
apt-get download \
  "gcc-14-base=$version" "gcc-14=$version" "g++-14=$version" "cpp-14=$version" \
  "gcc-14-riscv64-linux-gnu=$version" "g++-14-riscv64-linux-gnu=$version" \
  "cpp-14-riscv64-linux-gnu=$version" "libgcc-14-dev=$version" \
  "libstdc++-14-dev=$version" "libstdc++6=$version" "libgcc-s1=$version"
for package in ./*.deb; do
  dpkg-deb -x "$package" "$compiler_root"
done
"$compiler_root/usr/bin/g++-14" --version
