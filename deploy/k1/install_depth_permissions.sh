#!/usr/bin/env bash
# Does not stop services, reset USB, start streaming or change RGB permissions.
set -euo pipefail
if [[ $EUID != 0 ]]; then
  echo 'Run with sudo: bash /data/RVClaw/deploy/k1/install_depth_permissions.sh' >&2
  exit 1
fi
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
rule="$project_root/deploy/k1/udev/60-rvclaw-astra-depth.rules"
target=/etc/udev/rules.d/60-rvclaw-astra-depth.rules
if [[ -f "$target" ]] && ! cmp -s "$rule" "$target"; then
  echo 'An existing different rule was found; preserve it and review first.' >&2
  exit 2
fi
install -o root -g root -m 0644 "$rule" "$target"
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2bc5 --attr-match=idProduct=0403
udevadm settle --timeout=5
echo 'Only Astra Pro depth 2bc5:0403 permissions updated. Run the unprivileged probe next.'
