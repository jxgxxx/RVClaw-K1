#!/usr/bin/env bash
# Grant this login temporary access to the currently connected XFM debug USB.
set -euo pipefail
[[ $EUID -eq 0 && ${SUDO_UID:-0} -gt 0 ]] || {
  echo 'Run with sudo from your normal K1 login.' >&2; exit 1;
}
devices=()
for usb in /sys/bus/usb/devices/*; do
  [[ -f "$usb/idVendor" && -f "$usb/idProduct" ]] || continue
  [[ $(<"$usb/idVendor") == 2207 && $(<"$usb/idProduct") == 0001 ]] || continue
  printf -v node '/dev/bus/usb/%03d/%03d' "$(<"$usb/busnum")" "$(<"$usb/devnum")"
  [[ -c "$node" ]] || exit 1
  devices+=("$node")
done
[[ ${#devices[@]} -eq 1 ]] || {
  echo 'Expected exactly one XFM voice USB module; no permission changed.' >&2; exit 1;
}
/usr/bin/setfacl -m "u:${SUDO_UID}:rw" "${devices[0]}"
echo 'Current diagnostic USB permission granted; no reset, firmware or motion command sent.'
