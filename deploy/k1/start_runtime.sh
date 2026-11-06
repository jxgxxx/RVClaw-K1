#!/usr/bin/env bash
# Check the migrated robot devices, then optionally start the stable services.
set -eo pipefail
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
case "${1:---check}" in --check|--start) ;; *) echo 'Usage: start_runtime.sh --check | --start' >&2; exit 2 ;; esac
missing=0
for device in \
  /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00 \
  /dev/wheeltec_lidar \
  /dev/rvclaw-m260c-control \
  /dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0; do
  if [[ ! -c "$device" || ! -r "$device" || ! -w "$device" ]]; then
    printf 'Device missing or inaccessible: %s\n' "$device" >&2
    missing=1
  fi
done
if ! arecord -l 2>/dev/null | grep -q XFMDPV0018; then
  echo 'Voice microphone missing: XFMDPV0018' >&2
  missing=1
fi
if ! aplay -l 2>/dev/null | grep -q 'Device \[USB Audio Device\]'; then
  echo 'Voice speaker missing: USB Audio Device' >&2
  missing=1
fi
[[ $missing -eq 0 ]] || exit 3
if ! sudo -n -l /usr/bin/renice -n -10 -p "$$" >/dev/null 2>&1; then
  echo 'Control priority permission missing. Run: sudo bash /data/RVClaw/deploy/k1/install_control_priority.sh' >&2
  exit 5
fi
for artifact in "$ROBOT_ROOT/build/yolov8_mjpeg" \
  "$ROBOT_ROOT/build/meter-paddleocr/meter_paddleocr" \
  "$ROBOT_ROOT/runtime/k1/navigation/ros2_ws/install/setup.bash"; do
  [[ -r "$artifact" ]] || { printf 'Build artifact missing: %s\n' "$artifact" >&2; exit 4; }
done
echo 'Robot device preflight passed.'
[[ ${1:---check} == --start ]] || exit 0
systemctl --user enable rvclaw-meter-gauge.service rvclaw-camera-api.service \
  rvclaw-resource-scheduler.service rvclaw-vendor-command.service
systemctl --user start rvclaw-meter-gauge.service
systemctl --user start rvclaw-camera-api.service
systemctl --user start rvclaw-resource-scheduler.service
systemctl --user start rvclaw-vendor-command.service
echo 'Robot services started. No patrol or motion command was sent.'
