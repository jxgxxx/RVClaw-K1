#!/usr/bin/env bash
# Install the exact renice permission used by the unchanged stable route.
set -euo pipefail
[[ $(id -u) -eq 0 ]] || { echo 'Run with sudo.' >&2; exit 1; }
ROBOT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
rule_source="$ROBOT_ROOT/deploy/k1/sudoers/rvclaw-control-priority"
rule_destination=/etc/sudoers.d/rvclaw-control-priority
getent passwd firefly >/dev/null
test -x /usr/bin/renice
/usr/sbin/visudo -c -f "$rule_source"
if [[ -e "$rule_destination" || -L "$rule_destination" ]]; then
  [[ ! -L "$rule_destination" ]] || { echo 'Refusing a symbolic link.' >&2; exit 2; }
  cmp "$rule_source" "$rule_destination"
else
  install -o root -g root -m 440 "$rule_source" "$rule_destination"
fi
/usr/sbin/visudo -c
echo 'K1 control priority permission ready. No service or motion command was issued.'
