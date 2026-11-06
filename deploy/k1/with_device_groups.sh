#!/usr/bin/env bash
# Supply newly granted device groups even if the user manager predates setup.
set -euo pipefail
[[ $# -gt 0 ]] || exit 2
command_args=(/usr/bin/env)
if [[ -n ${LD_LIBRARY_PATH:-} ]]; then
  command_args+=("LD_LIBRARY_PATH=$LD_LIBRARY_PATH")
fi
command_args+=("$@")
printf -v quoted_command '%q ' "${command_args[@]}"
command="exec $quoted_command"
for device_group in video audio dialout; do
  id -nG | tr ' ' '\n' | grep -qx "$device_group" && continue
  printf -v quoted_command '%q' "$command"
  command="exec /usr/bin/sg $device_group -c $quoted_command"
done
exec /bin/bash -c "$command"
