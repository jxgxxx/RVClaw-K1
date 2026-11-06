#!/usr/bin/env bash
# K1 has eight cores. Let navigation and control children use the full pool.
set -euo pipefail
[[ $# -gt 0 ]] || { echo 'Missing control process executable.' >&2; exit 2; }
[[ $(getconf _NPROCESSORS_ONLN) -eq 8 ]] || { echo 'Unexpected K1 CPU layout.' >&2; exit 2; }
sudo -n /usr/bin/renice -n -10 -p "$$" >/dev/null
exec "$@"
