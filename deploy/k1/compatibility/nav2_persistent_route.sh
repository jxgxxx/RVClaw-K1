#!/usr/bin/env bash
# Run a route only after the persistent localization guard reaches HOT_READY.

set -eo pipefail
# Shared Studio selection. Reading configuration never starts hardware.
PROFILE_RUNTIME=/data/RVClaw/services/inspection_profiles/profile_runtime.py
if [[ -f $PROFILE_RUNTIME && -f /data/RVClaw/state/inspection-profile.json ]]; then
  PROFILE_EXPORTS=$(python3 "$PROFILE_RUNTIME" --shell) || exit 4
  eval "$PROFILE_EXPORTS"
  unset PROFILE_EXPORTS
fi
RVCLAW_STARTUP_STARTED=$SECONDS
record_startup_stage() {
  printf 'startup_timing stage=%s elapsed_s=%s\n' "$1" "$((SECONDS-RVCLAW_STARTUP_STARTED))"
}

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
ROOT=${RVCLAW_LAB_ROOT:-$DEFAULT_ROOT}
BASE_ROOT=${RVCLAW_BASE_ROOT:-/data/rvclaw-k3-lab}
SITE_DIR=${RVCLAW_SITE_DIR:-}
MAP_FILE=${RVCLAW_MAP_FILE:-}
CACHE_FILE=${RVCLAW_LOCALIZATION_CACHE:-$ROOT/state/localization-pose.json}
WAYPOINTS_FILE=${RVCLAW_NAV2_WAYPOINTS_FILE:-}
if [[ -n $SITE_DIR ]]; then
  # The map selector/UI supplies a site directory, never a code change.
  # Explicit file overrides are permitted only if the Python binding check
  # confirms that the waypoint catalog declares this very map.
  MAP_FILE=${MAP_FILE:-$SITE_DIR/map.yaml}
  WAYPOINTS_FILE=${WAYPOINTS_FILE:-$SITE_DIR/waypoints.yaml}
fi
ROUTE_NAMES=${RVCLAW_NAV2_WAYPOINT_ROUTE:-A,B,C,ORIGIN}
INSPECTION_CONFIG=${RVCLAW_INSPECTION_CONFIG:-}
CHECKPOINT_FILE=${RVCLAW_ROUTE_CHECKPOINT_FILE:-/data/RVClaw/runtime/inspection-route-checkpoint.json}
RESUME_CONTEXT=${RVCLAW_ROUTE_RESUME_CONTEXT:-}
FORCE_RELOCALIZATION=${RVCLAW_FORCE_RELOCALIZATION:-0}
ROUTE_ARM_TOKEN=WHEELS_GROUND_PS2_OFF_NAV2_WAYPOINT_ROUTE
CONTROL_NICE=${RVCLAW_CONTROL_NICE:--10}
INSPECTION_ARGS=()
RESUME_ARGS=()
REPOSITION_ARGS=()
if [[ $FORCE_RELOCALIZATION == 1 ]]; then
  REPOSITION_ARGS=(--mark-repositioned)
fi
if [[ -n $INSPECTION_CONFIG ]]; then
  INSPECTION_ARGS=(--inspection-config "$INSPECTION_CONFIG")
fi
if [[ -n $RESUME_CONTEXT ]]; then
  # This wrapper runs nav2_fast_localization.py to PASS immediately before the
  # route client.  Mark only this guarded path as allowed to rebase continuity
  # on the freshly verified current AMCL pose.
  RESUME_ARGS=(
    --resume-context "$RESUME_CONTEXT"
    --resume-localization-verified
  )
fi

if [[ $CONTROL_NICE != 0 && $CONTROL_NICE != -10 ]]; then
  echo "RVCLAW_CONTROL_NICE must be 0 or -10" >&2
  exit 2
fi

raise_process_priority() {
  local label=$1
  local pattern=$2
  local pid
  local observed_nice
  local -a pids=()
  local -a tids=()

  mapfile -t pids < <(pgrep -f "$pattern")
  if [[ ${#pids[@]} -ne 1 ]]; then
    echo "expected exactly one $label process, found ${#pids[@]}" >&2
    return 1
  fi
  pid=${pids[0]}
  for task_dir in /proc/"$pid"/task/*; do
    tids+=("${task_dir##*/}")
  done
  # Persistent processes normally already have this priority. Check every
  # thread, including any new workers, before invoking the privileged helper.
  if ps -L -o ni= -p "$pid" | awk -v wanted="$CONTROL_NICE" '$1 != wanted {exit 1}'; then
    echo "$label priority already verified: pid=$pid nice=$CONTROL_NICE"
    return 0
  fi
  sudo -n renice -n "$CONTROL_NICE" -p "${tids[@]}" >/dev/null
  observed_nice=$(ps -o ni= -p "$pid" | tr -d ' ')
  if [[ $observed_nice != "$CONTROL_NICE" ]]; then
    echo "$label priority readback failed: pid=$pid nice=$observed_nice" >&2
    return 1
  fi
  echo "$label priority verified: pid=$pid nice=$observed_nice"
}

configure_control_priority() {
  if [[ $CONTROL_NICE == 0 ]]; then
    echo "control priority override disabled"
    return 0
  fi
  # Inspect all dependency threads in one /proc pass. Repeating pgrep and ps
  # for each node costs several seconds on K1 even when priorities are ready.
  local helper=${RVCLAW_CONTROL_PRIORITY_HELPER:-/data/RVClaw/deploy/k1/apply_control_priority.py}
  if [[ -f $helper ]]; then
    python3 "$helper" --ready
    return
  fi
  raise_process_priority \
    hardware_node \
    '/rvclaw_stm32_base/hardware_node([[:space:]]|$)'
  if [[ ${RVCLAW_SENSOR_PIPELINE_PRIORITY:-0} == 1 ]]; then
    raise_process_priority scan_self_filter '/rvclaw_nav2_bringup/rvclaw-scan-self-filter([[:space:]]|$)'
    raise_process_priority odom_tf '/rvclaw_nav2_bringup/rvclaw-odom-tf([[:space:]]|$)'
    raise_process_priority amcl '/nav2_amcl/amcl([[:space:]]|$)'
  fi
  raise_process_priority \
    cmd_vel_adapter \
    '/rvclaw_nav2_bringup/cmd_vel_adapter([[:space:]]|$)'
  raise_process_priority \
    controller_server \
    '/nav2_controller/controller_server([[:space:]]|$)'
  raise_process_priority \
    planner_server \
    '/nav2_planner/planner_server([[:space:]]|$)'
  raise_process_priority \
    bt_navigator \
    '/nav2_bt_navigator/bt_navigator([[:space:]]|$)'
  raise_process_priority \
    behavior_server \
    '/nav2_behaviors/behavior_server([[:space:]]|$)'
  raise_process_priority \
    velocity_smoother \
    '/nav2_velocity_smoother/velocity_smoother([[:space:]]|$)'
  raise_process_priority \
    collision_monitor \
    '/nav2_collision_monitor/collision_monitor([[:space:]]|$)'
}

raise_route_priority() {
  local observed_nice

  if [[ $CONTROL_NICE == 0 ]]; then
    echo "route-client priority override disabled"
    return 0
  fi
  # exec below preserves the shell's nice value for the Python route client
  # and for its independent deadman keepalive thread. This keeps the 30 ms
  # heartbeat from being starved by RViz and vision inference bursts.
  sudo -n renice -n "$CONTROL_NICE" -p "$$"
  observed_nice=$(ps -o ni= -p "$$" | tr -d ' ')
  if [[ $observed_nice != "$CONTROL_NICE" ]]; then
    echo "route-client priority readback failed: pid=$$ nice=$observed_nice" >&2
    return 1
  fi
  echo "route-client priority verified: pid=$$ nice=$observed_nice"
}

if [[ ${1:-} == --describe && $# -eq 1 ]]; then
  cat <<EOF
hardware_stack_started=false
lidar_stack_started=false
nav2_stack_started=false
requires_existing_persistent_runtime=true
pre_route_gate=nav2_fast_localization.py with HOT_READY final state
ambiguous_static_match=bounded_global_scan_only_in_run_mode_then_revalidate
control_priority=$CONTROL_NICE for hardware_node, cmd_vel_adapter, controller_server, planner_server, bt_navigator, behavior_server, velocity_smoother, collision_monitor, and route_client
route=$ROUTE_NAMES
waypoints=$WAYPOINTS_FILE
inspection_config=${INSPECTION_CONFIG:-disabled}
motion_requires=--run and existing physical field supervision
EOF
  exit 0
fi

MODE=${1:-}
if [[ $MODE == --run && $# -eq 1 && -z ${RVCLAW_RUNTIME_LEASE:-} ]]; then
  exec python3 /data/RVClaw/deploy/k1/runtime_demand.py run --kind inspection -- bash "$0" "$@"
fi
if [[ $MODE == --run ]]; then
  mkdir -p /data/RVClaw/runtime/inspection-control
  exec 9>/data/RVClaw/runtime/inspection-control/route.lock
  flock -n 9 || { echo "another inspection route is running" >&2; exit 9; }
fi
if [[ $MODE != --preflight && $MODE != --run && $MODE != --inspection-readiness && $MODE != --runtime-admission ]] || [[ $# -ne 1 ]]; then
  printf 'Usage: %s --describe | --inspection-readiness | --preflight | --run\n' "$0" >&2
  exit 2
fi

if [[ -z $MAP_FILE || -z $WAYPOINTS_FILE || ! -f $MAP_FILE || ! -f $WAYPOINTS_FILE ]]; then
  echo "select an existing site map and waypoint catalog before navigation" >&2
  exit 2
fi

set +u
# The system ROS tree is read-only to the service account. Store Python's
# timestamp-validated bytecode in the runtime tree instead of recompiling
# generated message/action modules on every task. A failed cache setup falls
# back to Python's normal behaviour; it never bypasses admission checks.
ROUTE_PYCACHE=${PYTHONPYCACHEPREFIX:-$ROOT/cache/python}
if mkdir -p -- "$ROUTE_PYCACHE" && [[ -w $ROUTE_PYCACHE ]]; then
  export PYTHONPYCACHEPREFIX="$ROUTE_PYCACHE"
else
  echo "route Python cache unavailable; using normal imports" >&2
fi
unset ROUTE_PYCACHE

if [[ $MODE == --runtime-admission && ${RVCLAW_ROUTE_ROS_ENV_READY:-} == 1 \
      && ${ROS_DISTRO:-} == humble && -n ${AMENT_PREFIX_PATH:-} ]]; then
  # This child inherits the environment prepared by the outer route wrapper.
  # Readiness, selected-map binding and priority verification still run below.
  # Re-sourcing identical overlays just adds several seconds to every start.
  record_startup_stage environment_reused
else
  if [[ -f $SCRIPT_DIR/ros_environment_cache.py ]] \
      && ROS_SETUP_EXPORTS=$(python3 "$SCRIPT_DIR/ros_environment_cache.py" --root "$ROOT" --base-root "$BASE_ROOT"); then
    eval "$ROS_SETUP_EXPORTS"
    unset ROS_SETUP_EXPORTS
  else
    source /opt/ros/humble/setup.bash
    if [[ $BASE_ROOT != "$ROOT" && -f $BASE_ROOT/ros2_ws/install/setup.bash \
          && $(readlink -f "$BASE_ROOT/ros2_ws/install/local_setup.bash") != \
             $(readlink -f "$ROOT/ros2_ws/install/local_setup.bash") ]]; then
      source "$BASE_ROOT/ros2_ws/install/local_setup.bash"
    fi
    source "$ROOT/ros2_ws/install/local_setup.bash"
  fi
  export RVCLAW_ROUTE_ROS_ENV_READY=1
fi
set -u
export RVCLAW_MAP_FILE="$MAP_FILE"
record_startup_stage environment_ready
# The route client's production loaders validate map/catalog binding and the
# complete inspection plan before creating ROS nodes or preparing control.
# Combined startup keeps that order without two extra Python interpreter loads.

if [[ $MODE == --inspection-readiness ]]; then
  if [[ -z $INSPECTION_CONFIG ]]; then
    echo "RVCLAW_INSPECTION_CONFIG is required for --inspection-readiness" >&2
    exit 2
  fi
  exec python3 "$SCRIPT_DIR/rvclaw_waypoint_route_client.py" \
    --inspection-readiness-only \
    --waypoints-file "$WAYPOINTS_FILE" \
    --route "$ROUTE_NAMES" \
    "${INSPECTION_ARGS[@]}"
fi

# A matching YAML/catalog pair is insufficient when the running map_server
# still serves a previous site. This read-only check compares every /map cell
# and its geometry with the selected map before localization and motion.
# Voice can become ready before the persistent stack on a cold boot. Wait
# only for that service's bounded initialization, before priority inspection
# or any ROS command. A failed/cancelled wait must never fall through.
if [[ $MODE != --run ]]; then
  python3 "${RVCLAW_NAVIGATION_READY_HELPER:-/data/RVClaw/deploy/k1/wait_navigation_ready.py}" \
    --timeout "${RVCLAW_NAVIGATION_READY_TIMEOUT_S:-30}"
  record_startup_stage navigation_service_ready
  if [[ -f $PROFILE_RUNTIME && -f /data/RVClaw/state/inspection-profile.json ]]; then
    python3 "$PROFILE_RUNTIME" --ensure-runtime --map-file "$MAP_FILE" --verifier "$SCRIPT_DIR/verify_runtime_map.py" || exit 4
  fi
fi
if [[ $MODE == --runtime-admission ]]; then
  configure_control_priority
  record_startup_stage control_priority_ready
  exit 0
fi
if [[ ${RVCLAW_EARLY_CONTROL_PRIORITY:-0} == 1 && $MODE != --run ]]; then
  configure_control_priority
  record_startup_stage control_priority_ready
fi
# The route process verifies the complete live map and stationary pose in its
# own ROS context. The watchdog releases only its stationary deadline after
# admission; cancellation continues to reach the route's entire process group.

mkdir -p -- "$ROOT/runs/persistent-route"
RUN_ID=$(date +%Y%m%d-%H%M%S)
FAST_TRACE=${RVCLAW_FAST_LOCALIZATION_TRACE_FILE:-$ROOT/runs/persistent-route/$RUN_ID-fast-localization.json}
ROUTE_TRACE=${RVCLAW_ROUTE_TRACE_FILE:-$ROOT/runs/persistent-route/$RUN_ID-route.json}

# Reposition invalidation runs in the route's existing ROS context and
# stationary watchdog. Avoid a separate CLI discovery/daemon round trip.

if [[ $MODE == --run ]]; then
  PRIORITY_LOG=$ROOT/runs/persistent-route/$RUN_ID-control-priority.log
  if ! raise_route_priority >"$PRIORITY_LOG" 2>&1; then
    cat "$PRIORITY_LOG" >&2
    exit 7
  fi
  cat "$PRIORITY_LOG"
  record_startup_stage route_ready
  export RVCLAW_PREPARE_RUNTIME_IN_ROUTE=1
  MODE_ARGS=(--arm-token "$ROUTE_ARM_TOKEN" --checkpoint-file "$CHECKPOINT_FILE")
else
  MODE_ARGS=(--preflight-only)
fi

run_verified_route() {
  python3 "$SCRIPT_DIR/startup_guard.py" --stage localization --handoff-ready \
    --timeout "${RVCLAW_STATIONARY_START_TIMEOUT_S:-60}" -- \
    python3 "$SCRIPT_DIR/rvclaw_waypoint_route_client.py" \
    --localize-first --localization-cache-file "$CACHE_FILE" \
    --localization-trace-file "$FAST_TRACE" \
    "${MODE_ARGS[@]}" \
    --waypoints-file "$WAYPOINTS_FILE" --route "$ROUTE_NAMES" \
    "${RESUME_ARGS[@]}" "${REPOSITION_ARGS[@]}" "${INSPECTION_ARGS[@]}" --trace-file "$ROUTE_TRACE"
}

LOCALIZATION_RC=0
run_verified_route || LOCALIZATION_RC=$?

if [[ $LOCALIZATION_RC -eq 4 ]]; then
  # Exit 4 is MOTION_REQUIRED, not authorization to bypass localization.
  if ! python3 -c 'import json, sys; d=json.load(open(sys.argv[1], encoding="utf-8")); sys.exit(0 if d.get("status") == "MOTION_REQUIRED" and d.get("motion_authorized") is False else 1)' "$FAST_TRACE"; then
    echo "localization exit code and MOTION_REQUIRED trace disagree; refusing scan" >&2
    exit 4
  fi
  if [[ $MODE != --run ]]; then
    echo "localization needs movement; preflight never authorizes a scan" >&2
    exit 4
  fi
  GLOBAL_TRACE=$ROOT/runs/persistent-route/$RUN_ID-global-scan.json
  python3 "$SCRIPT_DIR/nav2_global_localization_scan.py" \
    --arm-token WHEELS_GROUND_PS2_OFF_AMCL_GLOBAL_SCAN \
    --trace-file "$GLOBAL_TRACE"
  FAST_TRACE=$ROOT/runs/persistent-route/$RUN_ID-post-scan-localization.json
  run_verified_route
  exit $?
fi
exit "$LOCALIZATION_RC"
