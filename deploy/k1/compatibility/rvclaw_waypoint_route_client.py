#!/usr/bin/env python3
"""Navigate a reviewed absolute waypoint route with fail-stop semantics."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback
from uuid import uuid4
import xml.etree.ElementTree as ET

# Prepare services/priorities while importing the ROS message stack. This
# child has no motion interface and shares the route's cancellation group.
# main still waits for successful admission before constructing any ROS node.
_runtime_admission = None
_runtime_admission_started = None
if (__name__ == '__main__' and os.environ.get('RVCLAW_PREPARE_RUNTIME_IN_ROUTE') == '1'
        and '--localize-first' in sys.argv
        and not any(flag in sys.argv for flag in ('--describe','--preflight-only','--help','-h'))):
    import atexit
    _runtime_admission_started = time.monotonic()
    _runtime_admission = subprocess.Popen(['bash', str(Path(__file__).resolve().with_name(
        'nav2_persistent_route.sh')), '--runtime-admission'])
    def _cleanup_runtime_admission():
        if _runtime_admission.poll() is None:
            _runtime_admission.terminate()
            try: _runtime_admission.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                _runtime_admission.kill(); _runtime_admission.wait(timeout=2.)
    atexit.register(_cleanup_runtime_admission)

from ament_index_python.packages import get_package_share_directory

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import DriveOnHeading, Spin
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(
    0,
    str(WORKSPACE_ROOT / "ros2_ws" / "src" / "rvclaw_nav2_bringup"),
)

import rvclaw_waypoint_client as WAYPOINT  # noqa: E402
import nav2_heading_spin_left_0350 as HEADING  # noqa: E402
from rvclaw_nav2_bringup import dynamic_obstacle_policy as DYNAMIC  # noqa: E402
from rvclaw_nav2_bringup.inspection_task_runtime import (  # noqa: E402
    InspectionExecutor,
    load_inspection_plan,
    plan_summary as inspection_plan_summary,
)
from rvclaw_nav2_bringup.navigation_recovery_policy import (  # noqa: E402
    PROMPTS as NAVIGATION_RECOVERY_PROMPTS,
    classify_navigation_failure,
    confirmed_localization_freshness_stop,
    retry_is_safe,
)
from rvclaw_nav2_bringup.waypoint_catalog import (  # noqa: E402
    load_waypoints,
)
from rvclaw_nav2_bringup.site_bundle_contract import (  # noqa: E402
    validate_map_waypoint_binding,
)
from rvclaw_nav2_bringup.safe_egress_policy import (  # noqa: E402
    EGRESS_ACTION_TIME_ALLOWANCE_S,
    EGRESS_BODY_FRONT_M,
    EGRESS_DISTANCE_M,
    EGRESS_LASER_X_M,
    EGRESS_MIN_DISTANCE_M,
    EGRESS_RESULT_TIMEOUT_S,
    EGRESS_RUNTIME_MAXIMUM_SCAN_AGE_S,
    EGRESS_RUNTIME_MINIMUM_SCAN_CHECKS,
    EGRESS_MODE_SIDE_NEAR,
    EGRESS_MODE_STANDARD,
    EGRESS_SIDE_NEAR_SAFETY_MARGIN_M,
    EGRESS_SPEED_M_S,
    detect_side_near_obstacle,
    evaluate_front_egress_scan,
    evaluate_runtime_egress_monitor,
    remaining_egress_distance,
    select_safe_egress,
)
from rvclaw_nav2_bringup.side_turn_departure_policy import (  # noqa: E402
    NOMINAL_ROTATION_RADIUS_M,
    TURN_REVIEW_MIN_ANGLE_RAD,
    check_nominal_static_rotation_disk,
    classify_side_turn_departure,
    check_front_stop_turn_sweep,
)
from rvclaw_nav2_bringup.staged_egress_policy import (  # noqa: E402
    MAX_STEPS as MAX_AUTONOMOUS_EGRESS_STEPS,
    step_budget,
    validate_chain_continuity,
    validate_completed_step,
)


ROUTE_ARM_TOKEN = "WHEELS_GROUND_PS2_OFF_NAV2_WAYPOINT_ROUTE"
# This equipment-level qualification is still missing: the 2-D lidar plane
# cannot certify upper chassis protrusions or the 10 mm side-near margin.
# Keep this recovery branch inactive even if the scan/map gates pass. This is
# not a per-site parameter and must not be enabled by a map/voice command.
SIDE_NEAR_EGRESS_FIELD_CERTIFIED = False
DEFAULT_ROUTE = ("A", "B", "C", "ORIGIN")
ROUTE_MAX_DURATION_S = 600.0
# Obstacle waiting and Nav2 retries share the transit deadline. Extend only
# ORIGIN travel; inspection legs and the separate final-heading limit retain
# their existing budgets.
ORIGIN_TRANSIT_ACTION_RESULT_TIMEOUT_S = 240.0
# Both stopped, independently verified return-recovery paths use the same
# bound. This is a correction budget, never a pose/arrival acceptance radius.
ORIGIN_VERIFIED_CORRECTION_MAX_M = 1.0
# The reviewed 0.309 m x 0.255 m footprint has a circumscribed radius of
# about 0.20 m.  Include the 0.05 m ORIGIN XY tolerance plus 0.10 m for AMCL
# and costmap rasterization so a future return preflight does not mistake the
# robot currently covering ORIGIN for an obstacle at that future goal. Runtime
# planning and collision checks remain mandatory when C->ORIGIN is executed.
MAX_DEFERRED_RETURN_DISTANCE_M = 0.35
MAX_INITIAL_YAW_INPUT_DEG = 1000000.0
ORIGIN_HEADING_SKIP_TOLERANCE_RAD = math.radians(4.0)
ORIGIN_HEADING_TOLERANCE_RAD = math.radians(12.0)
# Meter framing is much less tolerant than the final ORIGIN pose.  A 12-degree
# terminal error was enough to push the phone/display to the edge of the camera
# image, so inspection alignment has its own strict bound.
INSPECTION_HEADING_COMPLETION_TOLERANCE_RAD = math.radians(10.0)
ORIGIN_HEADING_ACTION_TIME_ALLOWANCE_S = 10
ORIGIN_HEADING_ACTION_RESULT_TIMEOUT_S = 15.0
ORIGIN_HEADING_MIN_GUARDED_RAD_S = 0.20
ORIGIN_HEADING_MAX_GUARDED_RAD_S = 0.60
# One continuous shortest-path turn. Collision monitoring still owns stopping;
# splitting a turn never makes a blocked swept envelope safe.
ORIGIN_HEADING_SPIN_CHUNK_MAX_RAD = math.pi
ORIGIN_HEADING_MAX_CHUNKS = 12
ORIGIN_HEADING_MAX_TOTAL_RAD = math.pi + math.radians(30.0)
ORIGIN_GOAL_XY_TOLERANCE_M = 0.05
ORIGIN_GOAL_YAW_TOLERANCE_RAD = 0.25
ORIGIN_POST_ENTRY_MAX_ROTATION_RAD = ORIGIN_HEADING_MAX_TOTAL_RAD
ORIGIN_REPLAN_DISTANCE_M = 0.25
CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M = 0.25
CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD = math.radians(30.0)
VERIFIED_RESUME_REBASE_KEY = "verified_resume_rebase_from_current_amcl"

# A lethal start cell can be a historical chassis return left in the static
# map.  The route may use a bounded forward escape only when the independent
# live-scan and static-map gates prove the newly swept corridor is clear.
NEAR_START_EGRESS_PATH_STATUS = GoalStatus.STATUS_ABORTED
NEAR_START_EGRESS_MIN_START_COST = WAYPOINT.BASE.MAXIMUM_START_COST + 1
# ``pre_arm_front_clearance_m`` is the raw forward range from the lidar, not
# the vehicle-front gap.  Convert the reviewed 0.30 m physical gap through
# the measured lidar/body geometry before admitting the adaptive policy.
NEAR_START_EGRESS_MIN_PHYSICAL_FRONT_GAP_M = 0.30
NEAR_START_EGRESS_MIN_FRONT_CLEARANCE_M = (
    EGRESS_BODY_FRONT_M
    + NEAR_START_EGRESS_MIN_PHYSICAL_FRONT_GAP_M
    - EGRESS_LASER_X_M
)

# Stop the position controller before final rotation. DWB's rotation critic
# latches its XY window while the non-stateful goal checker keeps reevaluating
# XY. A changed scan view can make these disagree and produce endless turns.
# Use the existing guarded Spin path, then independently validate final XY.
ORIGIN_NAVIGATION_MODE = "split_spin"
# Intermediate waypoints are position checkpoints.  Their catalog heading is
# retained as metadata. Runtime and preflight use a travel-bearing hint plus a
# dedicated goal checker that accepts any endpoint yaw after XY is reached.
INTERMEDIATE_NAVIGATION_MODE = "position_only"
POSITION_ONLY_BT_NAME = "navigate_to_pose_position_only.xml"
ORIGIN_FULL_POSE_BT_NAME = "navigate_to_pose_origin_heading.xml"
DEFAULT_ARRIVAL_EVENT_DIR = "/data/RVClaw/runtime/waypoint-arrivals"
DEFAULT_INSPECTION_RESULT_DIR = "/data/RVClaw/runs/route-inspections"
DEFAULT_ROUTE_CHECKPOINT_FILE = "/data/RVClaw/runtime/inspection-route-checkpoint.json"
DEFAULT_SPEECH_ACK_TIMEOUT_S = 12.0
# Native pause prompts can exceed twelve seconds. Match the inspection speech
# budget so the 20-second player can finish and acknowledge the actual audio.
# This changes speech acknowledgement only; motion and safety deadlines remain.
RECOVERY_SPEECH_ACK_TIMEOUT_S = 22.0
SPEECH_ACK_POLL_INTERVAL_S = 0.05

AMCL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def shortest_heading_delta(target_yaw: float, current_yaw: float) -> float:
    """Return the shortest signed rotation from current to target."""
    return WAYPOINT.BASE.normalize_angle(target_yaw - current_yaw)


def normalize_yaw_degrees(value: float) -> float:
    """Normalize any finite degree input to the ROS [-180, 180) convention."""
    if not math.isfinite(value):
        raise ValueError("initial yaw must be finite")
    if abs(value) > MAX_INITIAL_YAW_INPUT_DEG:
        raise ValueError(
            "initial yaw must be within +/-%.0f degrees"
            % MAX_INITIAL_YAW_INPUT_DEG
        )
    return math.degrees(WAYPOINT.BASE.normalize_angle(math.radians(value)))


def validate_initial_pose(x: float, y: float, yaw_deg: float):
    """Validate and normalize a user-supplied map-frame start pose."""
    values = (float(x), float(y), float(yaw_deg))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("initial pose values must be finite")
    # Metric coordinates belong to the selected map, whose extent and origin
    # are not fixed by this route client. The live map/localization preflight
    # rejects an impossible pose; a hardcoded +/-20 m box rejects valid sites.
    normalized_yaw_deg = normalize_yaw_degrees(values[2])
    return {
        "x": values[0],
        "y": values[1],
        "yaw_deg": normalized_yaw_deg,
        "yaw": math.radians(normalized_yaw_deg),
    }


def load_global_localization_pose(trace_path):
    """Read a converged pose from a passed AMCL global-scan trace."""
    path = Path(trace_path).expanduser().resolve()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("global localization trace is not readable") from exc
    if document.get("status") != "PASS":
        raise ValueError("global localization trace is not a PASS result")
    pose = document.get("localization", {}).get("final_pose")
    if not isinstance(pose, dict):
        raise ValueError("global localization trace has no final pose")
    try:
        result = validate_initial_pose(
            pose["x"], pose["y"], math.degrees(float(pose["yaw"]))
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("global localization final pose is invalid") from exc
    result["source"] = "amcl_global_localization_scan"
    return result


def _is_near_start_path_failure(node, path_error=None):
    """Recognize only the reviewed lethal-start/empty-path condition."""
    return (
        getattr(node, "start_cost", None) is not None
        and int(node.start_cost) >= NEAR_START_EGRESS_MIN_START_COST
        and getattr(node, "path_check_status", None)
        == NEAR_START_EGRESS_PATH_STATUS
        and getattr(node, "latest_amcl_pose", None) is not None
        and getattr(node, "pre_arm_front_clearance_m", None) is not None
        and float(node.pre_arm_front_clearance_m)
        >= NEAR_START_EGRESS_MIN_FRONT_CLEARANCE_M
    )


class SideNearTurnEgressRequired(RuntimeError):
    """The planner succeeded, but its departure turn is not sweep-clear."""


class SideNearTurnUnsafe(RuntimeError):
    """An uncertain near-side turn cannot be released automatically."""


class DepartureAlignmentRequired(RuntimeError):
    """A checked departure must finish a pure Spin before path tracking."""

    def __init__(self, target_yaw):
        super().__init__("checked egress requires pure departure alignment")
        self.target_yaw = float(target_yaw)


def _active_map_file():
    """Never silently compare a new-map pose with the old demonstration map."""
    configured = os.environ.get("RVCLAW_MAP_FILE", "").strip()
    if not configured:
        raise RuntimeError("current map is not configured for route safety checks")
    path = Path(configured).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError("current route map is unavailable: %s" % path)
    return str(path)


def _evaluate_node_egress_gates(node):
    """Evaluate live and static gates using the node's newest observations."""
    if node.latest_scan is None or node.latest_amcl_pose is None:
        raise RuntimeError("near-start egress requires a fresh scan and AMCL pose")
    map_file = _active_map_file()
    from rvclaw_nav2_bringup.scan_map_consistency import load_static_map

    grid = load_static_map(map_file)
    pose = node.latest_amcl_pose
    candidates = [
        {
            "x": pose["x"],
            "y": pose["y"],
            "yaw": pose["yaw"],
            "yaw_deg": math.degrees(pose["yaw"]),
        }
    ]
    side_obstacle = detect_side_near_obstacle(
        node.latest_scan.ranges,
        node.latest_scan.angle_min,
        node.latest_scan.angle_increment,
        node.latest_scan.range_min,
        node.latest_scan.range_max,
    )
    mode = EGRESS_MODE_SIDE_NEAR if side_obstacle["detected"] else EGRESS_MODE_STANDARD
    continuation_check = None
    if getattr(node, "prefer_shortest_egress", False):
        delta = node.egress_turn_delta_rad
        if delta is None or not math.isfinite(delta):
            raise SideNearTurnUnsafe("turn egress has no departure bearing")

        def continuation_check(distance_m):
            endpoint = dict(pose,
                x=pose["x"] + distance_m*math.cos(pose["yaw"]),
                y=pose["y"] + distance_m*math.sin(pose["yaw"]))
            map_turn = check_nominal_static_rotation_disk(grid, endpoint)
            scan_turn = check_front_stop_turn_sweep(
                node.latest_scan.ranges, node.latest_scan.angle_min,
                node.latest_scan.angle_increment, node.latest_scan.range_min,
                node.latest_scan.range_max, delta, forward_offset_m=distance_m)
            return {"status": "PASS" if map_turn["status"] == scan_turn["status"] == "PASS" else "FAIL",
                    "map_rotation": map_turn, "front_stop_turn_sweep": scan_turn,
                    "predicted_pose": endpoint, "requires_live_recheck_after_step": True}
    result = select_safe_egress(
        node.latest_scan.ranges,
        node.latest_scan.angle_min,
        node.latest_scan.angle_increment,
        node.latest_scan.range_min,
        node.latest_scan.range_max,
        grid,
        candidates,
        mode=mode,
        prefer_shortest=getattr(node, "prefer_shortest_egress", False),
        continuation_check=continuation_check,
    )
    if mode == EGRESS_MODE_SIDE_NEAR and not SIDE_NEAR_EGRESS_FIELD_CERTIFIED:
        result.update({
            "status": "FAIL",
            "decision": "REQUIRE_OPERATOR_REPOSITION",
            "selected_distance_m": None,
            "failures": list(result.get("failures", [])) + [
                "side-near escape is not certified for the complete robot envelope"
            ],
        })
    if side_obstacle["status"] != "PASS":
        failures = list(result.get("failures", []))
        failures.append("side obstacle is inside the side-near safety margin")
        result["status"] = "FAIL"
        result["decision"] = "REQUIRE_OPERATOR_REPOSITION"
        result["selected_distance_m"] = None
        result["failures"] = failures
    result["map_file"] = map_file
    result["pose"] = dict(pose)
    result["side_near_obstacle"] = side_obstacle
    return result


def _check_continuous_route_localization(node, continuity_anchor):
    """Validate pose continuity without repeating an ambiguous global scan search."""
    if continuity_anchor is None:
        return False
    if node.latest_amcl_pose is None:
        raise RuntimeError("continuous route localization requires a fresh AMCL pose")
    current = node.latest_amcl_pose
    if (
        isinstance(continuity_anchor, dict)
        and continuity_anchor.get(VERIFIED_RESUME_REBASE_KEY) is True
    ):
        # The persistent wrapper sets this marker only after the independent
        # fast-localization gate has returned PASS/HOT_READY.  A resumed segment
        # must therefore start from that freshly verified current pose, not from
        # the last completed waypoint: the robot may have travelled safely into
        # the failed segment before an obstacle forced a stop.
        result = {
            "status": "PASS",
            "mode": "VERIFIED_RESUME_REBASE",
            "failures": [],
            "anchor_pose": dict(current),
            "current_pose": dict(current),
            "position_jump_m": 0.0,
            "yaw_jump_rad": 0.0,
            "maximum_position_jump_m": CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M,
            "maximum_yaw_jump_rad": CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD,
        }
        node.localization_gate_result = result
        node.localization_gate_scan_sample_count = node.scan_samples
        node.localization_gate_checked_monotonic = time.monotonic()
        node.localization_gate_pose = dict(current)
        node._record("verified_resume_localization_rebase", result)
        return True
    try:
        position_jump_m = math.hypot(
            float(current["x"]) - float(continuity_anchor["x"]),
            float(current["y"]) - float(continuity_anchor["y"]),
        )
        yaw_jump_rad = abs(
            WAYPOINT.BASE.normalize_angle(
                float(current["yaw"]) - float(continuity_anchor["yaw"])
            )
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("continuous route localization anchor is invalid") from exc
    failures = []
    if position_jump_m > CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M:
        failures.append(
            "AMCL position jumped %.3f m between route segments (limit %.3f m)"
            % (position_jump_m, CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M)
        )
    if yaw_jump_rad > CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD:
        failures.append(
            "AMCL yaw jumped %.1f deg between route segments (limit %.1f deg)"
            % (
                math.degrees(yaw_jump_rad),
                math.degrees(CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD),
            )
        )
    result = {
        "status": "FAIL" if failures else "PASS",
        "mode": "CONTINUOUS_ROUTE",
        "failures": failures,
        "anchor_pose": dict(continuity_anchor),
        "current_pose": dict(current),
        "position_jump_m": position_jump_m,
        "yaw_jump_rad": yaw_jump_rad,
        "maximum_position_jump_m": CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M,
        "maximum_yaw_jump_rad": CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD,
    }
    node.localization_gate_result = result
    node.localization_gate_scan_sample_count = node.scan_samples
    node.localization_gate_checked_monotonic = time.monotonic()
    node.localization_gate_pose = dict(current)
    node._record("continuous_route_localization_gate", result)
    if failures:
        raise RuntimeError(
            "continuous route localization gate failed: %s" % "; ".join(failures)
        )
    return True


class LiveObstacleWait:
    """One nonblocking prompt per sustained stop, rearmed after stable clearance."""

    def __init__(self):
        self.started = None
        self.announced = False
        self.clear_started = None
        self.last_announced = None

    def update(self, blocked, now, clear=True):
        if not blocked:
            self.started = None
            if not clear:
                self.clear_started = None
                return False
            if self.clear_started is None:
                self.clear_started = now
            if now - self.clear_started >= 2.0:
                self.announced = False
            return False
        self.clear_started = None
        if self.started is None:
            self.started = now
        if (not self.announced and now - self.started >= .6
                and (self.last_announced is None or now - self.last_announced >= 3.0)):
            self.announced = True
            self.last_announced = now
            return True
        return False


def scan_polygon_hit_count(scan, points, tx, ty, yaw):
    """Count real scan returns in the live CollisionMonitor polygon frame."""
    if len(points) < 3:
        return 0
    count = 0
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment + yaw
        x, y = tx + distance * math.cos(angle), ty + distance * math.sin(angle)
        inside = False
        previous = points[-1]
        for point in points:
            if ((point.y > y) != (previous.y > y)
                    and x < (previous.x - point.x) * (y - point.y)
                    / (previous.y - point.y) + point.x):
                inside = not inside
            previous = point
        count += int(inside)
    return count


def departure_path_heading(poses, robot_pose, lookahead_m=0.50):
    """Use a path-length lookahead, independent of the first costmap cell."""
    if len(poses) < 2:
        raise SideNearTurnUnsafe("departure alignment path is unavailable")
    if not all(math.isfinite(p.get(k, math.nan)) for p in poses for k in ('x', 'y')):
        raise SideNearTurnUnsafe("departure alignment path is nonfinite")
    start = poses[0]
    if math.hypot(start['x']-robot_pose['x'],start['y']-robot_pose['y']) > .20:
        raise SideNearTurnUnsafe("departure path start differs from stopped pose")
    walked = 0.0
    endpoint = start
    repeated_start = False
    for previous, current in zip(poses, poses[1:]):
        walked += math.hypot(current['x']-previous['x'],current['y']-previous['y'])
        endpoint = current
        progress = math.hypot(endpoint['x']-start['x'],endpoint['y']-start['y'])
        if walked >= lookahead_m:
            if progress < .10:
                repeated_start = True
            if progress >= (lookahead_m if repeated_start else .10):
                break
    if math.hypot(endpoint['x']-start['x'],endpoint['y']-start['y']) < .10:
        raise SideNearTurnUnsafe("departure path has no supported direction")
    return math.atan2(endpoint['y']-start['y'],endpoint['x']-start['x'])


def departure_alignment_attempt_allowed(completed_turns, delta):
    if not math.isfinite(delta) or len(completed_turns) >= 2:
        return False
    if not completed_turns:
        return True
    if abs(delta) > math.radians(45):
        return False
    total = 0.0
    for result in completed_turns:
        raw = result.get('raw_yaw_delta_rad')
        if (result.get('status') != 'PASS' or result.get('final_armed') is not False
                or not isinstance(raw,(int,float)) or not math.isfinite(raw)):
            return False
        chunk_deltas = [c.get('raw_yaw_delta_rad') for c in result.get('spin_chunks', [])]
        if any(not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in chunk_deltas):
            return False
        chunks = sum(abs(value) for value in chunk_deltas)
        total += max(abs(raw),chunks)
    return total+abs(delta) <= math.radians(210)


def departure_verified_observations(result, stopped_pose):
    """Transfer the last two independently acquired, stopped Spin samples."""
    if (not isinstance(result, dict) or result.get('status') != 'PASS'
            or result.get('final_armed') is not False or result.get('final_state') != 'DISARMED'
            or result.get('post_stop_stable_samples', 0) < 3):
        return []
    linear = [result.get(key, math.inf) for key in
              ('max_guarded_linear_x_m_s', 'max_guarded_linear_y_m_s')]
    if any(not math.isfinite(v) or abs(v) > WAYPOINT.BASE.ZERO_EPSILON for v in linear):
        return []
    events = [event for event in result.get('events', [])
              if event.get('name') == 'stopped_heading_observations_stable']
    if not events:
        return []
    event = events[-1]
    age = result.get('duration_s', math.inf)-event.get('elapsed_s', -math.inf)
    samples = [_finite_pose_copy(p) for p in event.get('value', {}).get('observations', [])]
    if (not 0. <= age <= .5 or len(samples) < 3 or any(p is None for p in samples)
            or _finite_pose_copy(stopped_pose) is None):
        return []
    samples = samples[-3:]
    if any(math.hypot(a['x']-b['x'], a['y']-b['y']) > .05
           or abs(shortest_heading_delta(a['yaw'], b['yaw'])) > math.radians(3.)
           for a in samples for b in samples+[stopped_pose]):
        return []
    return samples[-2:]


class PositionOnlyWaypointClient(WAYPOINT.WaypointClient):
    """Navigate to a position without enforcing an endpoint heading."""

    def __init__(self, waypoint, start_pose_hint=None, continuity_anchor=None,
                 continuing_egress=False):
        super().__init__(waypoint)
        self.start_pose_hint = (
            None if start_pose_hint is None else dict(start_pose_hint)
        )
        self.continuity_anchor = (
            None if continuity_anchor is None else dict(continuity_anchor)
        )
        self.freshness_recovery_disarmed_only = bool(
            (continuity_anchor or {}).get('freshness_recovery_disarmed_only'))
        self.continuing_egress = bool(continuing_egress)
        self.align_departure_before_navigation = False
        self.departure_alignment_running = False
        self.departure_alignment_result = None
        self.departure_handoff = None
        self.navigation_amcl_handoff_verified = False
        self.side_turn_review = None
        self.live_obstacle_wait = LiveObstacleWait()
        self.live_obstacle_navigation_active = False
        self.live_obstacle_input_time = None
        self.live_obstacle_output = None
        self.live_obstacle_polygon = None
        self.live_obstacle_max_points = None
        self.live_obstacle_event_path = None
        self.live_obstacle_tf = Buffer()
        self.live_obstacle_tf_listener = TransformListener(self.live_obstacle_tf, self)
        self.live_obstacle_params = self.create_client(GetParameters, "/collision_monitor/get_parameters")
        self.create_subscription(Twist, "/rvclaw/nav2_cmd_vel_raw", self._on_live_obstacle_input, 10)
        self.create_subscription(PolygonStamped, "/rvclaw/collision_monitor/front_stop",
                                 self._on_live_obstacle_polygon, 10)
        self.create_timer(.10, self._check_live_obstacle_wait)
        self.position_only_behavior_tree = str(
            Path(get_package_share_directory("rvclaw_nav2_bringup"))
            / "behavior_trees"
            / POSITION_ONLY_BT_NAME
        )

    def _on_live_obstacle_input(self, message):
        self.live_obstacle_input_time = time.monotonic()

    def _prepare_in_motion_map_guard(self):
        super()._prepare_in_motion_map_guard()
        self.b_reference = None
        self.b_reference_offset = None
        self.b_reference_offset_at = -math.inf
        self.b_reference_last_check = -math.inf
        self.b_reference_last_scan_ns = 0
        self.b_reference_checks = 0
        self.b_arrival_offset = None
        self.b_arrival_offset_at = -math.inf
        if self.waypoint.get('name') == 'B':
            from concurrent.futures import ThreadPoolExecutor
            from b_localization_reference import BLocalizationReference
            self.b_reference = BLocalizationReference(
                Path(os.environ['RVCLAW_MAP_FILE']).parent/'b_localization_reference.json',
                self.motion_map_grid)
            self.b_reference_worker = ThreadPoolExecutor(max_workers=1,
                thread_name_prefix='b-reference')
            self.b_reference_pending = None
            self.b_reference_timer = self.create_timer(.05, self._poll_b_reference_result)

    def destroy_node(self):
        timer = getattr(self, 'b_reference_timer', None)
        if timer is not None:
            self.destroy_timer(timer)
            self.b_reference_timer = None
        worker = getattr(self, 'b_reference_worker', None)
        if worker is not None:
            worker.shutdown(wait=True, cancel_futures=True)
            self.b_reference_worker = None
        return super().destroy_node()

    def _poll_b_reference_result(self):
        # Consume completed work independently of another lidar callback and
        # its synchronous general-map search. The same .8 s evidence limit
        # applies to the complete interval from submission to consumption.
        if (not getattr(self, 'motion_map_guard_active', False)
                or self.motion_map_guard_tripped):
            return
        now = time.monotonic()
        pending = getattr(self, 'b_reference_pending', None)
        if pending is not None:
            future, sampled_at, pose, scan_ns, pose_ns, *timing = pending
            if not future.done():
                return  # One job only; never build a queue of old scans.
            self.b_reference_pending = None
            evidence = future.result()
            if now-sampled_at > .8:
                self.b_reference_offset = None
                self._record('b_reference_stale_result', dict(age_s=now-sampled_at))
            else:
                self.b_reference_checks += 1
                self._record('b_reference_check', dict(
                    index=self.b_reference_checks, scan_stamp_ns=scan_ns,
                    pose_stamp_ns=pose_ns, current_pose=pose,
                    observation_timing=timing[0] if timing else None,
                    elapsed_s=now-sampled_at, evidence=evidence))
                from b_localization_reference import offset_evidence
                # Reference matching guards localization disagreement; Nav2
                # owns ordinary arrival. The optional early cancellation here
                # could halt at B then require a different stationary view to
                # prove the cancellation, stranding an otherwise valid route.
                previous = self.b_reference_offset if now-self.b_reference_offset_at < 1.8 else None
                from b_localization_reference import preserves_offset_observation
                if preserves_offset_observation(previous, evidence, pose):
                    # Ambiguous evidence cannot confirm a stop, but a matching
                    # peak does not erase a recent independently proven offset.
                    # Keep its original age; only another PASS may confirm it.
                    stop = False
                else:
                    self.b_reference_offset, stop = offset_evidence(previous, evidence, pose)
                    self.b_reference_offset_at = now
                if stop:
                    self._trip_in_motion_map_guard(dict(status='B_REFERENCE_DISAGREEMENT',
                        method='B_reference', current_pose=pose, reference=evidence,
                        reason='two independent B scans show a consistent position offset'))
                    return

    def _check_in_motion_map_disagreement(self, message):
        self._poll_b_reference_result()
        if self.motion_map_guard_tripped:
            return
        super()._check_in_motion_map_disagreement(message)
        reference = getattr(self, 'b_reference', None)
        if reference is None or self.motion_map_guard_tripped:
            return
        if getattr(self, 'b_reference_pending', None) is not None:
            return
        now = time.monotonic()
        if now-self.b_reference_last_check < .5:
            return
        pose = _finite_pose_copy(self.latest_amcl_pose)
        stamp = message.header.stamp
        scan_ns = int(stamp.sec)*1000000000+int(stamp.nanosec)
        if scan_ns <= self.b_reference_last_scan_ns:
            return
        pose_ns = getattr(self, 'departure_latest_amcl_stamp_ns', 0)
        if (pose is None or not 4.0 <= pose['x'] <= 7.5 or not pose_ns):
            # AMCL updates less often than the lidar. An unpaired scan says
            # nothing against the previous match; its 1.8 s expiry still
            # applies. Resetting here erased valid evidence between updates.
            return
        self.b_reference_last_check = now
        alignment = self.scan_odometry_history.align(pose, pose_ns, scan_ns)
        if alignment is None:
            # Use a measured, bracketed scan just as the general map guard
            # does. A newer scan with an older pose can invent a B offset.
            for previous in reversed(self.motion_map_recent_scans):
                previous_ns = (previous.header.stamp.sec * 1000000000
                               + previous.header.stamp.nanosec)
                if (previous_ns <= self.b_reference_last_scan_ns
                        or not 0 <= scan_ns-previous_ns <= 500000000):
                    continue
                paired = self.scan_odometry_history.align(pose, pose_ns, previous_ns)
                if paired is not None:
                    alignment = dict(paired, scan_selection_delay_s=(scan_ns-previous_ns)/1e9)
                    message, scan_ns = previous, previous_ns
                    break
        if alignment is None:
            # The general map guard still runs above. This additional B
            # reference must not count an unaligned frame as confirmation.
            self._record('b_reference_alignment_unavailable', dict(
                scan_stamp_ns=scan_ns, pose_stamp_ns=pose_ns))
            return
        self.b_reference_last_scan_ns = scan_ns
        pose = alignment['pose']
        self.b_reference_pending = (
            self.b_reference_worker.submit(reference.search,message,pose,moving=True),
            now, pose, scan_ns, pose_ns, alignment)

    def _trip_in_motion_map_guard(self, evidence):
        if evidence.get('method') not in {'B_reference', 'B_reference_arrival'}:
            return super()._trip_in_motion_map_guard(evidence)
        # This is a requested localization stop, never a latch reset. Drop
        # every keepalive producer first so a later heartbeat cannot undo it.
        self.motion_map_guard_tripped = True
        self.motion_map_guard_active = False
        self.motion_map_last_evidence = evidence
        self._record('motion_map_guard_stop', evidence)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.publish_zero_command()

    def stop_for_b_reference(self):
        """Cancel and prove a stationary DISARMED state; never clear a fault."""
        self._call_arm(False)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.publish_zero_command()
        if self.goal_handle is None:
            raise RuntimeError('B reference stop has no navigation goal')
        pending = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, pending, timeout_sec=2.0)
        if not pending.done() or pending.exception() is not None:
            raise RuntimeError('B reference navigation cancellation timed out')
        result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=2.0)
        if (not result.done() or result.exception() is not None
                or result.result().status not in (
                    GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED)):
            raise RuntimeError('B reference navigation did not stop normally')
        self.wait_for(lambda: self.armed is False and self.state == 'DISARMED',
                      WAYPOINT.BASE.SERVICE_TIMEOUT_S, 'disarmed B reference stop')
        self._wait_for_stable_feedback_stop()
        if self.armed is not False or self.state != 'DISARMED':
            raise RuntimeError('B reference stop encountered a hardware fault')
        self._record('b_reference_controlled_stop', dict(
            final_state=self.state, final_armed=self.armed,
            action_status=result.result().status))
        return self._b_reference_stationary_seed()

    def _b_reference_stationary_seed(self, *, passive_resume=False):
        """Fresh stopped AMCL is a search center, never localization approval."""
        # A manually requested resume can inherit the previous emergency
        # cleanup latch. Reading stationary evidence must not reset that latch.
        # Normal navigation still applies every pre-arm gate before resetting.
        allowed = {'DISARMED'}
        if passive_resume:
            allowed.update(('WATCHDOG_LATCHED', 'ESTOP_LATCHED'))
        initial_state = self.state
        if self.armed is not False or initial_state not in allowed:
            raise RuntimeError('B reference requires an unarmed stationary controller')
        self._wait_for_stable_feedback_stop()
        self._wait_for_fresh_scan()
        stamp = self.latest_localization_scan.header.stamp
        scan_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        count = self.amcl_samples
        pending = self.amcl_nomotion_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, pending, timeout_sec=2.0)
        if not pending.done() or pending.exception() is not None:
            raise RuntimeError('B stopped localization refresh service failed')
        self.wait_for(lambda: self.amcl_samples > count
                      and getattr(self, 'departure_latest_amcl_stamp_ns', 0) >= scan_ns,
                      WAYPOINT.BASE.SERVICE_TIMEOUT_S, 'fresh stopped B localization seed')
        pose = _finite_pose_copy(self.latest_amcl_pose)
        if (pose is None or self.armed is not False or self.state != initial_state
                or self.latest_amcl_monotonic is None
                or time.monotonic() - self.latest_amcl_monotonic > .5):
            raise RuntimeError('B stopped localization seed is stale or controller changed')
        self._record('b_reference_stationary_seed', dict(
            pose=pose, scan_stamp_ns=scan_ns,
            pose_stamp_ns=self.departure_latest_amcl_stamp_ns,
            independently_verified=False))
        return pose

    def _departure_message_is_fresh(self, message):
        barrier = getattr(self, "departure_amcl_after_stamp_ns", None)
        if barrier is None:
            return True
        stamp = message.header.stamp
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec) > barrier

    def _on_amcl(self, message):
        # This node is not spun while the separate heading node controls Spin.
        # Queued poses/scans must not become fresh post-turn observations.
        if self._departure_message_is_fresh(message):
            super()._on_amcl(message)
            stamp = message.header.stamp
            self.departure_latest_amcl_stamp_ns = (
                int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            )
            # Bound diagnostics equally on every route leg. Include wheel
            # odometry so the next replay can distinguish sensor aliasing
            # from actual motion; this does not change navigation decisions.
            now = time.monotonic()
            scan = getattr(self, 'latest_localization_scan', None)
            if (getattr(self, 'motion_map_guard_active', False) and scan is not None
                    and getattr(self, 'route_observation_count', 0) < 120
                    and now-getattr(self, 'route_observation_time', -math.inf) >= 1.0):
                self.route_observation_time = now
                self.route_observation_count = getattr(self, 'route_observation_count', 0)+1
                scan_ns = int(scan.header.stamp.sec)*1000000000+int(scan.header.stamp.nanosec)
                alignment = self.scan_odometry_history.align(
                    self.latest_amcl_pose, self.departure_latest_amcl_stamp_ns, scan_ns)
                self._record('route_pose_observation', dict(
                    waypoint=self.waypoint['name'], pose=dict(self.latest_amcl_pose),
                    odom=None if self.latest_raw_pose is None else dict(self.latest_raw_pose),
                    pose_stamp_ns=self.departure_latest_amcl_stamp_ns,
                    scan_stamp_ns=scan_ns,
                    odom_stamp_ns=self.latest_raw_stamp_ns,
                    observation_timing=alignment,
                    feedback=dict(vx=self.latest_feedback_x, wz=self.latest_feedback_angular),
                    scan=dict(angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                              range_min=scan.range_min, range_max=scan.range_max,
                              ranges=[float(v) if math.isfinite(v) else None for v in scan.ranges])))

    def _on_scan(self, message):
        if self._departure_message_is_fresh(message):
            super()._on_scan(message)

    def _on_localization_scan(self, message):
        if self._departure_message_is_fresh(message):
            super()._on_localization_scan(message)

    def _refresh_departure_observations(self, stopped_pose):
        # A no-motion request sent before the next scan may publish nothing:
        # AMCL just processed that scan for the heading client. Wait for a new
        # scan FIRST, then request its update; never wait passively for AMCL
        # while the stationary filter needs another explicit request.
        stable = departure_verified_observations(
            self.departure_alignment_result, stopped_pose)
        transferred = len(stable)
        for sample_index in range(12):
            self._wait_for_fresh_scan()
            stamp = self.latest_localization_scan.header.stamp
            scan_stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            previous_samples = self.amcl_samples
            future = self.amcl_nomotion_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=WAYPOINT.BASE.AMCL_NOMOTION_RESPONSE_TIMEOUT_S,
            )
            if not future.done() or future.exception() is not None:
                raise SideNearTurnUnsafe("departure stopped AMCL refresh service failed")
            self.wait_for(
                lambda: (self.amcl_samples > previous_samples
                         and getattr(self, "departure_latest_amcl_stamp_ns", 0) >= scan_stamp_ns),
                WAYPOINT.BASE.SERVICE_TIMEOUT_S,
                "new-scan AMCL observation after departure Spin",
            )
            pose = _finite_pose_copy(self.latest_amcl_pose)
            if (pose is None
                    or math.hypot(pose["x"]-stopped_pose["x"], pose["y"]-stopped_pose["y"])
                    > CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M
                    or abs(shortest_heading_delta(pose["yaw"], stopped_pose["yaw"]))
                    > CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD):
                raise SideNearTurnUnsafe("departure pose changed after verified Spin")
            if stable and any(
                math.hypot(pose["x"]-old["x"], pose["y"]-old["y"]) > .05
                or abs(shortest_heading_delta(pose["yaw"], old["yaw"])) > math.radians(3.)
                for old in stable
            ):
                stable = []
            stable.append(pose)
            if len(stable) >= 3:
                self._record("departure_stopped_observations_verified", {
                    "fresh_updates": sample_index + 1, "observations": stable,
                    "transferred_spin_observations": transferred,
                    "last_amcl_stamp_ns": self.departure_latest_amcl_stamp_ns,
                    "last_scan_stamp_ns": scan_stamp_ns,
                })
                return
        raise SideNearTurnUnsafe("departure stopped AMCL pose did not stabilize")

    def _on_live_obstacle_polygon(self, message):
        self.live_obstacle_polygon = message

    def _on_monitor_command(self, message):
        super()._on_monitor_command(message)
        self.live_obstacle_output = message

    def _load_live_obstacle_configuration(self):
        if self.live_obstacle_params.service_is_ready():
            request = GetParameters.Request()
            request.names = ["front_stop.max_points", "front_stop.enabled"]
            pending = self.live_obstacle_params.call_async(request)
            def configured(done):
                try:
                    values = done.result().values
                    if values[0].type == 2 and values[1].type == 1 and values[1].bool_value:
                        self.live_obstacle_max_points = values[0].integer_value
                except Exception as exc:
                    self._record("live_obstacle_notification_config_error", str(exc))
            pending.add_done_callback(configured)

    def _wait_navigation_result(self, future, timeout_s):
        correction_deadline = getattr(self, "inspection_position_deadline", None)
        if correction_deadline is not None:
            remaining = correction_deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("inspection position correction deadline expired")
            timeout_s = min(timeout_s, remaining)
        self.live_obstacle_navigation_active = True
        self._load_live_obstacle_configuration()
        try:
            return super()._wait_navigation_result(future, timeout_s)
        finally:
            self.live_obstacle_navigation_active = False
            self._retire_live_obstacle_event()

    def _retire_live_obstacle_event(self):
        path = getattr(self, "live_obstacle_event_path", None)
        if path is not None:
            Path(path).unlink(missing_ok=True)
            speech_ack_path(path).unlink(missing_ok=True)
            self.live_obstacle_event_path = None

    def _check_live_obstacle_wait(self):
        now = time.monotonic()
        output = self.live_obstacle_output
        polygon = self.live_obstacle_polygon
        fresh = all(stamp is not None and 0. <= now - stamp <= .30 for stamp in (
            self.live_obstacle_input_time, self.latest_monitor_command_monotonic,
            self.latest_scan_monotonic,
        ))
        stopped = (output is not None and all(abs(value) <= 1e-4 for value in (
            output.linear.x, output.linear.y, output.angular.z,
        )) and abs(self.latest_feedback_x) <= WAYPOINT.BASE.MAX_POST_STOP_LINEAR_M_S
            and abs(self.latest_feedback_angular) <= WAYPOINT.BASE.MAX_POST_STOP_ANGULAR_RAD_S)
        active = (self.live_obstacle_navigation_active and self.armed is True
                  and self.state == "ACTIVE" and fresh and stopped
                  and (self.action_feedback_final_distance_m or 0.) > .15
                  and self.live_obstacle_max_points is not None and polygon is not None)
        hits = 0
        if active:
            try:
                transform = self.live_obstacle_tf.lookup_transform(
                    polygon.header.frame_id, self.latest_scan.header.frame_id, rclpy.time.Time())
                t = transform.transform
                yaw = WAYPOINT.BASE.quaternion_to_yaw(t.rotation)
                hits = scan_polygon_hit_count(self.latest_scan, polygon.polygon.points,
                                              t.translation.x, t.translation.y, yaw)
            except Exception:
                active = False  # A missing transform must never invent an obstacle.
        blocked = active and hits > self.live_obstacle_max_points
        # A STOP/SLOWDOWN toggle is not evidence that the obstruction cleared.
        # Rearm the prompt only after measured physical movement resumes.
        moving = (abs(self.latest_feedback_x) > .01
                  or abs(self.latest_feedback_angular) > .10)
        cleared = (fresh and self.armed is True and self.state == "ACTIVE" and moving)
        announce = self.live_obstacle_wait.update(blocked, now, clear=cleared)
        if not blocked:
            self._retire_live_obstacle_event()
        if announce:
            try:
                decision = {"code": "ROUTE_BLOCKED",
                            "started_speech_key": "navigation_obstacle_recovery_started"}
                self.live_obstacle_event_path = publish_navigation_recovery_event(
                    "navigation_recovery_started", self.waypoint, decision,
                    os.getenv("RVCLAW_ARRIVAL_EVENT_DIR", DEFAULT_ARRIVAL_EVENT_DIR),
                )
                self._record("live_obstacle_wait_announced", {"hits": hits, "debounce_s": .6})
            except Exception as exc:
                self._record("live_obstacle_notification_error", str(exc))

    def _compute_goal_pose(self):
        reference = getattr(self, "start_pose_hint", None) or self.latest_amcl_pose
        if reference is None:
            raise RuntimeError("position-only start pose is unavailable")
        delta_x = self.waypoint["x"] - reference["x"]
        delta_y = self.waypoint["y"] - reference["y"]
        if math.hypot(delta_x, delta_y) <= 1e-6:
            goal_yaw = reference["yaw"]
        else:
            goal_yaw = math.atan2(delta_y, delta_x)
        return {
            "x": self.waypoint["x"],
            "y": self.waypoint["y"],
            "yaw": goal_yaw,
        }

    def _configure_navigation_goal(self, goal):
        self._validate_position_only_behavior_tree()
        goal.behavior_tree = self.position_only_behavior_tree

    def _validate_position_only_behavior_tree(self):
        path = Path(self.position_only_behavior_tree)
        if not path.is_file():
            raise RuntimeError(
                "position-only behavior tree is unavailable: %s" % path
            )

    def _check_static_map_localization(self):
        _active_map_file()
        if _check_continuous_route_localization(self, self.continuity_anchor):
            return
        return super()._check_static_map_localization()

    def _pre_arm_gate(self, check_localization: bool = True):
        """Validate position while deliberately allowing the current heading."""
        correction_deadline = getattr(self, "inspection_position_deadline", None)
        if correction_deadline is not None and time.monotonic() >= correction_deadline:
            raise RuntimeError("inspection position correction deadline expired")
        WAYPOINT.BASE.Nav2ForwardGoalClient._pre_arm_gate(
            self,
            check_localization=check_localization,
        )
        if any(
            not math.isclose(
                self.goal_pose[key], self.waypoint[key], rel_tol=0.0, abs_tol=1e-9
            )
            for key in ("x", "y")
        ):
            raise RuntimeError("position-only waypoint goal was not preserved")
        self._validate_position_only_behavior_tree()

    def _stationary_departure_handoff_is_current(self):
        handoff = self.departure_handoff
        if (handoff is None or not self.departure_alignment_running
                or not isinstance(self.departure_alignment_result, dict)
                or self.departure_alignment_result.get('status') != 'PASS'
                or self.departure_alignment_result.get('final_armed') is not False
                or self.departure_alignment_result.get('final_state') != 'DISARMED'
                or self.armed is not False or self.state != 'DISARMED'
                or not self.stm32_connected or not self.feedback_connected
                or self.amcl_samples < 1
                or not 0. <= time.monotonic()-handoff['at'] <= 1.
                or not all(math.isfinite(v) for v in (
                    self.latest_feedback_x, self.latest_feedback_y, self.latest_feedback_angular))
                or max(abs(self.latest_feedback_x), abs(self.latest_feedback_y)) > .01
                or abs(self.latest_feedback_angular) > .03):
            return False
        for current, previous, distance_limit, yaw_limit in (
            (self.latest_raw_pose, handoff['raw'], .005, .01),
            (self.latest_amcl_pose, handoff['pose'], .01, math.radians(1.)),
        ):
            current = _finite_pose_copy(current)
            if (current is None or previous is None
                    or math.hypot(current['x']-previous['x'], current['y']-previous['y']) > distance_limit
                    or abs(shortest_heading_delta(current['yaw'], previous['yaw'])) > yaw_limit):
                return False
        return True

    def _prepare_navigation_localization(self):
        self.navigation_amcl_handoff_verified = self._stationary_departure_handoff_is_current()
        if self.navigation_amcl_handoff_verified:
            # _refresh_departure_observations proved three consistent stopped
            # poses, including a NEW scan/AMCL pair received by this node.
            # Preserve callback counts; do not pretend transferred samples
            # were new subscriptions or request that same scan three times.
            self._record('departure_stopped_localization_handoff', {
                'verified_observations': 3, 'received_amcl_samples': self.amcl_samples})
            return
        return super()._prepare_navigation_localization()

    def _navigation_localization_ready(self):
        return ((self.navigation_amcl_handoff_verified and self.amcl_samples >= 1)
                or super()._navigation_localization_ready())

    def _initial_path_preflight_required(self):
        """Plan once at the final pre-arm gate after a checked, current Spin.

        The mandatory final preflight always computes a NEW path and checks
        the current map/scan before any arming. No cached path is admitted.
        """
        handoff = self.departure_handoff
        current = self._stationary_departure_handoff_is_current()
        self.departure_handoff = None
        if not current:
            return True
        self._record('departure_plan_deferred_to_final_preflight', {
            'handoff_age_s': time.monotonic()-handoff['at'], 'fresh_plan_required': True})
        return False

    def _preflight_path(self, start_pose=None):
        super()._preflight_path(start_pose=start_pose)
        if start_pose is not None:
            return  # Future route segments have no current live scan.
        # Planning and action-result callbacks can outlive the scan freshness
        # window in this client. Pump both scan subscriptions again instead of
        # treating an aged local callback as a lidar outage. The same bounded
        # wait and freshness limits still reject an actual stopped publisher.
        before_samples = self.scan_samples
        before_time = getattr(self, "latest_scan_monotonic", None)
        self._wait_for_fresh_scan()
        self._record("post_path_scan_refresh", {
            "samples_before": before_samples,
            "samples_after": self.scan_samples,
            "previous_receive_time": before_time,
            "latest_age_s": time.monotonic() - self.latest_scan_monotonic,
        })
        scan = self.latest_scan
        scan_time = getattr(self, "latest_scan_monotonic", None)
        if (scan is None or self.latest_amcl_pose is None
                or scan_time is None
                or time.monotonic() - scan_time > WAYPOINT.BASE.MAXIMUM_SCAN_AGE_S):
            raise SideNearTurnUnsafe("side turn review requires a fresh scan and pose")
        fields = (scan.ranges, scan.angle_min, scan.angle_increment,
                  scan.range_min, scan.range_max)
        side = detect_side_near_obstacle(*fields)
        self._record("side_detection", side)
        if side["status"] != "PASS":
            raise SideNearTurnUnsafe("side obstacle overlaps reviewed chassis margin")
        delta = self.initial_path_heading_delta_rad
        if self.departure_alignment_running:
            target = departure_path_heading(self.path_alignment_poses, self.latest_amcl_pose)
            delta = shortest_heading_delta(target, self.latest_amcl_pose['yaw'])
            self._record('departure_path_alignment', dict(
                target_yaw_rad=target, heading_delta_rad=delta, lookahead_m=.50,
                path=self.path_alignment_poses))
        sharp = delta is not None and abs(delta) >= TURN_REVIEW_MIN_ANGLE_RAD
        if not sharp:
            return
        from rvclaw_nav2_bringup.scan_map_consistency import load_static_map
        from rvclaw_nav2_bringup.scan_trial_policy import evaluate_rotation_clearance
        map_file = _active_map_file()
        grid = load_static_map(map_file)
        rotation_scan = evaluate_rotation_clearance(
            *fields, minimum_radial_m=NOMINAL_ROTATION_RADIUS_M,
            minimum_coverage_deg=330.0,
        )
        rotation_map = check_nominal_static_rotation_disk(
            grid, self.latest_amcl_pose
        )
        review = classify_side_turn_departure(
            delta, side, rotation_scan, rotation_map,
            continuing_egress=self.continuing_egress,
        )
        self.side_turn_review = {
            **review, "side_detection": side,
            "scan_rotation": rotation_scan, "map_rotation": rotation_map,
            "heading_delta_deg": math.degrees(delta),
            "map_file": map_file,
        }
        self._record("side_turn_review", self.side_turn_review)
        if review["status"] == "REQUIRE_STRAIGHT_EGRESS":
            raise SideNearTurnEgressRequired(review["reason"])
        if review["status"] != "NORMAL":
            raise SideNearTurnUnsafe(review["reason"])
        if self.departure_alignment_running or self.waypoint.get("name") == "ORIGIN":
            stop_sweep = check_front_stop_turn_sweep(*fields, delta)
            self._record("departure_front_stop_turn_sweep", stop_sweep)
            if stop_sweep["status"] != "PASS" and stop_sweep.get('actual_stop_status') != 'PASS':
                delta = self._confirm_departure_turn_clear(delta, grid)
            elif stop_sweep["status"] != "PASS":
                # The nominal chassis disk and map have passed above. The
                # exact live STOP rectangle is also clear throughout this
                # turn; only the padded, discretized prediction is occupied.
                # Retain the live collision monitor and all motion guards.
                self._record('departure_turn_buffer_only', stop_sweep)
            if self.departure_alignment_running:
                raise DepartureAlignmentRequired(self.latest_amcl_pose["yaw"] + delta)
            self._record("departure_continuous_navigation", {
                "heading_delta_rad": delta,
                "controller": "FollowPath",
                "separate_spin": False,
            })

    def _confirm_departure_turn_clear(self, delta, grid):
        """Recheck a rejected sweep while already stopped; never reset/arm.

        Keep every original envelope. A transient edge return needs two
        consecutive clear fresh scans; persistent obstruction still fails.
        Healthy departures take the original path without this extra wait.
        """
        from rvclaw_nav2_bringup.scan_trial_policy import evaluate_rotation_clearance
        origin = _finite_pose_copy(self.latest_amcl_pose)
        raw = _finite_pose_copy(self.latest_raw_pose)
        deadline = time.monotonic()+1.2
        last_stamp = None
        consecutive = 0
        attempts = []
        if origin is None or raw is None:
            raise SideNearTurnUnsafe('departure recheck lacks stationary pose evidence')
        while time.monotonic() < deadline:
            feedback_count = self.feedback_samples
            scan_count = self.scan_samples
            localization_count = self.localization_scan_samples
            while time.monotonic() < deadline:
                rclpy.spin_once(self,timeout_sec=.01)
                if (self.feedback_samples > feedback_count and self.scan_samples > scan_count
                        and self.localization_scan_samples > localization_count):
                    break
            if time.monotonic() >= deadline:
                break
            pose = _finite_pose_copy(self.latest_amcl_pose)
            current_raw = _finite_pose_copy(self.latest_raw_pose)
            if (self.armed is not False or self.state != 'DISARMED'
                    or not self.stm32_connected or not self.feedback_connected
                    or self.feedback_samples <= feedback_count
                    or self.latest_scan_monotonic is None
                    or time.monotonic()-self.latest_scan_monotonic > WAYPOINT.BASE.MAXIMUM_SCAN_AGE_S
                    or self.latest_localization_scan_monotonic is None
                    or time.monotonic()-self.latest_localization_scan_monotonic > WAYPOINT.BASE.MAXIMUM_SCAN_AGE_S
                    or pose is None or current_raw is None
                    or max(abs(self.latest_feedback_x),abs(self.latest_feedback_y)) > .01
                    or abs(self.latest_feedback_angular) > .03
                    or math.hypot(current_raw['x']-raw['x'],current_raw['y']-raw['y']) > .01
                    or abs(shortest_heading_delta(current_raw['yaw'],raw['yaw'])) > .03
                    or math.hypot(pose['x']-origin['x'],pose['y']-origin['y']) > .03
                    or abs(shortest_heading_delta(pose['yaw'],origin['yaw'])) > math.radians(3)):
                raise SideNearTurnUnsafe('departure recheck lost stationary hardware confirmation')
            scan = self.latest_scan
            stamp = (scan.header.stamp.sec,scan.header.stamp.nanosec)
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            if time.monotonic() >= deadline:
                break
            fields = (scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)
            side = detect_side_near_obstacle(*fields)
            rotation_scan = evaluate_rotation_clearance(*fields,
                minimum_radial_m=NOMINAL_ROTATION_RADIUS_M, minimum_coverage_deg=330.)
            rotation_map = check_nominal_static_rotation_disk(grid,pose)
            # Retain the absolute path heading if stationary AMCL drifts slightly.
            current_delta = shortest_heading_delta(origin['yaw']+delta,pose['yaw'])
            review = classify_side_turn_departure(current_delta,side,rotation_scan,rotation_map,
                                                  continuing_egress=self.continuing_egress)
            sweep = check_front_stop_turn_sweep(*fields,current_delta)
            clear = review['status']=='NORMAL' and (
                sweep['status']=='PASS' or sweep.get('actual_stop_status')=='PASS')
            consecutive = consecutive+1 if clear else 0
            attempts.append(dict(scan_stamp=stamp,review=review,sweep=sweep))
            self._record('departure_turn_recheck',attempts[-1])
            if consecutive >= 2:
                return current_delta
        raise SideNearTurnUnsafe('departure turn intersects the forward stop envelope')

    def run_navigation(self):
        # Explicit compatibility callers may request a stopped Spin. The
        # inspection route uses continuous FollowPath admission on every leg.
        self.departure_alignment_running = bool(self.align_departure_before_navigation)
        completed_turns = []
        try:
            while True:
                try:
                    return super().run_navigation()
                except DepartureAlignmentRequired as required:
                    if self.armed is not False:
                        raise SideNearTurnUnsafe("departure alignment cannot overlap motion")
                    target_yaw = required.target_yaw
                    delta = shortest_heading_delta(target_yaw,self.latest_amcl_pose['yaw'])
                    if not departure_alignment_attempt_allowed(completed_turns,delta):
                        raise SideNearTurnUnsafe("departure replan exceeds bounded stopped alignment")
                # Each attempt reached this point through fresh path, map,
                # scan and turn-sweep gates. Never fall back to a forward arc.
                self._stop_deadman_keepalive()
                self.publish_deadman(False)
                self.publish_zero_command()
                before_turn = _finite_pose_copy(self.latest_amcl_pose)
                heading = FinalHeadingClient(
                    target_yaw,
                    completion_tolerance_rad=INSPECTION_HEADING_COMPLETION_TOLERANCE_RAD,
                    require_raw_delta_match=False,
                )
                try:
                    result = heading.run_to_absolute_heading()
                except BaseException as exc:
                    if not _heading_failure_is_safely_stopped(heading, exc):
                        heading.emergency_cleanup()
                    heading.capture_final_amcl_observation()
                    self._record('departure_heading_failure', _failure_result(heading, exc))
                    raise
                finally:
                    heading.destroy_node()
                self.departure_alignment_result = result
                completed_turns.append(result)
                self._record("departure_heading_stage", result)
                if result.get("status") != "PASS" or result.get("final_armed") is not False:
                    raise SideNearTurnUnsafe("departure Spin did not finish safely")
                self.continuity_anchor = continuity_anchor_after_departure_turn(
                    self.continuity_anchor, before_turn, result,
                )
                self._record("departure_continuity_handoff", {
                    "before_turn": before_turn,
                    "stopped_pose": result.get("final_amcl_pose"),
                    "continuity_anchor": self.continuity_anchor,
                })
                self.departure_amcl_after_stamp_ns = self.get_clock().now().nanoseconds
                self.latest_amcl_pose = None
                self.latest_amcl_monotonic = None
                self.amcl_samples = 0
                self.latest_scan = None
                self.latest_scan_monotonic = None
                self.scan_samples = 0
                self.latest_localization_scan = None
                self.latest_localization_scan_monotonic = None
                self.localization_scan_samples = 0
                self._refresh_departure_observations(result["final_amcl_pose"])
                self.departure_handoff = dict(at=time.monotonic(),
                    raw=_finite_pose_copy(self.latest_raw_pose),
                    pose=_finite_pose_copy(self.latest_amcl_pose))
                # Replan from the refreshed stopped pose. Permit one small
                # additional pure turn; repeated changes still stop the route.
        finally:
            self.departure_alignment_running = False
            self.departure_handoff = None
            self.navigation_amcl_handoff_verified = False

    def evaluate_navigation_evidence(self, metrics):
        """Position stage deliberately defers the final heading decision."""
        failures = super().evaluate_navigation_evidence(metrics)
        stale_distance = "NavigateToPose final feedback remained too far from goal"
        pose = _finite_pose_copy(self.final_amcl_pose)
        received = self.latest_amcl_monotonic
        # NavigateToPose feedback is periodic, not a terminal measurement.
        # A successful action with a fresh, stopped pose inside 7.5 cm must
        # not be rejected by the previous distance sample (observed: 16.4 cm
        # feedback versus 4.9 cm stopped pose). All other evidence stays.
        if (stale_distance in failures
                and metrics.get("action_status") == GoalStatus.STATUS_SUCCEEDED
                and metrics.get("final_armed") is False
                and self.state == "DISARMED"
                and pose is not None and received is not None
                and 0.0 <= time.monotonic()-received <= .75
                and math.hypot(pose['x']-self.waypoint['x'],
                               pose['y']-self.waypoint['y']) <= .075
                and not metrics.get("motion_map_guard_tripped")):
            failures.remove(stale_distance)
            self._record("terminal_pose_supersedes_periodic_distance", {
                "feedback_distance_m": metrics.get("action_feedback_final_distance_m"),
                "stopped_pose": pose, "maximum_position_error_m": .075,
            })
        return [
            failure
            for failure in failures
            if not failure.startswith("final AMCL yaw error exceeded")
        ]


class OriginPoseWaypointClient(WAYPOINT.WaypointClient):
    """Navigate to ORIGIN with a dedicated checker and bounded final rotation."""

    def __init__(self, waypoint, continuity_anchor=None):
        super().__init__(waypoint)
        self.continuity_anchor = (
            None if continuity_anchor is None else dict(continuity_anchor)
        )
        self.origin_behavior_tree = str(
            Path(get_package_share_directory("rvclaw_nav2_bringup"))
            / "behavior_trees"
            / ORIGIN_FULL_POSE_BT_NAME
        )
        self.origin_rotation_guard_active = False
        self.origin_xy_entered = False
        self.origin_xy_entry_pose = None
        self.origin_rotation_last_yaw = None
        self.origin_post_entry_total_rotation_rad = 0.0
        self.origin_rotation_limit_exceeded = False

    def _check_static_map_localization(self):
        _active_map_file()
        if _check_continuous_route_localization(self, self.continuity_anchor):
            return
        return super()._check_static_map_localization()

    def _validate_origin_behavior_tree(self):
        path = Path(self.origin_behavior_tree)
        if not path.is_file():
            raise RuntimeError(
                "ORIGIN full-pose behavior tree is unavailable: %s" % path
            )
        try:
            tree = ET.parse(path)
        except (OSError, ET.ParseError) as exc:
            raise RuntimeError(
                "ORIGIN full-pose behavior tree is invalid: %s" % exc
            ) from exc
        if tree.findall(".//RateController"):
            raise RuntimeError(
                "ORIGIN behavior tree must not time-replan during terminal rotation"
            )
        distance_controllers = tree.findall(".//DistanceController")
        if len(distance_controllers) != 1:
            raise RuntimeError(
                "ORIGIN behavior tree requires one distance-based replanner"
            )
        try:
            replan_distance_m = float(distance_controllers[0].get("distance"))
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                "ORIGIN distance replanner has an invalid threshold"
            ) from exc
        if not math.isclose(
            replan_distance_m,
            ORIGIN_REPLAN_DISTANCE_M,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise RuntimeError(
                "ORIGIN distance replanner threshold is not the reviewed %.2f m"
                % ORIGIN_REPLAN_DISTANCE_M
            )
        followers = tree.findall(".//FollowPath")
        if not followers or any(
            follower.get("goal_checker_id") != "origin_goal_checker"
            for follower in followers
        ):
            raise RuntimeError(
                "ORIGIN behavior tree does not select origin_goal_checker"
            )

    def _configure_navigation_goal(self, goal):
        self._validate_origin_behavior_tree()
        goal.behavior_tree = self.origin_behavior_tree

    def _pre_arm_gate(self, check_localization: bool = True):
        super()._pre_arm_gate(check_localization=check_localization)
        self._validate_origin_behavior_tree()

    def _start_origin_rotation_guard(self):
        self.origin_rotation_guard_active = True
        self.origin_xy_entered = False
        self.origin_xy_entry_pose = None
        self.origin_rotation_last_yaw = None
        self.origin_post_entry_total_rotation_rad = 0.0
        self.origin_rotation_limit_exceeded = False
        self._update_origin_rotation_guard()

    def _update_origin_rotation_guard(self):
        if not getattr(self, "origin_rotation_guard_active", False):
            return
        pose = getattr(self, "latest_amcl_pose", None)
        if pose is None:
            return
        goal = getattr(self, "goal_pose", None) or self.waypoint
        distance_m = math.hypot(pose["x"] - goal["x"], pose["y"] - goal["y"])
        if not self.origin_xy_entered:
            if distance_m > ORIGIN_GOAL_XY_TOLERANCE_M:
                return
            self.origin_xy_entered = True
            self.origin_xy_entry_pose = dict(pose)
            self.origin_rotation_last_yaw = pose["yaw"]
            return

        previous_yaw = self.origin_rotation_last_yaw
        self.origin_rotation_last_yaw = pose["yaw"]
        if previous_yaw is None:
            return
        delta = abs(shortest_heading_delta(pose["yaw"], previous_yaw))
        self.origin_post_entry_total_rotation_rad += delta
        if (
            self.origin_post_entry_total_rotation_rad
            > ORIGIN_POST_ENTRY_MAX_ROTATION_RAD
        ):
            self.origin_rotation_limit_exceeded = True

    def _on_amcl(self, message):
        super()._on_amcl(message)
        self._update_origin_rotation_guard()

    def _wait_navigation_result(self, future, timeout_s: float):
        """Fail-stop if terminal heading control accumulates more than 210 deg."""
        self._start_origin_rotation_guard()
        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=WAYPOINT.BASE.INTERVAL_S)
                if getattr(self, "motion_map_guard_tripped", False):
                    raise RuntimeError("in-motion scan/map localization disagreement; stopped")
                if self.origin_rotation_limit_exceeded:
                    details = {
                        "xy_entered": self.origin_xy_entered,
                        "xy_entry_pose": self.origin_xy_entry_pose,
                        "total_rotation_rad": self.origin_post_entry_total_rotation_rad,
                        "total_rotation_deg": math.degrees(
                            self.origin_post_entry_total_rotation_rad
                        ),
                        "limit_rad": ORIGIN_POST_ENTRY_MAX_ROTATION_RAD,
                        "limit_deg": math.degrees(
                            ORIGIN_POST_ENTRY_MAX_ROTATION_RAD
                        ),
                    }
                    self._record("origin_rotation_guard_exceeded", details)
                    raise RuntimeError(
                        "ORIGIN post-entry rotation exceeded %.1f deg safety limit"
                        % math.degrees(ORIGIN_POST_ENTRY_MAX_ROTATION_RAD)
                    )
                if future.done():
                    return future.result()
                if self.armed is False or self.state in (
                    "WATCHDOG_LATCHED",
                    "ESTOP_LATCHED",
                ):
                    raise RuntimeError(
                        "hardware safety stop during ORIGIN NavigateToPose "
                        "(state=%s reason=%s)"
                        % (self.state, self.transition_reason)
                    )
            raise RuntimeError("timed out waiting for ORIGIN NavigateToPose result")
        finally:
            self.origin_rotation_guard_active = False

    def result(self, status, error):
        result = super().result(status, error)
        result.update(
            {
                "origin_behavior_tree": ORIGIN_FULL_POSE_BT_NAME,
                "origin_goal_xy_tolerance_m": ORIGIN_GOAL_XY_TOLERANCE_M,
                "origin_goal_yaw_tolerance_rad": ORIGIN_GOAL_YAW_TOLERANCE_RAD,
                "origin_xy_entered": self.origin_xy_entered,
                "origin_xy_entry_pose": self.origin_xy_entry_pose,
                "origin_post_entry_total_rotation_rad": (
                    self.origin_post_entry_total_rotation_rad
                ),
                "origin_post_entry_total_rotation_deg": math.degrees(
                    self.origin_post_entry_total_rotation_rad
                ),
                "origin_post_entry_max_rotation_rad": (
                    ORIGIN_POST_ENTRY_MAX_ROTATION_RAD
                ),
                "origin_rotation_limit_exceeded": (
                    self.origin_rotation_limit_exceeded
                ),
            }
        )
        return result


class SafeEgressClient(WAYPOINT.BASE.Nav2ForwardGoalClient):
    """Move a short distance forward to clear a historical lethal start.

    This is intentionally a separate action from NavigateToPose.  It never
    reverses, strafes, or spins, and it fails closed on every fresh-scan
    violation.  After the motion it forces fresh AMCL samples and an
    independent static-map match before the route is retried.
    """

    COLLISION_PROFILE_PARAMETERS = (
        "front_stop.enabled",
        "front_slow.enabled",
        "side_near_stop.enabled",
    )

    def __init__(self, *, prefer_shortest=False, turn_delta_rad=None):
        super().__init__()
        self.prefer_shortest_egress = bool(prefer_shortest)
        self.egress_turn_delta_rad = turn_delta_rad
        self.egress_action = ActionClient(
            self,
            DriveOnHeading,
            "/drive_on_heading",
        )
        self.egress_feedback_samples = 0
        self.egress_feedback_distance_m = 0.0
        self.egress_feedback_invalid = False
        self.egress_runtime_scan_checks = 0
        self.egress_gate = None
        self.egress_distance_m = None
        self.egress_mode = EGRESS_MODE_STANDARD
        self.egress_start_raw_pose = None
        self.egress_final_raw_pose = None
        self.egress_start_amcl_pose = None
        self.egress_final_amcl_pose = None
        self.collision_profile_active = False
        self.collision_profile_restored = False
        self.collision_profile_before = None
        self.collision_profile_after = None
        self.collision_get_client = self.create_client(
            GetParameters, "/collision_monitor/get_parameters"
        )
        self.collision_set_client = self.create_client(
            SetParametersAtomically, "/collision_monitor/set_parameters_atomically"
        )

    def _collision_parameters(self):
        return self.COLLISION_PROFILE_PARAMETERS

    def _read_collision_profile(self):
        if not self.collision_get_client.wait_for_service(
            timeout_sec=WAYPOINT.BASE.SERVICE_TIMEOUT_S
        ):
            raise RuntimeError("collision monitor parameter service is unavailable")
        request = GetParameters.Request()
        request.names = list(self._collision_parameters())
        future = self.collision_get_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=WAYPOINT.BASE.SERVICE_TIMEOUT_S
        )
        if not future.done() or future.result() is None:
            raise RuntimeError("collision monitor parameter read timed out")
        values = future.result().values
        if len(values) != len(request.names):
            raise RuntimeError("collision monitor parameter read was incomplete")
        profile = {}
        for name, value in zip(request.names, values):
            if value.type != 1:
                raise RuntimeError("collision monitor parameter %s is not boolean" % name)
            profile[name] = bool(value.bool_value)
        return profile

    def _write_collision_profile(self, profile):
        if not self.collision_set_client.wait_for_service(
            timeout_sec=WAYPOINT.BASE.SERVICE_TIMEOUT_S
        ):
            raise RuntimeError("collision monitor atomic parameter service is unavailable")
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=bool(profile[name])).to_parameter_msg()
            for name in self._collision_parameters()
        ]
        future = self.collision_set_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=WAYPOINT.BASE.SERVICE_TIMEOUT_S
        )
        if not future.done() or future.result() is None:
            raise RuntimeError("collision monitor atomic parameter write timed out")
        response = future.result()
        if not response.result.successful:
            raise RuntimeError(
                "collision monitor rejected profile: %s" % response.result.reason
            )
        observed = self._read_collision_profile()
        expected = {name: bool(profile[name]) for name in self._collision_parameters()}
        if observed != expected:
            raise RuntimeError("collision monitor profile readback mismatch")
        return observed

    def _enter_side_near_profile(self):
        before = self._read_collision_profile()
        expected = {
            "front_stop.enabled": True,
            "front_slow.enabled": True,
            "side_near_stop.enabled": False,
        }
        if before != expected:
            raise RuntimeError(
                "collision monitor was not in the ordinary safe profile before side-near mode"
            )
        self.collision_profile_before = before
        try:
            after = self._write_collision_profile(
                {
                    **before,
                    "front_stop.enabled": False,
                    "front_slow.enabled": False,
                    "side_near_stop.enabled": True,
                }
            )
        except BaseException:
            # Atomic writes should not partially apply, but fail closed if a
            # service reports an error after changing the profile.
            try:
                self._write_collision_profile(before)
            except BaseException as restore_exc:
                self.publish_estop(True)
                self._record("collision_profile_restore_error", str(restore_exc))
            raise
        self.collision_profile_after = after
        self.collision_profile_active = True
        self._record("collision_profile_entered", after)

    def _restore_collision_profile(self):
        if self.collision_profile_before is None:
            return
        if not self.collision_profile_active and self.collision_profile_restored:
            return
        profile = self.collision_profile_before or {
            "front_stop.enabled": True,
            "front_slow.enabled": True,
            "side_near_stop.enabled": False,
        }
        restored = self._write_collision_profile(profile)
        self.collision_profile_restored = True
        self.collision_profile_active = False
        self._record("collision_profile_restored", restored)

    def emergency_cleanup(self):
        cleanup_error = None
        try:
            super().emergency_cleanup()
        finally:
            try:
                self._restore_collision_profile()
            except BaseException as exc:
                cleanup_error = exc
                self.publish_estop(True)
                self._record("collision_profile_restore_error", str(exc))
        if cleanup_error is not None:
            raise RuntimeError(
                "collision monitor profile restoration failed: %s" % cleanup_error
            )

    def _on_egress_feedback(self, message):
        value = float(message.feedback.distance_traveled)
        if math.isfinite(value) and value >= 0.0:
            self.egress_feedback_samples += 1
            self.egress_feedback_distance_m = value
        else:
            self.egress_feedback_invalid = True
            self._record(
                "egress_feedback_invalid",
                {"distance_traveled": repr(value)},
            )

    def _wait_egress_result(self, future):
        deadline = time.monotonic() + EGRESS_RESULT_TIMEOUT_S
        previous_scan_samples = self.scan_samples
        response = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=WAYPOINT.BASE.INTERVAL_S)
            if self.egress_feedback_invalid:
                self._cancel_goal()
                raise RuntimeError(
                    "safe egress received invalid DriveOnHeading feedback"
                )
            if self.scan_samples > previous_scan_samples:
                previous_scan_samples = self.scan_samples
                if self.latest_scan_monotonic is None:
                    raise RuntimeError("egress received a scan without a timestamp")
                scan_age = time.monotonic() - self.latest_scan_monotonic
                remaining_distance_m = remaining_egress_distance(
                    self.egress_distance_m,
                    self.egress_feedback_distance_m,
                )
                scan_gate = evaluate_front_egress_scan(
                    self.latest_scan.ranges,
                    self.latest_scan.angle_min,
                    self.latest_scan.angle_increment,
                    self.latest_scan.range_min,
                    self.latest_scan.range_max,
                    distance_m=remaining_distance_m,
                    mode=self.egress_mode,
                )
                runtime = evaluate_runtime_egress_monitor(
                    scan_gate,
                    scan_age,
                    maximum_scan_age_s=EGRESS_RUNTIME_MAXIMUM_SCAN_AGE_S,
                )
                runtime["selected_distance_m"] = self.egress_distance_m
                runtime["distance_traveled_m"] = self.egress_feedback_distance_m
                runtime["remaining_distance_m"] = remaining_distance_m
                self.egress_runtime_scan_checks += 1
                self._record("egress_runtime_scan", runtime)
                if runtime["status"] != "PASS":
                    self._cancel_goal()
                    raise RuntimeError(
                        "safe egress runtime scan gate failed: %s"
                        % "; ".join(runtime["failures"])
                    )
            if self.armed is False or self.state in (
                "WATCHDOG_LATCHED",
                "ESTOP_LATCHED",
            ):
                raise RuntimeError(
                    "hardware safety stop during safe egress (state=%s reason=%s)"
                    % (self.state, self.transition_reason)
                )
            if future.done():
                response = future.result()
                if self.egress_runtime_scan_checks >= EGRESS_RUNTIME_MINIMUM_SCAN_CHECKS:
                    return response
        raise RuntimeError("timed out waiting for safe egress result")

    def run_egress(self):
        self.wait_for(
            lambda: self.transport_attached is True,
            WAYPOINT.BASE.STARTUP_TIMEOUT_S,
            "hardware transport attachment for safe egress",
        )
        self.wait_for(
            lambda: (
                self.feedback_connected is True
                and self.stm32_connected is True
                and self.voltage_v is not None
                and self.voltage_v >= WAYPOINT.BASE.MIN_BATTERY_VOLTAGE_V
                and self.latest_raw_pose is not None
            ),
            WAYPOINT.BASE.STARTUP_TIMEOUT_S,
            "fresh feedback and raw odometry for safe egress",
        )
        # /amcl_pose is volatile and AMCL may not publish while the chassis is
        # stationary.  A newly created egress client must therefore request a
        # no-motion update before requiring a pose from its own subscription.
        self._request_amcl_nomotion_update()
        self.wait_for(
            lambda: self.latest_amcl_pose is not None,
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,
            "fresh AMCL pose for safe egress",
        )
        self._wait_for_fresh_scan()
        self.egress_gate = _evaluate_node_egress_gates(self)
        self._record("egress_preflight_gate", self.egress_gate)
        if self.egress_gate["status"] != "PASS":
            raise RuntimeError(
                "safe egress preflight gate failed: %s"
                % "; ".join(self.egress_gate["failures"])
            )
        self.egress_distance_m = float(
            self.egress_gate["selected_distance_m"]
        )
        self.egress_mode = self.egress_gate.get("mode", EGRESS_MODE_STANDARD)
        if self.egress_mode == EGRESS_MODE_SIDE_NEAR and not SIDE_NEAR_EGRESS_FIELD_CERTIFIED:
            raise RuntimeError("side-near escape is not field certified")
        if self.egress_mode == EGRESS_MODE_SIDE_NEAR:
            self._enter_side_near_profile()
        if not self.egress_action.wait_for_server(
            timeout_sec=WAYPOINT.BASE.STARTUP_TIMEOUT_S
        ):
            raise RuntimeError("DriveOnHeading action server is unavailable")
        if self.armed:
            raise RuntimeError("hardware controller unexpectedly started armed")

        self.publish_estop(False)
        self._call_reset()
        self.wait_for(
            lambda: self.state == "DISARMED",
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,
            "DISARMED state after safe egress reset",
        )
        deadline = time.monotonic() + WAYPOINT.BASE.DEADMAN_PRIME_S
        while time.monotonic() < deadline:
            self.publish_deadman(True)
            rclpy.spin_once(self, timeout_sec=WAYPOINT.BASE.INTERVAL_S)
        self._start_deadman_keepalive()
        self._call_arm(True)
        self.wait_for(
            lambda: self.armed is True,
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,
            "armed state for safe egress",
        )

        self.egress_start_raw_pose = dict(self.latest_raw_pose)
        self.egress_start_amcl_pose = dict(self.latest_amcl_pose)
        self.egress_feedback_invalid = False
        goal = DriveOnHeading.Goal()
        goal.target.x = self.egress_distance_m
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(EGRESS_SPEED_M_S)
        goal.time_allowance.sec = int(EGRESS_ACTION_TIME_ALLOWANCE_S)
        goal.time_allowance.nanosec = 0
        send_future = self.egress_action.send_goal_async(
            goal,
            feedback_callback=self._on_egress_feedback,
        )
        self.goal_handle = self._wait_future_with_deadman(
            send_future,
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,
            "DriveOnHeading goal acceptance",
        )
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError("DriveOnHeading safe egress goal was rejected")
        self._record("egress_goal_accepted", True)
        result_response = self._wait_egress_result(
            self.goal_handle.get_result_async()
        )
        action_status = int(result_response.status)
        self._record("egress_action_status", action_status)
        if action_status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                "DriveOnHeading safe egress did not report success (status=%d)"
                % action_status
            )
        for _ in range(WAYPOINT.BASE.POST_ACTION_ZERO_FRAMES):
            self.publish_deadman(True)
            self.publish_zero_command()
            rclpy.spin_once(self, timeout_sec=WAYPOINT.BASE.INTERVAL_S)
        self._call_arm(False)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.wait_for(
            lambda: self.armed is False
            and self.state in ("DISARMED", "WATCHDOG_LATCHED", "ESTOP_LATCHED"),
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,
            "disarmed state after safe egress",
        )
        self._wait_for_stable_feedback_stop()
        self.egress_final_raw_pose = dict(self.latest_raw_pose)
        self.egress_final_amcl_pose = dict(self.latest_amcl_pose)

        # Motion invalidates the old localization cache.  Requiring fresh
        # AMCL samples here prevents the retry from planning from stale pose.
        self.localization_gate_scan_sample_count = -1
        self.localization_gate_result = None
        self.localization_gate_pose = None
        self._request_amcl_nomotion_update()
        self._wait_for_fresh_scan()
        self._check_static_map_localization()
        self._restore_collision_profile()
        return {
            "status": "PASS",
            "distance_m": self.egress_distance_m,
            "speed_m_s": EGRESS_SPEED_M_S,
            "runtime_scan_checks": self.egress_runtime_scan_checks,
            "feedback_samples": self.egress_feedback_samples,
            "feedback_distance_m": self.egress_feedback_distance_m,
            "preflight_gate": self.egress_gate,
            "start_raw_pose": self.egress_start_raw_pose,
            "final_raw_pose": self.egress_final_raw_pose,
            "start_amcl_pose": self.egress_start_amcl_pose,
            "final_amcl_pose": self.egress_final_amcl_pose,
            "relocalization_gate": self.localization_gate_result,
            "collision_profile_before": self.collision_profile_before,
            "collision_profile_after": self.collision_profile_after,
            "collision_profile_restored": self.collision_profile_restored,
            "mode": self.egress_mode,
            "final_armed": self.armed,
            "final_state": self.state,
        }


def exception_diagnostics(exc):
    """Return a non-empty, structured representation of any exception."""
    exception_type = type(exc).__name__
    message = str(exc)
    error = message if message else exception_type
    return {
        "error": error,
        "error_type": exception_type,
        "error_repr": repr(exc),
        "traceback": "".join(
            traceback.format_exception(type(exc), exc, exc.__traceback__)
        ),
    }


def _failure_result(node, exc):
    diagnostic = exception_diagnostics(exc)
    result = node.result("FAIL", diagnostic["error"])
    result.update(diagnostic)
    result["departure_diagnostics"] = _departure_diagnostics(node)
    return result


def _heading_failure_is_safely_stopped(node, exc):
    """Preserve verified stopped failures without manufacturing an emergency."""
    timeout = isinstance(exc, HEADING.StoppedHeadingControlTimeout)
    if not timeout and not isinstance(exc, StoppedHeadingLocalizationChanged):
        return False
    try:
        if timeout and isinstance(node, HEADING.Nav2HeadingClient):
            return node.cancel_after_control_timeout()
        # A parent navigation node may still have the pre-turn DDS state queued.
        node.wait_for(lambda: node.armed is False and node.state == 'DISARMED',
                      HEADING.SERVICE_TIMEOUT_S, 'stopped heading handoff')
        node._wait_for_stable_feedback_stop()
    except Exception:
        return False
    return (node.armed is False and node.state == 'DISARMED'
            and node.transport_attached is True
            and node.feedback_connected is True and node.stm32_connected is True)


def strong_uncontested_stopped_scan(gate):
    """Missing competitor is not ambiguity when the known pose fits tightly."""
    if (gate.get('status')!='PASS' or gate.get('uniqueness',{}).get('failures')!=[
            'stationary scan did not produce a distinct second match']):
        return False
    candidate=gate.get('candidate',{})
    comparison=gate.get('comparison',{})
    ray=gate.get('ray_consistency',{})
    values=[candidate.get('mean_likelihood'),candidate.get('known_endpoint_ratio'),
            comparison.get('candidate_to_best_ratio'),comparison.get('candidate_to_best_distance_m'),
            comparison.get('candidate_to_best_yaw_deg'),ray.get('conflicting_ray_ratio')]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        return False
    score,known,ratio,distance,yaw,conflict=values
    return (score>=.90 and known>=.90 and ratio>=.98 and 0<=distance<=.05
            and 0<=yaw<=3. and ray.get('status')=='PASS' and 0<=conflict<=.10)


def stopped_scan_with_free_ray_uniqueness(gate):
    """Apply the global matcher's existing joint evidence to a stopped proof.

    Endpoint ties need the measured free space as well. Keep the original
    winner, all pose/ray limits, and the existing static uniqueness thresholds.
    A different free-space winner cannot authorize a new pose here.
    """
    from rvclaw_nav2_bringup.scan_map_consistency import evaluate_static_match_gate
    if gate.get('status') != 'PASS':
        return gate
    matches=[]
    for candidate in gate.get('best_matches', []):
        ray=candidate.get('ray_consistency') or {}
        score=candidate.get('mean_likelihood')
        conflict=ray.get('conflicting_ray_ratio')
        unknown=ray.get('unknown_sample_ratio')
        if (ray.get('status') != 'PASS'
                or not all(isinstance(v,(int,float)) and math.isfinite(v)
                           for v in (score,conflict,unknown))
                or not 0.<=score<=1. or not 0.<=conflict<=1. or not 0.<=unknown<=1.):
            return gate
        matches.append(dict(candidate,ray_joint_likelihood=score*(1.-conflict)*(1.-unknown)))
    if len(matches)<2:
        return gate
    uniqueness=evaluate_static_match_gate(dict(matches=matches))
    return dict(gate,uniqueness=uniqueness)


def _departure_diagnostics(node):
    """Keep the actual preflight reason even when egress creates a new node."""
    fields = (
        "side_turn_review", "initial_path_heading_delta_rad",
        "path_departure_poses", "path_check_status", "path_check_pose_count",
        "path_alignment_poses",
        "path_check_error", "start_cost", "start_clearance_mode",
        "pre_arm_front_clearance_m", "latest_amcl_pose", "rotation_clearance",
    )
    return {field: getattr(node, field, None) for field in fields}


def _finite_pose_copy(value):
    if not isinstance(value, dict):
        return None
    try:
        pose = dict(value)
        for key in ("x", "y", "yaw"):
            pose[key] = float(pose[key])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(pose[key]) for key in ("x", "y", "yaw")):
        return None
    return pose


def continuity_anchor_after_departure_turn(anchor, before_turn, result):
    """Account only for a verified, disarmed, position-preserving Spin."""
    before = _finite_pose_copy(before_turn)
    after = _finite_pose_copy(result.get("final_amcl_pose"))
    try:
        linear = [float(result[key]) for key in (
            "max_guarded_linear_x_m_s", "max_guarded_linear_y_m_s",
        )]
        error = float(result["final_amcl_error_rad"])
    except (KeyError, TypeError, ValueError) as exc:
        raise SideNearTurnUnsafe("departure Spin lacks stopped motion evidence") from exc
    if (result.get("status") != "PASS" or result.get("final_armed") is not False
            or result.get("final_state") != "DISARMED"
            or before is None or after is None
            or not all(math.isfinite(v) and abs(v) <= WAYPOINT.BASE.ZERO_EPSILON for v in linear)
            or not math.isfinite(error)
            or abs(error) > INSPECTION_HEADING_COMPLETION_TOLERANCE_RAD
            or math.hypot(after["x"] - before["x"], after["y"] - before["y"])
            > CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M):
        raise SideNearTurnUnsafe("departure Spin did not preserve a verified stopped position")
    # An absent anchor still requires the ordinary independent map gate.
    if anchor is None:
        return None
    previous = _finite_pose_copy(anchor)
    if (previous is None
            or math.hypot(before["x"] - previous["x"], before["y"] - previous["y"])
            > CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M
            or abs(WAYPOINT.BASE.normalize_angle(before["yaw"] - previous["yaw"]))
            > CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD):
        raise SideNearTurnUnsafe("departure Spin began outside the continuity anchor")
    return after


def update_continuity_anchor_after_task(continuity_anchor, task_result):
    """Apply a verified stopped inspection pose to the route continuity anchor."""

    anchor = _finite_pose_copy(continuity_anchor)
    if anchor is None or not isinstance(task_result, dict):
        return anchor

    if task_result.get("final_armed") is False:
        task_pose = _finite_pose_copy(task_result.get("final_amcl_pose"))
        if task_pose is not None:
            return task_pose
        try:
            task_yaw = float(task_result.get("final_heading_rad"))
        except (TypeError, ValueError):
            task_yaw = math.nan
        if math.isfinite(task_yaw):
            anchor["yaw"] = WAYPOINT.BASE.normalize_angle(task_yaw)
            return anchor

    for attempt in reversed(task_result.get("heading_attempts") or []):
        if not isinstance(attempt, dict) or attempt.get("final_armed") is not False:
            continue
        attempt_pose = _finite_pose_copy(attempt.get("final_amcl_pose"))
        if attempt_pose is not None:
            return attempt_pose
        try:
            attempt_yaw = float(attempt.get("final_amcl_yaw_rad"))
        except (TypeError, ValueError):
            continue
        if math.isfinite(attempt_yaw):
            anchor["yaw"] = WAYPOINT.BASE.normalize_angle(attempt_yaw)
            return anchor
    return anchor


def continuity_anchor_from_results(results):
    """Recover the last stopped pose when continuing a checkpointed route."""

    for result in reversed(results or []):
        if not isinstance(result, dict) or result.get("status") != "PASS":
            continue
        anchor = _finite_pose_copy(result.get("final_amcl_pose"))
        if anchor is None:
            continue
        return update_continuity_anchor_after_task(
            anchor,
            result.get("inspection_task"),
        )
    return None


def route_start_continuity_anchor(initial_results, resume_localization_verified):
    """Select the first segment anchor for a new or resumed route invocation."""

    if resume_localization_verified:
        return {VERIFIED_RESUME_REBASE_KEY: True}
    return continuity_anchor_from_results(initial_results)


def heading_correction_delta(remaining, response_gain=1.0, *, correcting=False):
    amount = abs(remaining)
    # Do not deliberately undershoot or insert fixed .45-rad stops. A changed
    # stopped map heading may still require a bounded corrective action.
    amount = amount / max(1.0, min(3.0, response_gain))
    if correcting or response_gain > 1.0:
        # Leave room for delayed map updates and coast on corrective turns.
        # The first continuous turn is unchanged; final acceptance is unchanged.
        amount *= .70
    return math.copysign(min(amount, ORIGIN_HEADING_SPIN_CHUNK_MAX_RAD), remaining)


def measured_heading_turn_complete(target, feedback, raw, feedback_age, raw_age):
    """Agreeing fresh odometry/action progress, within at most .01 rad."""
    values = (target, feedback, raw, feedback_age, raw_age)
    if not all(v is not None and math.isfinite(v) for v in values):
        return False
    if abs(target) <= WAYPOINT.BASE.ZERO_EPSILON:
        return False
    tolerance = min(.01, .05 * abs(target))
    return (0. <= feedback_age <= .15 and 0. <= raw_age <= .15
            and target * feedback > 0. and target * raw > 0.
            and abs(feedback - target) <= tolerance
            and abs(raw - target) <= tolerance
            and abs(feedback - raw) <= .02)


class StoppedHeadingLocalizationChanged(RuntimeError):
    """A completed, disarmed turn changed its map frame; verify before moving."""


def stopped_heading_frame_change(before, after, raw_delta):
    before, after = _finite_pose_copy(before), _finite_pose_copy(after)
    if before is None or after is None or raw_delta is None or not math.isfinite(raw_delta):
        return None
    translation = math.hypot(after['x']-before['x'], after['y']-before['y'])
    yaw_change = shortest_heading_delta(after['yaw'], before['yaw'])
    yaw_disagreement = abs(shortest_heading_delta(yaw_change, raw_delta))
    if (translation <= WAYPOINT.MAX_FINAL_POSITION_ERROR_M
            and yaw_disagreement <= ORIGIN_HEADING_TOLERANCE_RAD):
        return None
    return dict(translation_m=translation, map_yaw_delta_rad=yaw_change,
                raw_yaw_delta_rad=raw_delta, yaw_disagreement_rad=yaw_disagreement)


def map_heading_stop_observation(start, pose, target_yaw, requested, raw_progress,
                                 pose_stamp_ns, now_ns, history, angular_velocity):
    """Project a fresh map yaw using bracketed measured odometry; stop only.

    Count coherent independent map observations during the approach, rather
    than requiring two callbacks to land inside a short endpoint window.
    A later stopped absolute-map check still owns completion and correction.
    """
    if (start is None or pose is None or requested is None or raw_progress is None
            or not all(math.isfinite(v) for v in
                       (target_yaw, requested, raw_progress, angular_velocity))
            or not all(math.isfinite(pose.get(k, math.nan)) for k in
                       ('x','y','yaw','cov_x','cov_y','cov_yaw'))
            or not all(math.isfinite(start.get(k, math.nan)) for k in ('x','y','yaw'))
            or not all(0<=pose[k]<=limit for k,limit in
                       (('cov_x',.05),('cov_y',.05),('cov_yaw',.02)))
            or math.hypot(pose['x']-start['x'],pose['y']-start['y'])
               > WAYPOINT.MAX_FINAL_POSITION_ERROR_M
            or abs(requested)<=.35 or requested*raw_progress<=0
            or not .5*abs(requested)<=abs(raw_progress)<=abs(requested)+.21
            or pose_stamp_ns<=0 or not 0<=now_ns-pose_stamp_ns<=350000000
            or not history.samples):
        return None
    current_stamp, current_raw = history.samples[-1]
    if (not 0<=now_ns-current_stamp<=150000000
            or not 0<=current_stamp-pose_stamp_ns<=250000000):
        return None
    measured_at_pose = history.at(pose_stamp_ns)
    if measured_at_pose is None:
        return None  # No extrapolation across missing or reset wheel history.
    delta = shortest_heading_delta(current_raw['yaw'],measured_at_pose['yaw'])
    at_pose_progress = raw_progress-delta
    map_progress = shortest_heading_delta(pose['yaw'],start['yaw'])
    gain = 1.
    if abs(at_pose_progress)>.30 and at_pose_progress*map_progress>0:
        gain = max(1.,min(2.5,abs(map_progress/at_pose_progress)))
    projected_yaw = math.atan2(math.sin(pose['yaw']+gain*delta),
                               math.cos(pose['yaw']+gain*delta))
    return dict(pose_stamp_ns=pose_stamp_ns,odom_stamp_ns=current_stamp,
        pose_age_s=(now_ns-pose_stamp_ns)/1e9,
        odom_age_s=(now_ns-current_stamp)/1e9,
        measured_yaw_delta_rad=delta,measured_response_gain=gain,
        projected_yaw_rad=projected_yaw,
        remaining_rad=math.copysign(1.,requested)*shortest_heading_delta(target_yaw,projected_yaw),
        stopping_lead_rad=min(.10,abs(angular_velocity)*gain*.10))


class FinalHeadingClient(HEADING.Nav2HeadingClient):
    """Run one bounded relative Spin to an absolute map-frame heading."""

    def __init__(
        self,
        target_yaw: float,
        *,
        completion_tolerance_rad: float = ORIGIN_HEADING_SKIP_TOLERANCE_RAD,
        require_raw_delta_match: bool = True,
        accept_terminal_action_failure: bool = False,
        max_direction_reversals: int = 2,
        verify_stopped_map_frame: bool = False,
        stopped_verifier=None,
    ):
        super().__init__()
        self.target_yaw = WAYPOINT.BASE.normalize_angle(target_yaw)
        self.completion_tolerance_rad = float(completion_tolerance_rad)
        if (
            not math.isfinite(self.completion_tolerance_rad)
            or self.completion_tolerance_rad <= 0.0
            or self.completion_tolerance_rad > ORIGIN_HEADING_TOLERANCE_RAD
        ):
            raise ValueError("invalid final-heading completion tolerance")
        self.require_raw_delta_match = bool(require_raw_delta_match)
        self.accept_terminal_action_failure = bool(accept_terminal_action_failure)
        self.max_direction_reversals = int(max_direction_reversals)
        self.verify_stopped_map_frame = bool(verify_stopped_map_frame)
        self.stopped_verifier = stopped_verifier
        if self.max_direction_reversals < 0:
            raise ValueError("max direction reversals must be non-negative")
        self.terminal_action_failure_accepted = False
        self.amcl_nomotion_client = self.create_client(
            Empty,
            "/request_nomotion_update",
        )
        self.latest_amcl_yaw = None
        self.amcl_samples = 0
        self.requested_delta_yaw = None
        self.skipped = False
        self.spin_chunks = []
        self.overall_start_raw_yaw = None
        self.final_amcl_yaw = None
        self.final_amcl_error_rad = None
        self.latest_heading_amcl_pose = None
        self.final_heading_amcl_pose = None
        self.monitor_output_samples = 0
        self.latest_monitor_command_monotonic = None
        self.spin_command_samples = 0
        self.latest_spin_command_monotonic = None
        self.expected_spin_delta = None
        self.turn_raw_progress = None
        self.latest_turn_feedback_monotonic = None
        self.turn_stop_requested = False
        self.turn_cancel_future = None
        self.turn_measured_completion = False
        self.turn_scan_latest = None
        self.turn_scan_reference = None
        self.turn_scan_pending = None
        self.turn_scan_worker = None
        self.turn_scan_last_stamp = 0
        self.turn_scan_matches = 0
        self.turn_scan_previous = None
        self.turn_scan_stop_requested = False
        self.turn_scan_evidence = None
        self.turn_map_stop_requested = False
        self.turn_map_confirmations = 0
        self.turn_map_scan_delta = None
        self.turn_map_quality_stamp_ns = 0
        self.turn_odometry_history = WAYPOINT.BASE.ScanOdometryHistory()
        self.latest_heading_amcl_stamp_ns = 0
        self.create_subscription(LaserScan, '/scan_localization', self._on_turn_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_final_amcl,
            AMCL_QOS,
        )
        self.create_subscription(
            Twist,
            "/rvclaw/nav2_cmd_vel",
            self._on_monitor_command,
            10,
        )

    def _on_raw_odometry(self, message):
        previous = self.latest_raw_yaw
        super()._on_raw_odometry(message)
        if self.turn_raw_progress is not None and previous is not None:
            self.turn_raw_progress += shortest_heading_delta(self.latest_raw_yaw, previous)
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        pose = message.pose.pose
        self.turn_odometry_history.add(stamp,dict(
            x=float(pose.position.x),y=float(pose.position.y),yaw=self.latest_raw_yaw),
            (message.header.frame_id,message.child_frame_id))

    def destroy_node(self):
        if self.turn_scan_worker is not None:
            self.turn_scan_worker.shutdown(wait=True, cancel_futures=True)
            self.turn_scan_worker = None
        return super().destroy_node()

    def _on_turn_scan(self, message):
        self.turn_scan_latest = message
        if (self.turn_scan_reference is None or self.armed is not True
                or self.turn_stop_requested or self.turn_scan_pending is not None):
            return
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= self.turn_scan_last_stamp or not 0 <= age <= .20:
            return
        from rvclaw_nav2_bringup.turn_scan_reference import endpoints
        try:
            points = endpoints(message)
        except ValueError:
            return
        self.turn_scan_last_stamp = stamp
        self.turn_scan_pending = (self.turn_scan_worker.submit(
            self.turn_scan_reference.match, points, self.turn_raw_progress),
            stamp, time.monotonic())

    def _prepare_turn_scan_reference(self):
        self.turn_scan_reference = None
        self.turn_scan_matches = 0
        self.turn_scan_previous = None
        self.turn_scan_stop_requested = False
        self.turn_scan_evidence = None
        if self.require_raw_delta_match:
            return
        # No extra process survives a turn. One worker, one pending sample;
        # hardware/deadman callbacks never wait for scan fitting.
        from concurrent.futures import ThreadPoolExecutor
        from rvclaw_nav2_bringup.turn_scan_reference import TurnScanReference, endpoints
        message = self.turn_scan_latest
        deadline = time.monotonic()+.30
        while self.armed is False and self.state == 'DISARMED':
            stamp = (0 if message is None else
                     message.header.stamp.sec*10**9+message.header.stamp.nanosec)
            if (message is not None and
                    0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= .25):
                break
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=.01)
            message = self.turn_scan_latest
        if message is None:
            self._record('turn_scan_reference_unavailable','no full-view scan')
            return
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if not 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= .25:
            self._record('turn_scan_reference_unavailable','stale full-view scan')
            return
        try:
            self.turn_scan_reference = TurnScanReference(endpoints(message,maximum=720))
        except ValueError:
            self._record('turn_scan_reference_unavailable','insufficient full-view geometry')
            return
        if self.turn_scan_worker is None:
            self.turn_scan_worker = ThreadPoolExecutor(max_workers=1,thread_name_prefix='turn-scan')
        self.turn_scan_pending = None
        self.turn_scan_last_stamp = stamp
        self._record('turn_scan_reference_ready',dict(scan_stamp_ns=stamp))

    def _poll_turn_scan(self):
        pending = self.turn_scan_pending
        if pending is None or not pending[0].done():
            return
        self.turn_scan_pending = None
        if self.turn_stop_requested:
            # The first stop owns its measurement. A result delivered after
            # disarming must not relabel a raw stop as a scan-triggered stop
            # or overwrite the scan that actually caused cancellation.
            return
        match = pending[0].result()
        age = (self.get_clock().now().nanoseconds-pending[1])/1e9
        previous = self.turn_scan_previous
        if match['status'] != 'PASS' or not 0 <= age <= .40:
            self.turn_scan_evidence = dict(match,age_s=age,rejected=True)
            self.turn_scan_matches = 0
            self.turn_scan_previous = None
            return
        delta = match['delta_yaw_rad']
        if previous is not None and (pending[1] <= previous['stamp_ns']
                or abs(delta-previous['delta']) > max(.15,2.*(pending[1]-previous['stamp_ns'])/1e9)):
            self.turn_scan_matches = 0
        self.turn_scan_matches += 1
        self.turn_scan_previous = dict(delta=delta,stamp_ns=pending[1])
        self.turn_scan_evidence = dict(match,age_s=age,scan_stamp_ns=pending[1],
            consecutive=self.turn_scan_matches,raw_delta_rad=self.turn_raw_progress)
        from rvclaw_nav2_bringup.turn_scan_reference import scan_turn_stop
        # Bound latency compensation with independently observed angle/raw
        # ratio. This only requests disarm; stopped absolute-map proof still
        # decides success and any subsequent correction.
        gain = (1.0 if self.turn_raw_progress is None or abs(self.turn_raw_progress)<.10 else
                max(1.,min(2.5,abs(delta/self.turn_raw_progress))))
        if (self.armed is True and self.state == 'ACTIVE'
                and self.transport_attached is True and self.feedback_connected is True
                and self.stm32_connected is True and self.action_feedback_samples >= 2
                and self.latest_raw_yaw_monotonic is not None
                and 0 <= time.monotonic()-self.latest_raw_yaw_monotonic <= .15
                and self.max_guarded_linear_x <= WAYPOINT.BASE.ZERO_EPSILON
                and self.max_guarded_linear_y <= WAYPOINT.BASE.ZERO_EPSILON
                and scan_turn_stop(self.expected_spin_delta,delta,
                    self.latest_feedback_angular*gain,age,self.turn_scan_matches)):
            self.turn_scan_stop_requested = True
            self._record('independent_scan_turn_stop',self.turn_scan_evidence)
            self._stop_completed_turn()

    def _on_action_feedback(self, message):
        super()._on_action_feedback(message)
        self.latest_turn_feedback_monotonic = time.monotonic()

    def _measure_settled_turn(self):
        """Measure the completed turn, including coast, against its start scan.

        The scan that requested disarm predates physical settling. Comparing
        that earlier angle with stopped AMCL falsely reports a frame change.
        Three fresh stopped scans may resolve it without resetting localization.
        """
        reference = getattr(self, 'turn_scan_reference', None)
        if reference is None:
            return None
        from rvclaw_nav2_bringup.turn_scan_reference import endpoints
        started = time.monotonic(); checks = []; stamp_before = 0
        while len(checks) < 3 and time.monotonic()-started < 1.5:
            rclpy.spin_once(self, timeout_sec=.01)
            scan = self.turn_scan_latest
            if scan is None: continue
            stamp = scan.header.stamp.sec*10**9+scan.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= stamp_before or not 0 <= age <= .25: continue
            stamp_before = stamp
            if (self.armed is not False or self.state != 'DISARMED'
                    or self.transport_attached is not True or self.feedback_connected is not True
                    or self.stm32_connected is not True
                    or self.latest_raw_yaw_monotonic is None
                    or not 0<=time.monotonic()-self.latest_raw_yaw_monotonic<=.15
                    or not all(math.isfinite(v) for v in
                        (self.latest_feedback_x,self.latest_feedback_y,self.latest_feedback_angular))
                    or max(abs(self.latest_feedback_x),abs(self.latest_feedback_y)) > .01
                    or abs(self.latest_feedback_angular) > .03):
                return None
            expected = self.turn_raw_progress
            previous = self.turn_scan_evidence or {}
            if (previous.get('status')=='PASS' and not previous.get('rejected')
                    and abs(previous.get('raw_delta_rad') or 0)>.10):
                gain=max(1.,min(2.5,abs(previous['delta_yaw_rad']/previous['raw_delta_rad'])))
                expected=previous['delta_yaw_rad']+gain*(self.turn_raw_progress-previous['raw_delta_rad'])
            match = reference.match(endpoints(scan), expected)
            if match['status'] != 'PASS': return None
            checks.append(dict(match, scan_stamp_ns=stamp))
        if (len(checks) != 3 or max(c['delta_yaw_rad'] for c in checks)
                - min(c['delta_yaw_rad'] for c in checks) > math.radians(3)):
            return None
        result = dict(status='PASS',delta_yaw_rad=sum(c['delta_yaw_rad'] for c in checks)/3,
                      checks=checks,elapsed_s=time.monotonic()-started,motion_authorized=False)
        self._record('settled_independent_turn_measurement', result)
        return result

    def _request_measured_turn_stop(self):
        if (self.turn_stop_requested or self.armed is not True or self.state != 'ACTIVE'
                or self.transport_attached is not True or self.feedback_connected is not True
                or self.stm32_connected is not True or self.action_feedback_samples < 2
                or not math.isfinite(self.max_guarded_linear_x)
                or not math.isfinite(self.max_guarded_linear_y)
                or self.max_guarded_linear_x > WAYPOINT.BASE.ZERO_EPSILON
                or self.max_guarded_linear_y > WAYPOINT.BASE.ZERO_EPSILON):
            return
        now = time.monotonic()
        feedback_at = self.latest_turn_feedback_monotonic
        raw_at = self.latest_raw_yaw_monotonic
        if not measured_heading_turn_complete(self.expected_spin_delta,
                self.action_feedback_final_rad, self.turn_raw_progress,
                None if feedback_at is None else now-feedback_at,
                None if raw_at is None else now-raw_at):
            return
        self._record('measured_turn_stop_requested', dict(
            requested_yaw_rad=self.expected_spin_delta,
            feedback_yaw_rad=self.action_feedback_final_rad,
            raw_yaw_rad=self.turn_raw_progress))
        self.turn_stop_requested = True
        # Disarm before cancel/zero draining: no further action output may
        # move the chassis while its result is being delivered.
        self._call_arm(False)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.publish_zero_command()
        self.turn_cancel_future = self.goal_handle.cancel_goal_async()

    def _stop_completed_turn(self):
        if self.turn_stop_requested:
            return
        self.turn_stop_requested = True
        self._call_arm(False)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.publish_zero_command()
        self.turn_cancel_future = self.goal_handle.cancel_goal_async()

    def _wait_future_with_deadman(self, future, timeout_s, description):
        if description != 'final heading Spin result':
            return super()._wait_future_with_deadman(future, timeout_s, description)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.publish_deadman(not self.turn_stop_requested)
            rclpy.spin_once(self, timeout_sec=HEADING.INTERVAL_S)
            if not self.turn_stop_requested:
                self._check_unexpected_control_stop()
            if future.done():
                return future.result()
            if self.state in ('WATCHDOG_LATCHED', 'ESTOP_LATCHED'):
                raise RuntimeError('hardware safety stop during final heading Spin')
            self._poll_turn_scan()
            self._request_map_heading_stop()
            self._request_measured_turn_stop()
        raise RuntimeError('timed out waiting for %s' % description)

    def _accept_measured_turn_cancellation(self, status):
        if not self.turn_stop_requested or status != GoalStatus.STATUS_CANCELED:
            return False
        future = self.turn_cancel_future
        if future is None:
            return False
        self.wait_for(future.done, HEADING.SERVICE_TIMEOUT_S, 'measured Spin cancellation response')
        response = future.result()
        accepted = (response is not None and response.return_code == 0 and any(
            bytes(item.goal_id.uuid) == bytes(self.goal_handle.goal_id.uuid)
            for item in response.goals_canceling))
        return bool(accepted)

    def _on_final_amcl(self, message):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp<=self.latest_heading_amcl_stamp_ns:
            return
        self.amcl_samples += 1
        yaw = WAYPOINT.BASE.quaternion_to_yaw(
            message.pose.pose.orientation
        )
        self.latest_amcl_yaw = yaw
        covariance = message.pose.covariance
        self.latest_heading_amcl_pose = {
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "yaw": float(yaw),
            "cov_x": float(covariance[0]),
            "cov_y": float(covariance[7]),
            "cov_yaw": float(covariance[35]),
        }

        self.latest_heading_amcl_stamp_ns = stamp

    def _request_map_heading_stop(self):
        # Only stop sooner. Actual success still needs fresh stopped AMCL,
        # unchanged final tolerances and hardware/map-frame checks.
        if (self.turn_stop_requested
                or self.armed is not True or self.state!='ACTIVE'
                or self.transport_attached is not True or self.feedback_connected is not True
                or self.stm32_connected is not True or self.action_feedback_samples<2
                or self.latest_raw_yaw_monotonic is None
                or not 0<=time.monotonic()-self.latest_raw_yaw_monotonic<=.15
                or not math.isfinite(self.max_guarded_linear_x)
                or not math.isfinite(self.max_guarded_linear_y)
                or self.max_guarded_linear_x>WAYPOINT.BASE.ZERO_EPSILON
                or self.max_guarded_linear_y>WAYPOINT.BASE.ZERO_EPSILON):
            return
        observation = map_heading_stop_observation(
            getattr(self,'turn_start_amcl_pose',None),self.latest_heading_amcl_pose,
            self.target_yaw,self.expected_spin_delta,self.turn_raw_progress,
            self.latest_heading_amcl_stamp_ns,self.get_clock().now().nanoseconds,
            self.turn_odometry_history,self.latest_feedback_angular)
        if observation is None:
            self.turn_map_confirmations = 0
            return
        stamp = observation['pose_stamp_ns']
        if stamp > self.turn_map_quality_stamp_ns:
            self.turn_map_quality_stamp_ns = stamp
            self.turn_map_confirmations += 1
        if (self.turn_map_confirmations<2 or observation['remaining_rad']>
                .5*self.completion_tolerance_rad+observation['stopping_lead_rad']):
            return
        scan=self.turn_scan_evidence or {}
        scan_age=(self.get_clock().now().nanoseconds-scan.get('scan_stamp_ns',0))/1e9
        if (scan.get('status')=='PASS' and not scan.get('rejected')
                and scan.get('consecutive',0)>=2 and 0<=scan_age<=.40):
            self.turn_map_scan_delta=scan['delta_yaw_rad']
        # A discrepancy is a reason to verify after disarming, not to keep
        # turning past an observed absolute target. Stopping here cannot
        # authorize completion; the stopped map and final heading gates follow.
        self.turn_map_stop_requested=True
        self._record('absolute_heading_stop_requested',dict(pose=self.latest_heading_amcl_pose,
            raw_delta_rad=self.turn_raw_progress,stamp_ns=self.latest_heading_amcl_stamp_ns,
            confirmations=self.turn_map_confirmations,observation=observation))
        self._stop_completed_turn()

    def capture_final_amcl_observation(self):
        """Preserve the last trusted stopped pose, including failure exits."""

        if self.latest_amcl_yaw is None:
            return
        self.final_amcl_yaw = float(self.latest_amcl_yaw)
        self.final_amcl_error_rad = shortest_heading_delta(
            self.target_yaw,
            self.final_amcl_yaw,
        )
        if isinstance(self.latest_heading_amcl_pose, dict):
            self.final_heading_amcl_pose = dict(self.latest_heading_amcl_pose)

    def _verify_stopped_heading_map(self):
        """Corroborate an in-place turn with three new full-view map scans.

        This never sets pose, resets a latch or arms. The pre-turn map frame
        was already verified; a translation or hardware fault is not eligible.
        """
        from nav2_fast_localization import FastLocalizationClient
        from rvclaw_nav2_bringup.scan_map_consistency import load_static_map
        pose = _finite_pose_copy(self.final_heading_amcl_pose)
        before = _finite_pose_copy(self.turn_start_amcl_pose)
        if (pose is None or before is None or
                math.hypot(pose['x']-before['x'],pose['y']-before['y']) > WAYPOINT.MAX_FINAL_POSITION_ERROR_M):
            return dict(status='FAIL',reason='stopped heading translated outside the existing limit')
        if not all(math.isfinite(self.final_heading_amcl_pose.get(k,math.nan)) and
                   0<=self.final_heading_amcl_pose[k]<=limit for k,limit in
                   (('cov_x',.05),('cov_y',.05),('cov_yaw',.02))):
            return dict(status='FAIL',reason='stopped heading covariance is invalid')
        class Proof:
            _endpoints=staticmethod(FastLocalizationClient._endpoints)
            _ray_endpoints=FastLocalizationClient._ray_endpoints
            def _scan_pose_gate(self, pose, scan):
                return FastLocalizationClient._scan_pose_gate(
                    self, pose, scan, include_all_matches=True)
        proof=Proof()
        proof.grid=load_static_map(_active_map_file())
        started=time.monotonic()
        latest=self.turn_scan_latest
        last_stamp=(0 if latest is None else
                    latest.header.stamp.sec*10**9+latest.header.stamp.nanosec)
        checks=[]
        resets=0
        while len(checks)<3 and time.monotonic()-started<4.0:
            rclpy.spin_once(getattr(self, 'stopped_proof_node', self),timeout_sec=.02)
            scan=self.turn_scan_latest
            if scan is None:continue
            stamp=scan.header.stamp.sec*10**9+scan.header.stamp.nanosec
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp<=last_stamp or not 0<=age<=.25:continue
            last_stamp=stamp
            current=_finite_pose_copy(self.latest_heading_amcl_pose)
            if (self.armed is not False or self.state!='DISARMED'
                    or self.transport_attached is not True or self.feedback_connected is not True
                    or self.stm32_connected is not True
                    or not all(math.isfinite(v) for v in (
                        self.latest_feedback_x,self.latest_feedback_y,self.latest_feedback_angular))
                    or max(abs(self.latest_feedback_x),abs(self.latest_feedback_y))>.01
                    or abs(self.latest_feedback_angular)>.03
                    or current is None
                    or abs(scan.angle_increment)*(len(scan.ranges)-1)<math.radians(330.)):
                return dict(status='FAIL',reason='stopped heading evidence lost freshness or continuity')
            # A first stopped AMCL observation can still be settling. Restart
            # the same bounded three-scan proof at the fresh pose; do not jump
            # immediately to a whole-map search or accept a mixed-pose batch.
            if (math.hypot(current['x']-before['x'],current['y']-before['y'])
                    > WAYPOINT.MAX_FINAL_POSITION_ERROR_M
                    or not all(math.isfinite(current.get(k,math.nan))
                               and 0<=current[k]<=limit for k,limit in
                               (('cov_x',.05),('cov_y',.05),('cov_yaw',.02)))):
                return dict(status='FAIL',reason='stopped pose moved outside the existing proof limit')
            if (math.hypot(current['x']-pose['x'],current['y']-pose['y'])>.05
                    or abs(shortest_heading_delta(current['yaw'],pose['yaw']))>math.radians(3.)):
                pose=current
                checks=[]
                resets+=1
            gate=stopped_scan_with_free_ray_uniqueness(proof._scan_pose_gate(pose,scan))
            checks.append(dict(stamp_ns=stamp,gate=gate))
            if gate['status']!='PASS':
                return dict(status='FAIL',reason='fresh full-view map/ray heading proof failed',checks=checks)
        unique=sum(c['gate']['uniqueness']['status']=='PASS' for c in checks)
        allowed_missing=lambda c:c['gate']['uniqueness'].get('failures')==[
            'stationary scan did not produce a distinct second match']
        no_conflict=all(c['gate']['uniqueness']['status']=='PASS' or allowed_missing(c) for c in checks)
        strong = all(c['gate']['uniqueness']['status']=='PASS'
                     or strong_uncontested_stopped_scan(c['gate']) for c in checks)
        return dict(status='PASS' if len(checks)==3 and no_conflict and (unique>=2 or strong) else 'FAIL',
                    pose=pose,checks=checks,elapsed_s=time.monotonic()-started,
                    settling_resets=resets,motion_authorized=False)

    def _on_monitor_command(self, message):
        self.monitor_output_samples += 1
        self.latest_monitor_command_monotonic = time.monotonic()
        angular = float(message.angular.z)
        linear = (float(message.linear.x), float(message.linear.y))
        if (
            self.expected_spin_delta is not None
            and all(math.isfinite(v) for v in (*linear, angular))
            and all(abs(v) <= WAYPOINT.BASE.ZERO_EPSILON for v in linear)
            and angular * self.expected_spin_delta > 0.0
            and abs(angular) <= ORIGIN_HEADING_MAX_GUARDED_RAD_S
        ):
            self.spin_command_samples += 1
            self.latest_spin_command_monotonic = self.latest_monitor_command_monotonic

    def _refresh_amcl_heading_once(self):
        # One update immediately after a changed scan view may be transient.
        # Require three fresh, mutually close stopped observations before
        # another chunk or a success decision. Never turn to chase that first
        # transient pose, and never reuse a pre-turn heading.
        stable = []
        for sample_index in range(12):
            # Several no-motion requests can otherwise update from the same
            # scan before the lidar has acquired another view. Three callbacks
            # on one frame are not three independent stopped observations.
            previous_scan_samples = self.scan_samples
            self.wait_for(lambda: self.scan_samples > previous_scan_samples,
                          HEADING.SERVICE_TIMEOUT_S,
                          "a new lidar scan for stopped heading verification")
            previous_samples = self.amcl_samples
            future = self.amcl_nomotion_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=HEADING.SERVICE_TIMEOUT_S)
            if not future.done() or future.exception() is not None:
                raise RuntimeError("AMCL no-motion heading refresh timed out")
            self.wait_for(lambda: self.amcl_samples > previous_samples,
                          HEADING.SERVICE_TIMEOUT_S, "fresh AMCL heading after Spin chunk")
            pose = _finite_pose_copy(self.latest_heading_amcl_pose)
            if pose is None:
                raise RuntimeError("fresh stopped AMCL pose is unavailable")
            if stable and any(
                math.hypot(pose["x"] - old["x"], pose["y"] - old["y"]) > 0.05
                or abs(shortest_heading_delta(pose["yaw"], old["yaw"])) > math.radians(3.0)
                for old in stable
            ):
                stable = []
            stable.append(pose)
            if len(stable) >= 3:
                self._record("stopped_heading_observations_stable", {
                    "fresh_updates": sample_index + 1, "observations": stable,
                })
                return
        raise RuntimeError("stopped AMCL heading did not stabilize after Spin")

    def _wait_for_post_goal_command_stream(
        self,
        baseline_samples: int,
        goal_dispatched_monotonic: float,
        result_future=None,
    ):
        def stream_is_fresh():
            if result_future is not None and result_future.done():
                raise RuntimeError("Spin ended before a valid command stream; refusing hardware arming")
            latest = self.latest_spin_command_monotonic
            if latest is None or latest < goal_dispatched_monotonic:
                return False
            age_s = time.monotonic() - latest
            return (
                self.spin_command_samples
                >= baseline_samples + WAYPOINT.BASE.PREARM_NAV2_COMMAND_MIN_SAMPLES
                and 0.0 <= age_s <= WAYPOINT.BASE.MAX_PREARM_NAV2_COMMAND_AGE_S
            )

        self.wait_for(
            stream_is_fresh,
            WAYPOINT.BASE.PREARM_NAV2_COMMAND_TIMEOUT_S,
            "fresh post-goal Spin command stream before hardware arming",
        )
        self._record(
            "prearm_spin_command_stream",
            {
                "baseline_samples": baseline_samples,
                "observed_samples": self.spin_command_samples,
                "new_samples": self.spin_command_samples - baseline_samples,
                "latest_age_s": (
                    None
                    if self.latest_spin_command_monotonic is None
                    else time.monotonic()
                    - self.latest_spin_command_monotonic
                ),
            },
        )

    def _wait_for_amcl_heading(self):
        if not self.amcl_nomotion_client.wait_for_service(
            timeout_sec=HEADING.STARTUP_TIMEOUT_S
        ):
            raise RuntimeError("AMCL no-motion update service is unavailable")
        self._refresh_amcl_heading_once()
        self._record("amcl_nomotion_updates_confirmed", 3)

    def _evaluate(self):
        failures = []
        if (
            self.action_status != GoalStatus.STATUS_SUCCEEDED
            and not self.terminal_action_failure_accepted
            and not self.turn_measured_completion
        ):
            failures.append("Nav2 Spin did not report success")
        if self.action_feedback_samples < 2:
            failures.append("Nav2 Spin emitted too few feedback samples")
        if self.requested_delta_yaw is None:
            failures.append("requested Spin delta is unavailable")
        elif abs(self.requested_delta_yaw) > ORIGIN_HEADING_SKIP_TOLERANCE_RAD:
            if self.guarded_angular_max is None:
                failures.append("hardware node emitted no guarded Spin samples")
            else:
                if (
                    abs(self.guarded_angular_max) > ORIGIN_HEADING_MAX_GUARDED_RAD_S
                    or self.guarded_angular_min is None
                    or abs(self.guarded_angular_min) > ORIGIN_HEADING_MAX_GUARDED_RAD_S
                ):
                    failures.append("guarded Spin exceeded the angular ceiling")
            if self.requested_delta_yaw > 0.0 and self.guarded_angular_min is None:
                failures.append("Spin direction samples are unavailable")
            elif self.requested_delta_yaw < 0.0 and self.guarded_angular_max is None:
                failures.append("Spin direction samples are unavailable")
            if self.max_guarded_linear_x > WAYPOINT.BASE.ZERO_EPSILON:
                failures.append("Spin contained guarded linear X velocity")
            if self.max_guarded_linear_y > WAYPOINT.BASE.ZERO_EPSILON:
                failures.append("Spin contained guarded lateral velocity")
            raw_delta = self.raw_yaw_delta()
            if raw_delta is None:
                failures.append("raw heading delta is unavailable")
            elif self.require_raw_delta_match and abs(
                WAYPOINT.BASE.normalize_angle(raw_delta - self.requested_delta_yaw)
            ) > ORIGIN_HEADING_TOLERANCE_RAD:
                failures.append("raw heading did not reach the requested shortest delta")
            if len(self.spin_chunks) > ORIGIN_HEADING_MAX_CHUNKS:
                failures.append("Spin exceeded the bounded chunk count")
            total_requested = sum(
                abs(float(item["requested_yaw_rad"])) for item in self.spin_chunks
            )
            if total_requested > ORIGIN_HEADING_MAX_TOTAL_RAD:
                failures.append("Spin exceeded the bounded total rotation")
            directions = [
                math.copysign(1.0, float(item["requested_yaw_rad"]))
                for item in self.spin_chunks
                if abs(float(item["requested_yaw_rad"]))
                > WAYPOINT.BASE.ZERO_EPSILON
            ]
            direction_reversals = sum(
                current != previous
                for previous, current in zip(directions, directions[1:])
            )
            if direction_reversals > self.max_direction_reversals:
                failures.append(
                    "Spin exceeded the bounded direction correction limit "
                    "(%d > %d)"
                    % (direction_reversals, self.max_direction_reversals)
                )
            if self.final_amcl_error_rad is None:
                failures.append("final AMCL heading error is unavailable")
            elif abs(self.final_amcl_error_rad) > ORIGIN_HEADING_TOLERANCE_RAD:
                failures.append("final AMCL heading did not reach the target tolerance")
        if abs(self.latest_feedback_x) > HEADING.MAX_POST_STOP_LINEAR_M_S:
            failures.append("post-Spin X feedback remained above the stop limit")
        if abs(self.latest_feedback_y) > HEADING.MAX_POST_STOP_LINEAR_M_S:
            failures.append("post-Spin Y feedback remained above the stop limit")
        if abs(self.latest_feedback_angular) > HEADING.MAX_POST_STOP_ANGULAR_RAD_S:
            failures.append("post-Spin angular feedback remained above the stop limit")
        if self.armed is not False:
            failures.append("hardware controller did not finish disarmed")
        return failures

    def run_to_absolute_heading(self):
        self._wait_for_amcl_heading()
        self.requested_delta_yaw = shortest_heading_delta(
            self.target_yaw,
            self.latest_amcl_yaw,
        )
        if abs(self.requested_delta_yaw) <= self.completion_tolerance_rad:
            self.skipped = True
            self.action_status = GoalStatus.STATUS_SUCCEEDED
            self.start_raw_yaw = self.latest_raw_yaw
            self.final_raw_yaw = self.latest_raw_yaw
            self.capture_final_amcl_observation()
            return self.result("PASS", None)

        self.wait_for(
            lambda: (
                self.transport_attached is True
                and self.feedback_connected is True
                and self.stm32_connected is True
                and self.voltage_v is not None
                and self.voltage_v >= HEADING.MIN_BATTERY_VOLTAGE_V
            ),
            HEADING.STARTUP_TIMEOUT_S,
            "fresh feedback and safe voltage",
        )
        self.wait_for(
            lambda: (
                self.scan_samples >= HEADING.MINIMUM_SCAN_SAMPLES
                and self.raw_odom_samples >= HEADING.MINIMUM_RAW_ODOM_SAMPLES
                and self.costmap_samples >= HEADING.MINIMUM_COSTMAP_SAMPLES
                and self.latest_raw_yaw is not None
            ),
            HEADING.STARTUP_TIMEOUT_S,
            "scan, raw odometry, and local costmap readiness",
        )
        if not self.spin_action.wait_for_server(timeout_sec=HEADING.STARTUP_TIMEOUT_S):
            raise RuntimeError("Nav2 Spin action server is unavailable")
        if self.armed:
            raise RuntimeError("hardware controller unexpectedly started armed")
        self.start_raw_yaw = self.latest_raw_yaw
        self.overall_start_raw_yaw = self.latest_raw_yaw

        self.publish_estop(False)
        self._call_reset()
        self.wait_for(
            lambda: self.state == "DISARMED",
            HEADING.SERVICE_TIMEOUT_S,
            "DISARMED state after reset",
        )
        action_started = time.monotonic()
        remaining = float(self.requested_delta_yaw)
        chunk_index = 0
        response_gain = 1.0
        while abs(remaining) > self.completion_tolerance_rad:
            chunk_index += 1
            if chunk_index > ORIGIN_HEADING_MAX_CHUNKS:
                raise StoppedHeadingLocalizationChanged("final heading exceeded the bounded Spin chunk count")
            # Dispatch the entire shortest turn; no fixed-angle stop/restart.
            chunk = heading_correction_delta(remaining, response_gain,
                                             correcting=chunk_index > 1)
            chunk_start_amcl_yaw = self.latest_amcl_yaw
            chunk_start_amcl_pose = _finite_pose_copy(self.latest_heading_amcl_pose)
            previous_requested = sum(
                abs(float(item["requested_yaw_rad"])) for item in self.spin_chunks
            )
            available = ORIGIN_HEADING_MAX_TOTAL_RAD - previous_requested
            if available < .02:
                raise StoppedHeadingLocalizationChanged("final heading exceeded the bounded total rotation")
            # A small residual must not reject an otherwise legal correction
            # merely because its proposed angle straddles the shared budget.
            # Spend only what remains; retain the final stopped pose checks.
            chunk = math.copysign(min(abs(chunk), available), chunk)
            directions = [math.copysign(1.0, item["requested_yaw_rad"])
                          for item in self.spin_chunks] + [math.copysign(1.0, chunk)]
            reversals = sum(a != b for a, b in zip(directions, directions[1:]))
            if reversals > self.max_direction_reversals:
                raise StoppedHeadingLocalizationChanged("Spin exceeded the bounded direction correction limit before dispatch")
            self.turn_raw_progress = 0.0
            self.turn_stop_requested = False
            self.turn_cancel_future = None
            self.turn_measured_completion = False
            self.turn_map_stop_requested = False
            self.turn_map_confirmations = 0
            self.turn_map_quality_stamp_ns = 0
            self.turn_map_scan_delta = None
            self.turn_start_amcl_pose = chunk_start_amcl_pose
            self._prepare_turn_scan_reference()
            self.action_feedback_samples = 0
            self.latest_turn_feedback_monotonic = None
            goal = Spin.Goal()
            goal.target_yaw = float(chunk)
            # Longer continuous angles need their real travel time. The
            # hardware deadman, command/scan freshness and route limits stay.
            self.action_time_allowance_s = max(ORIGIN_HEADING_ACTION_TIME_ALLOWANCE_S,
                math.ceil(abs(chunk) / ORIGIN_HEADING_MIN_GUARDED_RAD_S + 2.0))
            goal.time_allowance.sec = self.action_time_allowance_s
            goal.time_allowance.nanosec = 0
            self.expected_spin_delta = chunk
            self.latest_spin_command_monotonic = None
            self._start_deadman_keepalive()
            deadline = time.monotonic() + HEADING.DEADMAN_PRIME_S
            while time.monotonic() < deadline:
                self.publish_deadman(True)
                rclpy.spin_once(self, timeout_sec=HEADING.INTERVAL_S)
            command_stream_baseline = self.spin_command_samples
            goal_dispatched_monotonic = time.monotonic()
            send_future = self.spin_action.send_goal_async(
                goal,
                feedback_callback=self._on_action_feedback,
            )
            self.goal_handle = self._wait_future_with_deadman(
                send_future,
                HEADING.SERVICE_TIMEOUT_S,
                "final heading Spin goal acceptance",
            )
            if self.goal_handle is None or not self.goal_handle.accepted:
                raise RuntimeError("final heading Spin goal was rejected")
            self._record("action_goal_accepted", {"chunk_index": chunk_index, "target_yaw_rad": chunk})
            result_future = self.goal_handle.get_result_async()
            self._wait_for_post_goal_command_stream(
                command_stream_baseline,
                goal_dispatched_monotonic,
                result_future,
            )
            if result_future.done():
                raise RuntimeError("Spin ended before hardware arming")
            self._call_arm(True)
            self.wait_for(
                lambda: self.armed is True,
                HEADING.SERVICE_TIMEOUT_S,
                "armed state after fresh Spin command stream",
            )
            self.wait_for(
                lambda: self.state == "ACTIVE"
                or self.state in ("WATCHDOG_LATCHED", "ESTOP_LATCHED"),
                HEADING.SERVICE_TIMEOUT_S,
                "ACTIVE state from post-arm Spin command stream",
            )
            if self.state != "ACTIVE":
                raise RuntimeError(
                    "hardware did not enter ACTIVE after Spin arming "
                    "(state=%s reason=%s)"
                    % (self.state, self.transition_reason)
                )
            self._record("post_goal_hardware_active", {"chunk_index": chunk_index})
            result_response = self._wait_future_with_deadman(
                result_future,
                max(ORIGIN_HEADING_ACTION_RESULT_TIMEOUT_S, self.action_time_allowance_s + 5.),
                "final heading Spin result",
            )
            self.action_status = int(result_response.status)
            self.turn_measured_completion = self._accept_measured_turn_cancellation(self.action_status)
            chunk_raw_delta = self.turn_raw_progress
            if (self.turn_measured_completion and not self.turn_scan_stop_requested
                    and not self.turn_map_stop_requested
                    and abs(chunk_raw_delta-chunk) > .04):
                raise RuntimeError('measured Spin stop exceeded the settling rotation bound')
            self.spin_chunks.append(
                {
                    "index": chunk_index,
                    "requested_yaw_rad": chunk,
                    "action_status": self.action_status,
                    "measured_completion": self.turn_measured_completion,
                    "action_time_allowance_s": self.action_time_allowance_s,
                    "raw_yaw_delta_rad": chunk_raw_delta,
                    "feedback_samples": self.action_feedback_samples,
                    "independent_scan_stop": self.turn_scan_stop_requested,
                    "absolute_heading_stop": self.turn_map_stop_requested,
                    "scan_evidence": self.turn_scan_evidence,
                }
            )
            self._call_arm(False)
            self._stop_deadman_keepalive()
            self.publish_deadman(False)
            for _ in range(HEADING.POST_ACTION_ZERO_FRAMES):
                self.publish_zero_command()
                rclpy.spin_once(self, timeout_sec=HEADING.INTERVAL_S)
            self.wait_for(
                lambda: self.armed is False and self.state == "DISARMED",
                HEADING.SERVICE_TIMEOUT_S,
                "DISARMED state after Spin chunk",
            )
            self._wait_for_stable_feedback_stop()
            self._refresh_amcl_heading_once()
            self.capture_final_amcl_observation()
            independent_verified_pose = None
            measured_delta = self.turn_raw_progress
            if self.turn_scan_stop_requested and self.turn_scan_evidence is not None:
                measured_delta = self.turn_scan_evidence['delta_yaw_rad']
            elif self.turn_map_stop_requested and self.turn_map_scan_delta is not None:
                measured_delta = self.turn_map_scan_delta
            stopped_frame_change = stopped_heading_frame_change(
                chunk_start_amcl_pose, self.final_heading_amcl_pose, measured_delta)
            if stopped_frame_change is not None:
                settled_turn = self._measure_settled_turn()
                if settled_turn is not None:
                    measured_delta = settled_turn['delta_yaw_rad']
                    self.spin_chunks[-1]['settled_scan_evidence'] = settled_turn
                    stopped_frame_change = stopped_heading_frame_change(
                        chunk_start_amcl_pose,self.final_heading_amcl_pose,measured_delta)
            if (self.turn_scan_stop_requested and self.final_amcl_error_rad is not None
                    and abs(self.final_amcl_error_rad)>self.completion_tolerance_rad
                    and stopped_frame_change is not None):
                # A small residual after a coherent measured turn is a normal
                # correction, not evidence of a new map frame. Only reconcile
                # independently when translation or measured yaw disagrees.
                review = self._verify_stopped_heading_map()
                verified = self.final_heading_amcl_pose if review['status']=='PASS' else None
                if verified is None:
                    verifier = self.stopped_verifier or _verify_stopped_return_pose
                    verified, review = verifier(dict(
                        name='HEADING',x=self.final_heading_amcl_pose['x'],
                        y=self.final_heading_amcl_pose['y'],yaw=self.target_yaw))
                self._record('stopped_turn_independent_reverification',review)
                if verified is None:
                    raise StoppedHeadingLocalizationChanged(
                        'stopped heading localization could not be independently verified')
                self._refresh_amcl_heading_once()
                self.capture_final_amcl_observation()
                independent_verified_pose = _finite_pose_copy(verified)
            remaining = shortest_heading_delta(
                self.target_yaw,
                self.latest_amcl_yaw,
            )
            map_progress = shortest_heading_delta(self.latest_amcl_yaw, chunk_start_amcl_yaw)
            if map_progress * chunk > 0.0:
                response_gain = max(response_gain, min(3.0, abs(map_progress / chunk)))
            self._record(
                "spin_chunk_completed",
                {
                    "index": chunk_index,
                    "remaining_yaw_rad": remaining,
                    "amcl_yaw_rad": self.latest_amcl_yaw,
                    "response_gain": response_gain,
                },
            )
            within_tolerance = abs(remaining) <= self.completion_tolerance_rad
            if self.action_status != GoalStatus.STATUS_SUCCEEDED and not self.turn_measured_completion:
                if self.accept_terminal_action_failure and within_tolerance:
                    self.terminal_action_failure_accepted = True
                    self._record(
                        "terminal_spin_status_tolerated",
                        {
                            "chunk_index": chunk_index,
                            "action_status": self.action_status,
                            "remaining_yaw_rad": remaining,
                        },
                    )
                    break
                raise RuntimeError(
                    "Nav2 Spin chunk %d did not report success" % chunk_index
                )
            if (
                (chunk_raw_delta is None or abs(chunk_raw_delta) < min(0.10, .5*abs(chunk)))
                and not within_tolerance
            ):
                raise RuntimeError(
                    "Nav2 Spin chunk %d produced insufficient raw heading change"
                    % chunk_index
                )
            if chunk_raw_delta is not None and chunk * chunk_raw_delta < 0.0:
                raise RuntimeError(
                    "Nav2 Spin chunk %d produced opposite-direction raw heading"
                    % chunk_index
                )
            if self.verify_stopped_map_frame:
                change = stopped_heading_frame_change(chunk_start_amcl_pose,
                    self.final_heading_amcl_pose, measured_delta)
                if change is not None and independent_verified_pose is not None:
                    # The full verifier has just established the stopped map
                    # frame. Do not reject that same frame again using the
                    # pre-relocalization turn origin and another local search.
                    current = _finite_pose_copy(self.final_heading_amcl_pose)
                    if (current is not None and math.hypot(current['x']-independent_verified_pose['x'],
                            current['y']-independent_verified_pose['y'])<=.05
                            and abs(shortest_heading_delta(current['yaw'],independent_verified_pose['yaw']))<=math.radians(3.)):
                        self._record('stopped_verified_frame_handoff',dict(pose=current,change=change))
                        change = None
                if change is not None:
                    # The action has ended, hardware is disarmed and fresh
                    # stopped feedback was checked above. Do not chase a
                    # changed map frame with another reversal or latch an
                    # already stopped chassis as though an obstacle hit it.
                    self._record('origin_stopped_heading_frame_changed', change)
                    proof=self._verify_stopped_heading_map()
                    self._record('stopped_heading_map_corroboration',proof)
                    if proof['status']!='PASS':
                        raise StoppedHeadingLocalizationChanged(
                            'stopped ORIGIN heading localization changed; independent verification required')
                    # Continue the same bounded heading loop with the observed
                    # response gain. Position correction and heading correction
                    # no longer consume the same one-shot recovery allowance.

        self.action_elapsed_s = time.monotonic() - action_started
        self.action_result_raw_yaw = self.latest_raw_yaw
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        self.final_raw_yaw = self.latest_raw_yaw
        self.capture_final_amcl_observation()
        failures = self._evaluate()
        if failures:
            raise RuntimeError("; ".join(failures))
        return self.result("PASS", None)

    def result(self, status, error):
        result = super().result(status, error)
        result.update(
            {
                "target_yaw_rad": self.target_yaw,
                "target_yaw_deg": math.degrees(self.target_yaw),
                "requested_delta_yaw_rad": self.requested_delta_yaw,
                "requested_delta_yaw_deg": (
                    None
                    if self.requested_delta_yaw is None
                    else math.degrees(self.requested_delta_yaw)
                ),
                "spin_chunk_max_rad": ORIGIN_HEADING_SPIN_CHUNK_MAX_RAD,
                "spin_max_chunks": ORIGIN_HEADING_MAX_CHUNKS,
                "spin_max_total_rad": ORIGIN_HEADING_MAX_TOTAL_RAD,
                "spin_max_direction_reversals": self.max_direction_reversals,
                "spin_chunks": self.spin_chunks,
                "origin_heading_skipped": self.skipped,
                "origin_heading_tolerance_rad": ORIGIN_HEADING_TOLERANCE_RAD,
                "completion_tolerance_rad": self.completion_tolerance_rad,
                "require_raw_delta_match": self.require_raw_delta_match,
                "terminal_action_failure_accepted": self.terminal_action_failure_accepted,
                "final_amcl_yaw_rad": self.final_amcl_yaw,
                "final_amcl_error_rad": self.final_amcl_error_rad,
                "final_amcl_pose": (
                    None
                    if self.final_heading_amcl_pose is None
                    else dict(self.final_heading_amcl_pose)
                ),
            }
        )
        return result


def origin_pose_within_goal_tolerance(current_pose, waypoint):
    """Return XY/yaw evidence when a stationary pose already satisfies ORIGIN."""

    current = _finite_pose_copy(current_pose)
    goal = _finite_pose_copy(waypoint)
    if current is None or goal is None:
        return None
    distance_m = math.hypot(current["x"] - goal["x"], current["y"] - goal["y"])
    yaw_error_rad = abs(shortest_heading_delta(goal["yaw"], current["yaw"]))
    if (
        distance_m > ORIGIN_GOAL_XY_TOLERANCE_M
        or yaw_error_rad > ORIGIN_GOAL_YAW_TOLERANCE_RAD
    ):
        return None
    return {
        "distance_m": distance_m,
        "yaw_error_rad": yaw_error_rad,
        "yaw_error_deg": math.degrees(yaw_error_rad),
    }


def _try_stationary_origin_completion(node, waypoint):
    """Complete ORIGIN without arming when fresh AMCL already satisfies it."""

    try:
        node._request_amcl_nomotion_update()
        if (
            node.amcl_samples < WAYPOINT.BASE.MINIMUM_AMCL_SAMPLES
            or node.armed is not False
            or str(node.state).upper() != "DISARMED"
        ):
            return None
        node._check_static_map_localization()
        current = _finite_pose_copy(node.latest_amcl_pose)
        evidence = origin_pose_within_goal_tolerance(current, waypoint)
        if evidence is None:
            return None
        node.goal_pose = dict(waypoint)
        node.start_amcl_pose = dict(current)
        node.final_amcl_pose = dict(current)
        if getattr(node, "latest_raw_pose", None) is not None:
            node.start_raw_pose = dict(node.latest_raw_pose)
            node.final_raw_pose = dict(node.latest_raw_pose)
        result = node.result("PASS", None)
        result.update(
            {
                "origin_navigation_skipped": True,
                "origin_completion_mode": "STATIONARY_WITHIN_TOLERANCE",
                "origin_stationary_evidence": evidence,
            }
        )
        return result
    except BaseException as exc:
        try:
            node._record("origin_stationary_completion_probe_failed", str(exc))
        except BaseException:
            pass
        return None


def _verify_stopped_arrival_pose(node):
    """Verify the arrived pose without resetting AMCL or searching a new pose."""
    node._wait_for_stable_feedback_stop()
    node._request_amcl_nomotion_update()
    pose = _finite_pose_copy(node.latest_amcl_pose)
    if pose is None:
        return None, dict(status='FAIL', reason='stopped arrival pose unavailable')

    class StoppedArrivalProof:
        stopped_proof_node = node
        final_heading_amcl_pose = pose
        turn_start_amcl_pose = pose

        def __getattr__(self, name):
            return getattr(node, {
                'turn_scan_latest': 'latest_localization_scan',
                'latest_heading_amcl_pose': 'latest_amcl_pose',
            }.get(name, name))

    report = FinalHeadingClient._verify_stopped_heading_map(StoppedArrivalProof())
    report['method'] = 'stopped_arrival_map_ray_consensus_without_pose_reset'
    return (report['pose'] if report['status']=='PASS' else None), report


def _verify_stopped_return_pose(waypoint, *, use_origin_reference=True, require_disarmed=False):
    # This helper has no motion interface. A new full-view consensus and the
    # independent final gate must pass before any position correction is used.
    from nav2_fast_localization import FastLocalizationClient
    stop_node = PositionOnlyWaypointClient(dict(waypoint))
    try:
        stop_node.wait_for(
            lambda: stop_node.armed is False
            and stop_node.stm32_connected is True
            and stop_node.state in ({'DISARMED'} if require_disarmed else {'DISARMED', 'ESTOP_LATCHED'}),
            WAYPOINT.BASE.STARTUP_TIMEOUT_S,
            'disarmed hardware before stopped return verification',
        )
        stop_node._wait_for_stable_feedback_stop()
    except Exception as exc:
        return None, {'status': 'FAIL', 'error': 'return chassis is not confirmed stopped: %s' % exc,
                      'motion_authorized': False}
    finally:
        stop_node.destroy_node()
    class ReturnVerifier(FastLocalizationClient):
        def _set_initial_pose(self, pose, mode):
            # A close, already converged live pose does not need particle
            # redistribution. Keep the global search and all final map/ray
            # gates, but avoid reinitializing AMCL just to verify the same pose.
            self._assert_disarmed()
            requested=time.monotonic()
            self._call(self.nomotion_client,Empty.Request(),'fresh stopped return observation')
            self._wait_for(lambda:any(s['received_monotonic']>=requested for s in self.amcl_samples),
                           3.0,'fresh stopped return observation')
            current=self.amcl_samples[-1]
            close=(all(math.isfinite(current.get(k,math.nan)) for k in
                       ('x','y','yaw','cov_x','cov_y','cov_yaw'))
                and all(0<=current[k]<=limit for k,limit in
                        (('cov_x',.05),('cov_y',.05),('cov_yaw',.02)))
                # Global raster matching is quantized. This is the existing
                # stationary convergence drift bound, not arrival tolerance;
                # the unchanged independent final pose/ray gate still follows.
                and math.hypot(current['x']-pose['x'],current['y']-pose['y'])<=.10
                and abs(shortest_heading_delta(current['yaw'],pose['yaw']))<=math.radians(3.))
            self.return_pose_unchanged=bool(close)
            if close:
                self._record('return_live_pose_preserved',dict(pose=current,candidate=pose))
                return time.monotonic()
            return super()._set_initial_pose(pose,mode)

        def _nomotion_convergence(self, after_monotonic):
            if not getattr(self,'return_pose_unchanged',False):
                return super()._nomotion_convergence(after_monotonic)
            from rvclaw_nav2_bringup.amcl_localization_observer import summarize_global_pose_samples
            # No pose was changed. Retain the original final five-sample,
            # two-second settling window; the 12 reset warmup updates are
            # unnecessary when no particle reset occurred.
            for _ in range(5):
                self._assert_disarmed()
                self._call(self.nomotion_client,Empty.Request(),'unchanged return stationary update')
                end=time.monotonic()+.4
                while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
            samples=[s for s in self.amcl_samples if s['received_monotonic']>=after_monotonic]
            summary=summarize_global_pose_samples(samples,minimum_samples=5,
                maximum_position_drift_m=.10,maximum_yaw_drift_deg=10.,
                maximum_position_variance=.10,maximum_yaw_variance=math.radians(10.)**2)
            self._record('nomotion_convergence',summary)
            return summary

        def _global_candidate(self, scans):
            pose, report = super()._global_candidate(scans)
            if pose is None or self.origin_reference is None:
                return pose, report
            full = self._fresh_auxiliary_scans(after_monotonic=time.monotonic())
            if len(full) < 3:
                return None, dict(status='FAIL',reason='fresh ORIGIN calibration scans unavailable',global_match=report)
            calibrated, reference_report = self.origin_reference.consensus(
                full[-3:],pose,global_report=report)
            self._record('origin_calibrated_reference',reference_report)
            if calibrated is None:
                # Sparse geometry recorded facing +X is an optional refinement
                # at other headings. Do not discard a unique whole-map pose
                # merely because that template loses score. Confirm the actual
                # global pose against every fresh scan before retaining it.
                from origin_reference_policy import unique_global_reference_fallback
                gates = [self._scan_pose_gate(pose,s) for s in full[-3:]]
                fallback = unique_global_reference_fallback(report,pose,reference_report,gates)
                self._record('origin_unique_global_fallback',fallback)
                if fallback['status']=='PASS':
                    self.auxiliary_candidate_used = True
                    self.auxiliary_candidate_signatures = [self._signature(s) for s in full[-3:]]
                    return pose, dict(status='PASS',global_match=report,
                        reference=reference_report,reference_fallback=fallback)
            if (calibrated is None or math.hypot(calibrated['x']-pose['x'],calibrated['y']-pose['y'])>.25
                    or abs(shortest_heading_delta(calibrated['yaw'],pose['yaw']))>math.radians(5)):
                return None, dict(status='FAIL',reason='ORIGIN calibration and global map disagree',
                    global_match=report,reference=reference_report)
            self.auxiliary_candidate_used = True
            self.auxiliary_candidate_signatures = [self._signature(s) for s in full[-3:]]
            return calibrated, dict(status='PASS',global_match=report,reference=reference_report)
    node = ReturnVerifier(Path(os.environ['RVCLAW_MAP_FILE']),
                          Path(os.environ['RVCLAW_LOCALIZATION_CACHE']))
    node.origin_reference = None
    reference_path = Path(os.environ['RVCLAW_MAP_FILE']).parent/'origin_localization_reference.json'
    if use_origin_reference and reference_path.exists() and waypoint.get('name') == 'ORIGIN':
        from b_localization_reference import BLocalizationReference
        node.origin_reference = BLocalizationReference(reference_path,node.grid,waypoint='ORIGIN')
    node.force_static_verification = True
    try:
        evidence = node.run(20.0)
        if (evidence['status'] != 'PASS' or node.final_verified_pose is None
                or evidence.get('motion_authorized') is not False
                or evidence.get('final_runtime_state', {}).get('state') != 'HOT_READY'):
            return None, evidence
        if require_disarmed and not (
                evidence.get('final_runtime_state', {}).get('controller_state') == 'DISARMED'
                and evidence.get('final_runtime_state', {}).get('armed') is False
                and evidence.get('final_runtime_state', {}).get('hardware_connected') is True):
            return None, dict(evidence, recovery_refused='controller state changed during verification')
        return dict(node.final_verified_pose), evidence
    except Exception as exc:
        return None, node.result('FAIL', 'return_stationary_verification', str(exc))
    finally:
        node.destroy_node()


def _corroborated_return_pose_change(first, verified_pose):
    """Extra evidence for a correction beyond the usual 0.75 m envelope.

    A globally verified stationary pose must also agree with the ray-checked
    alternative observed before stopping. The motion observation must be time
    aligned, inside its one-metre search radius, and contradict the old pose's
    free rays. No new search, pose publication, latch reset or motion here.
    """
    evidence = (first.get('metrics') or {}).get('motion_map_last_evidence') or {}
    alternative = evidence.get('ray_checked_alternative') or {}
    timing = evidence.get('observation_timing') or {}
    current_ray = evidence.get('current_ray_consistency') or {}
    ray = alternative.get('ray_consistency') or {}
    center = _finite_pose_copy(timing.get('pose'))
    candidate = _finite_pose_copy(alternative)
    if (evidence.get('status') != 'DISAGREEMENT' or timing.get('status') != 'ALIGNED'
            or current_ray.get('status') != 'FAIL'
            or ray.get('status') != 'PASS' or center is None or candidate is None):
        return False
    try:
        values = [float(alternative['mean_likelihood']), float(alternative['known_endpoint_ratio']),
                  float(current_ray['conflicting_ray_ratio']), float(timing['skew_s'])]
    except (KeyError, TypeError, ValueError):
        return False
    if not all(math.isfinite(v) for v in values):
        return False
    score, known, conflicts, skew = values
    return (score >= .90 and known >= .90 and conflicts > .25 and abs(skew) <= .25
            and math.hypot(candidate['x']-center['x'],candidate['y']-center['y']) <= 1.0
            and math.hypot(candidate['x']-verified_pose['x'],candidate['y']-verified_pose['y']) <= .25
            and abs(shortest_heading_delta(candidate['yaw'],verified_pose['yaw'])) <= math.radians(12.))


def _run_origin(waypoint, continuity_anchor=None, continuing_egress=False):
    first = _run_origin_once(waypoint, continuity_anchor, continuing_egress)
    if (first.get('error') == 'in-motion scan/map localization disagreement; stopped'
            and classify_navigation_failure(first).get('code') == 'LOCALIZATION_PROTECTIVE_STOP'
            and first.get('final_armed') is False):
        # The guard must stop the chassis first. Only ORIGIN may attempt one
        # stationary, globally unique relocalization followed by one bounded
        # correction through the normal path, scan, costmap and arm gates.
        pose, evidence = _verify_stopped_return_pose(waypoint, use_origin_reference=False)
        first['origin_stationary_reverification'] = evidence
        if pose is None:
            return first
        distance = math.hypot(pose['x'] - waypoint['x'], pose['y'] - waypoint['y'])
        first['origin_verified_distance_m'] = distance
        # This is interrupted transit, not the final heading correction. Bound
        # the verified pose *change* and route progress, rather than rejecting
        # a valid remaining route just because it is longer than one metre.
        stopped_pose = _finite_pose_copy(first.get('final_amcl_pose'))
        start_pose = _finite_pose_copy(first.get('start_amcl_pose'))
        displacement = (None if stopped_pose is None else
                        math.hypot(pose['x']-stopped_pose['x'],pose['y']-stopped_pose['y']))
        first['origin_verified_pose_change_m'] = displacement
        remaining_limit = (ORIGIN_VERIFIED_CORRECTION_MAX_M if start_pose is None else
                           math.hypot(start_pose['x']-waypoint['x'],start_pose['y']-waypoint['y'])+.25)
        first['origin_verified_correction_limit_m'] = remaining_limit
        corroborated = (displacement is not None and .75 < displacement <= 1.0
                        and _corroborated_return_pose_change(first, pose))
        first['origin_return_pose_change_corroborated'] = bool(corroborated)
        if (displacement is None or (displacement > .75 and not corroborated)
                or distance > remaining_limit):
            first['origin_recovery_refused_reason'] = 'verified return pose exceeds bounded transit recovery'
            return first
        corrected = _run_origin_once(waypoint, continuity_anchor=pose)
        corrected['origin_completion_attempts'] = [first]
        corrected['origin_stationary_reverification'] = evidence
        corrected['origin_position_correction_count'] = 1
        corrected['origin_localization_guard_recovery'] = True
        corrected['origin_verified_distance_m'] = distance
        corrected['origin_verified_correction_limit_m'] = remaining_limit
        corrected['origin_verified_pose_change_m'] = displacement
        # Transit recovery and final-heading verification are separate stages.
        # Do not return a heading failure merely because XY used its recovery.
        first = corrected
    heading_frame_changed = (
        first.get('origin_heading_reverification_required') is True
        and first.get('final_armed') is False and first.get('final_state') == 'DISARMED')
    if (first.get('error') != 'final AMCL position outside ORIGIN tolerance after stopped heading'
            and not heading_frame_changed):
        return first
    pre_heading = first.get('origin_pre_heading_verification') or {}
    if (pre_heading.get('status') == 'PASS'
            and 0 <= time.monotonic()-first.get('origin_pre_heading_checked_monotonic',-math.inf) <= 5.):
        pose, evidence = first.get('origin_pre_heading_pose'), pre_heading
    else:
        pose, evidence = _verify_stopped_return_pose(waypoint)
    first['origin_stationary_reverification'] = evidence
    if pose is None:
        return first
    distance = math.hypot(pose['x']-waypoint['x'],pose['y']-waypoint['y'])
    yaw_error = abs(shortest_heading_delta(waypoint['yaw'],pose['yaw']))
    if distance <= WAYPOINT.MAX_FINAL_POSITION_ERROR_M and yaw_error <= ORIGIN_HEADING_TOLERANCE_RAD:
        original = dict(first)
        first.update(status='PASS',error=None,final_amcl_pose=pose,
                     final_amcl_yaw_rad=pose['yaw'],final_amcl_error_rad=shortest_heading_delta(waypoint['yaw'],pose['yaw']),
                     origin_final_position_error_m=distance)
        first['origin_completion_attempts'] = [original]
        first['origin_heading_reverification_required'] = False
        first['origin_heading_stage'] = dict(first.get('origin_heading_stage') or {},
            status='PASS',error=None,final_amcl_pose=pose,
            final_amcl_yaw_rad=pose['yaw'],final_amcl_error_rad=shortest_heading_delta(waypoint['yaw'],pose['yaw']))
        return first
    # One short correction through the usual costmap/controller safety gates.
    # Never recurse or chase a distant relocalization hypothesis.
    if distance > ORIGIN_VERIFIED_CORRECTION_MAX_M:
        first['error'] = 'verified return position is outside bounded correction distance'
        return first
    if (heading_frame_changed and distance <= .25
            and yaw_error > ORIGIN_HEADING_TOLERANCE_RAD):
        # A newly verified near-origin pose needs a small absolute heading
        # correction, not a fresh departure toward a centimetre-scale XY path.
        # Recheck XY after turning; never relax the final position tolerance.
        corrected = _finish_origin_heading(waypoint, dict(first,
            status='PASS', error=None, final_amcl_pose=pose))
        corrected['origin_completion_attempts'] = [first]
        corrected['origin_stationary_reverification'] = evidence
        corrected['origin_heading_only_recovery'] = True
        corrected['origin_position_correction_count'] = first.get('origin_position_correction_count',0)
        if corrected.get('error') != 'final AMCL position outside ORIGIN tolerance after stopped heading':
            # This recovery already spent its bounded heading budget. Do not
            # let the route retry from the old C anchor or repeat a full turn.
            if corrected.get('status') != 'PASS':
                corrected['automatic_recovery_forbidden'] = True
            return corrected
        first = corrected
        pose = _finite_pose_copy(corrected.get('final_amcl_pose'))
        if pose is None:
            return corrected
    if first.get('origin_position_correction_count',0)>=1:
        first['origin_recovery_refused_reason']='bounded return position correction already used'
        return first
    corrected = _run_origin_once(waypoint, continuity_anchor=pose)
    corrected['origin_completion_attempts'] = [first]
    corrected['origin_stationary_reverification'] = evidence
    corrected['origin_position_correction_count'] = 1
    return corrected


def _run_origin_navigation(node):
    """Give synchronous ORIGIN travel its own budget, then restore the default."""
    # Use the same continuous Nav2 controller as the other departures. A
    # separate Spin here overshot, corrected, stopped and then replanned into
    # another turn. Keep the current path/scan/rotation-sweep admission below.
    if isinstance(node, PositionOnlyWaypointClient):
        node.align_departure_before_navigation = False
    previous_timeout = WAYPOINT.BASE.ACTION_RESULT_TIMEOUT_S
    WAYPOINT.BASE.ACTION_RESULT_TIMEOUT_S = ORIGIN_TRANSIT_ACTION_RESULT_TIMEOUT_S
    try:
        try:
            result = node.run_navigation()
        except StoppedHeadingLocalizationChanged as exc:
            if not _heading_failure_is_safely_stopped(node, exc):
                raise
            pose, proof = _verify_stopped_arrival_pose(node)
            node._record('departure_automatic_stopped_recovery', proof)
            if pose is None:
                raise
            # One stopped, independently proved restart of the segment. It
            # retains normal path/sweep/scan gates and never resets a latch.
            node.continuity_anchor = dict(pose)
            result = node.run_navigation()
            result['departure_automatic_recovery_count'] = 1
        result["origin_transit_timeout_s"] = ORIGIN_TRANSIT_ACTION_RESULT_TIMEOUT_S
        return result
    finally:
        WAYPOINT.BASE.ACTION_RESULT_TIMEOUT_S = previous_timeout


def _try_stopped_origin_position(node, waypoint):
    """Prove an already reached XY target without a zero-length navigation."""
    node._request_amcl_nomotion_update()
    current = _finite_pose_copy(node.latest_amcl_pose)
    if current is None or math.hypot(
        current['x']-waypoint['x'],current['y']-waypoint['y']
    ) > ORIGIN_GOAL_XY_TOLERANCE_M:
        return None
    if (node.armed is not False or node.state not in {
        'DISARMED','ESTOP_LATCHED','WATCHDOG_LATCHED'
    } or node.transport_attached is not True or node.feedback_connected is not True
        or node.stm32_connected is not True):
        raise RuntimeError('ORIGIN position skip requires connected disarmed hardware')
    node._wait_for_stable_feedback_stop()
    stopped_state=node.state
    for _ in range(3):
        node._wait_for_fresh_scan()
        samples=node.amcl_samples
        future=node.amcl_nomotion_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(
            node,future,timeout_sec=WAYPOINT.BASE.AMCL_NOMOTION_RESPONSE_TIMEOUT_S)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('ORIGIN stopped pose refresh failed')
        node.wait_for(lambda:node.amcl_samples>samples,
            WAYPOINT.BASE.SERVICE_TIMEOUT_S,'new stopped AMCL observation at ORIGIN')
        node._check_static_map_localization()
        if (node.armed is not False or node.state != stopped_state
                or node.transport_attached is not True or node.feedback_connected is not True
                or node.stm32_connected is not True):
            raise RuntimeError('hardware changed during ORIGIN stopped verification')
        current=_finite_pose_copy(node.latest_amcl_pose)
        if current is None or math.hypot(
            current['x']-waypoint['x'],current['y']-waypoint['y']
        ) > ORIGIN_GOAL_XY_TOLERANCE_M:
            return None
    node.goal_pose=dict(waypoint)
    node.start_amcl_pose=dict(current)
    node.final_amcl_pose=dict(current)
    result=node.result('PASS',None)
    result.update(position_stage='SKIPPED_NEAR_GOAL',
        deferred_reason='ORIGIN position already within 0.05 m',
        current_goal_distance_m=math.hypot(
            current['x']-waypoint['x'],current['y']-waypoint['y']))
    return result


def _run_origin_once(waypoint, continuity_anchor=None, continuing_egress=False):
    if ORIGIN_NAVIGATION_MODE == "absolute_pose":
        if continuity_anchor is None:
            node = OriginPoseWaypointClient(waypoint)
        else:
            node = OriginPoseWaypointClient(
                waypoint,
                continuity_anchor=continuity_anchor,
            )
        try:
            result = _try_stationary_origin_completion(node, waypoint)
            if result is None:
                result = _run_origin_navigation(node)
        except BaseException as exc:
            try:
                node.emergency_cleanup()
            except BaseException as cleanup_exc:
                node._record("cleanup_error", str(cleanup_exc))
            result = _failure_result(node, exc)
        finally:
            node.destroy_node()
        result.update(
            {
                "origin_navigation_mode": ORIGIN_NAVIGATION_MODE,
                "origin_pose_goal": {
                    "x": waypoint["x"],
                    "y": waypoint["y"],
                    "yaw": waypoint["yaw"],
                },
            }
        )
        return result

    # Position and heading are deliberately separated so the controller's
    # translation-only progress checker cannot abort valid in-place rotation.
    position_waypoint = dict(waypoint)
    if continuity_anchor is None:
        position_node = PositionOnlyWaypointClient(position_waypoint, continuing_egress=continuing_egress)
    else:
        position_node = PositionOnlyWaypointClient(
            position_waypoint,
            continuity_anchor=continuity_anchor,
            continuing_egress=continuing_egress,
        )
    near_pose = None
    near_candidate = False
    near_error = None
    try:
        position_node._request_amcl_nomotion_update()
        if position_node.latest_amcl_pose is not None:
            current = position_node.latest_amcl_pose
            if math.hypot(
                current["x"] - waypoint["x"],
                current["y"] - waypoint["y"],
            ) <= ORIGIN_GOAL_XY_TOLERANCE_M:
                near_candidate = True
                # A prior failure cleanup intentionally leaves the chassis
                # ESTOP_LATCHED. This is stopped, not a reason to dispatch a
                # zero-distance NavigateToPose and demand motion-only samples.
                if position_node.armed is not False or position_node.state not in {
                    "DISARMED", "ESTOP_LATCHED", "WATCHDOG_LATCHED",
                }:
                    raise RuntimeError("ORIGIN position skip requires a disarmed controller")
                position_node._wait_for_stable_feedback_stop()
                for _ in range(3):
                    position_node._wait_for_fresh_scan()
                    pose_count = position_node.amcl_samples
                    future = position_node.amcl_nomotion_client.call_async(Empty.Request())
                    rclpy.spin_until_future_complete(
                        position_node, future,
                        timeout_sec=WAYPOINT.BASE.AMCL_NOMOTION_RESPONSE_TIMEOUT_S,
                    )
                    if not future.done() or future.exception() is not None:
                        raise RuntimeError("ORIGIN stopped pose refresh failed")
                    position_node.wait_for(
                        lambda: position_node.amcl_samples > pose_count,
                        WAYPOINT.BASE.SERVICE_TIMEOUT_S,
                        "new stopped AMCL observation at ORIGIN",
                    )
                position_node._check_static_map_localization()
                current = _finite_pose_copy(position_node.latest_amcl_pose)
                if current is not None and math.hypot(
                    current["x"] - waypoint["x"], current["y"] - waypoint["y"]
                ) <= ORIGIN_GOAL_XY_TOLERANCE_M:
                    near_pose = dict(current)
    except Exception as exc:
        near_error = exc if near_candidate else None
        position_node._record("stationary_origin_validation_error", str(exc))
        near_pose = None
    if near_error is not None:
        try:
            position_node.emergency_cleanup()
        except Exception as cleanup_exc:
            position_node._record("cleanup_error", str(cleanup_exc))
        position_result = _failure_result(position_node, near_error)
    elif near_pose is not None:
        position_node.goal_pose = dict(waypoint)
        position_node.start_amcl_pose = dict(near_pose)
        position_node.final_amcl_pose = dict(near_pose)
        position_result = position_node.result("PASS", None)
        position_result.update(
            {
                "position_stage": "SKIPPED_NEAR_GOAL",
                "deferred_reason": "ORIGIN position already within 0.05 m",
                "current_goal_distance_m": math.hypot(
                    near_pose["x"] - waypoint["x"],
                    near_pose["y"] - waypoint["y"],
                ),
            }
        )
    else:
        try:
            position_result = _run_origin_navigation(position_node)
        except BaseException as exc:
            # A departure turn can move AMCL across the existing 5 cm XY
            # threshold. The refreshed plan then has no initial direction.
            # Only this preflight error can use stopped XY proof; full-map
            # verification and the final heading stage below still follow.
            stopped_result = None
            if (str(exc) in {'near-obstacle path has no usable initial direction',
                             'departure path has no supported direction'}
                    and position_node.armed is False
                    and position_node.state == 'DISARMED'):
                try:
                    stopped_result = _try_stopped_origin_position(position_node, waypoint)
                except Exception as stopped_exc:
                    position_node._record('stationary_origin_validation_error', str(stopped_exc))
            if stopped_result is not None:
                position_result = stopped_result
                position_node._record('origin_short_plan_completed_while_stopped',
                                      stopped_result['current_goal_distance_m'])
            else:
                checked_egress_required = (
                    position_node.armed is False
                    and (isinstance(exc, SideNearTurnEgressRequired)
                         or _is_near_start_path_failure(position_node, exc))
                )
                stopped_preflight=False
                if (isinstance(exc,SideNearTurnUnsafe)
                        and position_node.armed is False and position_node.state=='DISARMED'
                        and getattr(position_node,'goal_handle',None) is None
                        and position_node.transport_attached is True
                        and position_node.feedback_connected is True
                        and position_node.stm32_connected is True):
                    try:
                        position_node._wait_for_stable_feedback_stop()
                        stopped_preflight=(position_node.armed is False and position_node.state=='DISARMED'
                            and position_node.transport_attached is True
                            and position_node.feedback_connected is True
                            and position_node.stm32_connected is True)
                    except Exception:
                        stopped_preflight=False
                safely_stopped_review = _heading_failure_is_safely_stopped(position_node, exc)
                if not stopped_preflight and not safely_stopped_review:
                    try:
                        position_node.emergency_cleanup()
                    except BaseException as cleanup_exc:
                        position_node._record("cleanup_error", str(cleanup_exc))
                position_result = _failure_result(position_node, exc)
                if safely_stopped_review and isinstance(exc, StoppedHeadingLocalizationChanged):
                    position_result['automatic_recovery_forbidden']=True
                    position_node._record('stopped_localization_review_failed_disarmed',str(exc))
                elif stopped_preflight:
                    position_result['error']='departure obstacle blocks the turn: %s' % exc
                    position_node._record('preflight_turn_blocked_while_disarmed',str(exc))
                position_result['checked_egress_required'] = checked_egress_required
    verified_pose, verification = None, None
    if position_result.get('final_amcl_pose') is None:
        # Preflight can fail after a successful departure turn, before Nav2
        # fills final_amcl_pose. Preserve its fresh stopped observation so a
        # retry cannot compare the origin against the previous C waypoint.
        if position_node.armed is False and position_node.state == 'DISARMED':
            try:
                position_node._request_amcl_nomotion_update()
                position_result['final_amcl_pose'] = _finite_pose_copy(position_node.latest_amcl_pose)
            except Exception as exc:
                position_result['automatic_recovery_forbidden'] = True
                position_node._record('origin_retry_pose_unavailable', str(exc))
    if position_result.get('status') == 'PASS':
        try:
            verified_pose, verification = _verify_stopped_arrival_pose(position_node)
        except Exception as exc:
            verification = dict(status='FAIL', reason=str(exc), motion_authorized=False)
        position_result['origin_arrival_pose_proof'] = verification
    position_node.destroy_node()
    if position_result.get("status") != "PASS":
        return {
            **position_result,
            "waypoint": waypoint,
            "origin_navigation_mode": ORIGIN_NAVIGATION_MODE,
            "origin_position_stage": position_result,
            "origin_heading_stage": None,
        }

    # A corridor alias may satisfy AMCL XY, then disappear only after turning.
    # Prove the stationary position before the final heading, using the same
    # globally unique map/ray gates and the operator-selected ORIGIN reference.
    if verified_pose is None:
        verified_pose, verification = _verify_stopped_return_pose(waypoint)
    position_result['origin_pre_heading_verification'] = verification
    if verified_pose is None:
        return dict(position_result,status='FAIL',
            error='ORIGIN stationary localization could not be verified before final heading',
            waypoint=waypoint,origin_navigation_mode=ORIGIN_NAVIGATION_MODE,
            origin_position_stage=position_result,origin_heading_stage=None,
            automatic_recovery_forbidden=True)
    verified_error = math.hypot(verified_pose['x']-waypoint['x'],verified_pose['y']-waypoint['y'])
    if verified_error > WAYPOINT.MAX_FINAL_POSITION_ERROR_M:
        return dict(position_result,status='FAIL',
            error='final AMCL position outside ORIGIN tolerance after stopped heading',
            waypoint=waypoint,origin_navigation_mode=ORIGIN_NAVIGATION_MODE,
            origin_position_stage=position_result,origin_heading_stage=None,
            origin_pre_heading_verification=verification,origin_pre_heading_pose=verified_pose,
            origin_pre_heading_checked_monotonic=time.monotonic(),
            origin_final_position_error_m=verified_error)
    position_result['final_amcl_pose'] = verified_pose
    return _finish_origin_heading(waypoint, position_result)


def _finish_origin_heading(waypoint, position_result):
    """Use the same guarded absolute turn for arrival and stopped recovery."""
    heading_node = FinalHeadingClient(
        waypoint["yaw"],
        # Use the existing absolute-heading acceptance limit as the loop's
        # stopping limit as well. Chasing a stricter four-degree micro-turn
        # after meeting that limit caused a second reversal and a false FAIL.
        completion_tolerance_rad=ORIGIN_HEADING_TOLERANCE_RAD,
        # Raw wheel yaw and map-frame AMCL yaw can differ after a long turn.
        # Absolute stopped AMCL heading decides this absolute map-frame goal;
        # finite, correctly directed raw motion and bounded rotation are still
        # mandatory for every chunk.
        require_raw_delta_match=False,
        verify_stopped_map_frame=True,
    )
    try:
        heading_result = heading_node.run_to_absolute_heading()
    except StoppedHeadingLocalizationChanged as exc:
        try:
            # Recheck fresh telemetry, including any real latch arriving
            # during the stopped AMCL refresh. Such a fault keeps the usual
            # emergency path and must never enter automatic reverification.
            heading_node._wait_for_stable_feedback_stop()
            if (heading_node.armed is not False or heading_node.state != 'DISARMED'
                    or heading_node.transport_attached is not True
                    or heading_node.feedback_connected is not True
                    or heading_node.stm32_connected is not True):
                raise RuntimeError('hardware is not safely disarmed for heading verification')
            heading_node.capture_final_amcl_observation()
            heading_result = _failure_result(heading_node, exc)
            heading_result['origin_heading_reverification_required'] = True
        except BaseException as stop_exc:
            heading_node.emergency_cleanup()
            heading_node.capture_final_amcl_observation()
            heading_result = _failure_result(heading_node, stop_exc)
    except BaseException as exc:
        try:
            if not _heading_failure_is_safely_stopped(heading_node, exc):
                heading_node.emergency_cleanup()
        except BaseException as cleanup_exc:
            heading_node._record("cleanup_error", str(cleanup_exc))
        heading_node.capture_final_amcl_observation()
        heading_result = _failure_result(heading_node, exc)
    finally:
        heading_node.destroy_node()
    final_pose = _finite_pose_copy(heading_result.get("final_amcl_pose"))
    final_position_error = None if final_pose is None else math.hypot(
        final_pose["x"] - waypoint["x"], final_pose["y"] - waypoint["y"])
    heading_result["origin_final_position_error_m"] = final_position_error
    heading_result["origin_final_position_tolerance_m"] = WAYPOINT.MAX_FINAL_POSITION_ERROR_M
    if heading_result.get("status") == "PASS" and (
        final_position_error is None
        or final_position_error > WAYPOINT.MAX_FINAL_POSITION_ERROR_M
    ):
        heading_result["status"] = "FAIL"
        heading_result["error"] = "final AMCL position outside ORIGIN tolerance after stopped heading"
    if heading_result.get("status") != "PASS":
        return {
            **heading_result,
            "waypoint": waypoint,
            "origin_navigation_mode": ORIGIN_NAVIGATION_MODE,
            "origin_position_stage": position_result,
            "origin_heading_stage": heading_result,
        }
    return {
        **heading_result,
        "status": "PASS",
        "error": None,
        "waypoint": waypoint,
        "origin_navigation_mode": ORIGIN_NAVIGATION_MODE,
        "origin_position_stage": position_result,
        "origin_heading_stage": heading_result,
        "final_armed": heading_result.get("final_armed"),
        "final_state": heading_result.get("final_state"),
    }


def _run_inspection_heading_once(target_yaw, waypoint_name=None, *, recovering_timeout=False):
    """Align the chassis through the existing guarded Nav2 Spin path."""

    node = FinalHeadingClient(
        target_yaw,
        completion_tolerance_rad=INSPECTION_HEADING_COMPLETION_TOLERANCE_RAD,
        require_raw_delta_match=False,
        accept_terminal_action_failure=True,
        max_direction_reversals=2,
        stopped_verifier=(lambda target: _verify_stopped_inspection_pose(
            target,20.,'C',allow_any_heading=True)) if waypoint_name=='C' else None,
    )
    try:
        if recovering_timeout:
            node.wait_for(lambda: node.armed is not None and node.state is not None
                and node.transition_reason is not None, 2., 'heading recovery control state')
            node._wait_for_stable_feedback_stop()
            if (node.armed is not False or node.state != 'DISARMED'
                    or node.transition_reason != 'deadman_timeout'
                    or node.transport_attached is not True
                    or node.feedback_connected is not True or node.stm32_connected is not True):
                raise RuntimeError('heading recovery rejected changed hardware stop state')
        result = node.run_to_absolute_heading()
    except StoppedHeadingLocalizationChanged as exc:
        # A safely stopped pose-review failure is not a physical emergency.
        # Preserve the localization failure and paused state, without latching
        # an already disarmed chassis or bypassing any live safety fault.
        try:
            node._wait_for_stable_feedback_stop()
            if (node.armed is not False or node.state!='DISARMED'
                    or node.transport_attached is not True or node.feedback_connected is not True
                    or node.stm32_connected is not True):
                raise RuntimeError('inspection heading is not safely disarmed')
            node.capture_final_amcl_observation()
            result = _failure_result(node, exc)
            result['automatic_recovery_forbidden'] = True
        except BaseException as fault:
            node.emergency_cleanup()
            node.capture_final_amcl_observation()
            result = _failure_result(node, fault)
    except BaseException as exc:
        timeout_stopped = False
        try:
            safely_stopped = _heading_failure_is_safely_stopped(node, exc)
            timeout_stopped = (safely_stopped
                and isinstance(exc, HEADING.StoppedHeadingControlTimeout)
                and getattr(node, 'heading_timeout_stop_confirmed', False) is True)
            if not safely_stopped:
                node.emergency_cleanup()
        except BaseException as cleanup_exc:
            node._record("cleanup_error", str(cleanup_exc))
        node.capture_final_amcl_observation()
        result = _failure_result(node, exc)
        result['heading_timeout_stop_confirmed'] = timeout_stopped
    finally:
        node.destroy_node()
    result["inspection_heading_stage"] = True
    return result


def _run_inspection_heading(target_yaw, waypoint_name=None):
    """Retry a proven transient stop in place, never re-drive an arrived point."""
    started = time.monotonic()
    first = _run_inspection_heading_once(target_yaw, waypoint_name)
    if not (first.get('status') == 'FAIL'
            and first.get('heading_timeout_stop_confirmed') is True
            and first.get('final_armed') is False
            and first.get('final_state') == 'DISARMED'
            and first.get('final_transition_reason') == 'deadman_timeout'
            and time.monotonic() - started < 25.):
        return first
    # A new client uses normal fresh feedback, scan, pose, command-stream and
    # collision admission. Never clear a real latch or reuse the old goal.
    time.sleep(.30)
    result = _run_inspection_heading_once(target_yaw, waypoint_name, recovering_timeout=True)
    result['heading_automatic_recovery'] = dict(
        attempts=1, first_failure=first, recovered=result.get('status') == 'PASS')
    return result


def _build_inspection_task_runner(plan, run_id):
    executor = InspectionExecutor(plan)

    def run_task(waypoint, route_index, navigation_result):
        try:
            point = plan.point(str(waypoint.get("name", "")))
            deadline = time.monotonic() + (0.0 if point is None else point.timeout_s)
            task_executor = executor
            arrival = navigation_result.get('inspection_arrival')
            if waypoint.get('name') in ('A','C') and point is not None and point.enabled:
                if not isinstance(arrival,dict) or arrival.get('status')!='PASS':
                    raise RuntimeError('inspection requires completed observation alignment')
                deadline=min(deadline,arrival['started_monotonic']+point.timeout_s)
            b_arrival = navigation_result.get('b_arrival')
            if waypoint.get('name') == 'B' and point is not None and point.enabled:
                if (not isinstance(b_arrival, dict) or b_arrival.get('status') != 'PASS'
                        or b_arrival.get('check', {}).get('decision') != 'ARRIVED'):
                    raise RuntimeError('B inspection requires independently verified arrival')
                # Preparation and camera work share the existing point budget.
                deadline = min(deadline, b_arrival['started_monotonic'] + point.timeout_s)
                arrival = b_arrival
                # XY and observation heading were checked before arrival was
                # announced. Do not start another AMCL-only position chase
                # after that announcement; bounded camera heading search stays.
                from dataclasses import replace
                points = dict(plan.points)
                point = replace(point, stopped_position_tolerance_m=None)
                points['B'] = point
                task_executor = InspectionExecutor(replace(plan, points=points),
                    client=executor.client, evidence_session=executor.evidence_session)
            position_review = point is not None and point.stopped_position_tolerance_m is not None
            # Reuse A/C's just-completed heading once. Searching other camera
            # headings still uses the guarded action. A stale receipt fails
            # closed instead of starting another turn after announcing arrival.
            first_heading = True
            def align(target_yaw):
                nonlocal first_heading
                if first_heading and isinstance(arrival,dict):
                    first_heading=False
                    if (time.monotonic()-arrival['checked_monotonic']>5.
                            or abs(shortest_heading_delta(target_yaw,arrival['target_yaw']))>1e-6):
                        raise RuntimeError('prepared inspection heading expired or target changed')
                    return dict(arrival['heading'],prepared_before_arrival=True)
                first_heading=False
                return (_run_inspection_heading(target_yaw,waypoint_name='C')
                    if waypoint.get('name')=='C' else _run_inspection_heading(target_yaw))
            result = task_executor.execute(
                waypoint,
                route_index,
                navigation_result,
                align,
                run_id=run_id,
                correct_position=_run_inspection_position,
                registration_failure_limit=(4 if waypoint.get('name') == 'A' else 2)
                    if position_review else None,
                deadline_monotonic=deadline,
            )
            return _retry_inspection_after_position_verification(
                task_executor, point, waypoint, route_index, result, run_id, deadline,
            )
        except BaseException as exc:
            point = plan.point(str(waypoint.get("name", "")))
            return {
                "status": "FAIL",
                "task_succeeded": False,
                "failure_policy": (
                    "abort" if point is None else point.failure_policy
                ),
                "motion_authorized": False,
                "task_id": None if point is None else point.task_id,
                "meter_id": None if point is None else point.meter_id,
                "kind": None if point is None else point.kind,
                **exception_diagnostics(exc),
            }

    def prepare(waypoint, navigation_result):
        point = plan.point(str(waypoint.get('name', '')))
        if point is None or not point.enabled or waypoint.get('name') not in ('A','B','C'):
            return navigation_result
        # Use the inspection plan's fixed camera heading, just as execute does.
        target = dict(waypoint, yaw=point.base_yaw_rad(waypoint))
        if waypoint.get('name')=='B':
            return _prepare_b_arrival(target, navigation_result, timeout_s=point.timeout_s)
        return _prepare_inspection_arrival(target,navigation_result,timeout_s=point.timeout_s)

    run_task.prepare_arrival = prepare
    return run_task


def _prepare_inspection_arrival(waypoint, navigation_result, *, timeout_s=90.):
    """A/C align before speech; C additionally proves its scene-bound stop."""
    result=dict(navigation_result)
    receipt=dict(status='FAIL',started_monotonic=time.monotonic(),target_yaw=waypoint['yaw'])
    deadline=receipt['started_monotonic']+min(90.,timeout_s)
    result['inspection_arrival']=receipt
    heading=navigation_result
    try:
        if (heading.get('status')!='PASS' or heading.get('final_armed') is not False
                or heading.get('final_state')!='DISARMED'):
            raise RuntimeError('inspection arrival requires a successful DISARMED stop')
        heading=dict(_run_inspection_heading(waypoint['yaw'],waypoint_name='C')
            if waypoint.get('name')=='C' else _run_inspection_heading(waypoint['yaw']))
        receipt['heading']=heading
        pose=_finite_pose_copy(heading.get('final_amcl_pose'))
        if (heading.get('status')!='PASS' or heading.get('final_armed') is not False
                or heading.get('final_state')!='DISARMED' or pose is None
                or abs(shortest_heading_delta(pose['yaw'],waypoint['yaw']))
                    > INSPECTION_HEADING_COMPLETION_TOLERANCE_RAD):
            raise RuntimeError('observation heading did not finish safely before arrival')
        if waypoint['name']=='C':
            from c_stopped_arrival import review_stopped_arrival
            check=review_stopped_arrival(_active_map_file(),pose,waypoint)
            receipt['geometry']=check
            heading.update(final_armed=check.get('final_armed'),final_state=check.get('final_state'))
            if (check.get('status')=='REVIEW' and check.get('decision') in ('RELOCALIZE','REAPPROACH')
                    and check.get('stationary_confirmed') is True
                    and check.get('final_armed') is False and check.get('final_state')=='DISARMED'
                    and deadline-time.monotonic()>43.):
                # The scene-bound review is evidence to stop and verify, never
                # a pose to drive from. Use fresh independent admission first.
                verified, proof=_verify_stopped_inspection_pose(pose,20.,'C')
                receipt['position_verification']=proof
                if verified is not None and proof.get('status')=='PASS':
                    check=review_stopped_arrival(_active_map_file(),verified,waypoint)
                    receipt['verified_geometry']=check
                    if (check.get('status')=='REVIEW' and check.get('decision')=='REAPPROACH'
                            and check.get('stationary_confirmed') is True
                            and check.get('final_armed') is False and check.get('final_state')=='DISARMED'
                            and deadline-time.monotonic()>23.):
                        correction=_run_inspection_position(waypoint,verified,
                            min(30.,deadline-time.monotonic()-18.))
                        receipt['position_correction']=correction
                        if (correction.get('status')!='PASS' or correction.get('final_armed') is not False
                                or correction.get('final_state')!='DISARMED'):
                            heading=correction
                            raise RuntimeError('C verified reapproach did not finish safely')
                        heading=dict(_run_inspection_heading(waypoint['yaw'],waypoint_name=waypoint.get('name')))
                        receipt['heading']=heading
                        if (heading.get('status')!='PASS' or heading.get('final_armed') is not False
                                or heading.get('final_state')!='DISARMED'):
                            raise RuntimeError('C reapproach heading did not finish safely')
                        check=review_stopped_arrival(_active_map_file(),heading['final_amcl_pose'],waypoint,
                            timeout_s=min(8.,deadline-time.monotonic()))
                        receipt['corrected_geometry']=check
                    heading.update(final_armed=check.get('final_armed'),final_state=check.get('final_state'))
            if (check.get('status')!='PASS' or check.get('decision')!='ARRIVED'
                    or check.get('final_armed') is not False or check.get('final_state')!='DISARMED'):
                result['failure_code']='ARRIVAL_VERIFICATION_INCOMPLETE'
                raise RuntimeError(check.get('error','C stopped geometry does not confirm arrival'))
            pose=check['amcl_pose']
            heading['final_amcl_pose']=pose
        if time.monotonic()>deadline:
            raise RuntimeError('inspection arrival preparation deadline expired')
        receipt.update(status='PASS',checked_monotonic=time.monotonic())
        result.update(final_amcl_pose=pose,final_armed=False,final_state='DISARMED',
                      final_transition_reason='operator_disarm')
    except Exception as exc:
        receipt['error']=str(exc)
        result.update(status='FAIL',error=str(exc),automatic_recovery_forbidden=True,
            final_armed=heading.get('final_armed'),final_state=heading.get('final_state'),
            final_transition_reason=heading.get('final_transition_reason'),
            final_amcl_pose=heading.get('final_amcl_pose'))
        if heading.get('status') == 'FAIL' and heading.get('error'):
            result['error'] = '%s: %s' % (str(exc), heading['error'])
    return result


def _prepare_b_arrival(waypoint, navigation_result, *, timeout_s=90.):
    # Admit stationary camera work through the same stopped heading path as A.
    # Sparse reference ambiguity is not evidence of a physical position error
    # and must not force translation before the camera can even see the meter.
    # Keep the original 12 cm / 10 degree envelope; this receipt authorizes
    # observation only. Departure still passes the ordinary motion guards.
    started = time.monotonic()
    prepared = _prepare_inspection_arrival(waypoint, navigation_result, timeout_s=timeout_s)
    if prepared.get('status') != 'PASS':
        return prepared
    receipt = prepared['inspection_arrival']
    heading = receipt['heading']
    pose = _finite_pose_copy(heading.get('final_amcl_pose'))
    distance = math.inf if pose is None else math.hypot(
        pose['x']-waypoint['x'], pose['y']-waypoint['y'])
    if distance <= .12:
        prepared['b_arrival'] = dict(receipt,
            check=dict(status='PASS', decision='ARRIVED',
                basis='SAFE_STATIONARY_OBSERVATION', distance_m=distance,
                tolerance_m=.12, amcl_pose=pose, motion_authorized=False),
            corrections=0, verifications=0, attempts=[])
        return prepared
    # A real out-of-envelope stop retains the existing independently proved
    # correction path, within the same whole-point deadline. Reuse the turn
    # already completed instead of making the chassis turn twice.
    first_heading = True
    def align(target):
        nonlocal first_heading
        if first_heading:
            first_heading = False
            if abs(shortest_heading_delta(target, waypoint['yaw'])) <= 1e-6:
                return heading
        return _run_inspection_heading(target)
    from b_stopped_arrival import prepare_arrival, review_stopped_arrival
    return prepare_arrival(waypoint, prepared,
        align=align,
        review=lambda pose, target, timeout: review_stopped_arrival(
            _active_map_file(), pose, target, timeout_s=timeout),
        verify=lambda pose, timeout: _verify_stopped_inspection_pose(
            pose, timeout, 'B', allow_any_heading=True),
        navigate=_run_inspection_position,
        timeout_s=max(0., timeout_s-(time.monotonic()-started)))


def inspection_local_position_candidate(grid, scans, reference):
    """Prove a unique nearby pose; repeated corridor walls must remain ambiguous."""
    from rvclaw_nav2_bringup.scan_map_consistency import (
        evaluate_match_consensus, filter_ray_inconsistent_matches,
        scan_endpoints, search_local_matches,
    )
    searches = []
    for scan in scans:
        endpoints = scan_endpoints(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, maximum_range_m=12.0, maximum_points=240,
        )
        search = search_local_matches(
            grid, endpoints, reference["x"], reference["y"], reference["yaw"],
            position_radius_m=.75, yaw_radius_deg=15.0,
            position_step_m=.05, yaw_step_deg=2.5, maximum_matches=16,
        )
        rays = scan_endpoints(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, maximum_range_m=12.0, maximum_points=480,
        )
        searches.append(filter_ray_inconsistent_matches(grid, rays, search))
    report = evaluate_match_consensus(
        searches, require_global_uniqueness=True, minimum_unique_scan_count=2,
        minimum_best_score=.85, minimum_known_endpoint_ratio=.90,
        maximum_pair_distance_m=.10, maximum_pair_yaw_deg=3.0,
    )
    pose = _finite_pose_copy(report.get("pose"))
    # A boundary winner may be a clipped peak outside this local search.
    if (pose is None or math.hypot(pose["x"]-reference["x"], pose["y"]-reference["y"]) > .70
            or abs(shortest_heading_delta(pose["yaw"], reference["yaw"])) > math.radians(12.5)):
        report["status"] = "FAIL"
        report["failures"].append("nearby pose lies outside the supported interior")
    report["searches"] = searches
    report["input_scans"] = [dict(
        angle_min=s.angle_min, angle_increment=s.angle_increment,
        range_min=s.range_min, range_max=s.range_max,
        ranges=[float(v) if math.isfinite(v) else None for v in s.ranges],
    ) for s in scans]
    return (pose if report["status"] == "PASS" else None), report


def _verify_stopped_inspection_pose(reference, timeout_s, waypoint_name=None, *, allow_any_heading=False):
    from nav2_fast_localization import FastLocalizationClient
    deadline = time.monotonic() + min(20.0, timeout_s)

    class InspectionPositionVerifier(FastLocalizationClient):
        def _call(self, client, request, name, timeout_s=15.0):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("inspection position verification deadline expired")
            return super()._call(client, request, name, min(timeout_s, remaining))

        def _wait_for(self, predicate, timeout_s, description):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("inspection position verification deadline expired")
            return super()._wait_for(predicate, min(timeout_s, remaining), description)

        def _bounded_candidate(self, scans):
            self._assert_disarmed()
            # Full view is stationary evidence only. Driving retains its
            # existing forward scan and collision monitor configuration.
            full = self._fresh_auxiliary_scans(after_monotonic=time.monotonic())
            if not full:
                return None, dict(status="FAIL", reason="fresh full-view scans unavailable")
            if self.c_reference is not None:
                pose, report = self.c_reference.consensus(full,reference)
                # Save the bounded stationary evidence as well as scores, so
                # an ambiguous stop remains replayable after the car is moved.
                report['input_scans'] = [dict(
                    angle_min=s.angle_min, angle_increment=s.angle_increment,
                    range_min=s.range_min, range_max=s.range_max,
                    ranges=[float(v) if math.isfinite(v) else None for v in s.ranges],
                ) for s in full[-3:]]
                if pose is not None and allow_any_heading:
                    # An approach view must also support the candidate in
                    # the driving scan before changing AMCL. Otherwise fall
                    # back to the known observation heading while still using
                    # the original pose; never apply a full-view-only guess.
                    primary = self.c_reference.pose_gate(scans[-1], pose)
                    report['approach_primary_gate'] = primary
                    if primary.get('status') != 'PASS':
                        pose = None
                        report['status'] = 'FAIL'
                        report.setdefault('failures', []).append(
                            'approach primary view does not independently support C pose')
            else:
                pose, report = inspection_local_position_candidate(self.grid, full, reference)
            if pose is not None:
                self.auxiliary_candidate_used = True
                self.auxiliary_candidate_signatures = [self._signature(s) for s in full]
            self._record("inspection_local_position_candidate", report)
            return pose, report

        def _scan_pose_gate(self, pose, scan):
            if self.c_reference is not None:
                return self.c_reference.pose_gate(scan,pose)
            return super()._scan_pose_gate(pose,scan)

        def _set_initial_pose(self, pose, mode):
            if time.monotonic() >= deadline:
                raise RuntimeError("inspection position verification deadline expired")
            return super()._set_initial_pose(pose, "local")

    node = InspectionPositionVerifier(
        Path(os.environ["RVCLAW_MAP_FILE"]), Path(os.environ["RVCLAW_LOCALIZATION_CACHE"]),
        start_region=(reference["x"], reference["y"], 1.5 if waypoint_name == 'B' else .75),
    )
    # A failed camera view is evidence against simply reusing HOT_READY.
    # Require fresh stationary scans and the existing independent final gate.
    node.force_static_verification = True
    node.c_reference = None
    try:
        if waypoint_name == 'B':
            from b_localization_reference import BLocalizationReference
            node.c_reference = BLocalizationReference(
                Path(os.environ['RVCLAW_MAP_FILE']).parent/'b_localization_reference.json',node.grid)
        if waypoint_name == 'C':
            from c_stationary_reference import StationaryReference
            node.c_reference = StationaryReference(
                Path(os.environ['RVCLAW_MAP_FILE']).parent/'c_stationary_reference.json',node.grid,
                allow_any_heading=allow_any_heading)
        evidence = node.run(min(20.0, timeout_s))
        pose = _finite_pose_copy(node.final_verified_pose)
        if (evidence.get("status") != "PASS" or pose is None
                or evidence.get("motion_authorized") is not False
                or evidence.get("final_runtime_state", {}).get("state") != "HOT_READY"
                or math.hypot(pose["x"]-reference["x"], pose["y"]-reference["y"]) > (1.35 if waypoint_name == 'B' else .75)
                or abs(shortest_heading_delta(pose["yaw"], reference["yaw"])) > math.radians(15)):
            return None, evidence
        return pose, evidence
    except Exception as exc:
        return None, node.result("FAIL", "inspection_local_position", str(exc))
    finally:
        node.destroy_node()


def _retry_inspection_after_position_verification(
    executor, point, waypoint, index, first, run_id, deadline,
):
    if (point is None or point.stopped_position_tolerance_m is None
            or not first.get("position_verification_requested")
            or first.get("task_succeeded") or first.get("position_correction_count", 0) > 0
            or first.get("final_armed") is not False or first.get("final_state") != "DISARMED"):
        return first
    reference = _finite_pose_copy(first.get("final_amcl_pose"))
    remaining = deadline - time.monotonic()
    if reference is None or remaining <= 35.0:
        first['position_verification_skipped_reason'] = 'no stopped pose or insufficient recovery time'
        return first
    pose, evidence = _verify_stopped_inspection_pose(
        reference, min(20.0, remaining - 15.0),str(waypoint.get('name','')))
    first["position_verification"] = evidence
    if pose is None:
        # Only an applied hypothesis invalidates the earlier route pose.
        changed = any(e.get("event") in {"runtime_hypothesis_invalidated", "initial_pose_set"}
                      for e in evidence.get("events", []))
        if changed:
            first["failure_policy"] = "abort"
            first["failure_stage"] = "position_alignment"
            first["error"] = "inspection position could not be independently verified"
        elif (point.verify_position_before_correction and deadline-time.monotonic()>15.0):
            # Rejected evidence cannot authorize translation or remove the
            # pre-existing bounded heading-only observation fallback.
            fallback = executor.execute(waypoint,index,first,_run_inspection_heading,
                run_id=run_id,correct_position=None,deadline_monotonic=deadline)
            fallback['initial_capture_attempt'] = first
            fallback['position_verification'] = evidence
            return fallback
        return first
    first["final_amcl_pose"] = pose
    distance = math.hypot(pose["x"]-waypoint["x"], pose["y"]-waypoint["y"])
    first["verified_position_distance_m"] = distance
    # A matching pose does not justify moving the catalog or chasing an OCR
    # failure. Keep the original result if there is no proven position error.
    if distance <= point.stopped_position_tolerance_m and not point.verify_position_before_correction:
        return first
    remaining = deadline - time.monotonic()
    if distance > .75 or remaining <= 15.0:
        first["failure_policy"] = "abort"
        first["error"] = "verified inspection position exceeds correction budget"
        return first
    correction_needed = distance > point.stopped_position_tolerance_m
    correction = (dict(_run_inspection_position(waypoint, pose, min(30.0, remaining - 10.0)))
                  if correction_needed else dict(status='PASS',final_armed=False,
                      final_state='DISARMED',final_amcl_pose=pose,motion_authorized=False))
    first["position_correction"] = correction
    first["position_correction_count"] = int(correction_needed)
    stopped = _finite_pose_copy(correction.get("final_amcl_pose"))
    if (correction.get("status") != "PASS" or correction.get("final_armed") is not False
            or correction.get("final_state") != "DISARMED" or stopped is None
            or math.hypot(stopped["x"]-waypoint["x"], stopped["y"]-waypoint["y"])
            > point.stopped_position_tolerance_m):
        first["failure_policy"] = "abort"
        first["error"] = "verified inspection reapproach did not finish safely"
        first["final_armed"] = correction.get("final_armed")
        first["final_state"] = correction.get("final_state")
        if stopped is not None:
            first["final_amcl_pose"] = stopped
        return first
    retried = executor.execute(
        waypoint, index, correction, _run_inspection_heading, run_id=run_id,
        correct_position=None, deadline_monotonic=deadline,
    )
    retried["initial_capture_attempt"] = first
    retried["position_correction_count"] = int(correction_needed)
    retried["position_verification"] = evidence
    retried["position_correction"] = correction
    return retried


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument(
        "--extract-global-pose",
        metavar="TRACE",
        help="print a converged global-localization pose as shell assignments",
    )
    parser.add_argument("--trace-file")
    parser.add_argument("--localize-first", action="store_true")
    parser.add_argument("--mark-repositioned", action="store_true",
                        help="invalidate the old pose inside stationary startup before resuming")
    parser.add_argument("--localization-cache-file")
    parser.add_argument("--localization-trace-file")
    parser.add_argument(
        "--checkpoint-file",
        default=os.getenv("RVCLAW_ROUTE_CHECKPOINT_FILE", ""),
        help="durable paused-route checkpoint; cleared after a complete route",
    )
    parser.add_argument(
        "--resume-context",
        default=os.getenv("RVCLAW_ROUTE_RESUME_CONTEXT", ""),
        help="checkpoint whose completed inspection results precede this route",
    )
    parser.add_argument(
        "--resume-localization-verified",
        action="store_true",
        help=(
            "rebase the first resumed segment on the current AMCL pose after an "
            "independent fast-localization PASS"
        ),
    )
    parser.add_argument("--arm-token")
    parser.add_argument("--waypoints-file", default=WAYPOINT.DEFAULT_WAYPOINTS_FILE)
    parser.add_argument(
        "--route",
        default=",".join(DEFAULT_ROUTE),
        help="comma-separated waypoint names, in execution order",
    )
    parser.add_argument(
        "--inspection-config",
        default=os.getenv("RVCLAW_INSPECTION_CONFIG", ""),
        help=(
            "optional waypoint inspection YAML; omitted keeps the validated "
            "navigation-only route unchanged"
        ),
    )
    parser.add_argument(
        "--inspection-readiness-only",
        action="store_true",
        help="perform read-only Studio API and meter calibration checks",
    )
    return parser.parse_args()


def _load_route(path: str, route_text: str):
    waypoints = load_waypoints(path)
    names = tuple(part.strip() for part in route_text.split(",") if part.strip())
    if not names:
        raise ValueError("route must contain at least one waypoint")
    if len(set(names)) != len(names):
        raise ValueError("route must not contain duplicate waypoint names")
    selected = []
    for name in names:
        if name not in waypoints:
            available = ", ".join(sorted(waypoints))
            raise ValueError(
                "unknown waypoint %s; available: %s" % (name, available)
            )
        waypoint = waypoints[name]
        if math.hypot(waypoint["x"], waypoint["y"]) > WAYPOINT.MAX_WAYPOINT_DISTANCE_M:
            raise ValueError(
                "waypoint %s is outside the reviewed map distance limit" % name
            )
        selected.append(waypoint)
    return names, selected


def _validate_trace(path_text: str) -> Path:
    if not path_text:
        raise ValueError("trace-file is required")
    path = Path(path_text).resolve()
    try:
        path.relative_to(WORKSPACE_ROOT.resolve())
    except ValueError as exc:
        raise ValueError("trace file must remain under the workspace root") from exc
    if path.exists() or not path.parent.is_dir():
        raise ValueError("trace file must be new and its parent must exist")
    return path


def load_route_resume_context(path_text, route_names):
    """Load only completed waypoint records from the matching paused route."""

    if not path_text:
        return []
    path = Path(path_text).expanduser().resolve()
    document = json.loads(path.read_text(encoding="utf-8"))
    if int(document.get("schema_version", 0)) != 1:
        raise ValueError("unsupported route resume checkpoint schema")
    requested = tuple(
        str(name).strip().upper() for name in document.get("route_requested", ())
    )
    completed = tuple(
        str(name).strip().upper() for name in document.get("completed_waypoints", ())
    )
    remaining = tuple(
        str(name).strip().upper() for name in document.get("remaining_waypoints", ())
    )
    expected_remaining = tuple(str(name).strip().upper() for name in route_names)
    if not requested or not remaining:
        raise ValueError("resume checkpoint route is incomplete")
    if any(not name for name in requested + completed + remaining):
        raise ValueError("resume checkpoint contains an empty waypoint")
    if len(set(requested)) != len(requested):
        raise ValueError("resume checkpoint route contains duplicates")
    if completed + remaining != requested:
        raise ValueError("resume checkpoint progress is not a contiguous route prefix")
    if remaining != expected_remaining:
        raise ValueError("resume checkpoint does not match the requested remaining route")
    failed_waypoint = str(document.get("failed_waypoint", "")).strip().upper()
    if failed_waypoint and failed_waypoint != remaining[0]:
        raise ValueError("resume checkpoint failed waypoint does not match route head")
    prior = []
    prior_names = []
    for item in document.get("route_results", ()):
        if not isinstance(item, dict) or item.get("status") != "PASS":
            continue
        waypoint = item.get("waypoint") or {}
        name = str(waypoint.get("name", "")).upper()
        if not name or name in remaining:
            continue
        prior.append(dict(item))
        prior_names.append(name)
    if tuple(prior_names) != completed:
        raise ValueError("resume checkpoint completed results do not match progress")
    return prior


def write_route_checkpoint(path_text, route_names, result):
    """Atomically save a resumable fail-stop checkpoint, or clear it on success."""

    if not path_text:
        return None
    path = Path(path_text).expanduser().resolve()
    if result.get("status") == "PASS":
        path.unlink(missing_ok=True)
        return None
    resume_plan = result.get("resume_plan")
    if not isinstance(resume_plan, dict):
        return None
    path.parent.mkdir(parents=True, exist_ok=True)
    completed_waypoints = [
        str(name).strip().upper()
        for name in result.get("completed_waypoints", ())
    ]
    remaining_waypoints = [
        str(name).strip().upper()
        for name in resume_plan.get("remaining_waypoints", ())
    ]
    requested_route = completed_waypoints + remaining_waypoints
    if not remaining_waypoints or any(not name for name in requested_route):
        raise ValueError("cannot write an incomplete route checkpoint")
    if len(set(requested_route)) != len(requested_route):
        raise ValueError("cannot write a checkpoint with duplicate waypoints")
    failed_waypoint = str(resume_plan.get("failed_waypoint", "")).strip().upper()
    if failed_waypoint and failed_waypoint != remaining_waypoints[0]:
        raise ValueError("checkpoint failed waypoint does not match remaining route")
    payload = {
        "schema_version": 1,
        "timestamp": time.time(),
        "route_requested": requested_route,
        "completed_waypoints": completed_waypoints,
        "remaining_waypoints": remaining_waypoints,
        "failed_waypoint": failed_waypoint or remaining_waypoints[0],
        "failure_code": resume_plan.get("failure_code"),
        "operator_action": resume_plan.get("operator_action"),
        "resume_command": resume_plan.get("resume_command"),
        "requires_global_localization": bool(
            resume_plan.get("requires_global_localization")
        ),
        "route_results": list(result.get("route", ())),
    }
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent,
        prefix=".inspection-route-checkpoint-", suffix=".tmp", delete=False,
    ) as stream:
        stream.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        stream.write("\n")
        stream.flush()
        temporary = Path(stream.name)
    os.replace(temporary, path)
    return path


def describe_plan(route_names):
    return {
        "hardware_io": False,
        "goal_mode": "absolute_waypoint_route",
        "frame_id": "map",
        "route": list(route_names),
        "required_arm_token": ROUTE_ARM_TOKEN,
        "single_navigation_stack": True,
        "initial_pose_required_once": True,
        "initial_pose_input": "RVCLAW_NAV2_INITIAL_POSE=x,y,yaw_deg or three RVCLAW_NAV2_INITIAL_* variables",
        "initial_heading_periodic": True,
        "global_localization_pose_import": "--extract-global-pose TRACE",
        "fresh_scan_gate_per_waypoint": True,
        "independent_static_map_localization_gate_at_route_start": True,
        "continuous_localization_gate_between_waypoints": True,
        "continuous_localization_max_position_jump_m": (
            CONTINUOUS_ROUTE_MAX_POSITION_JUMP_M
        ),
        "continuous_localization_max_yaw_jump_rad": (
            CONTINUOUS_ROUTE_MAX_YAW_JUMP_RAD
        ),
        "path_preflight_before_operator_gate": True,
        "all_route_segments_preflighted": True,
        "near_start_egress_supported": True,
        "near_start_egress_state": "NEAR_START_EGRESS_REQUIRED",
        "near_start_egress_distance_m": EGRESS_DISTANCE_M,
        "near_start_egress_minimum_distance_m": EGRESS_MIN_DISTANCE_M,
        "near_start_egress_adaptive_distance": True,
        "near_start_egress_speed_m_s": EGRESS_SPEED_M_S,
        "near_start_egress_max_checked_steps": MAX_AUTONOMOUS_EGRESS_STEPS,
        "near_start_egress_review_scope": "every_non_origin_segment",
        "near_start_egress_requires_relocalization": True,
        "near_start_egress_runtime_scan_checks": EGRESS_RUNTIME_MINIMUM_SCAN_CHECKS,
        "near_start_egress_modes": [EGRESS_MODE_STANDARD],
        "side_near_egress_field_certified": SIDE_NEAR_EGRESS_FIELD_CERTIFIED,
        "side_near_egress_policy": "STOP_UNTIL_FULL_ENVELOPE_CERTIFICATION",
        "near_start_egress_side_near_margin_m": EGRESS_SIDE_NEAR_SAFETY_MARGIN_M,
        "near_start_egress_profile_restored_before_success": True,
        "post_stop_check_per_waypoint": True,
        "fail_stop": True,
        "dynamic_obstacle_recovery": {
            "policy_version": DYNAMIC.POLICY_VERSION,
            "default_action_on_uncertainty": DYNAMIC.ACTION_HOLD_STOP,
            "flow": [
                DYNAMIC.ACTION_SLOW_AND_REPLAN,
                DYNAMIC.ACTION_REPLAN_ALTERNATE,
                DYNAMIC.ACTION_STOP_WAIT_REPLAN,
                DYNAMIC.ACTION_SPIN_SEARCH,
                DYNAMIC.ACTION_ABORT_GOAL,
            ],
            "max_replan_attempts": DYNAMIC.MAX_REPLAN_ATTEMPTS,
            "max_wait_attempts": DYNAMIC.MAX_WAIT_ATTEMPTS,
            "max_spin_attempts": DYNAMIC.MAX_SPIN_ATTEMPTS,
            "spin_requires_full_swept_clearance": True,
            "requires_fresh_motion_feedback": True,
            "progress_stall_policy": "stop_then_bounded_replan_then_abort",
            "reverse_allowed": False,
            "lateral_allowed": False,
        },
        "automatic_reverse": False,
        "origin_navigation_mode": ORIGIN_NAVIGATION_MODE,
        "intermediate_navigation_mode": INTERMEDIATE_NAVIGATION_MODE,
        "intermediate_heading_policy": "path_bearing_hint_with_relaxed_yaw_goal",
        "intermediate_behavior_tree": POSITION_ONLY_BT_NAME,
        "origin_pose_goal_includes_heading": ORIGIN_NAVIGATION_MODE == "absolute_pose",
        "origin_heading_mode": (
            "dwb_full_pose_with_origin_goal_checker_and_rotation_guard"
            if ORIGIN_NAVIGATION_MODE == "absolute_pose"
            else "position_stop_then_bounded_spin_and_final_xy_validation"
        ),
        "origin_behavior_tree": (ORIGIN_FULL_POSE_BT_NAME
                                 if ORIGIN_NAVIGATION_MODE == "absolute_pose" else POSITION_ONLY_BT_NAME),
        "origin_replan_policy": ("distance_triggered" if ORIGIN_NAVIGATION_MODE == "absolute_pose"
                                 else "position_only_then_no_replan_spin"),
        "origin_replan_distance_m": (ORIGIN_REPLAN_DISTANCE_M
                                     if ORIGIN_NAVIGATION_MODE == "absolute_pose" else None),
        "origin_goal_checker": ("origin_goal_checker" if ORIGIN_NAVIGATION_MODE == "absolute_pose"
                                else "position_goal_checker"),
        "origin_goal_xy_tolerance_m": ORIGIN_GOAL_XY_TOLERANCE_M,
        "origin_goal_yaw_tolerance_rad": ORIGIN_GOAL_YAW_TOLERANCE_RAD,
        "origin_post_entry_max_rotation_rad": ORIGIN_POST_ENTRY_MAX_ROTATION_RAD,
        "arrival_event_waypoints": ["A", "B", "C"],
        "arrival_event_directory_default": DEFAULT_ARRIVAL_EVENT_DIR,
        "origin_heading_skip_tolerance_deg": math.degrees(
            ORIGIN_HEADING_SKIP_TOLERANCE_RAD
        ),
        "origin_heading_tolerance_deg": math.degrees(
            ORIGIN_HEADING_TOLERANCE_RAD
        ),
        "origin_heading_spin_chunk_max_rad": ORIGIN_HEADING_SPIN_CHUNK_MAX_RAD,
        "origin_heading_max_chunks": ORIGIN_HEADING_MAX_CHUNKS,
        "origin_heading_max_total_rad": ORIGIN_HEADING_MAX_TOTAL_RAD,
        "origin_transit_timeout_s": ORIGIN_TRANSIT_ACTION_RESULT_TIMEOUT_S,
        "origin_verified_correction_limit_m": ORIGIN_VERIFIED_CORRECTION_MAX_M,
        "max_route_duration_s": ROUTE_MAX_DURATION_S,
    }


def preflight_exit_status(result):
    """Return a shell status for a route preflight result.

    A near-start lethal cell is an expected, reviewable state when the
    independent live-scan and static-map egress gates pass.  It must reach the
    operator gate so the runtime client can perform the bounded egress.  Any
    other preflight failure remains a hard failure.
    """
    if result.get("status") == "PASS":
        return 0
    egress_gate = result.get("safe_egress_gate") or {}
    if (
        result.get("status") == "REQUIRE_SAFE_EGRESS"
        and result.get("egress_required") is True
        and result.get("egress_state") == "NEAR_START_EGRESS_REQUIRED"
        and isinstance(egress_gate, dict)
        and egress_gate.get("status") == "PASS"
    ):
        return 0
    return 1


def _run_inspection_position(waypoint, observed_pose, timeout_s):
    """Reapproach once through the same path, scan and hardware safety gates."""
    return _run_one(
        waypoint,
        continuity_anchor=observed_pose,
        allow_near_start_egress=False,
        navigation_timeout_s=timeout_s,
    )


def _run_one(
    waypoint,
    allow_near_start_egress=False,
    continuity_anchor=None,
    navigation_timeout_s=None,
    _b_reference_recovery_used=False,
):
    """One normal leg plus one independently proved freshness recovery."""
    result = _run_one_attempt(waypoint, allow_near_start_egress, continuity_anchor,
                              navigation_timeout_s, _b_reference_recovery_used)
    if not confirmed_localization_freshness_stop(result):
        return result
    stopped = _finite_pose_copy(result.get('final_amcl_pose'))
    if stopped is None:
        return result
    pose, proof = _verify_stopped_return_pose(waypoint, use_origin_reference=False, require_disarmed=True)
    result['localization_freshness_verification'] = proof
    if pose is None or math.hypot(pose['x']-stopped['x'], pose['y']-stopped['y']) > .75:
        return result
    pose = dict(pose, freshness_recovery_disarmed_only=True)
    retried = _run_one_attempt(waypoint, False, pose,
                               navigation_timeout_s, _b_reference_recovery_used)
    retried['localization_freshness_attempts'] = [result]
    retried['localization_freshness_recovery_count'] = 1
    return retried


def _run_one_attempt(
    waypoint,
    allow_near_start_egress=False,
    continuity_anchor=None,
    navigation_timeout_s=None,
    _b_reference_recovery_used=False,
):
    if waypoint.get("name") == "ORIGIN":
        result = _run_origin(waypoint, continuity_anchor=continuity_anchor)
        if allow_near_start_egress and result.get('checked_egress_required') is True:
            return _run_staged_egress_then_retry(waypoint, result)
        return result
    if continuity_anchor is None:
        node = PositionOnlyWaypointClient(dict(waypoint))
    else:
        node = PositionOnlyWaypointClient(
            dict(waypoint),
            continuity_anchor=continuity_anchor,
        )
    if navigation_timeout_s is not None:
        node.inspection_position_deadline = time.monotonic() + float(navigation_timeout_s)
    node_destroyed = False
    try:
        if (waypoint.get('name') == 'B' and isinstance(continuity_anchor, dict)
                and continuity_anchor.get(VERIFIED_RESUME_REBASE_KEY) is True):
            # A local resume match can repeat the same corridor alias. Before
            # planning or arming B, prove the full stationary reference again.
            node.wait_for(lambda: node.armed is False and node.state is not None,
                          WAYPOINT.BASE.STARTUP_TIMEOUT_S, 'disarmed B resume')
            seed = node._b_reference_stationary_seed(passive_resume=True)
            node.destroy_node()
            node_destroyed = True
            pose, verification = _verify_stopped_inspection_pose(
                seed, 20., 'B', allow_any_heading=True)
            if pose is None:
                failed = _failure_result(node, RuntimeError(
                    'B resume localization could not be independently verified'))
                failed.update(automatic_recovery_forbidden=True,
                              b_resume_verification=verification)
                return failed
            resumed = _run_one_attempt(waypoint, allow_near_start_egress=allow_near_start_egress,
                continuity_anchor=pose, navigation_timeout_s=navigation_timeout_s,
                _b_reference_recovery_used=_b_reference_recovery_used)
            resumed['b_resume_verification'] = verification
            return resumed
        result = node.run_navigation()
        result.update(
            {
                "intermediate_navigation_mode": INTERMEDIATE_NAVIGATION_MODE,
                "catalog_waypoint_yaw": waypoint["yaw"],
            }
        )
        return result
    except BaseException as exc:
        if allow_near_start_egress and (
            _is_near_start_path_failure(node, exc)
            or isinstance(exc, SideNearTurnEgressRequired)
        ):
            initial_failure = exception_diagnostics(exc)
            initial_failure["departure_diagnostics"] = _departure_diagnostics(node)
            try:
                node.emergency_cleanup()
            except BaseException as cleanup_exc:
                node._record("cleanup_error", str(cleanup_exc))
            node.destroy_node()
            node_destroyed = True
            return _run_staged_egress_then_retry(waypoint, initial_failure)
        evidence = getattr(node, 'motion_map_last_evidence', None) or {}
        b_requested_stop = (waypoint.get('name') == 'B' and evidence.get('method') in {'B_reference','B_reference_arrival'}
                and isinstance(exc, RuntimeError)
                and str(exc) == 'in-motion scan/map localization disagreement; stopped')
        controlled_stop = False
        stopped = None
        try:
            if b_requested_stop:
                stopped = node.stop_for_b_reference()
                controlled_stop = True
            else:
                node.emergency_cleanup()
        except BaseException as cleanup_exc:
            node._record('cleanup_error', str(cleanup_exc))
            node.emergency_cleanup()
        failed = _failure_result(node, exc)
        if b_requested_stop:
            # The moving match only stops the robot. A new, disarmed, three-scan
            # consensus and the existing post-AMCL gates authorize any retry.
            failed['automatic_recovery_forbidden'] = True
            failed['b_reference_stop'] = evidence
            failed['b_reference_search_center'] = stopped
            if controlled_stop and not _b_reference_recovery_used:
                node.destroy_node()
                node_destroyed = True
                if (stopped is not None and failed.get('final_armed') is False
                        and failed.get('final_state') == 'DISARMED'
                        and failed.get('final_transition_reason') == 'operator_disarm'):
                    pose, verification = _verify_stopped_inspection_pose(
                        stopped, 20., 'B', allow_any_heading=True)
                    failed['b_reference_verification'] = verification
                    if pose is not None:
                        if (evidence.get('method')=='B_reference_arrival'
                                and math.hypot(pose['x']-waypoint['x'],pose['y']-waypoint['y'])<=.12):
                            # This is an intentional cancellation at a measured
                            # arrival, not a Nav2 success. The ordinary B heading
                            # and stopped geometry stages must still pass before
                            # arrival speech or camera work can start.
                            return dict(failed,status='PASS',error=None,
                                automatic_recovery_forbidden=False,final_amcl_pose=pose,
                                completion_mode='INDEPENDENT_GEOMETRY_STOP',
                                action_status=GoalStatus.STATUS_CANCELED)
                        retried = _run_one_attempt(waypoint, allow_near_start_egress=False,
                            continuity_anchor=pose, navigation_timeout_s=navigation_timeout_s,
                            _b_reference_recovery_used=True)
                        retried['b_reference_recovery'] = failed
                        return retried
        if isinstance(exc, SideNearTurnUnsafe):
            failed["automatic_recovery_forbidden"] = True
            failed["safety_pause_reason"] = "turn clearance cannot be certified"
        return failed
    finally:
        if not node_destroyed:
            node.destroy_node()


def _run_staged_egress_then_retry(waypoint, initial_failure):
    """Replan after *every* bounded step; never spin or blindly chain drives."""
    steps = []
    last_state = None
    while True:
        budget = step_budget(item["distance_m"] for item in steps)
        if budget["status"] != "ALLOW_CHECKED_STEP":
            return {
                "status": "FAIL", "error": "navigation obstacle: %s" % budget["reason"],
                "initial_path_failure": initial_failure,
                "safe_egress_steps": steps,
                "safe_egress": steps[-1] if steps else None,
                "waypoint": waypoint, "final_armed": False,
                "final_state": last_state,
                "automatic_recovery_forbidden": True,
                "safety_pause_reason": "autonomous escape budget exhausted",
            }
        egress = SafeEgressClient(
            prefer_shortest=(initial_failure.get("error_type") == "SideNearTurnEgressRequired"),
            turn_delta_rad=initial_failure.get("departure_diagnostics", {}).get(
                "initial_path_heading_delta_rad"),
        )
        try:
            egress_result = egress.run_egress()
            step_evidence = validate_completed_step(egress_result)
            if step_evidence["status"] == "PASS" and steps:
                step_evidence = validate_chain_continuity(steps[-1], egress_result)
            if step_evidence["status"] != "PASS":
                egress.emergency_cleanup()
        except BaseException as egress_exc:
            try:
                egress.emergency_cleanup()
            except BaseException as cleanup_exc:
                egress._record("cleanup_error", str(cleanup_exc))
            egress_result = egress.result("FAIL", str(egress_exc))
            egress_result.update(exception_diagnostics(egress_exc))
            step_evidence = {"status": "STOP", "reason": str(egress_exc)}
        finally:
            egress.destroy_node()
        if step_evidence["status"] != "PASS":
            return {
                "status": "FAIL",
                "error": "safe egress stopped: %s" % step_evidence["reason"],
                "initial_path_failure": initial_failure,
                "safe_egress_steps": steps,
                "safe_egress": egress_result,
                "step_evidence": step_evidence,
                "waypoint": waypoint,
                "final_armed": getattr(egress, "armed", None),
                "final_state": getattr(egress, "state", None),
                "automatic_recovery_forbidden": True,
                "safety_pause_reason": step_evidence["reason"],
            }
        steps.append(dict(egress_result))
        if waypoint.get('name') == 'ORIGIN':
            # Run the complete ORIGIN contract after the checked step:
            # position travel, stopped heading, and final position validation.
            result = _run_origin(waypoint, continuing_egress=True)
            last_state = result.get('final_state')
            if (result.get('status') != 'PASS'
                    and result.get('checked_egress_required') is True
                    and result.get('final_armed') is False):
                continue
            result.update({
                'near_start_egress_state': 'STAGED_EGRESS_RELOCALIZED_AND_REPLANNED',
                'safe_egress': steps[-1], 'safe_egress_steps': steps,
                'initial_path_failure': initial_failure,
            })
            return result
        retry_node = PositionOnlyWaypointClient(
            dict(waypoint), continuing_egress=True,
        )
        try:
            result = retry_node.run_navigation()
            result.update({
                "intermediate_navigation_mode": INTERMEDIATE_NAVIGATION_MODE,
                "catalog_waypoint_yaw": waypoint["yaw"],
                "near_start_egress_state": "STAGED_EGRESS_RELOCALIZED_AND_REPLANNED",
                "safe_egress": steps[-1],
                "safe_egress_steps": steps,
                "initial_path_failure": initial_failure,
            })
            return result
        except BaseException as retry_exc:
            try:
                retry_node.emergency_cleanup()
            except BaseException as cleanup_exc:
                retry_node._record("cleanup_error", str(cleanup_exc))
            last_state = getattr(retry_node, "state", None)
            can_stage_again = (
                getattr(retry_node, "armed", None) is False
                and (isinstance(retry_exc, SideNearTurnEgressRequired)
                     or _is_near_start_path_failure(retry_node, retry_exc))
            )
            if can_stage_again:
                # The next iteration must independently read a new scan,
                # current map and localized pose before moving another step.
                continue
            failed = _failure_result(retry_node, retry_exc)
            failed.update({
                "safe_egress": steps[-1],
                "safe_egress_steps": steps,
                "initial_path_failure": initial_failure,
            })
            if isinstance(retry_exc, SideNearTurnUnsafe):
                failed["automatic_recovery_forbidden"] = True
                failed["safety_pause_reason"] = "turn clearance cannot be certified"
            return failed
        finally:
            retry_node.destroy_node()


def _preflight_one(waypoint, start_pose, defer_if_current_goal=False):
    if (
        waypoint.get("name") == "ORIGIN"
        and ORIGIN_NAVIGATION_MODE == "absolute_pose"
    ):
        node = WAYPOINT.WaypointClient(waypoint)
    else:
        node = PositionOnlyWaypointClient(
            dict(waypoint),
            start_pose_hint=start_pose,
        )
    try:
        result = node.run_path_preflight(start_pose=start_pose)
        if waypoint.get("name") != "ORIGIN":
            result.update(
                {
                    "intermediate_navigation_mode": INTERMEDIATE_NAVIGATION_MODE,
                    "intermediate_behavior_tree": POSITION_ONLY_BT_NAME,
                }
            )
        elif ORIGIN_NAVIGATION_MODE == "split_spin":
            result.update(
                {
                    "origin_position_navigation_mode": "position_only",
                    "origin_heading_navigation_mode": "bounded_shortest_relative_spin",
                    "intermediate_behavior_tree": POSITION_ONLY_BT_NAME,
                }
            )
        return result
    except BaseException as exc:
        current_pose = node.latest_amcl_pose
        current_goal_distance = None
        egress_gate = None
        if current_pose is not None:
            current_goal_distance = math.hypot(
                current_pose["x"] - waypoint["x"],
                current_pose["y"] - waypoint["y"],
            )
        if start_pose is None and (
            _is_near_start_path_failure(node, exc)
            or isinstance(exc, SideNearTurnEgressRequired)
        ):
            try:
                egress_gate = _evaluate_node_egress_gates(node)
            except BaseException as gate_exc:
                egress_gate = {
                    "status": "FAIL",
                    "failures": ["safe egress gate error: %s" % gate_exc],
                }
            if egress_gate.get("status") == "PASS":
                return {
                    "status": "REQUIRE_SAFE_EGRESS",
                    "error": None,
                    "egress_required": True,
                    "egress_state": "NEAR_START_EGRESS_REQUIRED",
                    "egress_reason": (
                        "side-near sharp turn is not sweep-clear; only a "
                        "bounded straight step has a separately reviewed gate"
                        if isinstance(exc, SideNearTurnEgressRequired)
                        else "lethal start cell; forward egress corridor is clear"
                    ),
                    "side_turn_review": getattr(node, "side_turn_review", None),
                    "safe_egress_gate": egress_gate,
                    "path_departure_poses": getattr(node, "path_departure_poses", None),
                    "current_goal_distance_m": current_goal_distance,
                    "goal_cost": node.goal_cost,
                    "goal_pose": node.goal_pose,
                    "start_pose": start_pose,
                    "start_source": "current_amcl",
                    "path_check_status": node.path_check_status,
                    "path_check_pose_count": node.path_check_pose_count,
                    "path_check_elapsed_s": node.path_check_elapsed_s,
                    "path_check_error": node.path_check_error,
                    "pre_arm_front_clearance_m": node.pre_arm_front_clearance_m,
                    "start_cost": node.start_cost,
                    "initial_path_heading_delta_rad": node.initial_path_heading_delta_rad,
                    "start_clearance_mode": node.start_clearance_mode,
                    "final_armed": node.armed,
                }
        if (
            ORIGIN_NAVIGATION_MODE == "split_spin"
            and waypoint.get("name") == "ORIGIN"
            and start_pose is None
            and current_goal_distance is not None
            and current_goal_distance <= ORIGIN_GOAL_XY_TOLERANCE_M
            and node.armed is False
        ):
            return {
                "status": "DEFERRED",
                "error": None,
                "deferred_reason": "ORIGIN position is already reached; heading-only stage",
                "current_goal_distance_m": current_goal_distance,
                "goal_cost": node.goal_cost,
                "goal_pose": waypoint,
                "start_pose": start_pose,
                "start_source": "current_amcl",
                "path_check_status": node.path_check_status,
                "path_check_pose_count": node.path_check_pose_count,
                "path_check_elapsed_s": node.path_check_elapsed_s,
                "path_check_error": node.path_check_error,
                "pre_arm_front_clearance_m": getattr(
                    node, "pre_arm_front_clearance_m", None
                ),
                "start_cost": getattr(node, "start_cost", None),
                "initial_path_heading_delta_rad": getattr(
                    node, "initial_path_heading_delta_rad", None
                ),
                "start_clearance_mode": getattr(
                    node, "start_clearance_mode", None
                ),
                "final_armed": node.armed,
            }
        if (
            defer_if_current_goal
            and str(exc) == "relative goal cell is not admitted free space"
            and current_goal_distance is not None
            and current_goal_distance <= MAX_DEFERRED_RETURN_DISTANCE_M
            and node.armed is False
        ):
            return {
                "status": "DEFERRED",
                "error": None,
                "deferred_reason": "final return goal overlaps current robot pose",
                "current_goal_distance_m": current_goal_distance,
                "goal_cost": node.goal_cost,
                "goal_pose": waypoint,
                "start_pose": start_pose,
                "start_source": "explicit",
                "path_check_status": node.path_check_status,
                "path_check_pose_count": node.path_check_pose_count,
                "path_check_elapsed_s": node.path_check_elapsed_s,
                "path_check_error": node.path_check_error,
                "pre_arm_front_clearance_m": getattr(
                    node, "pre_arm_front_clearance_m", None
                ),
                "start_cost": getattr(node, "start_cost", None),
                "initial_path_heading_delta_rad": getattr(
                    node, "initial_path_heading_delta_rad", None
                ),
                "start_clearance_mode": getattr(
                    node, "start_clearance_mode", None
                ),
                "final_armed": node.armed,
            }
        diagnostic = exception_diagnostics(exc)
        return {
            "status": "FAIL",
            **diagnostic,
            "static_map_localization_gate": getattr(node, "localization_gate_result", None),
            "side_turn_review": getattr(node, "side_turn_review", None),
            "safe_egress_gate": egress_gate,
            "path_departure_poses": getattr(node, "path_departure_poses", None),
            "current_goal_distance_m": current_goal_distance,
            "goal_cost": node.goal_cost,
            "goal_pose": waypoint,
            "start_pose": start_pose,
            "start_source": "explicit" if start_pose is not None else "current_amcl",
            "path_check_status": node.path_check_status,
            "path_check_pose_count": node.path_check_pose_count,
            "path_check_elapsed_s": node.path_check_elapsed_s,
            "path_check_error": node.path_check_error,
            "pre_arm_front_clearance_m": getattr(
                node, "pre_arm_front_clearance_m", None
            ),
            "start_cost": getattr(node, "start_cost", None),
            "initial_path_heading_delta_rad": getattr(
                node, "initial_path_heading_delta_rad", None
            ),
            "start_clearance_mode": getattr(node, "start_clearance_mode", None),
            "final_armed": node.armed,
        }
    finally:
        node.destroy_node()


def run_route_preflight(waypoints, preflight_one=_preflight_one):
    results = []
    failed_waypoints = []
    deferred_segments = []
    egress_segments = []
    start_pose = None
    start_name = "CURRENT"
    for index, waypoint in enumerate(waypoints, start=1):
        allow_deferred_current_goal = (
            waypoint.get("name") == "ORIGIN" and index == len(waypoints)
        )
        result = dict(
            preflight_one(
                waypoint,
                start_pose,
                allow_deferred_current_goal,
            )
        )
        result["route_index"] = index
        result["segment"] = "%s->%s" % (start_name, waypoint["name"])
        result["waypoint"] = waypoint
        results.append(result)
        if result.get("status") == "REQUIRE_SAFE_EGRESS" and index == 1:
            egress_segments.append(result["segment"])
        elif result.get("status") != "PASS":
            if result.get("status") == "DEFERRED":
                deferred_segments.append(result["segment"])
            else:
                failed_waypoints.append(waypoint["name"])
        goal_pose = result.get("goal_pose")
        start_pose = {
            "x": waypoint["x"],
            "y": waypoint["y"],
            "yaw": (
                goal_pose["yaw"]
                if isinstance(goal_pose, dict) and "yaw" in goal_pose
                else waypoint["yaw"]
            ),
        }
        start_name = waypoint["name"]
    if failed_waypoints:
        return {
            "status": "FAIL",
            "error": "path preflight failed for segment(s) ending at %s"
            % ",".join(failed_waypoints),
            "segments": results,
            "deferred_segments": deferred_segments,
        }
    return {
        "status": "PASS",
        "error": None,
        "segments": results,
        "deferred_segments": deferred_segments,
        "egress_required": bool(egress_segments),
        "egress_segments": egress_segments,
    }


def publish_arrival_event(waypoint, event_dir):
    """Atomically publish a verified A/B/C arrival for the voice service."""
    name = str(waypoint.get("name", "")).upper()
    sequence = {"A": "100", "B": "200", "C": "300"}
    if name not in sequence:
        return None
    target_dir = Path(event_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "event_id": str(uuid4()),
        "event": "waypoint_arrived",
        "waypoint": name,
        "status": "PASS",
        "timestamp": time.time(),
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target_dir,
        prefix=".arrival-",
        suffix=".tmp",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        stream.flush()
        temp_path = Path(stream.name)
    final_path = target_dir / (
        "%s-waypoint-%s-%s.json" % (sequence[name], name, payload["event_id"])
    )
    facts_dir = target_dir / "facts"
    facts_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=facts_dir,
        prefix=".arrival-", suffix=".tmp", delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        stream.flush()
        fact_temp = Path(stream.name)
    os.replace(fact_temp, facts_dir / final_path.name)
    os.replace(temp_path, final_path)
    return final_path


def publish_inspection_result_event(waypoint, task_result, event_dir):
    """Atomically publish a meter result for the independent voice service."""

    name = str(waypoint.get("name", "")).upper()
    sequence = {"A": "150", "B": "250", "C": "350"}
    spoken_name = {
        "A": "第一个巡检点",
        "B": "第二个巡检点",
        "C": "第三个巡检点",
    }
    if name not in sequence or task_result.get("status") == "SKIPPED":
        return None

    succeeded = (
        str(task_result.get("status", "")).upper() == "PASS"
        and task_result.get("task_succeeded") is True
    )
    failure_stage = None
    if succeeded:
        abnormal = task_result.get("is_anomaly") is True
        event_name = "meter_inspection_result"
        condition = "ABNORMAL" if abnormal else "NORMAL"
        speech_key = "meter_abnormal" if abnormal else "meter_normal"
        result_text = "异常" if abnormal else "正常"
        speech_text = "%s。仪表检测完成。结果%s。" % (
            spoken_name[name],
            result_text,
        )
        status = "PASS"
    else:
        failure_stage = str(task_result.get("failure_stage") or "vision").lower()
        condition = "UNKNOWN"
        if failure_stage == "heading_alignment":
            event_name = "meter_inspection_heading_failed"
            speech_key = "meter_heading_failed"
            speech_text = (
                "%s。观察方向调整未完成，未执行仪表识别。将继续后续任务。"
                % spoken_name[name]
            )
        elif failure_stage == "runtime":
            event_name = "meter_inspection_runtime_failed"
            speech_key = "meter_runtime_failed"
            speech_text = "%s。仪表巡检系统异常。将继续后续任务。" % spoken_name[name]
        else:
            failure_stage = "vision"
            event_name = "meter_inspection_failed"
            speech_key = "meter_unreadable"
            speech_text = "%s。仪表检测失败。将继续后续任务。" % spoken_name[name]
        status = "FAIL"

    target_dir = Path(event_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "event_id": str(uuid4()),
        "event": event_name,
        "waypoint": name,
        "status": status,
        "condition": condition,
        "speech_key": speech_key,
        "speech_text": speech_text,
        "timestamp": time.time(),
        "task_id": task_result.get("task_id"),
        "meter_id": task_result.get("meter_id"),
        "meter_kind": task_result.get("kind"),
        "value": task_result.get("value"),
        "unit": task_result.get("unit"),
        "confidence": task_result.get("confidence"),
        "failure_stage": failure_stage,
        "trial_count": task_result.get("trial_count", len(task_result.get("trials") or [])),
        "result_path": task_result.get("result_path"),
        "error": task_result.get("error"),
        "requires_ack": True,
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target_dir,
        prefix=".meter-result-",
        suffix=".tmp",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        stream.flush()
        temp_path = Path(stream.name)
    final_path = target_dir / (
        "%s-meter-%s-%s.json" % (sequence[name], name, payload["event_id"])
    )
    os.replace(temp_path, final_path)
    return final_path


def _inspection_count_word(value):
    words = {0: "零", 1: "一", 2: "两", 3: "三"}
    number = int(value)
    return words.get(number, str(number))


def _inspection_count_clause(value, nonzero_prefix, zero_text):
    number = int(value)
    if number == 0:
        return zero_text
    return "%s%s项" % (nonzero_prefix, _inspection_count_word(number))


def inspection_summary_text(summary):
    """Build the fixed Chinese summary announced after a safe ORIGIN return."""

    clauses = [
        _inspection_count_clause(
            summary["attempted"], "共检查", "没有执行巡检点检查"
        ),
        _inspection_count_clause(
            summary["completed"], "成功完成", "没有成功完成的点位"
        ),
    ]
    for key, prefix in (
        ("heading_failed", "观察方向失败"),
        ("vision_failed", "仪表检测失败"),
        ("runtime_failed", "系统异常"),
    ):
        number = int(summary[key])
        if number > 0:
            clauses.append("%s%s项" % (prefix, _inspection_count_word(number)))
    clauses.extend(
        [
            _inspection_count_clause(
                summary["normal"], "正常结果", "没有正常结果"
            ),
            _inspection_count_clause(
                summary["abnormal"], "异常结果", "没有异常结果"
            ),
        ]
    )
    return "巡检结束。%s。" % "。".join(clauses)


def summarize_inspection_results(route_results):
    """Count readable, failed, normal and abnormal point inspections once."""

    attempted = normal = abnormal = failed = skipped = 0
    heading_failed = vision_failed = runtime_failed = 0
    point_results = []
    for route_result in route_results:
        task = route_result.get("inspection_task")
        if not isinstance(task, dict):
            continue
        waypoint = str((route_result.get("waypoint") or {}).get("name", "")).upper()
        status = str(task.get("status", "")).upper()
        if status == "SKIPPED":
            skipped += 1
            continue
        attempted += 1
        if status == "PASS" and task.get("task_succeeded") is True:
            condition = "ABNORMAL" if task.get("is_anomaly") is True else "NORMAL"
            if condition == "ABNORMAL":
                abnormal += 1
            else:
                normal += 1
        else:
            condition = "FAILED"
            failed += 1
            failure_stage = str(task.get("failure_stage") or "vision").lower()
            if failure_stage == "heading_alignment":
                heading_failed += 1
            elif failure_stage == "runtime":
                runtime_failed += 1
            else:
                failure_stage = "vision"
                vision_failed += 1
        point_results.append(
            {
                "waypoint": waypoint,
                "condition": condition,
                "failure_stage": None if condition != "FAILED" else failure_stage,
                "result_path": task.get("result_path"),
                "error": task.get("error"),
            }
        )
    completed = normal + abnormal
    if attempted != completed + failed:
        raise RuntimeError("inspection summary accounting invariant failed")
    if failed != heading_failed + vision_failed + runtime_failed:
        raise RuntimeError("inspection failure-stage accounting invariant failed")
    summary = {
        "attempted": attempted,
        "completed": completed,
        "failed": failed,
        "heading_failed": heading_failed,
        "vision_failed": vision_failed,
        "runtime_failed": runtime_failed,
        "normal": normal,
        "abnormal": abnormal,
        "skipped": skipped,
        "outcome": "PASS" if failed == 0 else "COMPLETED_WITH_FAILURES",
        "point_results": point_results,
    }
    summary["speech_text"] = inspection_summary_text(summary)
    summary["speech_key"] = "inspection_summary"
    return summary


def publish_inspection_summary_event(summary, event_dir):
    """Atomically publish the final count summary after ORIGIN is reached."""

    if not isinstance(summary, dict) or int(summary.get("attempted", 0)) <= 0:
        return None
    target_dir = Path(event_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "event_id": str(uuid4()),
        "event": "inspection_summary",
        "status": "PASS",
        "timestamp": time.time(),
        "requires_ack": True,
        **summary,
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target_dir,
        prefix=".inspection-summary-",
        suffix=".tmp",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        stream.flush()
        temp_path = Path(stream.name)
    final_path = target_dir / (
        "850-inspection-summary-%s.json" % payload["event_id"]
    )
    os.replace(temp_path, final_path)
    return final_path


def publish_navigation_recovery_event(
    event_name,
    waypoint,
    decision,
    event_dir,
    *,
    attempt=None,
):
    """Publish one acknowledged progress or pause prompt for route recovery."""

    event_profiles = {
        "navigation_recovery_started": (
            "STARTED",
            "400",
            decision.get("started_speech_key"),
        ),
        "navigation_recovery_succeeded": (
            "PASS",
            "450",
            "navigation_recovery_succeeded",
        ),
        "navigation_paused": (
            "FAIL",
            "500",
            decision.get("paused_speech_key"),
        ),
    }
    if event_name not in event_profiles:
        raise ValueError("unsupported navigation recovery event: %s" % event_name)
    status, sequence, speech_key = event_profiles[event_name]
    prompt = NAVIGATION_RECOVERY_PROMPTS.get(speech_key)
    if prompt is None:
        raise ValueError("navigation recovery prompt is unavailable: %s" % speech_key)
    target_dir = Path(event_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "event_id": str(uuid4()),
        "event": event_name,
        "waypoint": str(waypoint.get("name", "")).upper(),
        "status": status,
        "failure_code": decision.get("code"),
        "attempt": attempt,
        "max_attempts": decision.get("max_attempts"),
        "speech_key": speech_key,
        "speech_text": prompt[0],
        "operator_action": decision.get("operator_action"),
        "resume_command": decision.get("resume_command"),
        "requires_global_localization": bool(
            decision.get("requires_global_localization")
        ),
        "requires_ack": True,
        "timestamp": time.time(),
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target_dir,
        prefix=".navigation-recovery-",
        suffix=".tmp",
        delete=False,
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        stream.flush()
        temp_path = Path(stream.name)
    final_path = target_dir / (
        "%s-%s-%s.json" % (sequence, event_name, payload["event_id"])
    )
    os.replace(temp_path, final_path)
    return final_path


def speech_ack_path(event_path):
    return Path(str(event_path) + ".ack")


def wait_for_speech_ack(
    event_path,
    timeout_s,
    *,
    clock=time.monotonic,
    sleeper=time.sleep,
):
    """Wait a bounded time for the voice owner to finish a prompt attempt."""

    timeout = max(0.0, float(timeout_s))
    if timeout <= 0.0:
        return {"status": "DISABLED", "played": None, "waited_s": 0.0}
    event_path = Path(event_path)
    try:
        event = json.loads(event_path.read_text(encoding="utf-8"))
        event_id = str(event.get("event_id", ""))
    except (OSError, ValueError, AttributeError):
        event_id = ""
    ack_path = speech_ack_path(event_path)
    started = clock()
    deadline = started + timeout
    while clock() < deadline:
        if ack_path.is_file():
            try:
                ack = json.loads(ack_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                ack = None
            if isinstance(ack, dict) and (
                not event_id or str(ack.get("event_id", "")) == event_id
            ):
                ack_path.unlink(missing_ok=True)
                return {
                    **ack,
                    "waited_s": round(max(0.0, clock() - started), 3),
                }
        sleeper(min(SPEECH_ACK_POLL_INTERVAL_S, max(0.0, deadline - clock())))
    return {
        "status": "TIMEOUT",
        "played": False,
        "event_id": event_id or None,
        "waited_s": round(max(0.0, clock() - started), 3),
        "error": "voice playback acknowledgement timed out",
    }


def _run_route_impl(
    waypoints,
    run_one=_run_one,
    clock=time.monotonic,
    arrival_event_dir=None,
    task_runner=None,
    speech_ack_timeout_s=0.0,
    navigation_speech_ack_timeout_s=0.0,
    sleeper=time.sleep,
    initial_results=None,
    resume_localization_verified=False,
    localization_profile=None,
    startup_continuity_anchor=None,
):
    if arrival_event_dir is not None:
        Path(arrival_event_dir).mkdir(parents=True, exist_ok=True)
    results = [dict(item) for item in (initial_results or ())]
    route_index_offset = len(results)
    started = clock()
    continuity_anchor = route_start_continuity_anchor(
        results,
        resume_localization_verified,
    )
    if continuity_anchor is None:
        continuity_anchor = _finite_pose_copy(startup_continuity_anchor)
    warnings = []
    for local_index, waypoint in enumerate(waypoints, start=1):
        index = route_index_offset + local_index
        if localization_profile is not None:
            localization_profile.select(waypoint)
        recovery_attempts = 0
        recovery_decision = None
        attempt_history = []
        retry_anchor = continuity_anchor
        while True:
            if run_one is _run_one:
                result = dict(
                    run_one(
                        waypoint,
                        allow_near_start_egress=True,
                        continuity_anchor=retry_anchor,
                    )
                )
            else:
                result = dict(run_one(waypoint))
            if (run_one is _run_one and waypoint.get('name') in ('A','B','C')
                    and result.get('status') == 'PASS'):
                prepare = getattr(task_runner, 'prepare_arrival', None)
                if prepare is not None:
                    result = prepare(waypoint, result)
            result["route_index"] = index
            result["navigation_attempt"] = recovery_attempts + 1
            if result.get("status") == "PASS":
                attempt_history.append(dict(result))
                if recovery_attempts:
                    result["navigation_recovered"] = True
                    result["navigation_recovery_attempts"] = recovery_attempts
                    result["navigation_failure_code"] = recovery_decision.get("code")
                    if arrival_event_dir is not None:
                        event_path = publish_navigation_recovery_event(
                            "navigation_recovery_succeeded",
                            waypoint,
                            recovery_decision,
                            arrival_event_dir,
                            attempt=recovery_attempts,
                        )
                        result["navigation_recovery_succeeded_event_file"] = str(
                            event_path
                        )
                        ack = wait_for_speech_ack(
                            event_path,
                            navigation_speech_ack_timeout_s,
                            clock=clock,
                            sleeper=sleeper,
                        )
                        result["navigation_recovery_succeeded_speech_ack"] = ack
                        if ack.get("status") not in {"PASS", "DISABLED"}:
                            warnings.append(
                                "navigation recovery speech was not confirmed at waypoint %s: %s"
                                % (waypoint["name"], ack.get("status"))
                            )
                result["navigation_attempts"] = attempt_history
                break

            decision = classify_navigation_failure(result)
            result["navigation_failure"] = decision
            can_retry = (
                retry_is_safe(result, decision)
                and result.get("automatic_recovery_forbidden") is not True
                and recovery_attempts < int(decision.get("max_attempts", 0))
                and clock() - started <= ROUTE_MAX_DURATION_S
            )
            result["automatic_recovery_eligible"] = can_retry
            attempt_history.append(dict(result))
            if not can_retry:
                result["navigation_attempts"] = attempt_history
                results.append(result)
                completed = [
                    item["waypoint"]["name"]
                    for item in results
                    if item.get("status") == "PASS"
                ]
                pause_event = None
                pause_ack = None
                if arrival_event_dir is not None:
                    pause_event = publish_navigation_recovery_event(
                        "navigation_paused",
                        waypoint,
                        decision,
                        arrival_event_dir,
                        attempt=recovery_attempts,
                    )
                    pause_ack = wait_for_speech_ack(
                        pause_event,
                        navigation_speech_ack_timeout_s,
                        clock=clock,
                        sleeper=sleeper,
                    )
                    result["navigation_paused_event_file"] = str(pause_event)
                    result["navigation_paused_speech_ack"] = pause_ack
                    if pause_ack.get("status") not in {"PASS", "DISABLED"}:
                        warnings.append(
                            "navigation pause speech was not confirmed at waypoint %s: %s"
                            % (waypoint["name"], pause_ack.get("status"))
                        )
                remaining = [
                    str(item.get("name", "")).upper()
                    for item in waypoints[local_index - 1 :]
                ]
                return {
                    "status": "FAIL",
                    "route_state": "PAUSED",
                    "error": "route paused after waypoint %s failed" % waypoint["name"],
                    "duration_s": clock() - started,
                    "route": results,
                    "completed_waypoints": completed,
                    "resume_plan": {
                        "failed_waypoint": str(waypoint.get("name", "")).upper(),
                        "remaining_waypoints": remaining,
                        "failure_code": decision.get("code"),
                        "operator_action": decision.get("operator_action"),
                        "resume_command": decision.get("resume_command"),
                        "requires_global_localization": bool(
                            decision.get("requires_global_localization")
                        ),
                    },
                    "warnings": warnings,
                }

            recovery_attempts += 1
            recovery_decision = decision
            if (arrival_event_dir is not None
                    and decision.get("started_speech_key") is not None):
                event_path = publish_navigation_recovery_event(
                    "navigation_recovery_started",
                    waypoint,
                    decision,
                    arrival_event_dir,
                    attempt=recovery_attempts,
                )
                attempt_history[-1]["navigation_recovery_started_event_file"] = str(
                    event_path
                )
                ack = wait_for_speech_ack(
                    event_path,
                    navigation_speech_ack_timeout_s,
                    clock=clock,
                    sleeper=sleeper,
                )
                attempt_history[-1]["navigation_recovery_started_speech_ack"] = ack
                if ack.get("status") not in {"PASS", "DISABLED"}:
                    warnings.append(
                        "navigation recovery start speech was not confirmed at waypoint %s: %s"
                        % (waypoint["name"], ack.get("status"))
                    )
            failed_pose = result.get("final_amcl_pose")
            if isinstance(failed_pose, dict):
                retry_anchor = dict(failed_pose)
            sleeper(float(decision.get("retry_delay_s", 0.0)))

        results.append(result)
        completed = [
            item["waypoint"]["name"]
            for item in results
            if item.get("status") == "PASS"
        ]
        final_amcl_pose = result.get("final_amcl_pose")
        if isinstance(final_amcl_pose, dict):
            continuity_anchor = dict(final_amcl_pose)
        elif run_one is _run_one and index < len(waypoints):
            return {
                "status": "FAIL",
                "error": (
                    "route stopped after waypoint %s returned no final AMCL pose"
                    % waypoint["name"]
                ),
                "duration_s": clock() - started,
                "route": results,
                "completed_waypoints": completed,
            }
        if arrival_event_dir is not None:
            event_path = publish_arrival_event(
                result.get("waypoint") or waypoint,
                arrival_event_dir,
            )
            if event_path is not None:
                result["arrival_event_file"] = str(event_path)
        if task_runner is not None:
            try:
                task_result = dict(task_runner(waypoint, index, result))
            except BaseException as exc:
                task_result = {
                    "status": "FAIL",
                    "task_succeeded": False,
                    "failure_policy": "abort",
                    "motion_authorized": False,
                    **exception_diagnostics(exc),
                }
            result["inspection_task"] = task_result
            if arrival_event_dir is not None:
                event_path = publish_inspection_result_event(
                    result.get("waypoint") or waypoint,
                    task_result,
                    arrival_event_dir,
                )
                if event_path is not None:
                    task_result["speech_event_file"] = str(event_path)
                    # Detection is already complete and hardware is stopped.
                    # The independent voice owner can speak its result while
                    # the next segment prepares; do not add 5+ idle seconds.
                    speech_ack = {"status": "QUEUED", "played": None,
                                  "waited_s": 0.0}
                    task_result["speech_ack"] = speech_ack
                    if speech_ack.get("status") not in {"PASS", "DISABLED", "QUEUED"}:
                        warnings.append(
                            "inspection speech was not confirmed at waypoint %s: %s"
                            % (waypoint["name"], speech_ack.get("status"))
                        )
            continuity_anchor = update_continuity_anchor_after_task(
                continuity_anchor,
                task_result,
            )
            if task_result.get("status") not in {"PASS", "SKIPPED"}:
                if task_result.get("failure_policy", "abort") == "abort":
                    completed_tasks = [
                        item["waypoint"]["name"]
                        for item in results[:-1]
                        if item.get("status") == "PASS"
                    ]
                    return {
                        "status": "FAIL",
                        "error": "inspection task failed at waypoint %s" % waypoint["name"],
                        "duration_s": clock() - started,
                        "route": results,
                        "completed_waypoints": completed_tasks,
                    }
        if clock() - started > ROUTE_MAX_DURATION_S:
            return {
                "status": "FAIL",
                "error": "route duration limit exceeded",
                "duration_s": clock() - started,
                "route": results,
                "completed_waypoints": completed,
            }
    summary = summarize_inspection_results(results)
    if (
        arrival_event_dir is not None
        and summary["attempted"] > 0
        and results
        and str(results[-1].get("waypoint", {}).get("name", "")).upper()
        == "ORIGIN"
    ):
        summary_event = publish_inspection_summary_event(summary, arrival_event_dir)
        if summary_event is not None:
            summary["speech_event_file"] = str(summary_event)
            summary_ack = wait_for_speech_ack(
                summary_event,
                speech_ack_timeout_s,
                clock=clock,
                sleeper=sleeper,
            )
            summary["speech_ack"] = summary_ack
            if summary_ack.get("status") not in {"PASS", "DISABLED"}:
                warnings.append(
                    "inspection summary speech was not confirmed: %s"
                    % summary_ack.get("status")
                )
    return {
        "status": "PASS",
        "error": None,
        "duration_s": clock() - started,
        "route": results,
        "completed_waypoints": [item["waypoint"]["name"] for item in results],
        "inspection_summary": summary,
        "warnings": warnings,
    }


_NATIVE_ROUTE_RUNNER = _run_one


def run_route(waypoints, run_one=_run_one, clock=time.monotonic,
              arrival_event_dir=None, task_runner=None, speech_ack_timeout_s=0.0,
              navigation_speech_ack_timeout_s=0.0, sleeper=time.sleep,
              initial_results=None, resume_localization_verified=False,
              startup_continuity_anchor=None):
    profile=None
    if run_one is _NATIVE_ROUTE_RUNNER:
        from inspection_localization_profile import InspectionLocalizationProfile
        profile=InspectionLocalizationProfile()
    result=None
    try:
        result=_run_route_impl(waypoints,run_one,clock,arrival_event_dir,task_runner,
            speech_ack_timeout_s,navigation_speech_ack_timeout_s,sleeper,
            initial_results,resume_localization_verified,localization_profile=profile,
            startup_continuity_anchor=startup_continuity_anchor)
        return result
    finally:
        if profile is not None:
            try:
                profile.close()
            except Exception as exc:
                # Keep the original failure and completed checkpoint. A failed
                # restoration must still prevent reporting a successful route.
                if result is not None:
                    result['localization_profile_cleanup_error'] = exception_diagnostics(exc)
                    if result.get('status') == 'PASS':
                        result.update(status='FAIL', error=str(exc))
                else:
                    print('AMCL profile cleanup failed: '+str(exc), file=sys.stderr)
            if result is not None:
                result['localization_profile_changes']=profile.events


def _localize_before_route(args):
    """Keep the full stationary admission in the already initialized context."""
    from nav2_fast_localization import PreservingFastLocalizationClient, _write_trace
    from verify_runtime_map import verify_runtime_map

    node = PreservingFastLocalizationClient(
        Path(_active_map_file()), Path(args.localization_cache_file), None,
    )
    try:
        started = time.monotonic()
        report = verify_runtime_map(node, _active_map_file())
        node._record("live_map_verified", dict(report, elapsed_s=time.monotonic()-started))
        if getattr(args, 'mark_repositioned', False):
            from std_srvs.srv import Trigger
            node._wait_runtime(10.0)
            node._assert_disarmed()
            response = node._call(node.invalidate_client, Trigger.Request(),
                                  'resume localization invalidation', timeout_s=5.0)
            if not response.success:
                raise RuntimeError('resume localization invalidation was rejected: '+response.message)
            node._wait_for(lambda: node.localization_valid is False
                           and node.localization_status.get('state') != 'HOT_READY',
                           5.0, 'invalidated localization before resume')
            node._record('resume_pose_invalidated', response.message)
        result = node.run(20.0)
    except BaseException as exc:
        result = node.result("FAIL", "error", str(exc))
    finally:
        if (isinstance(locals().get('result'),dict) and result.get('status')=='PASS'
                and node.amcl_samples):
            args.startup_continuity_anchor = _finite_pose_copy(node.amcl_samples[-1])
        node.destroy_node()
    _write_trace(args.localization_trace_file, result)
    # The full scan evidence is already durable in localization_trace_file.
    # Avoid serializing/writing it a second time before the first motion.
    print(json.dumps({key:result.get(key) for key in
        ('status','mode','error','elapsed_s','motion_authorized')},sort_keys=True),flush=True)
    return {"PASS": 0, "MOTION_REQUIRED": 4, "BLOCKED": 5}.get(result["status"], 1)


def main() -> int:
    args = _parse_args()
    try:
        # Configuration-only gate: the catalog and static safety grid must
        # belong to the same selected site before localization or any motion.
        validate_map_waypoint_binding(_active_map_file(), args.waypoints_file)
        route_names, waypoints = _load_route(args.waypoints_file, args.route)
        resume_results = load_route_resume_context(args.resume_context, route_names)
        inspection_plan = (
            load_inspection_plan(args.inspection_config)
            if args.inspection_config
            else None
        )
        if args.extract_global_pose:
            pose = load_global_localization_pose(args.extract_global_pose)
            print("export RVCLAW_NAV2_INITIAL_X=%.9f" % pose["x"])
            print("export RVCLAW_NAV2_INITIAL_Y=%.9f" % pose["y"])
            print("export RVCLAW_NAV2_INITIAL_YAW_DEG=%.9f" % pose["yaw_deg"])
            return 0
        if args.describe:
            description = describe_plan(route_names)
            description["inspection"] = (
                None
                if inspection_plan is None
                else inspection_plan_summary(inspection_plan)
            )
            print(json.dumps(description, indent=2, sort_keys=True))
            return 0
        if args.inspection_readiness_only:
            if inspection_plan is None:
                raise ValueError("--inspection-readiness-only requires --inspection-config")
            readiness = InspectionExecutor(inspection_plan).readiness()
            print(json.dumps(readiness, indent=2, ensure_ascii=False, sort_keys=True))
            return 0 if readiness["status"] == "PASS" else 1
        if args.resume_localization_verified and not args.resume_context:
            raise ValueError(
                "--resume-localization-verified requires --resume-context"
            )
        if args.mark_repositioned and not args.localize_first:
            raise ValueError("--mark-repositioned requires --localize-first")
        if os.environ.get('RVCLAW_PREPARE_RUNTIME_IN_ROUTE') == '1' and not args.localize_first:
            raise ValueError('runtime preparation requires guarded combined startup')
        if args.localize_first and (
                not args.localization_cache_file or not args.localization_trace_file
                or not os.environ.get("RVCLAW_STARTUP_READY_FD", "").isdigit()):
            raise ValueError("combined startup requires cache, trace and the startup watchdog")
        if not args.preflight_only and args.arm_token != ROUTE_ARM_TOKEN:
            raise ValueError("exact waypoint-route arm token is required")
        trace_path = _validate_trace(args.trace_file)
    except ValueError as exc:
        print("ARGUMENT_ERROR: %s" % exc, file=sys.stderr)
        return 2

    WAYPOINT.BASE.ACTION_RESULT_TIMEOUT_S = WAYPOINT.MAX_ACTION_RESULT_TIMEOUT_S
    if os.environ.pop('RVCLAW_PREPARE_RUNTIME_IN_ROUTE', '') == '1':
        # Imports and configuration-only validation overlap cold Nav2 startup.
        # The same bounded runtime/profile/priority admission must still pass
        # before creating ROS nodes, heartbeat publishers or arm requests.
        import nav2_fast_localization  # noqa: F401
        import verify_runtime_map  # noqa: F401
        started = time.monotonic()
        if _runtime_admission is None:
            subprocess.run(['bash', str(SCRIPT_DIR / 'nav2_persistent_route.sh'),
                            '--runtime-admission'], check=True)
        else:
            if _runtime_admission.wait(timeout=45.) != 0:
                raise RuntimeError('navigation runtime preparation failed before motion admission')
            print('startup_timing stage=runtime_admission_overlap total_s=%.3f'
                  % (time.monotonic()-_runtime_admission_started), flush=True)
        print('startup_timing stage=runtime_admission_after_imports duration_s=%.3f'
              % (time.monotonic()-started), flush=True)
    if not args.preflight_only:
        from control_keepalive import prepare_independent_publisher
        prepare_independent_publisher()
    rclpy.init()
    try:
        if args.localize_first:
            localization_code = _localize_before_route(args)
            if localization_code != 0:
                return localization_code
            # The parent watchdog owns this inherited pipe and continues to
            # forward cancellation after the stationary deadline is released.
            ready_fd = int(os.environ.pop("RVCLAW_STARTUP_READY_FD"))
            try:
                os.write(ready_fd, b"R")
            finally:
                os.close(ready_fd)
            print("startup_timing stage=localization_verified_in_route", flush=True)
        if not args.preflight_only:
            # The independent heartbeat inherited the verified control priority
            # before ROS initialization. Expensive route/scan orchestration must
            # not outrank the controller that supplies its velocity commands.
            os.setpriority(os.PRIO_PROCESS, 0, 0)
            print('route scheduling: main nice=0, independent heartbeat retains control priority',
                  flush=True)
        if args.preflight_only:
            result = run_route_preflight(waypoints)
        else:
            task_runner = (
                None
                if inspection_plan is None
                else _build_inspection_task_runner(inspection_plan, trace_path.stem)
            )
            result = run_route(
                waypoints,
                arrival_event_dir=os.getenv(
                    "RVCLAW_ARRIVAL_EVENT_DIR",
                    DEFAULT_ARRIVAL_EVENT_DIR,
                ),
                task_runner=task_runner,
                speech_ack_timeout_s=(
                    0.0
                    if inspection_plan is None
                    else inspection_plan.speech_ack_timeout_s
                ),
                navigation_speech_ack_timeout_s=RECOVERY_SPEECH_ACK_TIMEOUT_S,
                initial_results=resume_results,
                resume_localization_verified=args.resume_localization_verified,
                startup_continuity_anchor=getattr(args,'startup_continuity_anchor',None),
            )
    except BaseException as exc:
        diagnostic = exception_diagnostics(exc)
        result = {
            "status": "FAIL",
            **diagnostic,
            "duration_s": 0.0,
            "route": [],
            "completed_waypoints": [],
        }
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    if not args.preflight_only:
        try:
            checkpoint = write_route_checkpoint(
                args.checkpoint_file,
                route_names,
                result,
            )
            result["checkpoint_file"] = None if checkpoint is None else str(checkpoint)
        except (OSError, ValueError) as exc:
            result["checkpoint_error"] = str(exc)
    trace_path.write_text(
        json.dumps(
            {
                "goal_mode": "absolute_waypoint_route",
                "preflight_only": args.preflight_only,
                "route_requested": list(route_names),
                "inspection": (
                    None
                    if inspection_plan is None
                    else inspection_plan_summary(inspection_plan)
                ),
                **result,
            },
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    # The complete report is already in trace_path. Printing it again can
    # block finalization on tens of megabytes of duplicated diagnostics,
    # keeping chassis ownership busy after the stopped-state announcement.
    print(json.dumps(result if args.preflight_only else {
        "status": result.get("status"),
        "error": result.get("error"),
        "duration_s": result.get("duration_s"),
        "completed_waypoints": result.get("completed_waypoints", []),
        "checkpoint_file": result.get("checkpoint_file"),
        "checkpoint_error": result.get("checkpoint_error"),
        "trace_file": str(trace_path),
    }, ensure_ascii=False), flush=True)
    if args.preflight_only:
        return preflight_exit_status(result)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
