#!/usr/bin/env python3
"""Run one supervised 0.30 m relative NavigateToPose goal."""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Dict, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from stm32_control_client import (  # noqa: E402
    DEADMAN_PRIME_S,
    INTERVAL_S,
    MIN_BATTERY_VOLTAGE_V,
    SERVICE_TIMEOUT_S,
    STARTUP_TIMEOUT_S,
    ZeroSmokeClient,
)


ARM_TOKEN = "WHEELS_GROUND_PS2_OFF_NAV2_FORWARD_030"
FIELD_SITE = "wheels_ground_mapped_clear_space"
ACTION_NAME = "/navigate_to_pose"
PATH_ACTION_NAME = "/compute_path_to_pose"
TARGET_DISTANCE_M = 0.30
ACTION_RESULT_TIMEOUT_S = 20.0
PATH_RESULT_TIMEOUT_S = 20.0
DEADMAN_KEEPALIVE_PERIOD_S = 0.03
# Keep a trace-level warning margin below the hardware's 0.15 s timeout. This
# does not authorize motion or relax the hardware watchdog; it only records
# scheduler jitter so a field failure can be diagnosed from one route trace.
DEADMAN_KEEPALIVE_LATE_GAP_S = 0.10
MINIMUM_SCAN_SAMPLES = 5
MINIMUM_AMCL_SAMPLES = 3
MINIMUM_COSTMAP_SAMPLES = 1
MINIMUM_FRONT_CLEARANCE_M = 0.80
NEAR_START_HARD_STOP_CLEARANCE_M = 0.35
NEAR_START_ROTATION_SWEEP_CLEARANCE_M = 0.25
NEAR_START_DIRECT_PATH_CLEARANCE_M = 0.55
# Costmap values below 253 are inflated but not inscribed/lethal.  Permit a
# path-aware start in that range; the footprint, path escape check, front-stop
# polygon, and 0.35 m hard-stop still prevent a start that overlaps an obstacle.
MAXIMUM_START_COST = 252
MINIMUM_NEAR_START_ESCAPE_TURN_RAD = math.radians(20.0)
PATH_ESCAPE_LOOKAHEAD_M = 0.10
FRONT_HALF_ANGLE_RAD = math.radians(30.0)
MAXIMUM_SCAN_AGE_S = 0.30
LOCALIZATION_GATE_MINIMUM_KNOWN_RATIO = 0.85
LOCALIZATION_GATE_MINIMUM_BEST_SCORE = 0.80
LOCALIZATION_GATE_MINIMUM_CANDIDATE_RATIO = 0.78
LOCALIZATION_GATE_MAXIMUM_DISTANCE_M = 0.50
LOCALIZATION_GATE_MAXIMUM_YAW_DEG = 35.0
LOCALIZATION_GATE_NEAR_MISS_MINIMUM_BEST_SCORE = 0.78
LOCALIZATION_GATE_NEAR_MISS_MINIMUM_CANDIDATE_SCORE = 0.85
LOCALIZATION_GATE_NEAR_MISS_MINIMUM_KNOWN_RATIO = 0.95
LOCALIZATION_GATE_NEAR_MISS_MINIMUM_CANDIDATE_RATIO = 0.95
LOCALIZATION_GATE_NEAR_MISS_MAXIMUM_DISTANCE_M = 0.15
LOCALIZATION_GATE_NEAR_MISS_MAXIMUM_YAW_DEG = 10.0
LOCALIZATION_GATE_POSITION_STEP_M = 0.20
LOCALIZATION_GATE_YAW_STEP_DEG = 10.0
LOCALIZATION_GATE_MAXIMUM_SCAN_POINTS = 120
LOCALIZATION_GATE_CACHE_S = 15.0
LOCALIZATION_GATE_CACHE_POSITION_M = 0.05
LOCALIZATION_GATE_CACHE_YAW_RAD = math.radians(5.0)
TRUSTED_LOCALIZATION_STATUS_MAX_AGE_S = 1.5
TRUSTED_LOCALIZATION_SCAN_MAX_AGE_S = 0.75
TRUSTED_LOCALIZATION_REFRESH_TIMEOUT_S = 1.25
TRUSTED_LOCALIZATION_MAXIMUM_SCORING_POSE_DISTANCE_M = 0.10
TRUSTED_LOCALIZATION_MAXIMUM_SCORING_POSE_YAW_DEG = 5.0
TRUSTED_LOCALIZATION_MINIMUM_SCORE = 0.85
TRUSTED_LOCALIZATION_MINIMUM_KNOWN_RATIO = 0.95
TRUSTED_LOCALIZATION_MINIMUM_CANDIDATE_RATIO = 0.90
TRUSTED_LOCALIZATION_MAXIMUM_LOCAL_DISTANCE_M = 0.20
TRUSTED_LOCALIZATION_MAXIMUM_LOCAL_YAW_DEG = 12.0
TRUSTED_LOCALIZATION_MAXIMUM_CACHE_DISTANCE_M = 0.25
TRUSTED_LOCALIZATION_MAXIMUM_CACHE_YAW_DEG = 15.0
MOTION_MAP_CHECK_PERIOD_S = 0.60
MOTION_MAP_MAX_AMCL_AGE_S = 1.50
MOTION_MAP_REQUIRED_CONSECUTIVE = 3
# Values below 253 are inflated but not inscribed/lethal. Runtime footprint,
# path, and Collision Monitor checks remain authoritative in narrow spaces.
MAXIMUM_GOAL_COST = 252
MIN_GUARDED_LINEAR_X_M_S = 0.03
MAX_GUARDED_LINEAR_X_M_S = 0.35
MAX_GUARDED_ANGULAR_Z_RAD_S = 0.60
MAX_ACTION_FEEDBACK_FINAL_DISTANCE_M = 0.10
MIN_RAW_FORWARD_M = 0.10
MAX_RAW_FORWARD_M = 0.60
MAX_RAW_LATERAL_M = 0.20
MAX_RAW_YAW_DELTA_RAD = 0.40
MIN_AMCL_DISPLACEMENT_M = 0.08
MAX_AMCL_DISPLACEMENT_M = 0.60
MAX_POST_STOP_LINEAR_M_S = 0.03
MAX_POST_STOP_ANGULAR_RAD_S = 0.10
POST_ACTION_ZERO_FRAMES = 6
POST_ACTION_STOP_TIMEOUT_S = 2.0
POST_ACTION_STABLE_SAMPLES = 3
PREARM_NAV2_COMMAND_MIN_SAMPLES = 2
PREARM_NAV2_COMMAND_TIMEOUT_S = 2.0
# Nav2's first controller/TF update can outlive the normal stream wait.
# This budget applies only while wheels are disarmed; command freshness,
# hardware watchdogs and all in-motion deadlines remain unchanged.
PREARM_NAV2_INITIALIZATION_TIMEOUT_S = 8.0
MAX_PREARM_NAV2_COMMAND_AGE_S = 0.10
AMCL_NOMOTION_RESPONSE_TIMEOUT_S = 5.0
AMCL_NOMOTION_MAX_ATTEMPTS = 2
ZERO_EPSILON = 1e-9
COSTMAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
AMCL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def required_active_map_file():
    """Require the selected map; never silently use a demonstration map."""
    configured = os.environ.get("RVCLAW_MAP_FILE", "").strip()
    if not configured:
        raise RuntimeError("RVCLAW_MAP_FILE is required for navigation safety")
    path = Path(configured).expanduser().resolve()
    if not path.is_file() or path.suffix.lower() not in (".yaml", ".yml"):
        raise RuntimeError("active map YAML is unavailable: %s" % path)
    return str(path)


def apply_localization_near_miss_override(candidate, comparison, gate):
    """Admit only a narrowly missed coarse search backed by a strong AMCL pose."""
    result = dict(gate)
    failures = list(result.get("failures", []))
    observed = {
        "best_score": comparison.get("best_score"),
        "candidate_score": candidate.get("mean_likelihood"),
        "known_endpoint_ratio": candidate.get("known_endpoint_ratio"),
        "candidate_to_best_ratio": comparison.get("candidate_to_best_ratio"),
        "candidate_to_best_distance_m": comparison.get(
            "candidate_to_best_distance_m"
        ),
        "candidate_to_best_yaw_deg": comparison.get("candidate_to_best_yaw_deg"),
    }
    thresholds = {
        "minimum_best_score": LOCALIZATION_GATE_NEAR_MISS_MINIMUM_BEST_SCORE,
        "minimum_candidate_score": (
            LOCALIZATION_GATE_NEAR_MISS_MINIMUM_CANDIDATE_SCORE
        ),
        "minimum_known_endpoint_ratio": (
            LOCALIZATION_GATE_NEAR_MISS_MINIMUM_KNOWN_RATIO
        ),
        "minimum_candidate_to_best_ratio": (
            LOCALIZATION_GATE_NEAR_MISS_MINIMUM_CANDIDATE_RATIO
        ),
        "maximum_candidate_to_best_distance_m": (
            LOCALIZATION_GATE_NEAR_MISS_MAXIMUM_DISTANCE_M
        ),
        "maximum_candidate_to_best_yaw_deg": (
            LOCALIZATION_GATE_NEAR_MISS_MAXIMUM_YAW_DEG
        ),
    }
    try:
        values = {name: float(value) for name, value in observed.items()}
        finite = all(math.isfinite(value) for value in values.values())
    except (TypeError, ValueError):
        values = {}
        finite = False
    only_coarse_score_failed = (
        len(failures) == 1
        and failures[0].startswith("best static-map scan match is too weak")
    )
    applied = bool(
        only_coarse_score_failed
        and finite
        and values["best_score"] >= thresholds["minimum_best_score"]
        and values["candidate_score"] >= thresholds["minimum_candidate_score"]
        and values["known_endpoint_ratio"]
        >= thresholds["minimum_known_endpoint_ratio"]
        and values["candidate_to_best_ratio"]
        >= thresholds["minimum_candidate_to_best_ratio"]
        and values["candidate_to_best_distance_m"]
        <= thresholds["maximum_candidate_to_best_distance_m"]
        and values["candidate_to_best_yaw_deg"]
        <= thresholds["maximum_candidate_to_best_yaw_deg"]
    )
    result["near_miss_override"] = {
        "applied": applied,
        "observed": observed,
        "thresholds": thresholds,
    }
    if applied:
        result["status"] = "PASS"
        result["failures"] = []
    return result


def evaluate_trusted_hot_localization(
    runtime_status, runtime_valid, status_age_s, active_map_sha,
    amcl_pose, candidate, local_search,
):
    """Admit a bounded local match only for a continuously verified map pose.

    A repetitive corridor may have several strong global matches.  This does
    not make a known, continuously verified pose wrong.  Missing or stale
    runtime evidence fails this branch; the original global gate remains.
    """
    from rvclaw_nav2_bringup.scan_map_consistency import (
        compare_candidate_to_search,
        evaluate_localization_gate,
    )

    def finite(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    status = runtime_status if isinstance(runtime_status, dict) else {}
    cache = status.get("cache_validation")
    cache = cache if isinstance(cache, dict) else {}
    cached_pose = cache.get("pose")
    cached_pose = cached_pose if isinstance(cached_pose, dict) else {}
    runtime_score = status.get("scan_map_score")
    runtime_score = runtime_score if isinstance(runtime_score, dict) else {}
    failures = []
    age = finite(status_age_s)
    if age is None or age > TRUSTED_LOCALIZATION_STATUS_MAX_AGE_S:
        failures.append("verified localization status is missing or stale")
    if runtime_valid is not True or status.get("valid") is not True:
        failures.append("localization runtime is not valid")
    if status.get("state") != "HOT_READY" or status.get("pose_reusable") is not True:
        failures.append("localization runtime is not HOT_READY")
    if status.get("map_sha256") != active_map_sha:
        failures.append("localization runtime map identity differs from active map")
    if status.get("charging") is not False or status.get("runtime_gap_observed") is not False:
        failures.append("charging or a runtime gap invalidated pose continuity")
    if status.get("armed") is not False or status.get("controller_state") not in {"ESTOP_LATCHED", "DISARMED"}:
        failures.append("controller is not disarmed for localization verification")
    if status.get("hardware_connected") is not True or status.get("lidar_fresh") is not True:
        failures.append("hardware or lidar is not continuously available")
    epoch = finite(status.get("external_verification_epoch"))
    if epoch is None or epoch < 1:
        failures.append("pose has no completed external verification")
    scan_age = finite(status.get("scan_age_s"))
    if scan_age is None or scan_age > TRUSTED_LOCALIZATION_SCAN_MAX_AGE_S:
        failures.append("localization runtime scan is stale")
    if cache.get("status") != "PASS":
        failures.append("map-bound pose cache did not validate")
    runtime_mean = finite(runtime_score.get("mean_likelihood"))
    runtime_known = finite(runtime_score.get("known_endpoint_ratio"))
    if runtime_mean is None or runtime_mean < TRUSTED_LOCALIZATION_MINIMUM_SCORE:
        failures.append("continuous runtime scan/map score is weak")
    if runtime_known is None or runtime_known < TRUSTED_LOCALIZATION_MINIMUM_KNOWN_RATIO:
        failures.append("continuous runtime scan has too few known endpoints")

    current_pose = amcl_pose if isinstance(amcl_pose, dict) else {}
    values = [finite(pose.get(key)) for pose in (cached_pose, current_pose) for key in ("x", "y", "yaw")]
    pose_delta = None
    yaw_delta_deg = None
    if any(value is None for value in values):
        failures.append("cached or current AMCL pose is unavailable")
    else:
        cx, cy, cyaw, ax, ay, ayaw = values
        pose_delta = math.hypot(cx - ax, cy - ay)
        yaw_delta_deg = abs(math.degrees(math.atan2(math.sin(cyaw - ayaw), math.cos(cyaw - ayaw))))
        if pose_delta > TRUSTED_LOCALIZATION_MAXIMUM_CACHE_DISTANCE_M:
            failures.append("AMCL pose jumped away from verified cache")
        if yaw_delta_deg > TRUSTED_LOCALIZATION_MAXIMUM_CACHE_YAW_DEG:
            failures.append("AMCL heading jumped away from verified cache")

    scored_pose = candidate.get("pose")
    scored_pose = scored_pose if isinstance(scored_pose, dict) else {}
    scored_values = [
        finite(pose.get(key))
        for pose in (scored_pose, current_pose)
        for key in ("x", "y", "yaw")
    ]
    if any(value is None for value in scored_values):
        failures.append("scored scan or current AMCL pose is unavailable")
    else:
        sx, sy, syaw, ax, ay, ayaw = scored_values
        if math.hypot(sx - ax, sy - ay) > TRUSTED_LOCALIZATION_MAXIMUM_SCORING_POSE_DISTANCE_M:
            failures.append("AMCL moved since the scan/map score was computed")
        if abs(math.degrees(math.atan2(math.sin(syaw - ayaw), math.cos(syaw - ayaw)))) > TRUSTED_LOCALIZATION_MAXIMUM_SCORING_POSE_YAW_DEG:
            failures.append("AMCL heading changed since the scan/map score was computed")

    comparison = compare_candidate_to_search(candidate, local_search)
    local_gate = evaluate_localization_gate(
        candidate, comparison,
        minimum_known_endpoint_ratio=TRUSTED_LOCALIZATION_MINIMUM_KNOWN_RATIO,
        minimum_best_score=TRUSTED_LOCALIZATION_MINIMUM_SCORE,
        minimum_candidate_to_best_ratio=TRUSTED_LOCALIZATION_MINIMUM_CANDIDATE_RATIO,
        maximum_candidate_to_best_distance_m=TRUSTED_LOCALIZATION_MAXIMUM_LOCAL_DISTANCE_M,
        maximum_candidate_to_best_yaw_deg=TRUSTED_LOCALIZATION_MAXIMUM_LOCAL_YAW_DEG,
    )
    failures.extend(local_gate["failures"])
    candidate_mean = finite(candidate.get("mean_likelihood"))
    if candidate_mean is None or candidate_mean < TRUSTED_LOCALIZATION_MINIMUM_SCORE:
        failures.append("fresh candidate scan/map score is weak")
    return {
        "status": "PASS" if not failures else "FAIL",
        "mode": "TRUSTED_HOT_LOCAL",
        "failures": failures,
        "status_age_s": age,
        "runtime_scan_age_s": scan_age,
        "cache_position_delta_m": pose_delta,
        "cache_yaw_delta_deg": yaw_delta_deg,
        "comparison": comparison,
        "local_gate": local_gate,
        "best_matches": local_search.get("matches", [])[:3],
        "runtime_map_sha256": status.get("map_sha256"),
        "external_verification_epoch": status.get("external_verification_epoch"),
    }


def describe_plan() -> Dict[str, object]:
    return {
        "hardware_io": False,
        "field_site": FIELD_SITE,
        "required_arm_token": ARM_TOKEN,
        "action_name": ACTION_NAME,
        "path_action_name": PATH_ACTION_NAME,
        "path_preflight_required": True,
        "path_result_timeout_s": PATH_RESULT_TIMEOUT_S,
        "relative_goal_forward_m": TARGET_DISTANCE_M,
        "goal_heading": "preserve_current_amcl_yaw",
        "action_result_timeout_s": ACTION_RESULT_TIMEOUT_S,
        "minimum_front_clearance_m": MINIMUM_FRONT_CLEARANCE_M,
        "near_start_hard_stop_clearance_m": NEAR_START_HARD_STOP_CLEARANCE_M,
        "near_start_direct_path_clearance_m": NEAR_START_DIRECT_PATH_CLEARANCE_M,
        "maximum_start_cost": MAXIMUM_START_COST,
        "minimum_near_start_escape_turn_deg": math.degrees(
            MINIMUM_NEAR_START_ESCAPE_TURN_RAD
        ),
        "front_clearance_half_angle_deg": math.degrees(FRONT_HALF_ANGLE_RAD),
        "maximum_goal_cost": MAXIMUM_GOAL_COST,
        "maximum_guarded_linear_x_m_s": MAX_GUARDED_LINEAR_X_M_S,
        "maximum_guarded_angular_z_rad_s": MAX_GUARDED_ANGULAR_Z_RAD_S,
        "minimum_battery_voltage_v": MIN_BATTERY_VOLTAGE_V,
        "post_action_stop_timeout_s": POST_ACTION_STOP_TIMEOUT_S,
        "post_action_stable_samples": POST_ACTION_STABLE_SAMPLES,
        "hardware_arm_sequence": (
            "accepted_goal_then_fresh_nav2_command_stream_then_arm"
        ),
        "prearm_nav2_command_min_samples": PREARM_NAV2_COMMAND_MIN_SAMPLES,
        "prearm_nav2_command_timeout_s": PREARM_NAV2_COMMAND_TIMEOUT_S,
        "maximum_prearm_nav2_command_age_s": MAX_PREARM_NAV2_COMMAND_AGE_S,
        "amcl_nomotion_response_timeout_s": AMCL_NOMOTION_RESPONSE_TIMEOUT_S,
        "amcl_nomotion_max_attempts": AMCL_NOMOTION_MAX_ATTEMPTS,
        "collision_monitor_required": True,
        "fresh_scan_gate_required": True,
        "independent_static_map_localization_gate": True,
        "trusted_hot_local_map_gate": True,
        "trusted_hot_local_gate_requires_map_identity_and_fresh_runtime": True,
        "localization_gate_minimum_known_endpoint_ratio": (
            LOCALIZATION_GATE_MINIMUM_KNOWN_RATIO
        ),
        "localization_gate_minimum_best_score": LOCALIZATION_GATE_MINIMUM_BEST_SCORE,
        "localization_gate_minimum_candidate_to_best_ratio": (
            LOCALIZATION_GATE_MINIMUM_CANDIDATE_RATIO
        ),
        "localization_gate_maximum_candidate_to_best_distance_m": (
            LOCALIZATION_GATE_MAXIMUM_DISTANCE_M
        ),
        "localization_gate_maximum_candidate_to_best_yaw_deg": (
            LOCALIZATION_GATE_MAXIMUM_YAW_DEG
        ),
        "automatic_reverse": False,
        "direct_nonzero_velocity_publish": False,
        "launches_hardware_node": False,
        "transport_write_bypass": False,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--trace-file")
    parser.add_argument("--arm-token")
    return parser.parse_args()


def _validate_run_args(args: argparse.Namespace) -> Path:
    if args.arm_token != ARM_TOKEN:
        raise ValueError("exact first NavigateToPose arm token is required")
    if not args.trace_file:
        raise ValueError("trace-file is required")
    candidate = Path(args.trace_file)
    if not candidate.is_absolute():
        candidate = WORKSPACE_ROOT / candidate
    trace_path = candidate.resolve()
    try:
        trace_path.relative_to(WORKSPACE_ROOT.resolve())
    except ValueError as exc:
        raise ValueError(
            "trace file must remain under the workspace root"
        ) from exc
    if not trace_path.parent.is_dir():
        raise ValueError("trace file parent directory does not exist")
    if trace_path.exists():
        raise ValueError("trace file already exists")
    return trace_path


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_yaw(orientation) -> float:
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def relative_goal(
    x: float,
    y: float,
    yaw: float,
) -> Tuple[float, float, float]:
    return (
        x + TARGET_DISTANCE_M * math.cos(yaw),
        y + TARGET_DISTANCE_M * math.sin(yaw),
        yaw,
    )


def distance(first: Dict[str, float], second: Dict[str, float]) -> float:
    return math.hypot(second["x"] - first["x"], second["y"] - first["y"])


def body_delta(
    first: Dict[str, float],
    second: Dict[str, float],
) -> Tuple[float, float]:
    dx = second["x"] - first["x"]
    dy = second["y"] - first["y"]
    cosine = math.cos(first["yaw"])
    sine = math.sin(first["yaw"])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def minimum_front_range(scan: LaserScan) -> Optional[float]:
    minimum = None
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(normalize_angle(angle)) > FRONT_HALF_ANGLE_RAD:
            continue
        if not math.isfinite(value):
            continue
        if value < scan.range_min or value > scan.range_max:
            continue
        minimum = value if minimum is None else min(minimum, value)
    return minimum


def costmap_value_at(costmap: Costmap, x: float, y: float) -> Optional[int]:
    metadata = costmap.metadata
    resolution = float(metadata.resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        return None
    origin = metadata.origin
    origin_yaw = quaternion_to_yaw(origin.orientation)
    dx = x - float(origin.position.x)
    dy = y - float(origin.position.y)
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cell_x = math.floor(local_x / resolution)
    cell_y = math.floor(local_y / resolution)
    size_x = int(metadata.size_x)
    size_y = int(metadata.size_y)
    if cell_x < 0 or cell_y < 0 or cell_x >= size_x or cell_y >= size_y:
        return None
    index = cell_y * size_x + cell_x
    if index >= len(costmap.data):
        return None
    return int(costmap.data[index])


def path_escape_heading_delta(path, start_pose: Dict[str, float]) -> Optional[float]:
    """Return the initial path direction relative to the robot heading."""
    points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in path.poses]
    if not points or not all(math.isfinite(v) for p in points for v in p):
        return None
    for stamped_pose in path.poses:
        dx = float(stamped_pose.pose.position.x) - start_pose["x"]
        dy = float(stamped_pose.pose.position.y) - start_pose["y"]
        if math.hypot(dx, dy) < PATH_ESCAPE_LOOKAHEAD_M:
            continue
        return normalize_angle(math.atan2(dy, dx) - start_pose["yaw"])
    # A verified reapproach can be shorter than the normal 10 cm lookahead.
    # Use its actual endpoint only for a short, coherent planner path; a
    # duplicate point, local loop or path starting elsewhere is not direction.
    end_x, end_y = points[-1]
    dx, dy = end_x-start_pose['x'], end_y-start_pose['y']
    distance = math.hypot(dx, dy)
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:]))
    if (len(points) >= 2 and .05 <= distance < PATH_ESCAPE_LOOKAHEAD_M
            and math.hypot(points[0][0]-start_pose['x'],points[0][1]-start_pose['y']) <= .05
            and 0 < length <= .20):
        direction = math.atan2(dy,dx)
        if all(math.hypot(x-start_pose['x'],y-start_pose['y']) < .03
               or abs(normalize_angle(math.atan2(y-start_pose['y'],x-start_pose['x'])-direction)) <= math.pi/4
               for x,y in points):
            return normalize_angle(direction-start_pose['yaw'])
    return None


def classify_start_clearance(
    clearance_m: float,
    start_cost: Optional[int],
    initial_path_heading_delta_rad: Optional[float],
    rotation_clearance: Optional[Dict[str, object]] = None,
) -> str:
    """Admit normal starts or reviewed path-aware near-obstacle starts."""
    if clearance_m < NEAR_START_HARD_STOP_CLEARANCE_M:
        if clearance_m < NEAR_START_ROTATION_SWEEP_CLEARANCE_M:
            raise RuntimeError("front clearance is inside the rotation swept envelope")
        if (start_cost is None or start_cost > MAXIMUM_START_COST
                or initial_path_heading_delta_rad is None
                or abs(initial_path_heading_delta_rad) < MINIMUM_NEAR_START_ESCAPE_TURN_RAD
                or not isinstance(rotation_clearance, dict)
                or rotation_clearance.get("status") != "PASS"):
            raise RuntimeError("near-obstacle start lacks a safe turn and complete rotation sweep")
        return "PATH_TURN_WITH_SWEEP_CLEAR"
    if clearance_m >= MINIMUM_FRONT_CLEARANCE_M:
        return "NORMAL"
    if start_cost is None:
        raise RuntimeError("robot start is outside the global costmap")
    if start_cost > MAXIMUM_START_COST:
        raise RuntimeError("robot start cell is not admitted for near-obstacle mode")
    if initial_path_heading_delta_rad is None:
        raise RuntimeError("near-obstacle path has no usable initial direction")
    if (
        clearance_m < NEAR_START_DIRECT_PATH_CLEARANCE_M
        and abs(initial_path_heading_delta_rad)
        < MINIMUM_NEAR_START_ESCAPE_TURN_RAD
    ):
        raise RuntimeError(
            "near-obstacle path does not begin with a sufficient escape turn"
        )
    return "PATH_AWARE_NEAR_OBSTACLE"


def classify_future_segment_start(
    start_cost: Optional[int],
    initial_path_heading_delta_rad: Optional[float],
) -> str:
    """Validate a planned future start without using the current live scan."""
    if start_cost is None:
        raise RuntimeError("planned route start is outside the global costmap")
    if start_cost > MAXIMUM_START_COST:
        raise RuntimeError("planned route start cell is not admitted")
    if initial_path_heading_delta_rad is None:
        raise RuntimeError("planned route path has no usable initial direction")
    return "FUTURE_SEGMENT_RUNTIME_CLEARANCE_DEFERRED"


def evaluate_start_clearance_evidence(metrics: Dict[str, object]) -> list:
    failures = []
    mode = metrics.get("start_clearance_mode")
    try:
        observed_mode = classify_start_clearance(
            float(metrics.get("pre_arm_front_clearance_m", 0.0)),
            metrics.get("start_cost"),
            metrics.get("initial_path_heading_delta_rad"),
            metrics.get("rotation_clearance"),
        )
    except (RuntimeError, TypeError, ValueError) as exc:
        failures.append("start clearance evidence is unsafe: %s" % exc)
        return failures
    if mode is not None and mode != observed_mode:
        failures.append("start clearance mode does not match the recorded evidence")
    return failures


def evaluate_evidence(metrics: Dict[str, object]) -> list:
    failures = []
    if metrics.get("action_status") != GoalStatus.STATUS_SUCCEEDED:
        failures.append("NavigateToPose did not report success")
    if int(metrics.get("action_feedback_samples", 0)) < 2:
        failures.append("NavigateToPose emitted too few feedback samples")
    if float(metrics.get("action_feedback_final_distance_m", math.inf)) > (
        MAX_ACTION_FEEDBACK_FINAL_DISTANCE_M
    ):
        failures.append(
            "NavigateToPose final feedback remained too far from goal"
        )
    if int(metrics.get("motion_raw_odom_samples", 0)) < 5:
        failures.append("too few raw odometry samples during navigation")
    if int(metrics.get("motion_amcl_samples", 0)) < 2:
        failures.append("too few AMCL samples during navigation")
    if int(metrics.get("monitor_output_samples", 0)) < 1:
        failures.append("Collision Monitor output was not observed")

    guarded_min = metrics.get("guarded_linear_min_m_s")
    guarded_max = metrics.get("guarded_linear_max_m_s")
    if guarded_min is None or float(guarded_min) < -ZERO_EPSILON:
        failures.append("guarded command contained reverse velocity")
    if guarded_max is None or float(guarded_max) < MIN_GUARDED_LINEAR_X_M_S:
        failures.append(
            "guarded command did not reach minimum forward velocity"
        )
    if guarded_max is not None and float(guarded_max) > (
        MAX_GUARDED_LINEAR_X_M_S + ZERO_EPSILON
    ):
        failures.append("guarded command exceeded forward velocity ceiling")
    if float(metrics.get("max_guarded_linear_y_m_s", math.inf)) > ZERO_EPSILON:
        failures.append("guarded command contained lateral velocity")
    if float(metrics.get("max_guarded_angular_z_rad_s", math.inf)) > (
        MAX_GUARDED_ANGULAR_Z_RAD_S + ZERO_EPSILON
    ):
        failures.append("guarded command exceeded angular velocity ceiling")

    monitor_min = metrics.get("monitor_linear_min_m_s")
    if monitor_min is None or float(monitor_min) < -ZERO_EPSILON:
        failures.append("Collision Monitor output contained reverse velocity")

    raw_forward = float(metrics.get("raw_forward_m", math.nan))
    raw_lateral = float(metrics.get("raw_lateral_m", math.nan))
    raw_yaw = float(metrics.get("raw_yaw_delta_rad", math.nan))
    if not math.isfinite(raw_forward) or not (
        MIN_RAW_FORWARD_M <= raw_forward <= MAX_RAW_FORWARD_M
    ):
        failures.append(
            "raw forward displacement is outside the admitted window"
        )
    if not math.isfinite(raw_lateral) or abs(raw_lateral) > MAX_RAW_LATERAL_M:
        failures.append(
            "raw lateral displacement exceeded the admitted window"
        )
    if not math.isfinite(raw_yaw) or abs(raw_yaw) > MAX_RAW_YAW_DELTA_RAD:
        failures.append("raw yaw change exceeded the admitted window")

    amcl_displacement = float(metrics.get("amcl_displacement_m", math.nan))
    if not math.isfinite(amcl_displacement) or not (
        MIN_AMCL_DISPLACEMENT_M
        <= amcl_displacement
        <= MAX_AMCL_DISPLACEMENT_M
    ):
        failures.append("AMCL displacement is outside the admitted window")
    if int(metrics.get("goal_cost", 255)) > MAXIMUM_GOAL_COST:
        failures.append("goal cell cost exceeded the admitted limit")
    failures.extend(evaluate_start_clearance_evidence(metrics))
    if float(metrics.get("pre_arm_scan_age_s", math.inf)) > MAXIMUM_SCAN_AGE_S:
        failures.append("pre-arm scan was stale")

    if (
        int(metrics.get("post_stop_stable_samples", 0))
        < POST_ACTION_STABLE_SAMPLES
    ):
        failures.append("post-action feedback did not remain stably stopped")
    if abs(float(metrics.get("final_feedback_x_m_s", math.inf))) > (
        MAX_POST_STOP_LINEAR_M_S
    ):
        failures.append("post-action X feedback remained above stop limit")
    if abs(float(metrics.get("final_feedback_y_m_s", math.inf))) > (
        MAX_POST_STOP_LINEAR_M_S
    ):
        failures.append("post-action Y feedback remained above stop limit")
    if abs(float(metrics.get("final_feedback_angular_rad_s", math.inf))) > (
        MAX_POST_STOP_ANGULAR_RAD_S
    ):
        failures.append(
            "post-action angular feedback remained above stop limit"
        )
    if metrics.get("final_armed") is not False:
        failures.append("hardware controller did not finish disarmed")
    return failures


class ScanOdometryHistory:
    """Bounded wheel-odometry interpolation; no extrapolation or pose writes.

    A recent AMCL callback is not necessarily a pose at the lidar acquisition
    time. Keep sensor stamps so motion checks can compare the same instant.
    Missing brackets, discontinuities and incompatible frames fail closed to
    the caller's existing guard; they never manufacture an aligned sample.
    """

    def __init__(self):
        self.samples = deque(maxlen=256)
        self.frame = None

    def add(self, stamp_ns, pose, frame):
        if (stamp_ns <= 0 or not all(math.isfinite(pose.get(k, math.nan))
                for k in ('x', 'y', 'yaw')) or not all(frame)):
            self.samples.clear()
            self.frame = None
            return
        if self.frame != frame or (self.samples and stamp_ns < self.samples[-1][0]):
            self.samples.clear()
        self.frame = frame
        if self.samples and stamp_ns == self.samples[-1][0]:
            return
        if self.samples:
            last_ns, last = self.samples[-1]
            seconds = (stamp_ns-last_ns)/1e9
            angle = abs(math.atan2(math.sin(pose['yaw']-last['yaw']),
                                  math.cos(pose['yaw']-last['yaw'])))
            if (seconds > .15 or math.hypot(pose['x']-last['x'], pose['y']-last['y']) > .02+.6*seconds
                    or angle > .03+2.*seconds):
                self.samples.clear()
        self.samples.append((stamp_ns, dict(pose)))

    def at(self, stamp_ns):
        if not self.samples or not self.samples[0][0] <= stamp_ns <= self.samples[-1][0]:
            return None
        previous = self.samples[0]
        for current in self.samples:
            if current[0] == stamp_ns:
                return dict(current[1])
            if current[0] > stamp_ns:
                a, b = previous[1], current[1]
                fraction = (stamp_ns-previous[0])/(current[0]-previous[0])
                delta = math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw']))
                return dict(x=a['x']+fraction*(b['x']-a['x']),
                            y=a['y']+fraction*(b['y']-a['y']),
                            yaw=math.atan2(math.sin(a['yaw']+fraction*delta),
                                           math.cos(a['yaw']+fraction*delta)))
            previous = current
        return None

    def align(self, pose, pose_ns, scan_ns):
        if (pose is None or pose_ns <= 0 or scan_ns <= 0
                or abs(pose_ns-scan_ns) > 250000000
                or not all(math.isfinite(pose.get(k, math.nan)) for k in ('x','y','yaw'))):
            return None
        source, target = self.at(pose_ns), self.at(scan_ns)
        if source is None or target is None:
            return None
        dx, dy = target['x']-source['x'], target['y']-source['y']
        angle = pose['yaw']-source['yaw']
        c, s = math.cos(angle), math.sin(angle)
        yaw = pose['yaw']+math.atan2(math.sin(target['yaw']-source['yaw']),
                                    math.cos(target['yaw']-source['yaw']))
        aligned = dict(pose, x=pose['x']+c*dx-s*dy, y=pose['y']+s*dx+c*dy,
                       yaw=math.atan2(math.sin(yaw),math.cos(yaw)))
        return dict(pose=aligned, odom=target, pose_stamp_ns=pose_ns,
                    scan_stamp_ns=scan_ns, skew_s=(scan_ns-pose_ns)/1e9,
                    translation_m=math.hypot(dx,dy),
                    yaw_delta_rad=math.atan2(math.sin(yaw-pose['yaw']),math.cos(yaw-pose['yaw'])))


class Nav2ForwardGoalClient(ZeroSmokeClient):
    def __init__(self) -> None:
        super().__init__()
        self.compute_path_action = ActionClient(
            self,
            ComputePathToPose,
            PATH_ACTION_NAME,
        )
        self.navigate_action = ActionClient(self, NavigateToPose, ACTION_NAME)
        self.amcl_nomotion_client = self.create_client(
            Empty, "/request_nomotion_update"
        )
        self.scan_samples = 0
        self.amcl_samples = 0
        self.costmap_samples = 0
        self.monitor_output_samples = 0
        self.latest_monitor_command_monotonic: Optional[float] = None
        self.stamped_output_samples = 0
        self.latest_stamped_command_monotonic: Optional[float] = None
        self.latest_stamped_command_ns: Optional[int] = None
        self.maximum_stamped_command_age_s = 0.0
        self.raw_odom_samples = 0
        self.latest_scan_monotonic: Optional[float] = None
        self.latest_scan = None
        self.latest_localization_scan = None
        self.latest_localization_scan_monotonic = None
        self.localization_scan_samples = 0
        self.front_clearance_m: Optional[float] = None
        self.latest_raw_pose: Optional[Dict[str, float]] = None
        self.latest_amcl_pose: Optional[Dict[str, float]] = None
        self.latest_amcl_stamp_ns = 0
        self.latest_raw_stamp_ns = 0
        self.scan_odometry_history = ScanOdometryHistory()
        self.global_costmap: Optional[Costmap] = None
        self.latest_feedback_x = 0.0
        self.latest_feedback_y = 0.0
        self.latest_feedback_angular = 0.0
        self.feedback_samples = 0
        self.post_stop_stable_samples = 0
        self.post_stop_settle_s: Optional[float] = None
        self._deadman_keepalive_stop = None
        self._deadman_keepalive_thread = None
        self.deadman_keepalive_publish_count = 0
        self.deadman_keepalive_max_gap_s: Optional[float] = None
        self.deadman_keepalive_late_gap_count = 0
        self._deadman_keepalive_last_monotonic: Optional[float] = None
        self.guarded_linear_min: Optional[float] = None
        self.guarded_linear_max: Optional[float] = None
        self.monitor_linear_min: Optional[float] = None
        self.monitor_linear_max: Optional[float] = None
        self.action_feedback_samples = 0
        self.action_feedback_final_distance_m: Optional[float] = None
        self.action_status: Optional[int] = None
        self.action_elapsed_s: Optional[float] = None
        self.goal_handle = None
        self.start_raw_pose: Optional[Dict[str, float]] = None
        self.final_raw_pose: Optional[Dict[str, float]] = None
        self.start_amcl_pose: Optional[Dict[str, float]] = None
        self.final_amcl_pose: Optional[Dict[str, float]] = None
        self.start_raw_sample_count = 0
        self.start_amcl_sample_count = 0
        self.goal_pose: Optional[Dict[str, float]] = None
        self.goal_cost: Optional[int] = None
        self.path_check_status: Optional[int] = None
        self.path_check_pose_count = 0
        self.path_departure_poses = []
        self.path_check_elapsed_s: Optional[float] = None
        self.path_check_error: Optional[str] = None
        self.pre_arm_scan_age_s: Optional[float] = None
        self.pre_arm_front_clearance_m: Optional[float] = None
        self.start_cost: Optional[int] = None
        self.initial_path_heading_delta_rad: Optional[float] = None
        self.start_clearance_mode: Optional[str] = None
        self.rotation_clearance: Optional[Dict[str, object]] = None
        self.localization_gate_result: Optional[Dict[str, object]] = None
        self.localization_gate_scan_sample_count = -1
        self.localization_gate_checked_monotonic: Optional[float] = None
        self.localization_gate_pose: Optional[Dict[str, float]] = None
        self.localization_runtime_status: Optional[Dict[str, object]] = None
        self.localization_runtime_status_monotonic: Optional[float] = None
        self.localization_runtime_valid: Optional[bool] = None
        self.localization_runtime_valid_monotonic: Optional[float] = None
        self.latest_amcl_monotonic: Optional[float] = None
        self.motion_map_grid = None
        self.motion_map_guard_active = False
        self.motion_map_last_check_monotonic = 0.0
        self.motion_map_last_scan_stamp_ns = -1
        self.motion_map_recent_scans = deque(maxlen=12)
        self.motion_map_last_paired_stamp_ns = -1
        self.motion_map_disagreement_streak = 0
        self.motion_map_stale_pose_streak = 0
        self.motion_map_refresh_future = None
        self.motion_map_refresh_requested_at = -math.inf
        self.motion_map_checks = 0
        self.motion_map_guard_tripped = False
        self.motion_map_last_evidence = None

        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan_localization", self._on_localization_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl,
            AMCL_QOS,
        )
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._on_costmap,
            COSTMAP_QOS,
        )
        self.create_subscription(
            Twist,
            "/rvclaw/nav2_cmd_vel",
            self._on_monitor_command,
            10,
        )
        self.create_subscription(
            TwistStamped, "/rvclaw/cmd_vel/autonomy",
            self._on_stamped_command,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            String, "/rvclaw/localization/status",
            self._on_localization_runtime_status, 10,
        )
        self.create_subscription(
            Bool, "/rvclaw/localization/valid",
            self._on_localization_runtime_valid, 10,
        )

    def _on_localization_runtime_status(self, message: String) -> None:
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            document = None
        self.localization_runtime_status = document if isinstance(document, dict) else None
        self.localization_runtime_status_monotonic = time.monotonic()

    def _on_localization_runtime_valid(self, message: Bool) -> None:
        self.localization_runtime_valid = bool(message.data)
        self.localization_runtime_valid_monotonic = time.monotonic()

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_samples += 1
        self.latest_scan_monotonic = time.monotonic()
        self.latest_scan = message
        self.front_clearance_m = minimum_front_range(message)

    def _on_localization_scan(self, message: LaserScan) -> None:
        self.latest_localization_scan = message
        self.latest_localization_scan_monotonic = time.monotonic()
        self.localization_scan_samples += 1
        if self.motion_map_guard_active and not self.motion_map_guard_tripped:
            self._check_in_motion_map_disagreement(message)

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self.amcl_samples += 1
        self.latest_amcl_monotonic = time.monotonic()
        self.latest_amcl_stamp_ns = int(message.header.stamp.sec)*1000000000+int(message.header.stamp.nanosec)
        self.latest_amcl_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(pose.orientation),
            "cov_x": float(message.pose.covariance[0]),
            "cov_y": float(message.pose.covariance[7]),
            "cov_yaw": float(message.pose.covariance[35]),
        }

    def _prepare_in_motion_map_guard(self) -> None:
        """Load the active map before arming; never reinitialize localization."""
        from rvclaw_nav2_bringup.scan_map_consistency import load_static_map

        map_file = required_active_map_file()
        self.motion_map_grid = load_static_map(map_file)
        self.motion_map_disagreement_streak = 0
        self.motion_map_stale_pose_streak = 0
        self.motion_map_guard_tripped = False
        self._record("motion_map_guard_map_file", map_file)

    def _trip_in_motion_map_guard(self, evidence) -> None:
        self.motion_map_guard_tripped = True
        self.motion_map_guard_active = False
        self.motion_map_last_evidence = evidence
        self._record("motion_map_guard_stop", evidence)
        if evidence.get('status') == 'STALE_AMCL':
            self._stop_deadman_keepalive()
            self.publish_deadman(False)
            self.publish_zero_command()
            return
        self.publish_estop(True)
        self.publish_deadman(False)

    def _refresh_delayed_motion_pose(self, now, pose_age):
        # AMCL publishes on filter updates, not on every scan. Slow travel and
        # turn deceleration may not cross its displacement threshold before
        # the publication watchdog. Request the next normal measured update;
        # never spin/wait here or synthesize a pose from the old observation.
        pending = self.motion_map_refresh_future
        if (pose_age <= .75 or now-self.motion_map_refresh_requested_at < 1.
                or (pending is not None and not pending.done())
                or not self.amcl_nomotion_client.service_is_ready()):
            return
        if pending is not None:
            try:
                pending.result()
            except Exception as exc:
                self._record('motion_pose_refresh_error', str(exc))
        self.motion_map_refresh_requested_at = now
        try:
            self.motion_map_refresh_future = self.amcl_nomotion_client.call_async(Empty.Request())
            self._record('motion_pose_refresh_requested', dict(pose_callback_age_s=pose_age))
        except Exception as exc:
            self._record('motion_pose_refresh_error', str(exc))

    def _check_in_motion_map_disagreement(self, message: LaserScan) -> None:
        now = time.monotonic()
        self.motion_map_recent_scans.append(message)
        if now - self.motion_map_last_check_monotonic < MOTION_MAP_CHECK_PERIOD_S:
            return
        scan_stamp_ns = int(message.header.stamp.sec) * 1000000000 + int(
            message.header.stamp.nanosec
        )
        if scan_stamp_ns <= self.motion_map_last_scan_stamp_ns:
            return
        self.motion_map_last_scan_stamp_ns = scan_stamp_ns
        self.motion_map_last_check_monotonic = now
        if self.motion_map_grid is None or self.latest_amcl_pose is None:
            return
        pose_age = (math.inf if self.latest_amcl_monotonic is None
                    else now-self.latest_amcl_monotonic)
        self._refresh_delayed_motion_pose(now, pose_age)
        if (
            self.latest_amcl_monotonic is None
            or now - self.latest_amcl_monotonic > MOTION_MAP_MAX_AMCL_AGE_S
        ):
            moving = (
                abs(self.latest_feedback_x) > 0.03
                or abs(self.latest_feedback_y) > 0.03
                or abs(self.latest_feedback_angular) > 0.10
            )
            if moving:
                from rvclaw_nav2_bringup.scan_map_consistency import (
                    advance_in_motion_disagreement_streak,
                )
                self.motion_map_stale_pose_streak, trip = (
                    advance_in_motion_disagreement_streak(
                        self.motion_map_stale_pose_streak, "DISAGREEMENT",
                        MOTION_MAP_REQUIRED_CONSECUTIVE,
                    )
                )
                if trip:
                    self._trip_in_motion_map_guard({"status": "STALE_AMCL", "moving": True,
                        "pose_callback_age_s": pose_age,
                        "pose_stamp_ns": self.latest_amcl_stamp_ns,
                        "scan_stamp_ns": scan_stamp_ns,
                        "refresh_requested": self.motion_map_refresh_future is not None})
            else:
                self.motion_map_stale_pose_streak = 0
            self.motion_map_disagreement_streak = 0
            return

        self.motion_map_stale_pose_streak = 0

        from rvclaw_nav2_bringup.scan_map_consistency import (
            advance_in_motion_disagreement_streak,
            corroborate_motion_map_disagreement,
            evaluate_in_motion_pose_disagreement,
            evaluate_pose_ray_consistency,
            filter_ray_inconsistent_matches,
            qualify_motion_disagreement_geometry,
            predict_map_pose_from_odometry,
            scan_endpoints,
            score_pose,
            search_local_matches,
        )

        endpoints = scan_endpoints(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max,
            maximum_points=LOCALIZATION_GATE_MAXIMUM_SCAN_POINTS,
        )
        if len(endpoints) < 60:
            self.motion_map_disagreement_streak = 0
            return
        alignment = self.scan_odometry_history.align(
            self.latest_amcl_pose, self.latest_amcl_stamp_ns, scan_stamp_ns)
        # AMCL and odometry are delivered on separate subscriptions. Use a
        # recent, fully bracketed measured scan when the newest scan arrived
        # ahead of its pose/odometry. Keep the 250 ms interpolation limit and
        # original fallback; never extrapolate or discard a disagreement.
        if alignment is None:
            for previous in reversed(self.motion_map_recent_scans):
                previous_ns = (previous.header.stamp.sec * 1000000000
                               + previous.header.stamp.nanosec)
                if not 0 <= scan_stamp_ns - previous_ns <= 500000000:
                    continue
                paired = self.scan_odometry_history.align(
                    self.latest_amcl_pose, self.latest_amcl_stamp_ns, previous_ns)
                if paired is not None:
                    paired_endpoints = scan_endpoints(
                        previous.ranges, previous.angle_min, previous.angle_increment,
                        previous.range_min, previous.range_max,
                        maximum_points=LOCALIZATION_GATE_MAXIMUM_SCAN_POINTS)
                    if len(paired_endpoints) < 60:
                        continue
                    endpoints = paired_endpoints
                    alignment = dict(paired, scan_selection_delay_s=(scan_stamp_ns-previous_ns)/1e9)
                    break
        evaluated_stamp_ns = scan_stamp_ns if alignment is None else alignment['scan_stamp_ns']
        if evaluated_stamp_ns <= self.motion_map_last_paired_stamp_ns:
            return
        self.motion_map_last_paired_stamp_ns = evaluated_stamp_ns
        pose = self.latest_amcl_pose if alignment is None else alignment['pose']
        candidate = score_pose(
            self.motion_map_grid, endpoints, pose["x"], pose["y"], pose["yaw"]
        )
        search = search_local_matches(
            self.motion_map_grid, endpoints, pose["x"], pose["y"], pose["yaw"],
            position_radius_m=1.0, yaw_radius_deg=15.0,
            position_step_m=0.1, yaw_step_deg=5.0,
        )
        evidence = evaluate_in_motion_pose_disagreement(candidate, search)
        if evidence["status"] == "DISAGREEMENT":
            current_ray = evaluate_pose_ray_consistency(
                self.motion_map_grid, endpoints, pose,
            )
            ray_search = filter_ray_inconsistent_matches(
                self.motion_map_grid, endpoints, search,
            )
            if current_ray["status"] == "PASS" and ray_search.get("matches"):
                evidence = evaluate_in_motion_pose_disagreement(candidate, ray_search)
            evidence = qualify_motion_disagreement_geometry(
                evidence, current_ray, ray_search,
            )
            # Use the same physically possible alternatives for odometry.
            if ray_search.get("matches"):
                search = ray_search
                # Retain the measured alternative for stopped return recovery.
                # This diagnostic never updates AMCL or authorizes movement.
                best = search['matches'][0]
                evidence['ray_checked_alternative'] = {
                    key: best[key] for key in (
                        'x', 'y', 'yaw', 'mean_likelihood',
                        'known_endpoint_ratio', 'ray_consistency')
                }
        if (self.start_amcl_pose is not None and self.start_raw_pose is not None
                and self.latest_raw_pose is not None):
            predicted = predict_map_pose_from_odometry(
                self.start_amcl_pose, self.start_raw_pose,
                self.latest_raw_pose if alignment is None else alignment['odom'],
            )
            evidence = corroborate_motion_map_disagreement(
                evidence, pose, search, predicted,
            )
        evidence['observation_timing'] = (dict(status='ALIGNED', **alignment)
            if alignment is not None else dict(status='LEGACY_FALLBACK',
                pose_stamp_ns=self.latest_amcl_stamp_ns, scan_stamp_ns=scan_stamp_ns))
        self.motion_map_checks += 1
        self.motion_map_last_evidence = evidence
        self.motion_map_disagreement_streak, trip = (
            advance_in_motion_disagreement_streak(
                self.motion_map_disagreement_streak, evidence["status"],
                MOTION_MAP_REQUIRED_CONSECUTIVE,
            )
        )
        if trip:
            self._trip_in_motion_map_guard(evidence)

    def _on_costmap(self, message: Costmap) -> None:
        self.costmap_samples += 1
        self.global_costmap = message

    def _on_guarded_command(self, message: Twist) -> None:
        super()._on_guarded_command(message)
        value = float(message.linear.x)
        if self.guarded_linear_min is None:
            self.guarded_linear_min = value
            self.guarded_linear_max = value
        else:
            self.guarded_linear_min = min(self.guarded_linear_min, value)
            self.guarded_linear_max = max(self.guarded_linear_max, value)

    def _on_monitor_command(self, message: Twist) -> None:
        self.monitor_output_samples += 1
        self.latest_monitor_command_monotonic = time.monotonic()
        value = float(message.linear.x)
        if self.monitor_linear_min is None:
            self.monitor_linear_min = value
            self.monitor_linear_max = value
        else:
            self.monitor_linear_min = min(self.monitor_linear_min, value)
            self.monitor_linear_max = max(self.monitor_linear_max, value)

    def _on_stamped_command(self, message: TwistStamped) -> None:
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec))
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        self.maximum_stamped_command_age_s = max(
            self.maximum_stamped_command_age_s, age_s,
        )
        if (not 0.0 <= age_s <= 0.10
                or not all(math.isfinite(value) for value in
                           (message.twist.linear.x, message.twist.linear.y,
                            message.twist.angular.z))
                or (self.latest_stamped_command_ns is not None
                    and stamp_ns <= self.latest_stamped_command_ns)):
            return
        self.stamped_output_samples += 1
        self.latest_stamped_command_ns = stamp_ns
        self.latest_stamped_command_monotonic = time.monotonic()

    def _on_odometry(self, message) -> None:
        super()._on_odometry(message)
        pose = message.pose.pose
        twist = message.twist.twist
        self.raw_odom_samples += 1
        self.feedback_samples += 1
        self.latest_feedback_x = float(twist.linear.x)
        self.latest_feedback_y = float(twist.linear.y)
        self.latest_feedback_angular = float(twist.angular.z)
        if (not getattr(self, '_first_motion_reported', False)
                and self.armed is True and self.state == 'ACTIVE'
                and (abs(self.latest_feedback_x) > .02
                     or abs(self.latest_feedback_y) > .02
                     or abs(self.latest_feedback_angular) > .06)):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if 0. <= time.time() - stamp <= .3:
                self._first_motion_reported = True
                print('startup_timing stage=first_motion timestamp=%.6f' % stamp, flush=True)
        self.latest_raw_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(pose.orientation),
        }
        self.latest_raw_stamp_ns = int(message.header.stamp.sec)*1000000000+int(message.header.stamp.nanosec)
        self.scan_odometry_history.add(self.latest_raw_stamp_ns, self.latest_raw_pose,
                                       (message.header.frame_id, message.child_frame_id))

    def _on_action_feedback(self, message) -> None:
        value = float(message.feedback.distance_remaining)
        if not math.isfinite(value):
            return
        self.action_feedback_samples += 1
        self.action_feedback_final_distance_m = value

    def _wait_future_with_deadman(
        self,
        future,
        timeout_s: float,
        description: str,
    ):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)
            if future.done():
                return future.result()
        raise RuntimeError("timed out waiting for %s" % description)

    def _wait_for_post_goal_command_stream(
        self,
        baseline_samples: int,
        goal_dispatched_monotonic: float,
        stamped_baseline_samples: Optional[int] = None,
    ) -> None:
        """Wait disarmed for Nav2 initialization and both fresh command streams."""

        started = time.monotonic()

        def assert_disarmed_connection() -> None:
            if self.armed is not False or self.state != "DISARMED":
                raise RuntimeError(
                    "hardware safety state changed before Nav2 arming "
                    "(state=%s armed=%s reason=%s)"
                    % (self.state, self.armed, self.transition_reason)
                )
            if not (self.feedback_connected is True
                    and self.stm32_connected is True
                    and self.transport_attached is True):
                raise RuntimeError("hardware transport or fresh feedback lost before Nav2 arming")

        def stream_is_fresh() -> bool:
            assert_disarmed_connection()
            latest = self.latest_monitor_command_monotonic
            if latest is None or latest < goal_dispatched_monotonic:
                return False
            age_s = time.monotonic() - latest
            monitor_fresh = (
                self.monitor_output_samples
                >= baseline_samples + PREARM_NAV2_COMMAND_MIN_SAMPLES
                and 0.0 <= age_s <= MAX_PREARM_NAV2_COMMAND_AGE_S
            )
            # A longer initialization wait must not admit a stale scan or
            # moved chassis. The existing short age/stop limits still apply.
            scan_ready = (self.latest_scan_monotonic is not None
                and 0.0 <= time.monotonic() - self.latest_scan_monotonic <= MAXIMUM_SCAN_AGE_S
                and self._feedback_is_stopped())
            if stamped_baseline_samples is None:
                return monitor_fresh and scan_ready
            stamped_latest = self.latest_stamped_command_monotonic
            return (monitor_fresh and scan_ready and stamped_latest is not None
                    and stamped_latest >= goal_dispatched_monotonic
                    and self.stamped_output_samples >= stamped_baseline_samples + 3
                    and 0.0 <= time.monotonic() - stamped_latest <= 0.10)

        description = "fresh post-goal Nav2 command stream before hardware arming"
        try:
            self.wait_for(stream_is_fresh, PREARM_NAV2_INITIALIZATION_TIMEOUT_S, description)
        except RuntimeError as exc:
            if str(exc) == "timed out waiting for " + description:
                # Record the original disarmed failure before emergency
                # cleanup deliberately latches ESTOP and obscures its cause.
                self._record("prearm_command_timeout", {
                    "state": self.state, "armed": self.armed,
                    "transition_reason": self.transition_reason,
                    "feedback_connected": self.feedback_connected,
                    "stm32_connected": self.stm32_connected,
                    "transport_attached": self.transport_attached,
                    "wait_s": time.monotonic() - started,
                    "monitor_new_samples": self.monitor_output_samples - baseline_samples,
                    "stamped_new_samples": (None if stamped_baseline_samples is None else
                                            self.stamped_output_samples - stamped_baseline_samples),
                })
            raise
        self._record(
            "prearm_nav2_command_stream",
            {
                "wait_s": time.monotonic() - started,
                "baseline_samples": baseline_samples,
                "observed_samples": self.monitor_output_samples,
                "new_samples": self.monitor_output_samples - baseline_samples,
                "stamped_baseline_samples": stamped_baseline_samples,
                "stamped_output_samples": getattr(self, "stamped_output_samples", None),
                "latest_stamped_command_ns": getattr(self, "latest_stamped_command_ns", None),
                "maximum_stamped_command_age_s": getattr(self, "maximum_stamped_command_age_s", None),
                "latest_age_s": (
                    None
                    if self.latest_monitor_command_monotonic is None
                    else time.monotonic()
                    - self.latest_monitor_command_monotonic
                ),
            },
        )

    def _wait_navigation_result(self, future, timeout_s: float):
        """Fail immediately if the hardware safety owner stops the chassis."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)
            if getattr(self, "motion_map_guard_tripped", False):
                raise RuntimeError("in-motion scan/map localization disagreement; stopped")
            if future.done():
                return future.result()
            if self.armed is False or self.state in (
                "WATCHDOG_LATCHED",
                "ESTOP_LATCHED",
            ):
                # Armed/state/reason are separate DDS topics. Once stopped,
                # collect the pending reason before cleanup overwrites it.
                stopped_at = time.monotonic()
                while (self.transition_reason in {'active_autonomy', 'operator_arm'}
                       and time.monotonic() - stopped_at < 0.10):
                    rclpy.spin_once(self, timeout_sec=0.005)
                self._record('navigation_hardware_stop_observed', {
                    'armed': self.armed, 'state': self.state,
                    'transition_reason': self.transition_reason,
                })
                raise RuntimeError(
                    "hardware safety stop during NavigateToPose "
                    "(state=%s reason=%s)"
                    % (self.state, self.transition_reason)
                )
        self._record("navigation_result_timeout", {
            "state": self.state, "transition_reason": self.transition_reason,
            "armed": self.armed,
        })
        raise RuntimeError("timed out waiting for NavigateToPose result")

    def _start_deadman_keepalive(self) -> None:
        """Keep the short hardware deadman window alive during Nav2 waits."""
        if self._deadman_keepalive_thread is not None:
            return
        from control_keepalive import start_independent_publisher
        self._record('independent_deadman_pid', start_independent_publisher())
        stop_event = threading.Event()
        self._deadman_keepalive_stop = stop_event
        ready = threading.Event()
        startup_errors = []

        def keepalive() -> None:
            try:
                from control_keepalive import configure_keepalive_thread
                cpu = configure_keepalive_thread()
                self._record('deadman_thread_cpu', cpu)
            except Exception as exc:
                startup_errors.append(exc)
                ready.set()
                return
            ready.set()
            while not stop_event.is_set():
                try:
                    now = time.monotonic()
                    if self._deadman_keepalive_last_monotonic is not None:
                        gap_s = now - self._deadman_keepalive_last_monotonic
                        if (
                            self.deadman_keepalive_max_gap_s is None
                            or gap_s > self.deadman_keepalive_max_gap_s
                        ):
                            self.deadman_keepalive_max_gap_s = gap_s
                        if gap_s > DEADMAN_KEEPALIVE_LATE_GAP_S:
                            self.deadman_keepalive_late_gap_count += 1
                    self._deadman_keepalive_last_monotonic = now
                    self.publish_deadman(True)
                    self.deadman_keepalive_publish_count += 1
                except BaseException:
                    break
                stop_event.wait(DEADMAN_KEEPALIVE_PERIOD_S)

        thread = threading.Thread(
            target=keepalive,
            name="rvclaw-deadman-keepalive",
            daemon=True,
        )
        self._deadman_keepalive_thread = thread
        thread.start()
        if not ready.wait(1.0) or startup_errors:
            self._stop_deadman_keepalive()
            raise RuntimeError('deadman thread allocation failed before arming: %s'
                               % (startup_errors or 'startup timeout'))

    def _stop_deadman_keepalive(self) -> None:
        stop_event = self._deadman_keepalive_stop
        thread = self._deadman_keepalive_thread
        self._deadman_keepalive_stop = None
        self._deadman_keepalive_thread = None
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        from control_keepalive import pause_independent_publisher
        pause_independent_publisher()

    def _request_amcl_nomotion_update(self) -> None:
        """Collect fresh AMCL poses after subscribing to /amcl_pose."""
        if not self.amcl_nomotion_client.wait_for_service(
            timeout_sec=STARTUP_TIMEOUT_S
        ):
            raise RuntimeError("AMCL no-motion update service is unavailable")
        for sample_index in range(MINIMUM_AMCL_SAMPLES):
            previous_samples = self.amcl_samples
            response_confirmed = False
            for attempt in range(1, AMCL_NOMOTION_MAX_ATTEMPTS + 1):
                future = self.amcl_nomotion_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(
                    self,
                    future,
                    timeout_sec=AMCL_NOMOTION_RESPONSE_TIMEOUT_S,
                )
                if future.done() and future.exception() is None:
                    response_confirmed = True
                    break
                if attempt < AMCL_NOMOTION_MAX_ATTEMPTS:
                    self._record(
                        "amcl_nomotion_retry",
                        {"sample": sample_index + 1, "attempt": attempt},
                    )
            if not response_confirmed:
                raise RuntimeError("AMCL no-motion update service timed out")
            if self.amcl_samples <= previous_samples:
                # AMCL's no-motion service can successfully force an update
                # without publishing a duplicate pose when the robot is
                # stationary.  The service response is the refresh
                # acknowledgement. Reuse a pose only after the readiness
                # message count is met; otherwise wait before requesting again.
                if (
                    getattr(self, "latest_amcl_pose", None) is not None
                    and self.amcl_samples >= MINIMUM_AMCL_SAMPLES
                ):
                    self._record(
                        "amcl_nomotion_pose_reused",
                        {"sample": sample_index + 1},
                    )
                else:
                    self.wait_for(
                        lambda: self.amcl_samples > previous_samples,
                        SERVICE_TIMEOUT_S,
                        "fresh AMCL pose (initial) after no-motion update",
                    )
        self._record(
            "amcl_nomotion_updates_confirmed", MINIMUM_AMCL_SAMPLES
        )

    def _cancel_goal(self) -> None:
        if self.goal_handle is None:
            return
        try:
            future = self.goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not future.done():
                self.publish_estop(True)
                self.publish_deadman(False)
                rclpy.spin_once(self, timeout_sec=INTERVAL_S)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    "timed out waiting for NavigateToPose cancellation"
                )
            self._record("action_cancel_requested", True)
        except BaseException as exc:
            self._record("action_cancel_error", str(exc))

    def _feedback_is_stopped(self) -> bool:
        return (
            abs(self.latest_feedback_x) <= MAX_POST_STOP_LINEAR_M_S
            and abs(self.latest_feedback_y) <= MAX_POST_STOP_LINEAR_M_S
            and abs(self.latest_feedback_angular)
            <= MAX_POST_STOP_ANGULAR_RAD_S
        )

    def _wait_for_stable_feedback_stop(self) -> None:
        started = time.monotonic()
        deadline = started + POST_ACTION_STOP_TIMEOUT_S
        last_sample_count = self.feedback_samples
        stable_samples = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)
            if self.feedback_samples == last_sample_count:
                continue
            last_sample_count = self.feedback_samples
            if self._feedback_is_stopped():
                stable_samples += 1
                if stable_samples >= POST_ACTION_STABLE_SAMPLES:
                    self.post_stop_stable_samples = stable_samples
                    self.post_stop_settle_s = time.monotonic() - started
                    self._record("post_stop_stable_samples", stable_samples)
                    self._record("post_stop_settle_s", self.post_stop_settle_s)
                    return
            else:
                stable_samples = 0
        raise RuntimeError(
            "timed out waiting for stable post-goal feedback stop"
        )

    def _pre_arm_gate(self, check_localization: bool = True) -> None:
        if check_localization:
            if (
                self.latest_scan_monotonic is None
                or self.front_clearance_m is None
            ):
                raise RuntimeError("front scan clearance is unavailable")
            self.pre_arm_scan_age_s = time.monotonic() - self.latest_scan_monotonic
            if self.pre_arm_scan_age_s > MAXIMUM_SCAN_AGE_S:
                raise RuntimeError("front scan is stale before arming")
            self.pre_arm_front_clearance_m = self.front_clearance_m
        else:
            # A route preflight with an explicit start pose describes a future
            # waypoint.  The live scan belongs to the robot's current pose and
            # must be checked when that segment is actually armed, not while
            # validating an unvisited future segment.
            self.pre_arm_scan_age_s = None
            self.pre_arm_front_clearance_m = None
        if self.latest_amcl_pose is None or self.global_costmap is None:
            raise RuntimeError("AMCL pose or global costmap is unavailable")
        if check_localization:
            self._check_static_map_localization()
        self.goal_pose = self._compute_goal_pose()
        self.goal_cost = costmap_value_at(
            self.global_costmap,
            self.goal_pose["x"],
            self.goal_pose["y"],
        )
        if self.goal_cost is None:
            raise RuntimeError("relative goal is outside the global costmap")
        if self.goal_cost > MAXIMUM_GOAL_COST:
            raise RuntimeError("relative goal cell is not admitted free space")
        self.start_cost = costmap_value_at(
            self.global_costmap,
            self.latest_amcl_pose["x"],
            self.latest_amcl_pose["y"],
        )
        self._record("front_clearance_m", self.front_clearance_m)
        self._record("pre_arm_scan_age_s", self.pre_arm_scan_age_s)
        self._record("goal_cost", self.goal_cost)
        self._record("start_cost", self.start_cost)
        self._record("goal_pose", self.goal_pose)

    def _refresh_trusted_localization_callbacks(self) -> None:
        """Process queued status and scan callbacks after the bounded map search."""
        deadline = time.monotonic() + TRUSTED_LOCALIZATION_REFRESH_TIMEOUT_S
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            status_at = getattr(self, "localization_runtime_status_monotonic", None)
            scan_at = getattr(self, "latest_scan_monotonic", None)
            localization_scan_at = getattr(self, "latest_localization_scan_monotonic", None)
            valid_at = getattr(self, "localization_runtime_valid_monotonic", None)
            status = getattr(self, "localization_runtime_status", None) or {}
            if (
                status_at is not None and now - status_at <= 0.75
                and valid_at is not None and now - valid_at <= 0.75
                and self.localization_runtime_valid is status.get("valid")
                and scan_at is not None and now - scan_at <= MAXIMUM_SCAN_AGE_S
                and localization_scan_at is not None and now - localization_scan_at <= MAXIMUM_SCAN_AGE_S
            ):
                return
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)

    def _check_static_map_localization(self) -> None:
        """Reject stable AMCL poses that disagree with an independent map search."""
        if (
            self.localization_gate_result is not None
            and self.localization_gate_result.get("status") == "PASS"
            and self.localization_gate_result.get("mode") != "TRUSTED_HOT_LOCAL"
            and self.localization_gate_checked_monotonic is not None
            and self.localization_gate_pose is not None
            and time.monotonic() - self.localization_gate_checked_monotonic
            <= LOCALIZATION_GATE_CACHE_S
            and distance(self.localization_gate_pose, self.latest_amcl_pose)
            <= LOCALIZATION_GATE_CACHE_POSITION_M
            and abs(
                normalize_angle(
                    self.localization_gate_pose["yaw"]
                    - self.latest_amcl_pose["yaw"]
                )
            )
            <= LOCALIZATION_GATE_CACHE_YAW_RAD
        ):
            self._record("static_map_localization_gate_cache_hit", True)
            return
        if (
            self.localization_gate_scan_sample_count == self.localization_scan_samples
            and (self.localization_gate_result or {}).get("mode") != "TRUSTED_HOT_LOCAL"
        ):
            if self.localization_gate_result is not None and self.localization_gate_result.get(
                "status"
            ) != "PASS":
                raise RuntimeError(
                    "independent static-map localization gate failed: %s"
                    % "; ".join(self.localization_gate_result.get("failures", []))
                )
            return
        if (self.latest_localization_scan is None or self.latest_localization_scan_monotonic is None
                or time.monotonic() - self.latest_localization_scan_monotonic > MAXIMUM_SCAN_AGE_S):
            raise RuntimeError("latest LaserScan is unavailable for localization gate")
        map_file = required_active_map_file()
        try:
            from rvclaw_nav2_bringup.fast_start_policy import map_identity
            from rvclaw_nav2_bringup.scan_map_consistency import (
                compare_candidate_to_search,
                evaluate_localization_gate,
                load_static_map,
                scan_endpoints,
                score_pose,
                search_global_matches,
                search_local_matches,
            )

            grid = load_static_map(map_file)
            endpoints = scan_endpoints(
                self.latest_localization_scan.ranges,
                self.latest_localization_scan.angle_min,
                self.latest_localization_scan.angle_increment,
                self.latest_localization_scan.range_min,
                self.latest_localization_scan.range_max,
                maximum_points=LOCALIZATION_GATE_MAXIMUM_SCAN_POINTS,
            )
            candidate = score_pose(
                grid,
                endpoints,
                self.latest_amcl_pose["x"],
                self.latest_amcl_pose["y"],
                self.latest_amcl_pose["yaw"],
            )
            local_search = search_local_matches(
                grid, endpoints,
                self.latest_amcl_pose["x"],
                self.latest_amcl_pose["y"],
                self.latest_amcl_pose["yaw"],
                position_radius_m=0.30,
                yaw_radius_deg=20.0,
                position_step_m=0.05,
                yaw_step_deg=2.5,
            )
            # Local search blocks this single-threaded executor briefly. Drain
            # fresh ROS callbacks before checking the unchanged 1.5 s status
            # gate; never treat an old callback timestamp as current evidence.
            self._refresh_trusted_localization_callbacks()
            status_at_scan = getattr(self, "localization_runtime_status", None)
            status_at = getattr(self, "localization_runtime_status_monotonic", None)
            status_age = math.inf if status_at is None else time.monotonic() - status_at
            trusted_gate = evaluate_trusted_hot_localization(
                status_at_scan,
                getattr(self, "localization_runtime_valid", None),
                status_age,
                map_identity(map_file)["sha256"],
                self.latest_amcl_pose,
                candidate,
                local_search,
            )
            # The runtime monitor samples 60 endpoints; startup's independent
            # check samples 240. A marginal sparse score must be remeasured,
            # not sent straight to an ambiguous corridor-wide search.
            sparse_score = (status_at_scan or {}).get("scan_map_score") or {}
            if (trusted_gate["failures"] == ["continuous runtime scan/map score is weak"]
                    and sparse_score.get("endpoint_count") == 60
                    and .80 <= sparse_score.get("mean_likelihood", 0.) < .85):
                dense_support = self._recheck_hot_runtime_score(grid)
                if dense_support["status"] == "PASS":
                    refreshed_status = dict(self.localization_runtime_status)
                    refreshed_status["scan_map_score"] = dense_support["candidate"]
                    trusted_gate = evaluate_trusted_hot_localization(
                        refreshed_status, self.localization_runtime_valid,
                        time.monotonic() - self.localization_runtime_status_monotonic,
                        map_identity(map_file)["sha256"], self.latest_amcl_pose,
                        candidate, local_search)
                trusted_gate["dense_runtime_recheck"] = dense_support
            if trusted_gate["status"] == "PASS":
                if (
                    self.latest_scan_monotonic is None
                    or time.monotonic() - self.latest_scan_monotonic > MAXIMUM_SCAN_AGE_S
                ):
                    raise RuntimeError("fresh scan expired during trusted localization check")
                self.pre_arm_scan_age_s = time.monotonic() - self.latest_scan_monotonic
                self.pre_arm_front_clearance_m = self.front_clearance_m
                self.localization_gate_result = {
                    "status": "PASS",
                    "mode": "TRUSTED_HOT_LOCAL",
                    "failures": [],
                    "candidate": candidate,
                    "comparison": trusted_gate["comparison"],
                    "best_matches": trusted_gate["best_matches"],
                    "gate": trusted_gate,
                    "map_file": map_file,
                }
            else:
                search = search_global_matches(
                    grid,
                    endpoints,
                    position_step_m=LOCALIZATION_GATE_POSITION_STEP_M,
                    yaw_step_deg=LOCALIZATION_GATE_YAW_STEP_DEG,
                )
                comparison = compare_candidate_to_search(candidate, search)
                gate = evaluate_localization_gate(
                    candidate,
                    comparison,
                    minimum_known_endpoint_ratio=LOCALIZATION_GATE_MINIMUM_KNOWN_RATIO,
                    minimum_best_score=LOCALIZATION_GATE_MINIMUM_BEST_SCORE,
                    minimum_candidate_to_best_ratio=LOCALIZATION_GATE_MINIMUM_CANDIDATE_RATIO,
                    maximum_candidate_to_best_distance_m=LOCALIZATION_GATE_MAXIMUM_DISTANCE_M,
                    maximum_candidate_to_best_yaw_deg=LOCALIZATION_GATE_MAXIMUM_YAW_DEG,
                )
                gate = apply_localization_near_miss_override(
                    candidate,
                    comparison,
                    gate,
                )
                self.localization_gate_result = {
                    "status": gate["status"],
                    "mode": "GLOBAL_STATIC_SEARCH",
                    "failures": gate["failures"],
                    "candidate": candidate,
                    "comparison": comparison,
                    "best_matches": search.get("matches", [])[:3],
                    "gate": gate,
                    "trusted_hot_local_rejection": trusted_gate,
                    "map_file": map_file,
                }
        except BaseException as exc:
            self.localization_gate_result = {
                "status": "FAIL",
                "failures": ["static-map localization gate error: %s" % exc],
                "map_file": map_file,
            }
        self.localization_gate_scan_sample_count = self.localization_scan_samples
        self.localization_gate_checked_monotonic = time.monotonic()
        self.localization_gate_pose = dict(self.latest_amcl_pose)
        self._record("static_map_localization_gate", self.localization_gate_result)
        if self.localization_gate_result.get("status") != "PASS":
            raise RuntimeError(
                "independent static-map localization gate failed: %s"
                % "; ".join(self.localization_gate_result.get("failures", []))
            )

    def _recheck_hot_runtime_score(self, grid):
        """Two fresh stopped scans, using startup's unchanged score/ray gates.

        No pose publication, state reset or motion API is used here. Only the
        sparse monitor score can be superseded; the caller rechecks every
        runtime/continuity/local-search condition afterwards.
        """
        from rvclaw_nav2_bringup.scan_map_consistency import (
            scan_endpoints, score_pose, evaluate_pose_ray_consistency,
        )
        started = time.monotonic()
        deadline = started + 1.5
        pose = dict(self.latest_amcl_pose)
        initial_status = dict(self.localization_runtime_status)
        initial_feedback = self.feedback_samples
        checks = []
        result = {"status": "FAIL", "scans": checks, "motion_authorized": False}
        try:
            while time.monotonic() < deadline and len(checks) < 2:
                rclpy.spin_once(self, timeout_sec=INTERVAL_S)
                if (self.armed is not False or self.state not in {"DISARMED", "ESTOP_LATCHED"}
                        or not self._feedback_is_stopped()):
                    raise RuntimeError("dense localization recheck requires a stopped unarmed chassis")
                current = self.latest_amcl_pose
                if (distance(pose, current) > .05
                        or abs(normalize_angle(pose['yaw']-current['yaw'])) > math.radians(3)):
                    raise RuntimeError("pose changed during dense localization recheck")
                scan = self.latest_localization_scan
                received = self.latest_localization_scan_monotonic
                if scan is None or received is None or received < started:
                    continue
                stamp = scan.header.stamp.sec*10**9 + scan.header.stamp.nanosec
                if checks and (stamp-checks[-1]['stamp_ns'])/1e9 < .30:
                    continue
                age = (self.get_clock().now().nanoseconds-stamp)/1e9
                if not 0 <= age <= MAXIMUM_SCAN_AGE_S:
                    continue
                if (not scan.header.frame_id or scan.angle_increment <= 0
                        or (len(scan.ranges)-1)*scan.angle_increment < math.radians(330)):
                    raise RuntimeError("dense localization recheck needs full-frame scan coverage")
                if checks and scan.header.frame_id != checks[0]['frame_id']:
                    raise RuntimeError("scan frame changed during dense localization recheck")
                def points(count):
                    return scan_endpoints(scan.ranges,scan.angle_min,scan.angle_increment,
                        scan.range_min,scan.range_max,maximum_range_m=12.,maximum_points=count)
                candidate = score_pose(grid,points(240),pose['x'],pose['y'],pose['yaw'])
                rays = evaluate_pose_ray_consistency(grid,points(480),pose)
                passed = (candidate['endpoint_count'] >= 60
                          and candidate['mean_likelihood'] >= .85
                          and candidate['known_endpoint_ratio'] >= .95
                          and rays['status'] == 'PASS')
                checks.append(dict(status='PASS' if passed else 'FAIL',candidate=candidate,
                    ray_consistency=rays,stamp_ns=stamp,frame_id=scan.header.frame_id))
                if not passed:
                    raise RuntimeError("fresh dense scan does not support the verified pose")
            self._refresh_trusted_localization_callbacks()
            status = self.localization_runtime_status or {}
            if (len(checks) != 2 or time.monotonic() > deadline
                    or not 0 <= (self.get_clock().now().nanoseconds - checks[-1]['stamp_ns']) / 1e9 <= MAXIMUM_SCAN_AGE_S
                    or self.feedback_samples-initial_feedback < 3
                    or self.armed is not False or not self._feedback_is_stopped()
                    or self.state not in {"DISARMED", "ESTOP_LATCHED"}
                    or status.get('map_sha256') != initial_status.get('map_sha256')
                    or status.get('external_verification_epoch') != initial_status.get('external_verification_epoch')
                    or distance(pose,self.latest_amcl_pose) > .05
                    or abs(normalize_angle(pose['yaw']-self.latest_amcl_pose['yaw'])) > math.radians(3)):
                raise RuntimeError("dense localization evidence expired or continuity changed")
            candidate = dict(checks[-1]['candidate'])
            for key in ('mean_likelihood','known_endpoint_ratio','endpoint_count'):
                candidate[key] = min(check['candidate'][key] for check in checks)
            result.update(status='PASS',candidate=candidate)
        except Exception as exc:
            result['error'] = str(exc)
        result['duration_s'] = time.monotonic()-started
        self._record('trusted_hot_dense_recheck',result)
        return result

    def _preflight_path(
        self,
        start_pose: Optional[Dict[str, float]] = None,
    ) -> None:
        """Prove Nav2 can produce a path before admitting motor arming."""
        if not self.compute_path_action.wait_for_server(
            timeout_sec=STARTUP_TIMEOUT_S
        ):
            self.path_check_error = (
                "ComputePathToPose action server is unavailable"
            )
            raise RuntimeError(self.path_check_error)
        if self.goal_pose is None:
            self.path_check_error = "goal pose is unavailable for path preflight"
            raise RuntimeError(self.path_check_error)

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = self.goal_pose["x"]
        goal.goal.pose.position.y = self.goal_pose["y"]
        goal.goal.pose.orientation.z = math.sin(self.goal_pose["yaw"] / 2.0)
        goal.goal.pose.orientation.w = math.cos(self.goal_pose["yaw"] / 2.0)
        goal.planner_id = "GridBased"
        if start_pose is None:
            goal.use_start = False
        else:
            goal.use_start = True
            goal.start.header.frame_id = "map"
            goal.start.header.stamp = goal.goal.header.stamp
            goal.start.pose.position.x = start_pose["x"]
            goal.start.pose.position.y = start_pose["y"]
            goal.start.pose.orientation.z = math.sin(start_pose["yaw"] / 2.0)
            goal.start.pose.orientation.w = math.cos(start_pose["yaw"] / 2.0)

        started = time.monotonic()
        try:
            send_future = self.compute_path_action.send_goal_async(goal)
            handle = self._wait_future_with_deadman(
                send_future,
                SERVICE_TIMEOUT_S,
                "ComputePathToPose goal acceptance",
            )
            if handle is None or not handle.accepted:
                self.path_check_error = "ComputePathToPose goal was rejected"
                raise RuntimeError(self.path_check_error)
            result_future = handle.get_result_async()
            response = self._wait_future_with_deadman(
                result_future,
                PATH_RESULT_TIMEOUT_S,
                "ComputePathToPose result",
            )
            self.path_check_status = int(response.status)
            path = response.result.path
            self.path_check_pose_count = len(path.poses)
            self.path_departure_poses = [
                {
                    "x": float(stamped.pose.position.x),
                    "y": float(stamped.pose.position.y),
                }
                for stamped in path.poses[:12]
            ]
            self.path_alignment_poses = [
                {"x": float(stamped.pose.position.x), "y": float(stamped.pose.position.y)}
                for stamped in path.poses[:80]
            ]
            self.path_check_elapsed_s = time.monotonic() - started
            self._record("path_check_status", self.path_check_status)
            self._record("path_check_pose_count", self.path_check_pose_count)
            self._record("path_check_elapsed_s", self.path_check_elapsed_s)
            if self.path_check_status != GoalStatus.STATUS_SUCCEEDED:
                self.path_check_error = (
                    "ComputePathToPose did not report success (status=%d)"
                    % self.path_check_status
                )
                raise RuntimeError(self.path_check_error)
            if self.path_check_pose_count < 2:
                self.path_check_error = (
                    "ComputePathToPose returned an empty or one-pose path"
                )
                raise RuntimeError(self.path_check_error)
            if start_pose is None:
                # Planning may outlive the scan freshness window on K1. Refresh
                # before the base clearance check (a subclass refresh happens
                # too late if this check already raised). Keep the existing
                # age limit and wait for both live scan subscriptions.
                before_samples = self.scan_samples
                self._wait_for_fresh_scan()
                self.pre_arm_scan_age_s = time.monotonic() - self.latest_scan_monotonic
                self.pre_arm_front_clearance_m = self.front_clearance_m
                self._record("path_clearance_scan_refresh", {
                    "samples_before": before_samples,
                    "samples_after": self.scan_samples,
                    "latest_age_s": self.pre_arm_scan_age_s,
                    "front_clearance_m": self.pre_arm_front_clearance_m,
                })
            clearance_start_pose = (
                self.latest_amcl_pose if start_pose is None else start_pose
            )
            self.start_cost = costmap_value_at(
                self.global_costmap,
                clearance_start_pose["x"],
                clearance_start_pose["y"],
            )
            self.initial_path_heading_delta_rad = path_escape_heading_delta(
                path,
                clearance_start_pose,
            )
            if start_pose is None:
                self.rotation_clearance = None
                if self.pre_arm_front_clearance_m < NEAR_START_HARD_STOP_CLEARANCE_M:
                    if (self.latest_scan is None or self.latest_scan_monotonic is None
                            or time.monotonic() - self.latest_scan_monotonic > MAXIMUM_SCAN_AGE_S):
                        raise RuntimeError("rotation sweep scan is unavailable or stale")
                    from rvclaw_nav2_bringup.scan_trial_policy import (
                        evaluate_rotation_clearance,
                    )

                    scan = self.latest_scan
                    self.rotation_clearance = evaluate_rotation_clearance(
                        scan.ranges, scan.angle_min, scan.angle_increment,
                        scan.range_min, scan.range_max,
                        minimum_radial_m=NEAR_START_ROTATION_SWEEP_CLEARANCE_M,
                        minimum_coverage_deg=330.0,
                    )
                self.start_clearance_mode = classify_start_clearance(
                    self.pre_arm_front_clearance_m,
                    self.start_cost,
                    self.initial_path_heading_delta_rad,
                    self.rotation_clearance,
                )
            else:
                self.start_clearance_mode = classify_future_segment_start(
                    self.start_cost,
                    self.initial_path_heading_delta_rad,
                )
            self._record(
                "initial_path_heading_delta_rad",
                self.initial_path_heading_delta_rad,
            )
            self._record("start_clearance_mode", self.start_clearance_mode)
            self._record("rotation_clearance", self.rotation_clearance)
        except BaseException as exc:
            if self.path_check_error is None:
                self.path_check_error = str(exc)
            raise

    def run_path_preflight(
        self,
        start_pose: Optional[Dict[str, float]] = None,
    ) -> Dict[str, object]:
        """Validate one path without resetting latches, arming, or commanding."""
        self.wait_for(
            lambda: self.armed is False,
            STARTUP_TIMEOUT_S,
            "disarmed hardware state",
        )
        self._request_amcl_nomotion_update()
        self.wait_for(
            lambda: (
                self.scan_samples >= MINIMUM_SCAN_SAMPLES
                and self.amcl_samples >= MINIMUM_AMCL_SAMPLES
                and self.costmap_samples >= MINIMUM_COSTMAP_SAMPLES
            ),
            STARTUP_TIMEOUT_S,
            "scan, AMCL, and global costmap readiness for path preflight",
        )
        self._wait_for_fresh_scan()
        self._pre_arm_gate(check_localization=start_pose is None)
        self._preflight_path(start_pose=start_pose)
        return {
            "status": "PASS",
            "goal_pose": self.goal_pose,
            "start_pose": start_pose,
            "start_source": "explicit" if start_pose is not None else "current_amcl",
            "path_check_status": self.path_check_status,
            "path_check_pose_count": self.path_check_pose_count,
            "path_check_elapsed_s": self.path_check_elapsed_s,
            "path_check_error": self.path_check_error,
            "start_cost": self.start_cost,
            "initial_path_heading_delta_rad": self.initial_path_heading_delta_rad,
            "start_clearance_mode": self.start_clearance_mode,
            "rotation_clearance": self.rotation_clearance,
            "static_map_localization_gate": self.localization_gate_result,
            "final_armed": self.armed,
        }

    def _wait_for_fresh_scan(self) -> None:
        previous_samples = self.scan_samples
        previous_localization_samples = self.localization_scan_samples
        self.wait_for(
            lambda: (
                self.scan_samples > previous_samples
                and self.localization_scan_samples > previous_localization_samples
                and self.latest_localization_scan_monotonic is not None
                and time.monotonic() - self.latest_localization_scan_monotonic <= MAXIMUM_SCAN_AGE_S
                and self.latest_scan_monotonic is not None
                and time.monotonic() - self.latest_scan_monotonic
                <= MAXIMUM_SCAN_AGE_S
            ),
            SERVICE_TIMEOUT_S,
            "a fresh scan immediately before arming",
        )

    def _compute_goal_pose(self) -> Dict[str, float]:
        """Return this profile's map-frame goal based on the current pose."""
        goal_x, goal_y, goal_yaw = relative_goal(
            self.latest_amcl_pose["x"],
            self.latest_amcl_pose["y"],
            self.latest_amcl_pose["yaw"],
        )
        return {"x": goal_x, "y": goal_y, "yaw": goal_yaw}

    def _configure_navigation_goal(self, goal: NavigateToPose.Goal) -> None:
        """Allow reviewed subclasses to select a package-owned behavior tree."""

    def evaluate_navigation_evidence(
        self,
        metrics: Dict[str, object],
    ) -> list:
        failures = evaluate_evidence(metrics)
        if self.motion_map_guard_tripped:
            failures.append("in-motion scan/map localization guard stopped the robot")
        return failures

    def _initial_path_preflight_required(self) -> bool:
        return True

    def _prepare_navigation_localization(self) -> None:
        self._request_amcl_nomotion_update()

    def _navigation_localization_ready(self) -> bool:
        return self.amcl_samples >= MINIMUM_AMCL_SAMPLES

    def run_navigation(self) -> Dict[str, object]:
        self.wait_for(
            lambda: self.transport_attached is True,
            STARTUP_TIMEOUT_S,
            "hardware transport attachment",
        )
        self.wait_for(
            lambda: (
                self.feedback_connected is True
                and self.stm32_connected is True
                and self.voltage_v is not None
                and self.voltage_v >= MIN_BATTERY_VOLTAGE_V
            ),
            STARTUP_TIMEOUT_S,
            "fresh feedback and safe voltage",
        )
        self._prepare_navigation_localization()
        self.wait_for(
            lambda: (
                self.scan_samples >= MINIMUM_SCAN_SAMPLES
                and self._navigation_localization_ready()
                and self.costmap_samples >= MINIMUM_COSTMAP_SAMPLES
                and self.raw_odom_samples >= 5
                and self.latest_raw_pose is not None
            ),
            STARTUP_TIMEOUT_S,
            "scan, AMCL, odometry, and global costmap readiness",
        )
        if not self.navigate_action.wait_for_server(
            timeout_sec=STARTUP_TIMEOUT_S
        ):
            raise RuntimeError("NavigateToPose action server is unavailable")
        if self.armed:
            raise RuntimeError(
                "hardware controller unexpectedly started armed"
            )
        self._wait_for_fresh_scan()
        self._pre_arm_gate()
        if self._initial_path_preflight_required():
            self._preflight_path()
        self._prepare_in_motion_map_guard()

        if getattr(self, 'freshness_recovery_disarmed_only', False):
            if not (self.armed is False and self.state == 'DISARMED'
                    and self.transport_attached is True and self.feedback_connected is True
                    and self.stm32_connected is True):
                raise RuntimeError('freshness recovery cannot reset a changed hardware state')
            self._record('freshness_recovery_kept_disarmed_state', True)
        else:
            self.publish_estop(False)
            self._call_reset()
        self.wait_for(
            lambda: self.state == "DISARMED",
            SERVICE_TIMEOUT_S,
            "DISARMED state after reset",
        )
        deadline = time.monotonic() + DEADMAN_PRIME_S
        while time.monotonic() < deadline:
            self.publish_deadman(True)
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)
        self._start_deadman_keepalive()
        self._wait_for_fresh_scan()
        self._pre_arm_gate()
        self._preflight_path()
        self.start_raw_pose = dict(self.latest_raw_pose)
        self.start_amcl_pose = dict(self.latest_amcl_pose)
        self.start_raw_sample_count = self.raw_odom_samples
        self.start_amcl_sample_count = self.amcl_samples

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_pose["x"]
        goal.pose.pose.position.y = self.goal_pose["y"]
        goal.pose.pose.orientation.z = math.sin(self.goal_pose["yaw"] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_pose["yaw"] / 2.0)
        self._configure_navigation_goal(goal)

        command_stream_baseline = self.monitor_output_samples
        stamped_stream_baseline = self.stamped_output_samples
        action_started = time.monotonic()
        send_future = self.navigate_action.send_goal_async(
            goal,
            feedback_callback=self._on_action_feedback,
        )
        self.goal_handle = self._wait_future_with_deadman(
            send_future,
            SERVICE_TIMEOUT_S,
            "NavigateToPose goal acceptance",
        )
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")
        self._record("action_goal_accepted", True)

        result_future = self.goal_handle.get_result_async()
        self._wait_for_post_goal_command_stream(
            command_stream_baseline,
            action_started,
            stamped_stream_baseline,
        )
        self._call_arm(True)
        self.wait_for(
            lambda: self.armed is True,
            SERVICE_TIMEOUT_S,
            "armed state after fresh Nav2 command stream",
        )
        self.wait_for(
            lambda: self.state == "ACTIVE"
            or self.state in ("WATCHDOG_LATCHED", "ESTOP_LATCHED"),
            SERVICE_TIMEOUT_S,
            "ACTIVE state from post-arm Nav2 command stream",
        )
        if self.state != "ACTIVE":
            raise RuntimeError(
                "hardware did not enter ACTIVE after post-goal arming "
                "(state=%s reason=%s)"
                % (self.state, self.transition_reason)
            )
        self._record("post_goal_hardware_active", True)
        self.motion_map_guard_active = True
        try:
            result_response = self._wait_navigation_result(
                result_future,
                ACTION_RESULT_TIMEOUT_S,
            )
        finally:
            self.motion_map_guard_active = False
            self.action_elapsed_s = time.monotonic() - action_started
        self.action_status = int(result_response.status)
        self._record("action_status", self.action_status)

        # The navigation action is terminal. Revoke motion before draining
        # queued callbacks/zero frames; keeping it armed during that drain let
        # delayed controller commands carry the chassis past an arrived point.
        self._call_arm(False)
        self._stop_deadman_keepalive()
        self.publish_deadman(False)
        for _ in range(POST_ACTION_ZERO_FRAMES):
            self.publish_zero_command()
            rclpy.spin_once(self, timeout_sec=INTERVAL_S)
        self.wait_for(
            lambda: self.armed is False
            and self.state in ("DISARMED", "WATCHDOG_LATCHED", "ESTOP_LATCHED"),
            SERVICE_TIMEOUT_S,
            "explicit disarm or latched stopped state",
        )
        self._wait_for_stable_feedback_stop()
        self.final_raw_pose = dict(self.latest_raw_pose)
        self.final_amcl_pose = dict(self.latest_amcl_pose)

        if self.action_status != GoalStatus.STATUS_SUCCEEDED:
            self._record_failed_navigation_lifecycles()

        metrics = self.metrics()
        failures = self.evaluate_navigation_evidence(metrics)
        if failures:
            raise RuntimeError("; ".join(failures))
        return self.result("PASS", None)

    def _record_failed_navigation_lifecycles(self) -> None:
        """Inspect a stopped aborted action before attributing its distance error.

        Nav2 deactivation aborts an unfinished action with a nonzero remaining
        distance. That distance alone does not demonstrate a localization fault.
        No service here changes a lifecycle or grants motion.
        """
        if self.armed is not False:
            return
        clients = {}
        futures = {}
        try:
            for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator'):
                client = self.create_client(GetState, '/' + name + '/get_state')
                clients[name] = client
                futures[name] = client.call_async(GetState.Request())
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not all(f.done() for f in futures.values()):
                rclpy.spin_once(self, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
            states = {name: (future.result().current_state.id
                      if future.done() and future.exception() is None and future.result() else None)
                      for name, future in futures.items()}
            self.navigation_lifecycle_states = states
            self._record('failed_navigation_lifecycle_states', states)
        except Exception as error:
            self._record('failed_navigation_lifecycle_diagnostic_error', str(error))
        finally:
            for future in futures.values():
                if not future.done():
                    future.cancel()
            for client in clients.values():
                self.destroy_client(client)

    def metrics(self) -> Dict[str, object]:
        raw_forward = None
        raw_lateral = None
        raw_yaw = None
        if self.start_raw_pose is not None and self.final_raw_pose is not None:
            raw_forward, raw_lateral = body_delta(
                self.start_raw_pose,
                self.final_raw_pose,
            )
            raw_yaw = normalize_angle(
                self.final_raw_pose["yaw"] - self.start_raw_pose["yaw"]
            )
        amcl_displacement = None
        if (
            self.start_amcl_pose is not None
            and self.final_amcl_pose is not None
        ):
            amcl_displacement = distance(
                self.start_amcl_pose,
                self.final_amcl_pose,
            )
        return {
            "action_status": self.action_status,
            "action_feedback_samples": self.action_feedback_samples,
            "action_feedback_final_distance_m": (
                self.action_feedback_final_distance_m
            ),
            "motion_raw_odom_samples": (
                self.raw_odom_samples - self.start_raw_sample_count
            ),
            "motion_amcl_samples": (
                self.amcl_samples - self.start_amcl_sample_count
            ),
            "monitor_output_samples": self.monitor_output_samples,
            "stamped_output_samples": self.stamped_output_samples,
            "latest_stamped_command_ns": self.latest_stamped_command_ns,
            "maximum_stamped_command_age_s": self.maximum_stamped_command_age_s,
            "monitor_linear_min_m_s": self.monitor_linear_min,
            "monitor_linear_max_m_s": self.monitor_linear_max,
            "guarded_linear_min_m_s": self.guarded_linear_min,
            "guarded_linear_max_m_s": self.guarded_linear_max,
            "max_guarded_linear_y_m_s": self.max_guarded_linear_y,
            "max_guarded_angular_z_rad_s": self.max_guarded_angular_z,
            "raw_forward_m": raw_forward,
            "raw_lateral_m": raw_lateral,
            "raw_yaw_delta_rad": raw_yaw,
            "amcl_displacement_m": amcl_displacement,
            "goal_cost": self.goal_cost,
            "start_cost": self.start_cost,
            "front_clearance_m": self.front_clearance_m,
            "pre_arm_front_clearance_m": self.pre_arm_front_clearance_m,
            "pre_arm_scan_age_s": self.pre_arm_scan_age_s,
            "initial_path_heading_delta_rad": self.initial_path_heading_delta_rad,
            "start_clearance_mode": self.start_clearance_mode,
            # Completion must carry the same sweep evidence used at departure.
            "rotation_clearance": self.rotation_clearance,
            "post_stop_stable_samples": self.post_stop_stable_samples,
            "post_stop_settle_s": self.post_stop_settle_s,
            "final_feedback_x_m_s": self.latest_feedback_x,
            "final_feedback_y_m_s": self.latest_feedback_y,
            "final_feedback_angular_rad_s": self.latest_feedback_angular,
            "final_armed": self.armed,
            "motion_map_checks": self.motion_map_checks,
            "motion_map_guard_tripped": self.motion_map_guard_tripped,
            "motion_map_last_evidence": self.motion_map_last_evidence,
            "navigation_lifecycle_states": getattr(self, 'navigation_lifecycle_states', None),
            "deadman_keepalive_period_s": DEADMAN_KEEPALIVE_PERIOD_S,
            "deadman_keepalive_late_gap_s": DEADMAN_KEEPALIVE_LATE_GAP_S,
            "deadman_keepalive_publish_count": (
                self.deadman_keepalive_publish_count
            ),
            "deadman_keepalive_max_gap_s": self.deadman_keepalive_max_gap_s,
            "deadman_keepalive_late_gap_count": (
                self.deadman_keepalive_late_gap_count
            ),
        }

    def emergency_cleanup(self) -> None:
        self._stop_deadman_keepalive()
        if ((getattr(self, 'motion_map_last_evidence', None) or {}).get('status') == 'STALE_AMCL'
                and self.state not in {'WATCHDOG_LATCHED', 'ESTOP_LATCHED'}):
            try:
                self._call_arm(False)
                self.publish_deadman(False)
                self.publish_zero_command()
                if self.goal_handle is None:
                    raise RuntimeError('freshness stop has no exact navigation goal')
                pending = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, pending, timeout_sec=2.)
                response = pending.result() if pending.done() else None
                if (response is None or not (response.return_code == 3
                        or (response.return_code == 0 and any(
                            bytes(item.goal_id.uuid) == bytes(self.goal_handle.goal_id.uuid)
                            for item in response.goals_canceling)))):
                    raise RuntimeError('freshness-stop cancellation not confirmed')
                terminal = self.goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, terminal, timeout_sec=2.)
                if not terminal.done() or terminal.result().status not in {4, 5, 6}:
                    raise RuntimeError('freshness-stop goal still active')
                self.wait_for(lambda: self.armed is False and self.state == 'DISARMED',
                              2., 'disarmed localization freshness stop')
                self._wait_for_stable_feedback_stop()
                if not (self.armed is False and self.state == 'DISARMED'
                        and self.transport_attached is True and self.feedback_connected is True
                        and self.stm32_connected is True):
                    raise RuntimeError('freshness-stop hardware proof changed')
                self._record('localization_freshness_stop_confirmed', dict(
                    goal_terminal=True, stationary=True, hardware_connected=True))
                self.final_raw_pose = (None if self.latest_raw_pose is None else dict(self.latest_raw_pose))
                self.final_amcl_pose = (None if self.latest_amcl_pose is None else dict(self.latest_amcl_pose))
                return
            except Exception as exc:
                self._record('localization_freshness_stop_unconfirmed', str(exc))
        timeout_reason = self.transition_reason
        if (self.armed is False and self.state == 'WATCHDOG_LATCHED'
                and timeout_reason in {'host_scheduling_timeout', 'command_timeout',
                                       'deadman_timeout'}):
            try:
                self.publish_deadman(False)
                self.publish_zero_command()
                if self.goal_handle is not None:
                    future = self.goal_handle.cancel_goal_async()
                    deadline = time.monotonic() + 2.
                    while not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=.01)
                    response = future.result() if future.done() else None
                    if (response is None or not (
                            response.return_code == 3  # already terminal; prove below
                            or (response.return_code == 0 and any(
                                bytes(item.goal_id.uuid) == bytes(self.goal_handle.goal_id.uuid)
                                for item in response.goals_canceling)))):
                        raise RuntimeError('NavigateToPose watchdog-stop cancellation not confirmed')
                    terminal = self.goal_handle.get_result_async()
                    deadline = time.monotonic() + 2.
                    while not terminal.done() and time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=.01)
                    if not terminal.done() or terminal.result().status not in {4, 5, 6}:
                        raise RuntimeError('NavigateToPose watchdog-stop goal still active')
                self._wait_for_stable_feedback_stop()
                if not (self.armed is False and self.state == 'WATCHDOG_LATCHED'
                        and self.transition_reason == timeout_reason
                        and self.transport_attached is True and self.feedback_connected is True
                        and self.stm32_connected is True):
                    raise RuntimeError('watchdog-stop proof changed')
                marker = ('host_scheduling_stop_confirmed'
                          if timeout_reason == 'host_scheduling_timeout'
                          else 'watchdog_stop_confirmed')
                self._record(marker, True if timeout_reason == 'host_scheduling_timeout'
                             else {'transition_reason': timeout_reason,
                                   'goal_cancelled': True, 'stopped': True})
                # Preserve the original timeout for the existing one-retry
                # policy. Publishing emergency_stop here would turn a verified
                # software stop into an unrelated, nonrecoverable ESTOP.
                if self.latest_raw_pose is not None:
                    self.final_raw_pose = dict(self.latest_raw_pose)
                if self.latest_amcl_pose is not None:
                    self.final_amcl_pose = dict(self.latest_amcl_pose)
                return
            except Exception as exc:
                self._record('watchdog_stop_unconfirmed', str(exc))
        try:
            super().emergency_cleanup()
        finally:
            self._cancel_goal()
            # Three spin_once calls can all consume queued scan/odom events.
            # A successful disarm RPC is not a fresh controller-state sample.
            # Drain until actual stopped feedback arrives before classifying
            # the failure or considering the existing verified return retry.
            try:
                self.wait_for(
                    lambda: self.armed is False
                    and self.state in {'DISARMED', 'ESTOP_LATCHED', 'WATCHDOG_LATCHED'}
                    and self.transition_reason not in {'operator_arm', 'active_autonomy'},
                    2.0, 'controller stop acknowledgement after cleanup')
                self._wait_for_stable_feedback_stop()
                self._record('cleanup_stop_confirmed', True)
            except Exception as exc:
                self._record('cleanup_stop_confirmation_failed', str(exc))
            if self.latest_raw_pose is not None:
                self.final_raw_pose = dict(self.latest_raw_pose)
            if self.latest_amcl_pose is not None:
                self.final_amcl_pose = dict(self.latest_amcl_pose)

    def result(self, status: str, error: Optional[str]) -> Dict[str, object]:
        result = super().result(status, error)
        result.update(
            {
                "field_site": FIELD_SITE,
                "action_name": ACTION_NAME,
                "relative_goal_forward_m": TARGET_DISTANCE_M,
                "goal_pose": self.goal_pose,
                "path_check_status": self.path_check_status,
                "path_check_pose_count": self.path_check_pose_count,
                "path_check_elapsed_s": self.path_check_elapsed_s,
                "path_check_error": self.path_check_error,
                "action_elapsed_s": self.action_elapsed_s,
                "scan_samples": self.scan_samples,
                "amcl_samples": self.amcl_samples,
                "costmap_samples": self.costmap_samples,
                "start_raw_pose": self.start_raw_pose,
                "final_raw_pose": self.final_raw_pose,
                "start_amcl_pose": self.start_amcl_pose,
                "final_amcl_pose": self.final_amcl_pose,
                "static_map_localization_gate": self.localization_gate_result,
                "metrics": self.metrics(),
            }
        )
        return result


def main() -> int:
    args = _parse_args()
    if args.describe:
        print(json.dumps(describe_plan(), indent=2, sort_keys=True))
        return 0
    try:
        trace_path = _validate_run_args(args)
    except ValueError as exc:
        print("ARGUMENT_ERROR: %s" % exc, file=sys.stderr)
        return 2

    rclpy.init()
    node = Nav2ForwardGoalClient()
    try:
        result = node.run_navigation()
    except BaseException as exc:
        try:
            node.emergency_cleanup()
        except BaseException as cleanup_exc:
            node._record("cleanup_error", str(cleanup_exc))
        result = node.result("FAIL", str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    trace_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
