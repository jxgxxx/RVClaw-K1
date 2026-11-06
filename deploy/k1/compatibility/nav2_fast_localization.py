#!/usr/bin/env python3
"""Restore a trusted map pose without commanding robot motion.

The client first reuses a continuously verified runtime pose.  Otherwise it
tries a hard-invalidated cache only as a bounded local-search hypothesis, then
falls back to a static global coarse-to-fine match.  Every non-hot path uses
two independent lidar scans, AMCL no-motion updates, an independent final map
check, costmap clearing, and explicit runtime-manager acceptance.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap, SetInitialPose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent
sys.path.insert(
    0,
    str(WORKSPACE_ROOT / "ros2_ws" / "src" / "rvclaw_nav2_bringup"),
)

from rvclaw_nav2_bringup.amcl_localization_observer import (  # noqa: E402
    quaternion_to_yaw,
    summarize_global_pose_samples,
)
from rvclaw_nav2_bringup.fast_start_policy import (  # noqa: E402
    HOT_READY,
    TEMPORARY_SCAN_OBSTRUCTION,
    WARM_VALIDATION_REQUIRED,
    build_scan_signature,
    compare_scan_signatures,
    load_pose_cache,
    map_identity,
    validate_pose_candidate,
)
from rvclaw_nav2_bringup.scan_map_consistency import (  # noqa: E402
    compare_candidate_to_search,
    evaluate_localization_gate,
    evaluate_match_consensus,
    evaluate_static_match_gate,
    evaluate_pose_ray_consistency,
    filter_ray_inconsistent_matches,
    load_static_map,
    scan_endpoints,
    score_pose,
    search_global_matches_coarse_to_fine,
    search_local_matches,
    _select_distinct_matches,
)


DEFAULT_MAP_FILE = "/data/rvclaw-k3-lab/maps/slam-20260827-105711/map.yaml"
DEFAULT_CACHE_FILE = "/data/rvclaw-k3-lab/state/localization-pose.json"
STARTUP_TIMEOUT_S = 20.0
SERVICE_TIMEOUT_S = 15.0
INDEPENDENT_SCAN_COUNT = 2
GLOBAL_SCAN_COUNT = 3
INDEPENDENT_SCAN_SEPARATION_S = 0.30
SCAN_MAXIMUM_AGE_S = 0.75
SCAN_MAXIMUM_POINTS = 240
# AMCL already uses measured returns up to 12 m. The independent admission
# matcher must retain those same distant landmarks: discarding everything
# past 8 m removes the end-wall evidence in corridors and leaves translations
# along parallel walls nearly indistinguishable. Driving/collision ranges
# remain independent of this stationary localization evidence.
SCAN_LOCALIZATION_MAXIMUM_RANGE_M = 12.0
LOCAL_POSITION_RADIUS_M = 0.45
LOCAL_YAW_RADIUS_DEG = 35.0
LOCAL_POSITION_STEP_M = 0.05
LOCAL_YAW_STEP_DEG = 2.5
GLOBAL_COARSE_POSITION_STEP_M = 0.40
GLOBAL_COARSE_YAW_STEP_DEG = 20.0
GLOBAL_REFINE_POSITION_RADIUS_M = 0.40
GLOBAL_REFINE_YAW_RADIUS_DEG = 20.0
GLOBAL_REFINE_POSITION_STEP_M = 0.05
GLOBAL_REFINE_YAW_STEP_DEG = 2.5
# A global hypothesis starts with a 15 degree yaw prior. Give AMCL enough
# stationary scan updates to reach the unchanged 10 degree acceptance gate.
NOMOTION_UPDATE_COUNT = 12
NOMOTION_UPDATE_INTERVAL_S = 0.40
FINAL_SETTLING_UPDATE_COUNT = 5
POST_ACCEPT_TIMEOUT_S = 5.0
TEMPORARY_OBSTRUCTION_WAIT_S = 8.0


def describe_plan() -> Dict[str, object]:
    return {
        "hardware_io_on_describe": False,
        "motion_authorized": False,
        "publishes_velocity": False,
        "uses_motion_action": False,
        "independent_scan_count": INDEPENDENT_SCAN_COUNT,
        "global_scan_count": GLOBAL_SCAN_COUNT,
        "localization_maximum_range_m": SCAN_LOCALIZATION_MAXIMUM_RANGE_M,
        "localization_maximum_points": SCAN_MAXIMUM_POINTS,
        "hot_path": "reuse_continuously_verified_runtime_pose",
        "temporary_obstruction_path": "wait_for_scan_to_clear_without_global_relocalization",
        "warm_path": "scan_signature_then_bounded_local_map_match",
        "cold_path": "stationary_global_coarse_to_fine_map_match",
        "cold_path_motion_fallback": False,
        "auxiliary_stationary_scan_topic": "/scan_full",
        "auxiliary_scan_changes_navigation_fov": False,
        "auxiliary_pose_requires_unique_consensus_and_fresh_final_check": True,
        "motion_required_result": "MOTION_REQUIRED",
        "nomotion_update_count": NOMOTION_UPDATE_COUNT,
        "final_stationary_settling_updates": FINAL_SETTLING_UPDATE_COUNT,
        "clears_costmaps_after_verification": True,
        "accepts_pose_through_runtime_manager": True,
        "hard_invalidated_cache_directly_trusted": False,
        "hard_invalidated_cache_candidate_allowed": True,
        "optional_single_run_start_region": "explicit x y radius; yaw searched over full circle; no persistent site-specific code",
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--describe", action="store_true")
    parser.add_argument(
        "--scan-audit-only", action="store_true",
        help="Read three fresh scans and test global evidence without setting pose, accepting localization, clearing costmaps or moving.",
    )
    parser.add_argument("--trace-file")
    parser.add_argument(
        "--map-file",
        default=os.environ.get("RVCLAW_MAP_FILE", DEFAULT_MAP_FILE),
    )
    parser.add_argument(
        "--cache-file",
        default=os.environ.get("RVCLAW_LOCALIZATION_CACHE", DEFAULT_CACHE_FILE),
    )
    parser.add_argument("--startup-timeout-s", type=float, default=STARTUP_TIMEOUT_S)
    parser.add_argument("--verify-runtime-map", action="store_true",
                        help="Verify every live /map cell before localization in this ROS context.")
    parser.add_argument(
        "--start-region", type=float, nargs=3, metavar=("X", "Y", "RADIUS_M"),
        help="One-run rough physical location, never a pose or yaw; fresh scans, AMCL and final gate remain required.",
    )
    args = parser.parse_args()
    if args.start_region is not None:
        x, y, radius = args.start_region
        if not all(math.isfinite(value) for value in (x, y, radius)) or not 0.25 <= radius <= 1.5:
            parser.error("start region needs finite x/y and radius in [0.25, 1.5] m")
    return args


def _stamp_ns(message: LaserScan) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class FastLocalizationClient(Node):
    def __init__(self, map_file: Path, cache_file: Path, start_region=None) -> None:
        super().__init__("rvclaw_fast_localization")
        self.map_file = map_file.resolve()
        self.cache_file = cache_file.resolve()
        self.map_info = map_identity(self.map_file)
        self.grid = load_static_map(self.map_file)
        self.start_region = None if start_region is None else tuple(start_region)
        self.started_monotonic = time.monotonic()
        self.events: List[Dict[str, object]] = []

        self.transport_attached: Optional[bool] = None
        self.stm32_connected: Optional[bool] = None
        self.armed: Optional[bool] = None
        self.localization_valid: Optional[bool] = None
        self.localization_status: Dict[str, object] = {}
        self.scans: List[Tuple[float, int, LaserScan]] = []
        self.auxiliary_scans: List[Tuple[float, int, LaserScan]] = []
        self.auxiliary_candidate_used = False
        self.auxiliary_candidate_signatures = []
        self.force_static_verification = False
        self.prefer_full_global_evidence = False
        self.final_verified_pose = None
        self.amcl_samples: List[Dict[str, object]] = []

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Bool,
            "/rvclaw/control/hardware_transport_attached",
            lambda message: setattr(self, "transport_attached", bool(message.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            "/rvclaw/stm32/connected",
            lambda message: setattr(self, "stm32_connected", bool(message.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            "/rvclaw/control/armed",
            lambda message: setattr(self, "armed", bool(message.data)),
            state_qos,
        )
        self.create_subscription(
            Bool,
            "/rvclaw/localization/valid",
            lambda message: setattr(self, "localization_valid", bool(message.data)),
            state_qos,
        )
        self.create_subscription(
            String,
            "/rvclaw/localization/status",
            self._on_status,
            state_qos,
        )
        self.create_subscription(LaserScan, "/scan_localization", self._on_scan, scan_qos)
        # The driving/AMCL view may intentionally be cropped. Use the existing
        # body-filtered full scan only as extra stationary map evidence; never
        # publish it into the navigation/collision pipeline.
        self.create_subscription(LaserScan, "/scan_full", self._on_auxiliary_scan, scan_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            pose_qos,
        )

        self.initial_pose_client = self.create_client(
            SetInitialPose,
            "/set_initial_pose",
        )
        self.nomotion_client = self.create_client(
            Empty,
            "/request_nomotion_update",
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.accept_client = self.create_client(
            Trigger,
            "/rvclaw/localization/accept_verified_pose",
        )
        self.invalidate_client = self.create_client(
            Trigger, "/rvclaw/localization/mark_repositioned",
        )

    def _record(self, event: str, details) -> None:
        self.events.append(
            {
                "event": event,
                "elapsed_s": time.monotonic() - self.started_monotonic,
                "details": details,
            }
        )

    def _on_status(self, message: String) -> None:
        try:
            document = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(document, dict):
            self.localization_status = document

    def _on_scan(self, message: LaserScan) -> None:
        item = (time.monotonic(), _stamp_ns(message), message)
        if self.scans and item[1] == self.scans[-1][1]:
            return
        self.scans.append(item)
        del self.scans[:-20]

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        covariance = message.pose.covariance
        try:
            yaw = quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        except ValueError:
            yaw = math.nan
        self.amcl_samples.append(
            {
                "stamp_s": float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) / 1e9,
                "frame_id": str(message.header.frame_id),
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": float(yaw),
                "cov_x": float(covariance[0]),
                "cov_y": float(covariance[7]),
                "cov_yaw": float(covariance[35]),
                "received_monotonic": time.monotonic(),
            }
        )
        del self.amcl_samples[:-40]

    def _on_auxiliary_scan(self, message: LaserScan) -> None:
        count = len(message.ranges)
        values = (message.angle_min, message.angle_max, message.angle_increment)
        if (count < 10 or not all(math.isfinite(v) for v in values)
                or message.angle_increment <= 0.0 or _stamp_ns(message) <= 0
                or not message.header.frame_id
                or abs(message.angle_max - message.angle_min
                       - (count - 1) * message.angle_increment) > 2 * message.angle_increment
                or (count - 1) * message.angle_increment < math.radians(330.0)):
            return
        item = (time.monotonic(), _stamp_ns(message), message)
        if self.auxiliary_scans and item[1] <= self.auxiliary_scans[-1][1]:
            return
        self.auxiliary_scans.append(item)
        del self.auxiliary_scans[:-20]

    def _fresh_auxiliary_scans(self, timeout_s: float = 2.0, after_monotonic=None):
        self._assert_disarmed()
        threshold = (time.monotonic() - SCAN_MAXIMUM_AGE_S
                     if after_monotonic is None else float(after_monotonic))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._assert_disarmed()
            selected = []
            for item in self.auxiliary_scans:
                if item[0] < threshold:
                    continue
                if selected and item[0] - selected[-1][0] < INDEPENDENT_SCAN_SEPARATION_S:
                    continue
                if self.scans and item[2].header.frame_id != self.scans[-1][2].header.frame_id:
                    continue
                selected.append(item)
            if len(selected) >= GLOBAL_SCAN_COUNT:
                return [item[2] for item in selected[-GLOBAL_SCAN_COUNT:]]
        return []

    def _wait_for(self, predicate, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError("timed out waiting for %s" % description)

    def _call(self, client, request, name: str, timeout_s=SERVICE_TIMEOUT_S):
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            raise RuntimeError("%s service is unavailable" % name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(timeout_s))
        if not future.done() or future.exception() is not None:
            raise RuntimeError("%s service call failed" % name)
        result = future.result()
        if result is None:
            raise RuntimeError("%s returned no response" % name)
        return result

    def _hot_ready(self) -> bool:
        return bool(
            self.localization_valid is True
            and self.localization_status.get("state") == HOT_READY
            and not self.localization_status.get("charging", False)
        )

    def _temporary_obstruction(self) -> bool:
        return self.localization_status.get("state") == TEMPORARY_SCAN_OBSTRUCTION

    def _verify_hot_pose_support(self, timeout_s):
        """Check fresh scans once at startup before reusing a stopped pose.

        A manual relocation can retain HOT_READY when repeated walls still
        partly fit. Do not let the later pre-arm gate be the first to notice.
        This check neither searches globally nor reinitializes a supported pose.
        """
        runtime_score = self.localization_status.get('scan_map_score') or {}
        pose = runtime_score.get('pose') or {}
        if not all(isinstance(pose.get(key), (float, int))
                   and math.isfinite(pose[key]) for key in ('x', 'y', 'yaw')):
            self._record('hot_pose_support_rejected', {'reason': 'no finite runtime pose'})
            return False
        scans = self._independent_scans(after_monotonic=time.monotonic(),
                                        count=INDEPENDENT_SCAN_COUNT,
                                        timeout_s=min(3.0, float(timeout_s)))
        checks = []
        for scan in scans:
            self._assert_disarmed()
            candidate = score_pose(self.grid, self._endpoints(scan),
                                   pose['x'], pose['y'], pose['yaw'])
            rays = evaluate_pose_ray_consistency(self.grid, self._ray_endpoints(scan), pose)
            passed = (candidate.get('endpoint_count', 0) >= 60
                      and candidate.get('known_endpoint_ratio', 0.0) >= .95
                      and candidate.get('mean_likelihood', 0.0) >= .85
                      and rays['status'] == 'PASS')
            checks.append(dict(status='PASS' if passed else 'FAIL',
                               candidate=candidate, ray_consistency=rays))
        supported = self._hot_ready() and all(check['status']=='PASS' for check in checks)
        self._record('hot_pose_fresh_scan_support', dict(status='PASS' if supported else 'FAIL', scans=checks))
        return supported

    def _wait_for_temporary_obstruction_clear(self, timeout_s: float) -> str:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._hot_ready():
                return HOT_READY
            state = str(self.localization_status.get("state", ""))
            if state and state not in {
                TEMPORARY_SCAN_OBSTRUCTION,
                WARM_VALIDATION_REQUIRED,
            }:
                return state
        return str(self.localization_status.get("state", ""))

    def _wait_runtime(self, timeout_s: float) -> None:
        self._wait_for(
            lambda: (
                self.transport_attached is True
                and self.stm32_connected is True
                and self.armed is False
                and bool(self.localization_status)
                and len(self.scans) >= INDEPENDENT_SCAN_COUNT
            ),
            timeout_s,
            "disarmed hardware, lidar, and localization runtime",
        )
        if self.localization_status.get("charging", False):
            raise RuntimeError("localization runtime is explicitly in charging mode")

    def _assert_disarmed(self) -> None:
        if self.armed is not False:
            raise RuntimeError("hardware controller must remain explicitly disarmed")

    def _independent_scans(
        self,
        *,
        after_monotonic: Optional[float] = None,
        count: int = INDEPENDENT_SCAN_COUNT,
        timeout_s: float = STARTUP_TIMEOUT_S,
    ) -> List[LaserScan]:
        deadline = time.monotonic() + float(timeout_s)
        threshold = (
            time.monotonic() - SCAN_MAXIMUM_AGE_S
            if after_monotonic is None
            else float(after_monotonic)
        )
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            selected: List[Tuple[float, int, LaserScan]] = []
            for item in self.scans:
                if item[0] < threshold:
                    continue
                if selected and item[0] - selected[-1][0] < INDEPENDENT_SCAN_SEPARATION_S:
                    continue
                selected.append(item)
            if len(selected) >= int(count):
                result = [item[2] for item in selected[-int(count) :]]
                self._record(
                    "independent_scans_collected",
                    {"count": len(result), "stamps_ns": [_stamp_ns(scan) for scan in result]},
                )
                return result
        raise RuntimeError("fresh independent lidar scans are unavailable")

    @staticmethod
    def _signature(scan: LaserScan) -> Dict[str, object]:
        return build_scan_signature(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )

    @staticmethod
    def _endpoints(scan: LaserScan):
        return scan_endpoints(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            maximum_range_m=SCAN_LOCALIZATION_MAXIMUM_RANGE_M,
            maximum_points=SCAN_MAXIMUM_POINTS,
        )

    def _local_candidate(self, scans: List[LaserScan]):
        try:
            document = load_pose_cache(self.cache_file)
            validation = validate_pose_candidate(document, self.map_info)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return None, {"status": "FAIL", "failures": [str(error)]}
        if validation.get("status") != "PASS":
            return None, validation
        comparisons = [
            compare_scan_signatures(
                validation["scan_signature"],
                self._signature(scan),
            )
            for scan in scans
        ]
        if any(result.get("status") != "PASS" for result in comparisons):
            return None, {
                "status": "FAIL",
                "failures": [
                    "cached scan signature does not agree with both fresh scans"
                ],
                "candidate_validation": validation,
                "signature_comparisons": comparisons,
            }
        pose = validation["pose"]
        searches = [
            search_local_matches(
                self.grid,
                self._endpoints(scan),
                pose["x"],
                pose["y"],
                pose["yaw"],
                position_radius_m=LOCAL_POSITION_RADIUS_M,
                yaw_radius_deg=LOCAL_YAW_RADIUS_DEG,
                position_step_m=LOCAL_POSITION_STEP_M,
                yaw_step_deg=LOCAL_YAW_STEP_DEG,
            )
            for scan in scans
        ]
        # A cache is a hypothesis only; both fresh scans must also agree with
        # mapped free space. The continuously verified HOT_READY path stays
        # unchanged and never runs this cold/warm initialization code.
        searches = [
            filter_ray_inconsistent_matches(self.grid, self._ray_endpoints(scan), search)
            for scan, search in zip(scans, searches)
        ]
        consensus = evaluate_match_consensus(searches)
        report = {
            "status": consensus["status"],
            "candidate_validation": validation,
            "signature_comparisons": comparisons,
            "consensus": consensus,
            "searches": searches,
        }
        return consensus.get("pose") if consensus["status"] == "PASS" else None, report

    def _global_candidate(self, scans: List[LaserScan]):
        auxiliary_report = None
        auxiliary = []
        if getattr(self, 'prefer_full_global_evidence', False):
            auxiliary = self._fresh_auxiliary_scans(after_monotonic=time.monotonic())
            if auxiliary:
                candidate, auxiliary_report = self._global_scan_candidate(auxiliary)
                self._record('auxiliary_stationary_global_candidate', auxiliary_report)
                if candidate is not None:
                    self.auxiliary_candidate_used = True
                    self.auxiliary_candidate_signatures = [self._signature(scan) for scan in auxiliary]
                    return candidate, dict(auxiliary_report, scan_topic='/scan_full',
                                           primary_global_search_deferred=True)
        pose, report = self._global_scan_candidate(scans)
        if pose is not None:
            return pose, report
        if auxiliary_report is None:
            auxiliary = self._fresh_auxiliary_scans()
            if not auxiliary:
                return pose, dict(report, auxiliary_scan_unavailable=True)
            candidate, auxiliary_report = self._global_scan_candidate(auxiliary)
            self._record("auxiliary_stationary_global_candidate", auxiliary_report)
        else:
            candidate = None
        if candidate is None:
            candidate, combined_report = self._combined_global_candidate(
                scans, report, auxiliary, auxiliary_report,
            )
            self._record("combined_stationary_global_candidate", combined_report)
            if candidate is None:
                candidate, retried = self._retry_stationary_global_candidate(
                    scans, report, auxiliary, auxiliary_report, combined_report,
                )
                if candidate is None:
                    return None, dict(report, auxiliary_stationary_report=auxiliary_report,
                                      combined_stationary_report=combined_report,
                                      stationary_retry_report=retried)
                combined_report = retried
            report = dict(combined_report, primary_scan_report=report,
                          auxiliary_stationary_report=auxiliary_report)
        else:
            report = dict(auxiliary_report, primary_scan_report=report)
        self.auxiliary_candidate_used = True
        self.auxiliary_candidate_signatures = [self._signature(scan) for scan in auxiliary]
        return candidate, dict(report, scan_topic="/scan_full")

    def _combined_global_candidate(self, scans, report, auxiliary, auxiliary_report):
        """Retain independent evidence across stationary search batches.

        Both views can contain one unique observation and two ambiguous ones.
        Require the original two unique observations across distinct physical
        acquisitions, and agreement of ALL independent candidates. Never count
        two topic views of one scan twice, or mix laser coordinate frames.
        Full-map competitors, ray vetoes and final fresh checks stay intact.
        """
        failures = []
        observations = []
        for source, messages, evidence in (
            ("primary", scans, report), ("full", auxiliary, auxiliary_report),
        ):
            searches = evidence.get("searches", [])
            inputs = evidence.get("input_scans", [])
            if len(messages) != len(searches) or len(messages) != len(inputs):
                failures.append("stationary evidence does not match its scan acquisitions")
                continue
            for scan, search, saved in zip(messages, searches, inputs):
                stamp = _stamp_ns(scan)
                if (stamp <= 0 or stamp != saved.get("stamp_ns")
                        or not scan.header.frame_id
                        or search.get("ray_consistency_required") is not True
                        or any(item.get("ray_consistency", {}).get("status") != "PASS"
                               for item in search.get("matches", []))):
                    failures.append("stationary evidence lacks valid acquisition or ray checks")
                observations.append((stamp, source, scan, search, saved))
        if len({item[2].header.frame_id for item in observations}) != 1:
            failures.append("stationary scan views must use the same laser frame")
        independent = []
        separation_ns = int(INDEPENDENT_SCAN_SEPARATION_S * 1e9)
        for item in sorted(observations, key=lambda item: item[0]):
            if independent and item[0] - independent[-1][0] < separation_ns:
                continue
            independent.append(item)
        consensus = evaluate_match_consensus(
            [item[3] for item in independent], require_global_uniqueness=True,
            minimum_unique_scan_count=2,
        )
        if failures:
            consensus = dict(consensus, status="FAIL",
                             failures=consensus["failures"] + failures)
        combined = {
            "status": consensus["status"], "consensus": consensus,
            "method": "independent_acquisitions_across_stationary_views",
            "independent_stamps_ns": [item[0] for item in independent],
            "scan_sources": [item[1] for item in independent],
            "searches": [item[3] for item in independent],
            "input_scans": [item[4] for item in independent],
        }
        return consensus.get("pose") if consensus["status"] == "PASS" else None, combined

    def _retry_stationary_global_candidate(self, scans, report, auxiliary,
                                           auxiliary_report, combined):
        """Keep same-invocation evidence through bounded stationary retries.

        Do not restart proof collection at a process boundary. Only retry an
        otherwise consistent batch with at least one globally unique scan.
        Every new physical acquisition must agree; two unique acquisitions,
        full-map competitors and the final fresh pose/ray gates still apply.
        """
        agreement = evaluate_match_consensus(combined.get('searches', []))
        count = combined.get('consensus', {}).get('unique_pass_count', 0)
        if (agreement['status'] != 'PASS' or count != 1
                or len(combined.get('input_scans', [])) < GLOBAL_SCAN_COUNT):
            return None, dict(status='FAIL', reason='stationary evidence is not eligible for retry')
        by_stamp = {_stamp_ns(scan): scan for scan in scans + auxiliary}
        messages = [by_stamp[stamp] for stamp in combined['independent_stamps_ns']]
        evidence = combined
        for attempt in range(3):
            self._assert_disarmed()
            now_ns = time.time_ns()
            stamps = evidence['independent_stamps_ns']
            if any(stamp > now_ns or now_ns - stamp > 60_000_000_000 for stamp in stamps):
                return None, dict(status='FAIL', reason='stationary evidence expired')
            fresh = self._independent_scans(
                after_monotonic=time.monotonic(), count=1, timeout_s=2.0,
            )
            # The rotation fallback already uses this 120-point search.
            # Keep it stationary here and retain the previous unique proof.
            _, new_report = self._global_scan_candidate(
                fresh, maximum_points=120, maximum_range_m=8.0,
            )
            self._record('stationary_global_retry_scan', new_report)
            now_ns = time.time_ns()
            if any(stamp > now_ns or now_ns - stamp > 60_000_000_000 for stamp in stamps):
                return None, dict(status='FAIL', reason='stationary evidence expired during retry')
            pose, evidence = self._combined_global_candidate(
                messages, evidence, fresh, new_report,
            )
            evidence['retry_attempt_count'] = attempt + 1
            self._record('stationary_global_retry_consensus', evidence)
            if pose is not None:
                return pose, evidence
            agreement = evaluate_match_consensus(evidence['searches'])
            if agreement['status'] != 'PASS':
                return None, evidence
            by_stamp.update({_stamp_ns(scan): scan for scan in fresh})
            messages = [by_stamp[stamp] for stamp in evidence['independent_stamps_ns']]
        return None, evidence

    def _global_scan_candidate(self, scans: List[LaserScan], maximum_points=None,
                               maximum_range_m=SCAN_LOCALIZATION_MAXIMUM_RANGE_M):
        def search(scan):
            endpoints = (self._endpoints(scan) if maximum_points is None else scan_endpoints(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max,
                maximum_range_m=maximum_range_m,
                maximum_points=maximum_points,
            ))
            rays = (self._ray_endpoints(scan) if maximum_points is None else scan_endpoints(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max,
                maximum_range_m=maximum_range_m, maximum_points=480,
            ))
            return search_global_matches_coarse_to_fine(
                self.grid,
                endpoints,
                coarse_position_step_m=GLOBAL_COARSE_POSITION_STEP_M,
                coarse_yaw_step_deg=GLOBAL_COARSE_YAW_STEP_DEG,
                refine_position_radius_m=GLOBAL_REFINE_POSITION_RADIUS_M,
                refine_yaw_radius_deg=GLOBAL_REFINE_YAW_RADIUS_DEG,
                refine_position_step_m=GLOBAL_REFINE_POSITION_STEP_M,
                refine_yaw_step_deg=GLOBAL_REFINE_YAW_STEP_DEG,
                maximum_seeds=24,
                maximum_matches=24,
                ray_endpoints=rays,
            )
        # Each measured scan still searches the entire map independently.
        # The native kernel owns per-call scratch memory and releases the GIL.
        # Bound computation to three workers and preserve scan/result order.
        with ThreadPoolExecutor(max_workers=min(3, max(1, len(scans)))) as pool:
            searches = list(pool.map(search, scans))
        # Coarse-to-fine already checked these exact scan rays before spatial
        # suppression. Preserve that full report instead of checking survivors
        # again and replacing the report with only the second-pass survivors.
        consensus = evaluate_match_consensus(
            searches,
            require_global_uniqueness=True,
            minimum_unique_scan_count=2,
        )
        report = {
            "status": consensus["status"],
            "consensus": consensus,
            "searches": searches,
            # Retain exact inputs in the startup trace so a failed cold start
            # can be reproduced offline even after the robot is carried away.
            "input_scans": [
                {
                    "stamp_ns": (scan.header.stamp.sec * 1_000_000_000
                                 + scan.header.stamp.nanosec),
                    "angle_min": scan.angle_min,
                    "angle_increment": scan.angle_increment,
                    "range_min": scan.range_min,
                    "range_max": scan.range_max,
                    "ranges": [float(value) if math.isfinite(value) else None
                               for value in scan.ranges],
                }
                for scan in scans
            ],
        }
        return consensus.get("pose") if consensus["status"] == "PASS" else None, report

    def _bounded_candidate(self, scans: List[LaserScan]):
        """Explicit one-run area evidence, with unknown yaw and unchanged gates.

        This is an operator-supplied *region*, not an injected pose. Never
        borrow cached yaw, and reject inconsistent rays or ambiguous matches.
        """
        if self.start_region is None:
            raise RuntimeError("bounded search requires an explicit start region")
        x, y, radius = self.start_region
        searches = []
        for scan in scans:
            endpoints = self._endpoints(scan)
            coarse = search_local_matches(
                self.grid, endpoints, x, y, 0.0,
                position_radius_m=radius, yaw_radius_deg=180.0,
                position_step_m=0.10, yaw_step_deg=10.0,
                maximum_matches=16,
            )
            refined = []
            for seed in coarse["matches"][:8]:
                result = search_local_matches(
                    self.grid, endpoints, seed["x"], seed["y"], seed["yaw"],
                    position_radius_m=0.15, yaw_radius_deg=10.0,
                    position_step_m=0.05, yaw_step_deg=2.5,
                    maximum_matches=16,
                )
                refined.extend(
                    item for item in result["matches"]
                    if math.hypot(item["x"] - x, item["y"] - y) <= radius
                )
            search = dict(
                search_mode="bounded_region_coarse_to_fine",
                matches=_select_distinct_matches(
                    refined, minimum_position_separation_m=0.25,
                    minimum_yaw_separation_deg=15.0, maximum_matches=24,
                ),
            )
            searches.append(filter_ray_inconsistent_matches(
                self.grid, self._ray_endpoints(scan), search,
            ))
        # The unrestricted matcher demands a runner-up everywhere. In a
        # confirmed bounded area, a single high-quality surviving hypothesis
        # can be stronger evidence than inventing a second match. Two or more
        # survivors keep the original strict global uniqueness check.
        consensus = evaluate_match_consensus(searches)
        gates = []
        for search in searches:
            matches = search.get("matches", [])
            if len(matches) == 1:
                best = matches[0]
                sufficient = (
                    best["mean_likelihood"] >= 0.90
                    and best["known_endpoint_ratio"] >= 0.85
                    and best["ray_consistency"]["status"] == "PASS"
                )
                gate = {
                    "status": "PASS" if sufficient else "FAIL",
                    "reason": "single strong ray-consistent hypothesis in operator-confirmed bounded region",
                    "best_score": best["mean_likelihood"],
                }
            else:
                gate = evaluate_static_match_gate(search)
            gates.append(gate)
        count = sum(gate["status"] == "PASS" for gate in gates)
        if count < 2:
            consensus["status"] = "FAIL"
            consensus["failures"].append(
                "too few uniquely supported bounded scans (%d < 2)" % count
            )
        consensus["bounded_unique_gates"] = gates
        consensus["bounded_unique_pass_count"] = count
        report = {
            "status": consensus["status"], "consensus": consensus,
            "searches": searches, "start_region": {
                "x": x, "y": y, "radius_m": radius,
                "yaw_prior": None, "single_run_only": True,
            },
        }
        return consensus.get("pose") if consensus["status"] == "PASS" else None, report

    def _ray_endpoints(self, scan: LaserScan):
        return scan_endpoints(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            maximum_range_m=SCAN_LOCALIZATION_MAXIMUM_RANGE_M,
            maximum_points=480,
        )

    def _set_initial_pose(self, pose: Dict[str, float], mode: str) -> float:
        self._assert_disarmed()
        # Hold the runtime invalid while this new hypothesis is being tested.
        # Otherwise its coarse continuous-score check can publish HOT_READY
        # before the independent final gate, even when that gate rejects it.
        invalidation = self._call(self.invalidate_client, Trigger.Request(),
                                  'hold new localization hypothesis unverified')
        if not invalidation.success:
            raise RuntimeError('runtime did not invalidate the unverified pose')
        self._record('runtime_hypothesis_invalidated', invalidation.message)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        # A zero stamp asks TF/AMCL to use the latest available transform.  Using
        # ``now()`` here can be a few milliseconds newer than the newest odom TF
        # and intermittently produces a future-extrapolation warning.
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        yaw = float(pose["yaw"])
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        position_std = 0.10 if mode == "local" else 0.20
        yaw_std = math.radians(8.0 if mode == "local" else 15.0)
        message.pose.covariance[0] = position_std ** 2
        message.pose.covariance[7] = position_std ** 2
        message.pose.covariance[35] = yaw_std ** 2
        request = SetInitialPose.Request()
        request.pose = message
        self._call(self.initial_pose_client, request, "AMCL set_initial_pose")
        changed_at = time.monotonic()
        self._record(
            "initial_pose_set",
            {
                "mode": mode,
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw_deg": math.degrees(yaw),
            },
        )
        return changed_at

    def _nomotion_convergence(self, after_monotonic: float) -> Dict[str, object]:
        for _ in range(NOMOTION_UPDATE_COUNT):
            self._assert_disarmed()
            requested_stamp_s = self.get_clock().now().nanoseconds / 1e9
            self._call(
                self.nomotion_client,
                Empty.Request(),
                "AMCL request_nomotion_update",
            )
            deadline = time.monotonic() + NOMOTION_UPDATE_INTERVAL_S
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                # A new measured scan has already completed this requested
                # particle update. Advance to the next update rather than
                # idling out 400 ms. Keep all 12 updates and the unchanged
                # final five-sample, two-second convergence window.
                if any(s['stamp_s'] >= requested_stamp_s
                       for s in self.amcl_samples):
                    break
        samples = [
            sample
            for sample in self.amcl_samples
            if float(sample["received_monotonic"]) >= float(after_monotonic)
        ]
        warmup = summarize_global_pose_samples(
            samples,
            minimum_samples=3,
            maximum_position_drift_m=0.10,
            maximum_yaw_drift_deg=10.0,
            maximum_position_variance=0.10,
            maximum_yaw_variance=math.radians(10.0) ** 2,
        )
        self._record("nomotion_initial_settling", warmup)
        # Initial particle redistribution is expected to move the estimate.
        # Test convergence in a subsequent fresh stationary window, retaining
        # the same drift/covariance limits and the initial history for audit.
        settled_after = time.monotonic()
        for _ in range(FINAL_SETTLING_UPDATE_COUNT):
            self._assert_disarmed()
            self._call(self.nomotion_client, Empty.Request(), "AMCL final stationary update")
            deadline = time.monotonic() + NOMOTION_UPDATE_INTERVAL_S
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        final_samples = [s for s in self.amcl_samples
                         if float(s["received_monotonic"]) >= settled_after]
        summary = summarize_global_pose_samples(
            final_samples, minimum_samples=FINAL_SETTLING_UPDATE_COUNT,
            maximum_position_drift_m=0.10, maximum_yaw_drift_deg=10.0,
            maximum_position_variance=0.10,
            maximum_yaw_variance=math.radians(10.0) ** 2,
        )
        self._record("nomotion_convergence", summary)
        return summary

    def _scan_pose_gate(
        self,
        pose: Dict[str, float],
        scan: LaserScan,
        *,
        include_all_matches: bool = False,
    ) -> Dict[str, object]:
        endpoints = self._endpoints(scan)
        candidate = score_pose(
            self.grid,
            endpoints,
            pose["x"],
            pose["y"],
            pose["yaw"],
        )
        local_search = search_local_matches(
            self.grid,
            endpoints,
            pose["x"],
            pose["y"],
            pose["yaw"],
            position_radius_m=0.30,
            yaw_radius_deg=20.0,
            position_step_m=LOCAL_POSITION_STEP_M,
            yaw_step_deg=LOCAL_YAW_STEP_DEG,
        )
        local_search = filter_ray_inconsistent_matches(
            self.grid, self._ray_endpoints(scan), local_search,
        )
        comparison = compare_candidate_to_search(candidate, local_search)
        gate = evaluate_localization_gate(
            candidate,
            comparison,
            minimum_known_endpoint_ratio=0.85,
            minimum_best_score=0.80,
            minimum_candidate_to_best_ratio=0.80,
            maximum_candidate_to_best_distance_m=0.25,
            maximum_candidate_to_best_yaw_deg=15.0,
        )
        ray_gate = evaluate_pose_ray_consistency(
            self.grid, self._ray_endpoints(scan), pose,
        )
        if ray_gate["status"] != "PASS":
            gate = dict(gate, status="FAIL", failures=gate["failures"] + ray_gate["failures"])
        result = {
            "status": gate["status"],
            "failures": gate["failures"],
            "candidate": candidate,
            "comparison": comparison,
            # Terminal free-ray ranking must retain every competing pose.
            # Other callers keep the compact diagnostic payload.
            "best_matches": local_search.get("matches", [])[:None if include_all_matches else 3],
            "ray_consistency": ray_gate,
            "uniqueness": evaluate_static_match_gate(local_search),
        }
        return result

    def _independent_final_gate(self, pose, scan):
        primary = self._scan_pose_gate(pose, scan)
        result = primary
        if self.auxiliary_candidate_used:
            fresh = self._fresh_auxiliary_scans(after_monotonic=time.monotonic())
            # Independent initial stationary scans established a globally
            # unique pose (one view or combined batches). Repeating searches after
            # AMCL settles can exceed K1's guarded startup deadline. Prove
            # that fresh independent full-view scans still show the same
            # stationary scene, then apply the unchanged pose/ray gates.
            signature_checks = [
                [compare_scan_signatures(reference, self._signature(item))
                 for reference in self.auxiliary_candidate_signatures]
                for item in fresh
            ]
            signature_support = [
                any(check.get('status') == 'PASS' for check in checks)
                for checks in signature_checks
            ]
            full_gates = [self._scan_pose_gate(pose, item) for item in fresh]
            failures = [reason for gate in full_gates for reason in gate['failures']]
            if len(fresh) != GLOBAL_SCAN_COUNT or not self.auxiliary_candidate_signatures:
                failures.append('fresh full-view confirmation scans are unavailable')
            if not all(signature_support):
                failures.append('fresh full-view scans differ from the unique stationary global evidence')
            # A uniquely contradictory primary view remains a veto. An
            # ambiguous one must still pass absolute score and free-ray checks.
            for field, minimum in [('known_endpoint_ratio', .85), ('mean_likelihood', .80)]:
                value = float(primary['candidate'].get(field, 0.))
                if not math.isfinite(value) or value < minimum:
                    failures.append('primary scan candidate %s is insufficient' % field)
            ratio = primary['comparison'].get('candidate_to_best_ratio')
            yaw_error = primary['comparison'].get('candidate_to_best_yaw_deg')
            if ratio is None or not math.isfinite(ratio) or ratio < .80:
                failures.append('primary scan candidate score ratio is insufficient')
            if yaw_error is None or not math.isfinite(yaw_error) or yaw_error > 15.:
                failures.append('primary scan candidate yaw is inconsistent')
            failures += primary['ray_consistency']['failures']
            if primary['uniqueness']['status'] == 'PASS':
                failures += primary['failures']
            result = dict(primary, primary_final_evidence=primary,
                          auxiliary_final_evidence={
                              'status': 'PASS' if full_gates and not failures else 'FAIL',
                              'method': 'fresh_scan_continuity_after_unique_global_consensus',
                              'signature_support': signature_support,
                              'signature_checks': signature_checks,
                          },
                          auxiliary_pose_gates=full_gates,
                          evidence_scan_topic='/scan_full', failures=failures,
                          status='PASS' if full_gates and not failures else 'FAIL')
        self._record("independent_final_gate", result)
        if result["status"] == "PASS":
            self.final_verified_pose = dict(pose)
        return result

    def _refresh_verified_pose(self) -> None:
        if self.final_verified_pose is None:
            raise RuntimeError("no independently verified pose to accept")
        self._assert_disarmed()
        before = time.monotonic()
        self._call(self.nomotion_client, Empty.Request(), "AMCL refresh before runtime acceptance")
        self._wait_for(lambda: bool(self.amcl_samples)
                       and self.amcl_samples[-1]["received_monotonic"] >= before,
                       5.0, "fresh AMCL pose after independent evidence check")
        pose = self.amcl_samples[-1]
        old = self.final_verified_pose
        fields = ("x", "y", "yaw", "cov_x", "cov_y", "cov_yaw")
        if (pose["frame_id"] != "map" or not all(math.isfinite(pose[k]) for k in fields)
                or time.monotonic() - pose["received_monotonic"] > SCAN_MAXIMUM_AGE_S
                or math.hypot(pose["x"] - old["x"], pose["y"] - old["y"]) > 0.05
                or abs(math.atan2(math.sin(pose["yaw"] - old["yaw"]),
                                 math.cos(pose["yaw"] - old["yaw"]))) > math.radians(3.0)
                or max(pose["cov_x"], pose["cov_y"]) > 0.10
                or pose["cov_yaw"] > math.radians(10.0) ** 2):
            raise RuntimeError("fresh AMCL pose changed or became invalid after verification")
        self._record("fresh_verified_pose_before_acceptance", dict(pose))

    def _clear_costmaps_and_accept(self) -> None:
        self._assert_disarmed()
        self._call(
            self.clear_global_client,
            ClearEntireCostmap.Request(),
            "clear global costmap",
        )
        self._call(
            self.clear_local_client,
            ClearEntireCostmap.Request(),
            "clear local costmap",
        )
        self._refresh_verified_pose()
        response = self._call(
            self.accept_client,
            Trigger.Request(),
            "accept verified localization",
        )
        if not response.success:
            raise RuntimeError(
                "localization runtime rejected verified pose: %s" % response.message
            )
        self._wait_for(
            self._hot_ready,
            POST_ACCEPT_TIMEOUT_S,
            "runtime manager HOT_READY acceptance",
        )
        self._record("runtime_pose_accepted", self.localization_status)

    def run(self, startup_timeout_s: float) -> Dict[str, object]:
        self._wait_runtime(startup_timeout_s)
        start_region = getattr(self, "start_region", None)
        # K1 can use the same full-view global evidence first when the driving
        # view is cropped. Preserve the fresh primary-view veto at the final
        # gate, and leave the existing ORIGIN forced recheck ordering alone.
        self.prefer_full_global_evidence = (
            os.environ.get('RVCLAW_STATIC_GLOBAL_FULL_SCAN_FIRST') == '1'
            and not getattr(self, 'force_static_verification', False)
            and start_region is None)
        hot_rejected = False
        if start_region is None and not getattr(self, 'force_static_verification', False) and self._hot_ready():
            if self._verify_hot_pose_support(startup_timeout_s):
                return self.result("PASS", "hot", None)
            hot_rejected = True
        if start_region is None and self._temporary_obstruction():
            self._record(
                "temporary_obstruction_wait_started",
                self.localization_status,
            )
            final_state = self._wait_for_temporary_obstruction_clear(
                min(float(startup_timeout_s), TEMPORARY_OBSTRUCTION_WAIT_S)
            )
            if self._hot_ready():
                self._record(
                    "temporary_obstruction_cleared",
                    self.localization_status,
                )
                if self._verify_hot_pose_support(startup_timeout_s):
                    return self.result("PASS", "hot_after_obstruction", None)
                hot_rejected = True
            if final_state == TEMPORARY_SCAN_OBSTRUCTION:
                return self.result(
                    "BLOCKED",
                    "temporary_obstruction",
                    "temporary obstacle still obscures localization; keep the robot still and clear the lidar view",
                )

        scans = self._independent_scans(
            count=GLOBAL_SCAN_COUNT,
            timeout_s=startup_timeout_s,
        )
        if start_region is not None:
            pose, bounded_report = self._bounded_candidate(scans)
            self._record("bounded_region_candidate", bounded_report)
            mode = "bounded_region"
        elif getattr(self, 'force_static_verification', False) or hot_rejected:
            # A stopped return recheck must prove global uniqueness anew,
            # rather than inheriting the hypothesis that just drifted.
            pose = None
            self._record('forced_global_stationary_verification',
                         'rejected stopped runtime pose' if hot_rejected else True)
            mode = 'global'
        else:
            pose, local_report = self._local_candidate(scans[:INDEPENDENT_SCAN_COUNT])
            self._record("local_candidate", local_report)
            mode = "local"
        if pose is None and start_region is None:
            pose, global_report = self._global_candidate(scans)
            self._record("global_candidate", global_report)
            mode = "global"
        if pose is None:
            return self.result(
                "MOTION_REQUIRED", "stationary_match_ambiguous",
                "no unique pose could be proven without moving the robot",
            )

        def verify_stationary_candidate(candidate, candidate_mode):
            changed_at = self._set_initial_pose(candidate, candidate_mode)
            convergence = self._nomotion_convergence(changed_at)
            if convergence.get("status") != "PASS":
                if candidate_mode == "local":
                    # A stale local hypothesis may fail before the final map
                    # gate. Keep the chassis disarmed and let the same unique
                    # global fallback handle this rejection as well.
                    return changed_at, {
                        "status": "FAIL",
                        "failures": [
                            "AMCL did not converge after no-motion updates: %s"
                            % "; ".join(convergence.get("failures", []))
                        ],
                    }
                raise RuntimeError(
                    "AMCL did not converge after no-motion updates: %s"
                    % "; ".join(convergence.get("failures", []))
                )
            final_scan = self._independent_scans(
                after_monotonic=changed_at,
                count=1,
                timeout_s=startup_timeout_s,
            )[0]
            return changed_at, self._independent_final_gate(
                convergence["final_pose"], final_scan
            )

        changed_at, final_gate = verify_stationary_candidate(pose, mode)
        if final_gate.get("status") != "PASS" and mode == "local":
            # A scan signature can admit an old, hard-invalidated pose as a
            # candidate while a later independent map check rejects it.
            # Stay disarmed and try a fresh, unique global consensus once.
            self._record("local_final_gate_rejected", final_gate)
            fresh_scans = self._independent_scans(
                after_monotonic=changed_at,
                count=GLOBAL_SCAN_COUNT,
                timeout_s=startup_timeout_s,
            )
            pose, global_report = self._global_candidate(fresh_scans)
            self._record("global_candidate_after_local_rejection", global_report)
            if pose is None:
                return self.result(
                    "MOTION_REQUIRED",
                    "stationary_global_match_ambiguous",
                    "local pose rejected and no unique global pose could be proven",
                )
            changed_at, final_gate = verify_stationary_candidate(pose, "global")
            mode = "global_after_local_rejection"
        if final_gate.get("status") != "PASS":
            raise RuntimeError(
                "independent final map gate failed: %s"
                % "; ".join(final_gate.get("failures", []))
            )
        self._clear_costmaps_and_accept()
        return self.result("PASS", mode, None)

    def result(self, status: str, mode: str, error: Optional[str]):
        return {
            "status": status,
            "mode": mode,
            "error": error,
            "motion_authorized": False,
            "map": self.map_info,
            "cache_file": str(self.cache_file),
            "elapsed_s": time.monotonic() - self.started_monotonic,
            "final_runtime_state": self.localization_status,
            "events": self.events,
        }


class PreservingFastLocalizationClient(FastLocalizationClient):
    """Keep an already-converged pose only after independent candidate proof.

    This avoids particle redistribution and its warmup on every task. Moved,
    stale, uncertain or inconsistent poses retain the full reset/convergence
    path. All callers still execute independent final map/ray admission.
    """
    def _set_initial_pose(self, pose, mode):
        self._assert_disarmed()
        requested=time.monotonic()
        self.matching_live_pose_preserved=False
        if not self.amcl_samples:
            return super()._set_initial_pose(pose,mode)
        try:
            self._call(self.nomotion_client,Empty.Request(),'fresh stationary pose before reuse')
            self._wait_for(lambda:any(s['received_monotonic']>=requested for s in self.amcl_samples),
                           3.,'fresh stationary pose before reuse')
        except RuntimeError:
            return super()._set_initial_pose(pose,mode)
        current=self.amcl_samples[-1]
        close=(all(math.isfinite(current.get(k,math.nan)) for k in
                   ('x','y','yaw','cov_x','cov_y','cov_yaw'))
            and 0<=self.get_clock().now().nanoseconds/1e9-current.get('stamp_s',0)<=SCAN_MAXIMUM_AGE_S
            and all(0<=current[k]<=limit for k,limit in
                    (('cov_x',.05),('cov_y',.05),('cov_yaw',.02)))
            and math.hypot(current['x']-pose['x'],current['y']-pose['y'])<=.10
            and abs(math.atan2(math.sin(current['yaw']-pose['yaw']),
                               math.cos(current['yaw']-pose['yaw'])))<=math.radians(3.))
        self.matching_live_pose_preserved=bool(close)
        if close:
            self._record('independently_matched_live_pose_preserved',dict(pose=current,candidate=pose))
            return time.monotonic()
        return super()._set_initial_pose(pose,mode)

    def _nomotion_convergence(self, after_monotonic):
        if not getattr(self,'matching_live_pose_preserved',False):
            return super()._nomotion_convergence(after_monotonic)
        for _ in range(FINAL_SETTLING_UPDATE_COUNT):
            self._assert_disarmed()
            self._call(self.nomotion_client,Empty.Request(),'unchanged stationary pose update')
            end=time.monotonic()+NOMOTION_UPDATE_INTERVAL_S
            while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        samples=[s for s in self.amcl_samples if s['received_monotonic']>=after_monotonic]
        summary=summarize_global_pose_samples(samples,minimum_samples=FINAL_SETTLING_UPDATE_COUNT,
            maximum_position_drift_m=.10,maximum_yaw_drift_deg=10.,
            maximum_position_variance=.10,maximum_yaw_variance=math.radians(10.)**2)
        self._record('nomotion_convergence',summary)
        return summary


def _write_trace(path: Optional[str], result: Dict[str, object]) -> None:
    if not path:
        return
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = _parse_args()
    if args.describe:
        print(json.dumps(describe_plan(), indent=2, sort_keys=True))
        return 0
    rclpy.init()
    node = FastLocalizationClient(Path(args.map_file), Path(args.cache_file), args.start_region)
    try:
        if args.verify_runtime_map:
            from verify_runtime_map import verify_runtime_map
            started = time.monotonic()
            report = verify_runtime_map(node, args.map_file)
            node._record("live_map_verified", dict(report, elapsed_s=time.monotonic()-started))
        if args.scan_audit_only:
            # Diagnostic branch: subscriptions and pure calculations only.
            # Do not reuse run(): it sets pose and accepts runtime validity.
            node._wait_runtime(args.startup_timeout_s)
            scans = node._independent_scans(count=GLOBAL_SCAN_COUNT, timeout_s=args.startup_timeout_s)
            pose, report = (
                node._bounded_candidate(scans)
                if args.start_region is not None
                else node._global_candidate(scans)
            )
            node._record("read_only_global_candidate", report)
            result = node.result(report["status"], "scan_audit_only", None)
            result.update(diagnostic_only=True, pose_initialization_performed=False,
                          runtime_acceptance_performed=False, candidate_pose=pose)
        else:
            result = node.run(args.startup_timeout_s)
    except BaseException as error:
        result = node.result("FAIL", "error", str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    _write_trace(args.trace_file, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    if result["status"] == "PASS":
        return 0
    if result["status"] == "MOTION_REQUIRED":
        return 4
    if result["status"] == "BLOCKED":
        return 5
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
