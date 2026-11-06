"""Fail-closed review of a sharp departure turn beside an obstacle.

No motion is issued here. A successful Nav2 centerline plan cannot override
a failed rotation sweep. These are nominal checks; full-height protrusions
and pose uncertainty still need a supervised physical review.
"""

import math


TURN_REVIEW_MIN_ANGLE_RAD = math.radians(30.0)
NOMINAL_ROTATION_RADIUS_M = 0.25


def check_front_stop_turn_sweep(
    ranges, angle_min, angle_increment, range_min, range_max, heading_delta,
    *, laser_x=0.07725, front_m=0.35, half_width_m=0.17, margin_m=0.02,
    forward_offset_m=0.0,
):
    """Predict the existing forward STOP rectangle during a pure rotation.

    This supplements the chassis-disk checks; it never certifies unseen space
    or disables the live Collision Monitor. Apply the pose margin as a physical
    Euclidean distance to the rectangle. The minimum over the continuous turn
    occurs at an endpoint, a face normal, or a corner bearing, so no angular
    sampling or extra corner inflation is necessary.
    """
    if not math.isfinite(heading_delta) or abs(heading_delta) > math.pi:
        raise ValueError("invalid departure turn")
    if not all(math.isfinite(v) for v in
               (angle_min, angle_increment, range_min, range_max, laser_x,
                front_m, half_width_m, margin_m, forward_offset_m)) or angle_increment == 0:
        raise ValueError("invalid turn-sweep geometry")
    if front_m < 0.35 or half_width_m < 0.17 or margin_m < 0.02:
        raise ValueError("turn sweep smaller than the active stop envelope")
    lower, upper = sorted((0.0, heading_delta))
    corner = math.atan2(half_width_m, front_m)
    critical_bearings = (0.0, corner, -corner, math.pi/2, -math.pi/2, math.pi)
    maximum_radius = math.hypot(front_m, half_width_m) + margin_m
    margin_squared = margin_m * margin_m
    blocking = []
    actual_blocking = []
    count = 0
    for index, raw in enumerate(ranges):
        distance = float(raw)
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        count += 1
        angle = angle_min + index*angle_increment
        x, y = laser_x + distance*math.cos(angle)-forward_offset_m, distance*math.sin(angle)
        if _point_in_rotating_front_rectangle(x, y, heading_delta, front_m, half_width_m):
            actual_blocking.append({"x_m": x, "y_m": y, "range_m": distance})
        if math.hypot(x, y) > maximum_radius:
            continue
        bearing = math.atan2(y, x)
        turns = [0.0, heading_delta]
        for critical in critical_bearings:
            offset = (bearing-critical) % math.tau
            for turn in (offset, offset-math.tau):
                if lower <= turn <= upper:
                    turns.append(turn)
        for turn in turns:
            cosine, sine = math.cos(turn), math.sin(turn)
            px, py = cosine*x+sine*y, -sine*x+cosine*y
            dx = max(-px, 0.0, px-front_m)
            dy = max(0.0, abs(py)-half_width_m)
            if dx*dx + dy*dy <= margin_squared + 1e-12:
                blocking.append({"x_m": x, "y_m": y, "range_m": distance})
                break
    return {
        "status": "PASS" if count and not blocking else "FAIL",
        "valid_return_count": count,
        "blocking_return_count": len(blocking), "blocking_returns": blocking[:8],
        "actual_stop_status": "PASS" if count and not actual_blocking else "FAIL",
        "actual_stop_blocking_return_count": len(actual_blocking),
        "actual_stop_blocking_returns": actual_blocking[:8],
        "actual_stop_model": "analytic_continuous_rectangle",
        "heading_delta_rad": heading_delta, "margin_m": margin_m,
        "front_m": front_m, "half_width_m": half_width_m,
        "forward_offset_m": forward_offset_m,
        "sweep_model": "continuous_rectangle_distance",
        "unobserved_area_verified": False,
    }


def _point_in_rotating_front_rectangle(x, y, delta, front, half_width):
    """Exact angular interval intersection with the unchanged live STOP box."""
    radius = math.hypot(x, y)
    if radius > math.hypot(front, half_width) + 1e-12:
        return False
    if radius < 1e-12:
        return True
    lower = math.acos(min(1.0, front/radius))
    upper = math.asin(min(1.0, half_width/radius))
    if lower > upper + 1e-12:
        return False
    phi = math.atan2(y, x)
    start, end = sorted((phi, phi-delta))
    for left, right in ((lower, upper), (-upper, -lower)):
        for turn in range(-2, 3):
            shift = turn*2*math.pi
            if max(start,left+shift) <= min(end,right+shift)+1e-12:
                return True
    return False


def _world_to_cell(grid, world_x, world_y):
    dx, dy = world_x - grid.origin_x, world_y - grid.origin_y
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    cx = math.floor((cosine * dx + sine * dy) / grid.resolution)
    cy = math.floor((-sine * dx + cosine * dy) / grid.resolution)
    if 0 <= cx < grid.width and 0 <= cy < grid.height:
        return cx, cy
    return None


def check_nominal_static_rotation_disk(grid, pose, radius_m=NOMINAL_ROTATION_RADIUS_M):
    """Reject occupied, unknown, or outside-map cells in a rotation disk."""
    if not math.isfinite(radius_m) or radius_m < NOMINAL_ROTATION_RADIUS_M:
        raise ValueError("rotation radius smaller than reviewed nominal radius")
    if not math.isfinite(grid.resolution) or grid.resolution <= 0:
        raise ValueError("invalid map resolution")
    x, y = float(pose["x"]), float(pose["y"])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("invalid rotation center")
    center = _world_to_cell(grid, x, y)
    if center is None:
        return {"status": "FAIL", "reason": "rotation center outside map", "radius_m": radius_m}
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    reach = math.ceil(radius_m / grid.resolution) + 2
    unknown = occupied = 0
    for row in range(center[1] - reach, center[1] + reach + 1):
        for column in range(center[0] - reach, center[0] + reach + 1):
            local_x = (column + 0.5) * grid.resolution
            local_y = (row + 0.5) * grid.resolution
            cell_world_x = grid.origin_x + cosine * local_x - sine * local_y
            cell_world_y = grid.origin_y + sine * local_x + cosine * local_y
            # Include one extra cell to account for rasterized obstacle edges.
            if math.hypot(cell_world_x - x, cell_world_y - y) > radius_m + grid.resolution:
                continue
            if not (0 <= column < grid.width and 0 <= row < grid.height):
                unknown += 1
                continue
            value = int(grid.cells[row, column])
            if value < 0:
                unknown += 1
            elif value >= 65:
                occupied += 1
    center_distance = float(grid.occupied_distance_m[center[1], center[0]])
    status = "PASS" if not unknown and not occupied and center_distance >= radius_m else "FAIL"
    return {
        "status": status,
        "radius_m": radius_m,
        "unknown_or_out_of_map_cells": unknown,
        "occupied_cells": occupied,
        "center_to_obstacle_m": center_distance,
        "nominal_only": True,
    }


def classify_side_turn_departure(
    heading_delta, side_detection, scan_rotation, map_rotation, *, continuing_egress=False
):
    """Distinguish positive obstruction evidence from missing turn evidence.

    After a straight step the originally detected obstacle may be behind the
    robot. ``continuing_egress`` keeps the turning-sweep requirement active.
    """
    if not isinstance(side_detection, dict) or side_detection.get("status") != "PASS":
        return {"status": "STOP", "reason": "side clearance invalid or missing"}
    if heading_delta is None or not math.isfinite(heading_delta):
        return {"status": "STOP", "reason": "departure heading invalid or missing"}
    if abs(heading_delta) < TURN_REVIEW_MIN_ANGLE_RAD:
        return {"status": "NORMAL", "reason": "no sharp departure turn"}
    if not isinstance(scan_rotation, dict) or not isinstance(map_rotation, dict):
        return {"status": "STOP", "reason": "rotation evidence missing"}
    scan_pass = scan_rotation.get("status") == "PASS"
    map_pass = map_rotation.get("status") == "PASS"
    if scan_pass and map_pass and not side_detection.get("detected"):
        return {
            "status": "NORMAL",
            "reason": "nominal rotation checks pass with no near-side obstacle",
        }
    evidence = {
        "scan_status": scan_rotation.get("status"),
        "map_status": map_rotation.get("status"),
        "side_obstacle_detected": side_detection.get("detected") is True,
        "continuing_egress": bool(continuing_egress),
    }
    # Unknown map cells, absent/invalid scan coverage and a generic FAIL are
    # not positive obstacle detections. Moving forward does not repair missing
    # evidence and must not be the default response to it.
    if evidence["side_obstacle_detected"]:
        return dict(evidence, status="REQUIRE_STRAIGHT_EGRESS",
                    reason="near-side obstacle detected during sharp departure turn",
                    obstruction_source="live_side_scan")
    map_occupied = map_rotation.get("occupied_cells")
    if isinstance(map_occupied, (int, float)) and math.isfinite(map_occupied) and map_occupied > 0:
        return dict(evidence, status="REQUIRE_STRAIGHT_EGRESS",
                    reason="known map occupancy intersects the departure rotation disk",
                    obstruction_source="static_map_occupancy")
    nearest = scan_rotation.get("minimum_radial_m")
    required = scan_rotation.get("required_minimum_radial_m")
    coverage = scan_rotation.get("coverage_deg")
    required_coverage = scan_rotation.get("required_minimum_coverage_deg")
    values = (nearest, required, coverage, required_coverage)
    complete = all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
    if (complete and scan_rotation.get("valid_return_count", 0) > 0
            and coverage >= required_coverage and 0 <= nearest < required):
        return dict(evidence, status="REQUIRE_STRAIGHT_EGRESS",
                    reason="covered live scan contains a departure rotation-disk return",
                    obstruction_source="live_rotation_scan")
    return dict(evidence, status="STOP", obstruction_source=None,
                reason="departure turn evidence insufficient; no certified obstacle-based egress reason")
