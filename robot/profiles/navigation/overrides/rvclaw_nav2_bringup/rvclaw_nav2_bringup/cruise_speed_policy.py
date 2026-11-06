"""Bounded cruise on clear, straight transit; retain the reviewed approach speed."""
import math

import numpy as np

CRUISE_SPEED_M_S = .35
APPROACH_SPEED_M_S = .22
FINAL_APPROACH_DISTANCE_M = .45
FINAL_APPROACH_SPEED_M_S = .10
NEAR_OBSTACLE_SPEED_M_S = .22 * .4
NEAR_OBSTACLE_X_M = .55
CRUISE_EXIT_X_M = .95
CRUISE_ENTER_X_M = 1.10
CRUISE_GOAL_EXIT_M = 1.0
CRUISE_GOAL_ENTER_M = 1.2
CRUISE_MAX_ANGULAR_RAD_S = .15
CRUISE_MAX_SCAN_AGE_S = .20
CRUISE_MAX_FEEDBACK_AGE_S = .30
CRUISE_CLEAR_SCANS = 3
LASER_X_M = .07725
CORRIDOR_HALF_WIDTH_M = .23


def front_corridor(ranges, angle_min, angle_increment, range_min, range_max):
    """Use every ray; missing front coverage never authorizes higher speed.

    X is measured from base_footprint, matching front_stop/front_slow polygons.
    Infinite/invalid returns are not proof of free space for cruise permission.
    They do not change the existing adapter's independent scan/STOP gates.
    """
    if (not all(math.isfinite(v) for v in
            (angle_min, angle_increment, range_min, range_max))
            or not angle_increment or not 0. <= range_min < range_max):
        return dict(clearance_m=None, coverage_valid=False)
    values = np.asarray(ranges, dtype=np.float64)
    angles = angle_min+np.arange(len(values), dtype=np.float64)*angle_increment
    cosine, sine = np.cos(angles), np.sin(angles)
    front = cosine >= math.cos(math.radians(35.))
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    safe_values = np.where(valid, values, 0.)
    x, y = LASER_X_M+safe_values*cosine, safe_values*sine
    corridor = valid & (x >= 0.) & (np.abs(y) <= CORRIDOR_HALF_WIDTH_M)
    clearance = float(np.min(x[corridor])) if np.any(corridor) else math.inf
    front_total = int(np.count_nonzero(front))
    valid_front = front & valid
    front_valid = int(np.count_nonzero(valid_front))
    front_angles = np.degrees(np.arctan2(sine[valid_front], cosine[valid_front]))
    front_bins = np.unique(np.clip(((front_angles+35.)/10.).astype(np.intp), 0, 6))
    return dict(clearance_m=clearance,
        coverage_valid=(front_total >= 20 and front_valid/front_total >= .8
                        and len(front_bins) == 7),
        front_rays=front_total, valid_front_rays=front_valid)


def cruise_speed_limit(clearance_m, coverage_valid, scan_age_s, feedback_age_s,
                       distance_remaining_m, angular_z, clear_scans, cruising=False):
    """Select a linear ceiling only. Never alter angular/reverse/STOP output."""
    if (distance_remaining_m is not None and math.isfinite(distance_remaining_m)
            and 0. <= distance_remaining_m <= FINAL_APPROACH_DISTANCE_M
            and feedback_age_s is not None and math.isfinite(feedback_age_s)
            and 0. <= feedback_age_s <= CRUISE_MAX_FEEDBACK_AGE_S):
        # A final low-speed approach reduces coast after XY goal acceptance.
        # Keep the obstacle cap even when it is lower than this arrival cap.
        return min(FINAL_APPROACH_SPEED_M_S, NEAR_OBSTACLE_SPEED_M_S
            if clearance_m is not None and not math.isnan(clearance_m)
            and 0. <= clearance_m <= NEAR_OBSTACLE_X_M else FINAL_APPROACH_SPEED_M_S)
    if (clearance_m is not None and not math.isnan(clearance_m)
            and 0. <= clearance_m <= NEAR_OBSTACLE_X_M):
        return NEAR_OBSTACLE_SPEED_M_S
    if (not coverage_valid or clearance_m is None or math.isnan(clearance_m)
            or any(v is None or not math.isfinite(v) for v in
                   (scan_age_s, feedback_age_s, distance_remaining_m, angular_z))
            or not 0. <= scan_age_s <= CRUISE_MAX_SCAN_AGE_S
            or not 0. <= feedback_age_s <= CRUISE_MAX_FEEDBACK_AGE_S
            or abs(angular_z) > CRUISE_MAX_ANGULAR_RAD_S):
        return APPROACH_SPEED_M_S
    if cruising:
        permitted = clearance_m > CRUISE_EXIT_X_M and distance_remaining_m > CRUISE_GOAL_EXIT_M
    else:
        permitted = (clearance_m >= CRUISE_ENTER_X_M
            and distance_remaining_m >= CRUISE_GOAL_ENTER_M
            and clear_scans >= CRUISE_CLEAR_SCANS)
    return CRUISE_SPEED_M_S if permitted else APPROACH_SPEED_M_S
