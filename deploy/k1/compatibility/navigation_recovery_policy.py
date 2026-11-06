"""Pure, fail-closed policy for bounded waypoint-navigation recovery."""

from __future__ import annotations

from typing import Any, Mapping


PROMPTS = {
    "navigation_paused_arrival_review": ("到点定位确认未完成，巡检已暂停。请保持小车不动并检查定位。", "navigation-paused-arrival-review.wav"),
    "navigation_obstacle_recovery_started": ("路线受阻，正在等待障碍物移开并重新规划。", "navigation-obstacle-recovery-started.wav"),
    "navigation_localization_recovery_started": ("定位不稳定，正在自动重新定位。", "navigation-localization-recovery-started.wav"),
    "navigation_lidar_recovery_started": ("雷达数据短暂异常，正在等待雷达恢复。", "navigation-lidar-recovery-started.wav"),
    "navigation_recovery_succeeded": ("导航已恢复，继续巡检。", "navigation-recovery-succeeded.wav"),
    "navigation_paused_obstacle": ("导航仍然受阻，巡检已暂停。请移开小车周围或通往目标点的障碍物，然后唤醒我，说继续巡检。", "navigation-paused-obstacle.wav"),
    "navigation_paused_localization": ("定位失败，巡检已暂停。请把小车放到当前地图内特征明显且周围空旷的位置，然后唤醒我，说重新定位并继续巡检。", "navigation-paused-localization.wav"),
    "navigation_paused_lidar": ("雷达数据未恢复，巡检已暂停。请检查雷达电源和USB连接，恢复后唤醒我，说继续巡检。", "navigation-paused-lidar.wav"),
    "navigation_paused_chassis": ("底盘通信异常，巡检已暂停。请检查小车总电源、串口三和通信线，恢复后唤醒我，说继续巡检。", "navigation-paused-chassis.wav"),
    "navigation_paused_command_timeout": ("导航控制指令超时，巡检已暂停。请保持小车不动，唤醒我，说继续巡检。", "navigation-paused-command-timeout.wav"),
    "navigation_paused_deadman_timeout": ("导航控制心跳超时，巡检已暂停。请保持小车不动并检查导航服务。", "navigation-paused-deadman-timeout.wav"),
    "navigation_paused_heading": ("已到返航点，车头回正未完成。巡检已暂停。", "navigation-paused-heading.wav"),
    "navigation_paused_return_position": ("返航位置未达到要求，巡检已暂停。", "navigation-paused-return-position.wav"),
    "navigation_paused_safety": ("安全保护已触发，巡检已暂停。请清除周围危险并检查急停，确认安全后人工解除，再唤醒我，说继续巡检。", "navigation-paused-safety.wav"),
    "navigation_paused_low_battery": ("小车电量过低，巡检已暂停。请关闭运动电源并更换或充电，恢复后唤醒我，说继续巡检。", "navigation-paused-low-battery.wav"),
    "navigation_paused_system": ("导航系统异常，巡检已暂停。请保持小车不动并检查导航服务，恢复后唤醒我，说继续巡检。", "navigation-paused-system.wav"),
}


def _policy(code, automatic, attempts, delay, started, paused, action, command="继续巡检", relocalize=False):
    return dict(code=code, automatic=automatic, max_attempts=attempts, retry_delay_s=delay,
                started_speech_key=started, paused_speech_key=paused,
                operator_action=action, resume_command=command,
                requires_global_localization=relocalize)


POLICIES = {
    "ARRIVAL_VERIFICATION_INCOMPLETE": _policy("ARRIVAL_VERIFICATION_INCOMPLETE", False, 0, 0.0, None, "navigation_paused_arrival_review", "keep the robot stopped and inspect arrival geometry and timing before manual resume"),
    "CONTROL_HEARTBEAT_TIMEOUT": _policy("CONTROL_HEARTBEAT_TIMEOUT", False, 0, 0.0, None, "navigation_paused_deadman_timeout", "keep the robot stopped and inspect control-chain scheduling before a manual restart"),
    "ROUTE_BLOCKED": _policy("ROUTE_BLOCKED", True, 2, 3.0, "navigation_obstacle_recovery_started", "navigation_paused_obstacle", "clear obstacles without moving the robot"),
    "LOCALIZATION_UNSTABLE": _policy("LOCALIZATION_UNSTABLE", True, 1, 2.0, "navigation_localization_recovery_started", "navigation_paused_localization", "move to a feature-rich open area on the current map", "重新定位并继续巡检", True),
    "LOCALIZATION_PROTECTIVE_STOP": _policy("LOCALIZATION_PROTECTIVE_STOP", False, 0, 0.0, None, "navigation_paused_localization", "keep the robot stopped, verify localization and manually reset the protective latch", "重新定位并继续巡检", True),
    "LOCALIZATION_DATA_DELAY": _policy("LOCALIZATION_DATA_DELAY", False, 0, 0.0, None, "navigation_paused_localization", "stopped independent localization recovery was unavailable or exhausted", "重新定位并继续巡检", True),
    "LIDAR_DATA_TRANSIENT": _policy("LIDAR_DATA_TRANSIENT", True, 1, 2.0, "navigation_lidar_recovery_started", "navigation_paused_lidar", "check lidar power and USB without moving the robot"),
    "CHASSIS_COMMAND_TIMEOUT": _policy("CHASSIS_COMMAND_TIMEOUT", True, 1, 1.0, None, "navigation_paused_command_timeout", "keep the robot still while the controller is safely reset"),
    "CHASSIS_DEADMAN_TIMEOUT": _policy("CHASSIS_DEADMAN_TIMEOUT", True, 1, 1.0, None, "navigation_paused_deadman_timeout", "verify the controller stopped before one bounded retry"),
    "RETURN_HEADING_INCOMPLETE": _policy("RETURN_HEADING_INCOMPLETE", False, 0, 0.0, None, "navigation_paused_heading", "keep the robot stopped and inspect the final heading result"),
    "RETURN_POSITION_INCOMPLETE": _policy("RETURN_POSITION_INCOMPLETE", False, 0, 0.0, None, "navigation_paused_return_position", "keep the robot stopped and inspect the independently verified return position"),
    "LOW_BATTERY": _policy("LOW_BATTERY", False, 0, 0.0, None, "navigation_paused_low_battery", "power off motion and charge or replace the chassis battery"),
    "CHASSIS_COMMUNICATION_FAILED": _policy("CHASSIS_COMMUNICATION_FAILED", False, 0, 0.0, None, "navigation_paused_chassis", "check chassis power, UART3 and the STM32 cable"),
    "SAFETY_STOP_LATCHED": _policy("SAFETY_STOP_LATCHED", False, 0, 0.0, None, "navigation_paused_safety", "clear the hazard and use the manual safety-reset procedure"),
    "NAVIGATION_RESULT_TIMEOUT": _policy("NAVIGATION_RESULT_TIMEOUT", False, 0, 0.0, None, "navigation_paused_system", "keep the robot stopped and inspect navigation timing"),
    "NAVIGATION_SERVICE_UNAVAILABLE": _policy("NAVIGATION_SERVICE_UNAVAILABLE", False, 0, 0.0, None, "navigation_paused_system", "keep the robot still and restore the Nav2 runtime"),
    "UNCLASSIFIED_NAVIGATION_FAILURE": _policy("UNCLASSIFIED_NAVIGATION_FAILURE", False, 0, 0.0, None, "navigation_paused_system", "keep the robot still and inspect the route trace"),
}


def _failure_text(result: Mapping[str, Any]) -> str:
    fields = (
        result.get("error"),
        result.get("error_repr"),
        result.get("traceback"),
        result.get("path_check_error"),
        result.get("transition_reason"),
        result.get("final_transition_reason"),
        result.get("final_state"),
    )
    values = [str(value) for value in fields if value is not None]
    events = result.get("events")
    if isinstance(events, (list, tuple)):
        # A previous segment's stopped latch may be observed at subscription
        # startup and explicitly reset before this segment is armed. Retain all
        # in-motion latches, even if a later reset was recorded in this attempt.
        first_arm = next((i for i, event in enumerate(events)
                          if isinstance(event, Mapping)
                          and event.get("name") == "state"
                          and str(event.get("value", "")).upper()
                          in {"ARMED_IDLE", "ACTIVE"}), None)
        if first_arm is not None:
            prefix = events[:first_arm]
            states = [str(event.get("value", "")).upper() for event in prefix
                      if isinstance(event, Mapping) and event.get("name") == "state"]
            reset_confirmed = any(
                isinstance(event, Mapping)
                and event.get("name") == "transition_reason"
                and event.get("value") == "safety_latches_reset"
                for event in prefix
            )
            if states and states[-1] == "DISARMED" and reset_confirmed:
                events = events[first_arm:]
        for event in events:
            if not isinstance(event, Mapping):
                continue
            if event.get("name") in {"state", "transition_reason"}:
                values.append(str(event.get("value", "")))
    return " ".join(values).lower()


def _reported_voltage(result: Mapping[str, Any]):
    for container in (result, result.get("metrics")):
        if not isinstance(container, Mapping):
            continue
        for key in ("voltage_v", "battery_voltage_v"):
            try:
                return float(container[key])
            except (KeyError, TypeError, ValueError):
                pass
    return None


def classify_navigation_failure(result: Mapping[str, Any]) -> dict[str, Any]:
    text = _failure_text(result)
    voltage = _reported_voltage(result)
    prearm_timeout = _attributed_prearm_command_timeout(result)
    if "low battery" in text or "voltage below" in text or (
        voltage is not None and voltage < 9.0
    ):
        code = "LOW_BATTERY"
    elif "watchdog_latched" in text and "command_timeout" in text:
        code = "CHASSIS_COMMAND_TIMEOUT"
    elif ("watchdog_latched" in text and "host_scheduling_timeout" in text
          and "estop_latched" not in text and "emergency_stop_topic" not in text
          and any(isinstance(event, Mapping) and event.get('name') == 'host_scheduling_stop_confirmed'
                  and event.get('value') is True for event in result.get('events', []))):
        code = "CHASSIS_COMMAND_TIMEOUT"
    elif "deadman_timeout" in text and "estop_latched" not in text:
        code = "CHASSIS_DEADMAN_TIMEOUT"
    elif _attributed_deadman_cleanup(result):
        code = "CONTROL_HEARTBEAT_TIMEOUT"
    elif confirmed_localization_freshness_stop(result):
        code = "LOCALIZATION_DATA_DELAY"
    elif _attributed_localization_stop(result):
        code = "LOCALIZATION_PROTECTIVE_STOP"
    elif _prearm_localization_rejection(result):
        code = "LOCALIZATION_PROTECTIVE_STOP"
    elif _attributed_navigation_timeout(result):
        code = "NAVIGATION_RESULT_TIMEOUT"
    elif prearm_timeout:
        code = "NAVIGATION_SERVICE_UNAVAILABLE"
    elif any(x in text for x in ("estop_latched", "watchdog_latched", "safety stop", "unexpectedly started armed", "did not finish disarmed")):
        code = "SAFETY_STOP_LATCHED"
    elif any(x in str(result.get('error','')).lower() for x in (
        'final amcl position outside origin tolerance',
        'verified return position is outside bounded correction distance',
    )):
        code = "RETURN_POSITION_INCOMPLETE"
    elif (isinstance(result.get("origin_position_stage"), Mapping)
          and result["origin_position_stage"].get("status") == "PASS"
          and isinstance(result.get("origin_heading_stage"), Mapping)
          and result["origin_heading_stage"].get("status") == "FAIL"):
        code = "RETURN_HEADING_INCOMPLETE"
    elif any(x in text for x in ("hardware transport", "stm32", "serial", "fresh feedback", "feedback_connected")):
        code = "CHASSIS_COMMUNICATION_FAILED"
    elif _aborted_with_inactive_navigation(result):
        code = "NAVIGATION_SERVICE_UNAVAILABLE"
    elif (result.get('failure_code') == 'ARRIVAL_VERIFICATION_INCOMPLETE'
          and result.get('final_armed') is False and result.get('final_state') == 'DISARMED'
          and isinstance(result.get('inspection_arrival'), Mapping)
          and isinstance(result['inspection_arrival'].get('geometry'), Mapping)
          and result['inspection_arrival']['geometry'].get('stationary_confirmed') is True):
        code = "ARRIVAL_VERIFICATION_INCOMPLETE"
    elif any(x in text for x in ("static-map localization", "continuous route localization", "amcl", "localization", "pose jump")):
        code = "LOCALIZATION_UNSTABLE"
    elif any(x in text for x in ("fresh scan", "scan was stale", "lidar", "laser")):
        code = "LIDAR_DATA_TRANSIENT"
    elif any(x in text for x in ("action server is unavailable", "goal was rejected", "service is unavailable", "service unavailable")):
        code = "NAVIGATION_SERVICE_UNAVAILABLE"
    elif any(x in text for x in ("computepathtopose", "no valid path", "path_check", "empty or one-pose path", "start clearance", "goal cell cost", "lethal", "obstacle", "collision monitor", "navigatetopose did not report success", "failed to make progress", "progress checker", "action result timed out")):
        code = "ROUTE_BLOCKED"
    else:
        code = "UNCLASSIFIED_NAVIGATION_FAILURE"
    policy = dict(POLICIES[code])
    if prearm_timeout and code == "NAVIGATION_SERVICE_UNAVAILABLE":
        # Use the existing recording, with no automatic retry or latch reset.
        policy["paused_speech_key"] = "navigation_paused_command_timeout"
    for field in ("started_speech_key", "paused_speech_key"):
        key = policy.get(field)
        policy[field.replace("key", "text")] = None if key is None else PROMPTS[key][0]
    return policy


def _attributed_prearm_command_timeout(result: Mapping[str, Any]) -> bool:
    """Recognize a documented disarmed timeout followed only by cleanup."""
    if result.get("error") != "timed out waiting for fresh post-goal Nav2 command stream before hardware arming":
        return False
    if (result.get("final_armed") is not False
            or result.get("final_state") not in {"DISARMED", "ESTOP_LATCHED"}
            or result.get("final_transition_reason") not in {"operator_disarm", "emergency_stop_topic"}):
        return False
    events = result.get("events")
    if not isinstance(events, (list, tuple)) or not all(isinstance(e, Mapping) for e in events):
        return False
    markers = [i for i, event in enumerate(events) if event.get("name") == "prearm_command_timeout"]
    if len(markers) != 1:
        return False
    index = markers[0]
    cause = events[index].get("value")
    if (not isinstance(cause, Mapping) or cause.get("state") != "DISARMED"
            or cause.get("armed") is not False
            or cause.get("transition_reason") != "safety_latches_reset"
            or not all(cause.get(key) is True for key in
                       ("feedback_connected", "stm32_connected", "transport_attached"))):
        return False
    # A real armed or hardware-fault episode must never be relabelled.
    if any(event.get("name") == "state" and event.get("value") in
           {"ARMED_IDLE", "ACTIVE", "WATCHDOG_LATCHED"} for event in events):
        return False
    reset = [i for i, event in enumerate(events[:index]) if event.get("name") == "transition_reason"
             and event.get("value") == "safety_latches_reset"]
    if not reset or any(event.get("name") == "state" and event.get("value") != "DISARMED"
                        for event in events[reset[-1]+1:index]):
        return False
    after = events[index+1:]
    if any(event.get("name") == "transition_reason" and event.get("value") not in
           {"operator_disarm", "emergency_stop_topic"} for event in after):
        return False
    return (any(e.get("name") == "arm_response" and str(e.get("value", "")).startswith("controller disarmed;") for e in after)
            and any(e.get("name") == "action_cancel_requested" and e.get("value") is True for e in after)
            and any(e.get("name") == "cleanup_stop_confirmed" and e.get("value") is True for e in after))


def _aborted_with_inactive_navigation(result: Mapping[str, Any]) -> bool:
    metrics = result.get('metrics')
    if not isinstance(metrics, Mapping) or metrics.get('action_status') != 6:
        return False
    states = metrics.get('navigation_lifecycle_states')
    return (result.get('final_armed') is False
            and isinstance(states, Mapping)
            and any(type(states.get(name)) is int and states[name] in {1, 2, 4}
                    for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')))


def _attributed_navigation_timeout(result: Mapping[str, Any]) -> bool:
    """Distinguish a navigation timeout followed solely by deliberate cleanup.

    Classification never resets a latch or enables an automatic retry.
    Any earlier recorded safety latch keeps its existing priority.
    """
    if str(result.get("error", "")) != "timed out waiting for NavigateToPose result":
        return False
    if (result.get("final_armed") is not False
            or result.get("final_state") not in {"DISARMED", "ESTOP_LATCHED"}):
        return False
    if result.get("final_transition_reason") != "operator_disarm":
        return False
    events = result.get("events")
    if not isinstance(events, (list, tuple)):
        return False
    armed = False
    latches = 0
    cleanup_confirmed = False
    canceled = False
    reasons = []
    timeout_recorded = False
    for event in events:
        if not isinstance(event, Mapping):
            continue
        name, value = event.get("name"), event.get("value")
        if name == "state" and value in {"ARMED_IDLE", "ACTIVE"}:
            armed = True
        if not armed:
            continue
        if name == "navigation_result_timeout" and not latches:
            timeout_recorded = (isinstance(value, Mapping)
                and value.get("state") == "ACTIVE" and value.get("armed") is True
                and value.get("transition_reason") == "active_autonomy")
        if name == "state" and value in {"ESTOP_LATCHED", "WATCHDOG_LATCHED"}:
            if value != "ESTOP_LATCHED":
                return False
            latches += 1
        if latches and name == "transition_reason":
            reasons.append(value)
        if name == "arm_response" and latches == 1:
            cleanup_confirmed = str(value).startswith("controller disarmed;")
        if name == "action_cancel_requested" and value is True and cleanup_confirmed:
            canceled = True
    return (timeout_recorded and latches == 1 and reasons == ["operator_disarm"]
            and cleanup_confirmed and canceled)


def _attributed_deadman_cleanup(result: Mapping[str, Any]) -> bool:
    """Keep a recorded deadman stop visible through deliberate cleanup.

    A real/unknown latch keeps precedence. This only labels the cause and
    never clears a latch, extends a timeout, or permits automatic motion.
    """
    if result.get("final_state") != "ESTOP_LATCHED" or result.get("final_armed") is not False:
        return False
    if result.get("final_transition_reason") not in {"operator_disarm", "emergency_stop_topic"}:
        return False
    observed = [event.get('value') for event in result.get('events', [])
                if isinstance(event, Mapping)
                and event.get('name') == 'navigation_hardware_stop_observed']
    if len(observed) == 1:
        cause = observed[0]
        if (isinstance(cause, Mapping) and cause.get('armed') is False
                and cause.get('state') == 'DISARMED'
                and cause.get('transition_reason') == 'deadman_timeout'):
            index = next(i for i, event in enumerate(result['events'])
                         if event.get('name') == 'navigation_hardware_stop_observed')
            after = result['events'][index+1:]
            return (not any(event.get('name') == 'state'
                            and event.get('value') == 'WATCHDOG_LATCHED' for event in after)
                    and all(event.get('value') in {'operator_disarm', 'emergency_stop_topic'}
                            for event in after if event.get('name') == 'transition_reason')
                    and any(event.get('name') == 'arm_response'
                            and str(event.get('value')).startswith('controller disarmed;') for event in after)
                    and any(event.get('name') == 'action_cancel_requested'
                            and event.get('value') is True for event in after))
    armed = stopped = latched = cleanup = canceled = False
    observed_state = None
    for event in result.get("events", []):
        if not isinstance(event, Mapping):
            return False
        name, value = event.get("name"), event.get("value")
        if name == "state":
            observed_state = value
        if name == "state" and value in {"ARMED_IDLE", "ACTIVE"}:
            armed = True
        if not armed:
            continue
        if name == "transition_reason" and value not in {
            "operator_arm", "active_autonomy", "deadman_timeout", "operator_disarm"
        }:
            # State and reason are separate DDS topics. A confirmed DISARMED
            # deadman stop can arrive after the client's initial stop marker;
            # cleanup then publishes ESTOP repeatedly while canceling Nav2.
            if value != "emergency_stop_topic" or not (stopped and latched):
                return False
        if (name == "transition_reason" and value == "deadman_timeout"
                and observed_state == "DISARMED" and not latched):
            stopped = True
        if name == "state" and value in {"ESTOP_LATCHED", "WATCHDOG_LATCHED"}:
            if value != "ESTOP_LATCHED" or not stopped or latched:
                return False
            latched = True
        if latched and name == "transition_reason" and value not in {"operator_disarm", "emergency_stop_topic"}:
            return False
        if latched and name == "arm_response":
            cleanup = str(value).startswith("controller disarmed;")
        if cleanup and name == "action_cancel_requested" and value is True:
            canceled = True
    return (stopped and latched and cleanup and canceled
            and any(isinstance(e, Mapping) and e.get('name') == 'cleanup_stop_confirmed'
                    and e.get('value') is True for e in result.get('events', [])))


def _prearm_localization_rejection(result: Mapping[str, Any]) -> bool:
    """Name a failed map preflight without resetting any inherited latch."""
    if not str(result.get('error', '')).startswith('independent static-map localization gate failed:'):
        return False
    metrics = result.get('metrics') or {}
    gate = result.get('static_map_localization_gate') or {}
    return (result.get('final_armed') is False and gate.get('status') == 'FAIL'
            and metrics.get('action_feedback_samples') == 0
            and metrics.get('deadman_keepalive_publish_count') == 0
            and not any(event.get('name') == 'state'
                        and event.get('value') in {'ARMED_IDLE', 'ACTIVE'}
                        for event in result.get('events', []) if isinstance(event, Mapping)))


def confirmed_localization_freshness_stop(result: Mapping[str, Any]) -> bool:
    """Require this segment's own stop proof; never reinterpret a real latch."""
    if (result.get('status') != 'FAIL' or result.get('final_armed') is not False
            or result.get('final_state') != 'DISARMED'
            or result.get('error') != 'in-motion scan/map localization disagreement; stopped'):
        return False
    events = result.get('events', [])
    first_arm = next((i for i,e in enumerate(events) if isinstance(e,Mapping)
                      and e.get('name') == 'state' and e.get('value') in {'ARMED_IDLE','ACTIVE'}), None)
    if first_arm is None:
        return False
    segment = events[first_arm:]
    if any(isinstance(e,Mapping) and e.get('name') == 'state'
           and e.get('value') in {'ESTOP_LATCHED','WATCHDOG_LATCHED'} for e in segment):
        return False
    markers = [e.get('value') for e in segment if isinstance(e,Mapping)
               and e.get('name') == 'motion_map_guard_stop']
    proofs = [e.get('value') for e in segment if isinstance(e,Mapping)
              and e.get('name') == 'localization_freshness_stop_confirmed']
    return (len(markers) == 1 and isinstance(markers[0],Mapping)
            and markers[0].get('status') == 'STALE_AMCL'
            and len(proofs) == 1 and isinstance(proofs[0],Mapping)
            and all(proofs[0].get(k) is True for k in
                    ('goal_terminal','stationary','hardware_connected')))


def _attributed_localization_stop(result: Mapping[str, Any]) -> bool:
    """Preserve a recorded software stop cause without permitting a retry.

    A latch observed before this segment's own guard marker, or another
    hardware cause, still has safety priority. The cleanup latch is expected.
    """
    if result.get("final_armed") is not False or result.get("final_state") != "ESTOP_LATCHED":
        return False
    metrics = result.get("metrics")
    if not isinstance(metrics, Mapping) or metrics.get("motion_map_guard_tripped") is not True:
        return False
    error = str(result.get("error", "")).lower()
    if "in-motion scan/map localization disagreement; stopped" not in error:
        return False
    events = result.get("events")
    if not isinstance(events, (list, tuple)):
        return False
    armed_segment = False
    marker = False
    observed_state = None
    for event in events:
        if not isinstance(event, Mapping):
            continue
        name, value = event.get("name"), event.get("value")
        if name == "state":
            observed_state = value
        if name == "state" and value in {"ARMED_IDLE", "ACTIVE"}:
            armed_segment = True
        if name == "motion_map_guard_stop" and armed_segment:
            marker = isinstance(value, Mapping) and value.get("status") in {"DISAGREEMENT", "STALE_AMCL"}
        if armed_segment and name == "state" and value == "WATCHDOG_LATCHED":
            return False
        if armed_segment and name == "state" and value in {"ESTOP_LATCHED", "WATCHDOG_LATCHED"} and not marker:
            return False
        if marker and name == "transition_reason" and value not in {
            "active_autonomy", "emergency_stop_topic", "operator_disarm", "operator_arm"
        }:
            # The guard sends both ESTOP and deadman(False). Their independent
            # subscriptions can report the requested DISARMED release before
            # the ESTOP cleanup latch. This is not an unrelated hardware fault.
            # Only accept that release after this segment's own stop marker;
            # timeouts and releases while still armed keep safety precedence.
            if value != "deadman_released" or observed_state != "DISARMED":
                return False
    return marker and str(result.get("final_transition_reason", "")) in {
        "emergency_stop_topic", "operator_disarm"
    }


def retry_is_safe(result: Mapping[str, Any], decision: Mapping[str, Any]) -> bool:
    if decision.get("automatic") is not True or result.get("final_armed") is not False:
        return False
    state = str(result.get("final_state", "")).upper()
    if decision.get("code") in {"CHASSIS_COMMAND_TIMEOUT", "CHASSIS_DEADMAN_TIMEOUT"}:
        text = _failure_text(result)
        command_timeout = decision.get("code") == "CHASSIS_COMMAND_TIMEOUT"
        cause = "command_timeout" if command_timeout else "deadman_timeout"
        if command_timeout and "host_scheduling_timeout" in text:
            cause = "host_scheduling_timeout"
        return (
            state in {"DISARMED", "WATCHDOG_LATCHED"}
            and cause in text
            and (not command_timeout or "watchdog_latched" in text)
            and "estop_latched" not in text
        )
    return state == "DISARMED"
