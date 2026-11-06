"""Map identity and Chinese failure feedback from durable task/trace evidence.

Pure file interpretation: no ROS, hardware queries, writes, or motion.
"""
from pathlib import Path
from functools import lru_cache
import re


@lru_cache(maxsize=128)
def _map_identifier(path, mtime_ns, size):
    """Cache immutable configuration versions, never live robot state."""
    import yaml
    value = yaml.safe_load(Path(path).read_text())
    return value.get('map_id') if isinstance(value, dict) else None


def map_from_trace(trace):
    """Only explicit map file metadata establishes a task's coordinate frame."""
    if not isinstance(trace, dict):
        return None
    metadata = trace.get('map') if isinstance(trace.get('map'), dict) else {}
    values = [trace.get('map_file'), metadata.get('yaml_path')]
    for value in values:
        if value:
            directory = Path(str(value)).parent
            try:
                source = directory/'waypoints.yaml'
                stat = source.stat()
                identifier = _map_identifier(str(source), stat.st_mtime_ns, stat.st_size) or directory.name
            except (OSError,ValueError,TypeError):
                identifier = directory.name
            if re.fullmatch(r'[A-Za-z0-9_-]{1,96}', identifier):
                return identifier
    return None


def annotate_runs(runs, traces):
    """Attach context before summarising, including runs with no captured photos."""
    for run in runs:
        trace = traces.get(run.get('trace_file'))
        fast_path = run.get('fast_localization_trace_file')
        if not fast_path and run.get('run_dir'):
            fast_path = str(Path(run['run_dir'])/'fast-localization.json')
        fast = traces.get(fast_path)
        if not run.get('map_id'):
            run['map_id'] = map_from_trace(trace) or map_from_trace(fast)
        failure = failure_feedback(run, trace, fast)
        run.update(failure)


def failure_feedback(run, trace, fast):
    if run.get('state') not in {'failed', 'error', 'startup_failed'}:
        return {}
    completed = trace.get('completed_waypoints') or []
    # A final route failure is more specific than a preflight state snapshot.
    # Do not report startup failure after three confirmed meter stops.
    if 'ORIGIN' not in completed and re.search(r'waypoint\s+ORIGIN\s+failed',str(trace.get('error') or ''),re.I):
        return {'failure_reason':'返航未能完成，请到 RVBench 检查导航与定位状态后重试。','failure_phase':'return'}
    code = (trace.get('resume_plan') or {}).get('failure_code')
    route_failures = {
        'CHASSIS_COMMAND_TIMEOUT': '导航控制指令超时，巡检已暂停。',
        'CHASSIS_DEADMAN_TIMEOUT': '导航心跳超时，巡检已暂停。',
        'NAVIGATION_HEARTBEAT_TIMEOUT': '导航心跳超时，巡检已暂停。',
        'CONTROL_HEARTBEAT_TIMEOUT': '底盘控制心跳超时，巡检已安全暂停。',
        'SAFETY_STOP_LATCHED': '安全保护已触发，巡检已暂停。',
    }
    if code in route_failures:
        return {'failure_reason': route_failures[code], 'failure_phase': 'inspection'}
    # A successful startup snapshot is historical evidence, not the cause of
    # a later route failure. Its disarmed state precedes the normal arm step.
    runtime = (fast.get('final_runtime_state') or {}) if fast.get('status') == 'FAIL' else {}
    reasons = []
    if runtime.get('hardware_connected') is False or runtime.get('hardware_transport_attached') is False:
        reasons.append('底盘硬件未连接')
    if runtime.get('lidar_fresh') is False:
        reasons.append('雷达数据未就绪')
    if runtime.get('pose_message_fresh') is False:
        reasons.append('定位未就绪')
    if runtime.get('controller_state') == 'ESTOP_LATCHED':
        reasons.append('急停处于锁定状态')
    startup_failed = not completed and fast.get('status') == 'FAIL'
    if reasons:
        message = '；'.join(reasons) + '。请连接设备并确认现场条件后重试。'
    elif run.get('timed_out'):
        message = '巡检任务超时，已执行安全停止。请检查设备与巡检路线。'
    elif startup_failed:
        message = '硬件或定位检查未通过，未进入巡检路线。请到 RVBench 检查设备状态。'
    else:
        message = '巡检任务未能完成，请到 RVBench 检查设备与定位状态。'
    return {'failure_reason': message, 'failure_phase': 'startup' if startup_failed else 'inspection'}
