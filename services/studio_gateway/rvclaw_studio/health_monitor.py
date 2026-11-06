"""Bounded, passive hardware status and dashboard warning rules."""
from collections import deque
from pathlib import Path
import json
import math
import os
import subprocess
import threading
import time

RULES = {'temperature_celsius': 80, 'cpu_percent': 90, 'cpu_duration_s': 30, 'memory_percent': 90, 'storage_percent': 90, 'sensor_timeout_s': 5, 'observer_timeout_s': 8}


def number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def ratio(item):
    total, used = number(item.get('total_bytes')), number(item.get('used_bytes'))
    return round(used / total * 100, 1) if total and used is not None else None


def status(key, label, state, reason, advice='', **extra):
    return {'id': key, 'label': label, 'state': state, 'status': {'ok': '正常', 'idle': '待机', 'error': '异常', 'unknown': '待确认'}[state], 'reason': reason, 'advice': advice, **extra}


def sample_runtime(project=Path('/data/RVClaw'), system=Path('/'), now=None):
    """Read existing scheduler/device state without opening ports or spawning ROS."""
    now = time.time() if now is None else now
    try:
        path = project / 'runtime/k1/resource-scheduler/status.json'
        if path.stat().st_size > 8192:
            return None
        data = json.loads(path.read_text())
        sampled_at = number(data.get('sampled_at'))
        if sampled_at is None or not 0 <= now - sampled_at <= 15:
            return None
        group = system / f'sys/fs/cgroup/user.slice/user-{os.getuid()}.slice/user@{os.getuid()}.service/app.slice/rvclaw-nav2-persistent.service/cgroup.procs'
        try:
            navigation_running = bool(group.read_text().strip())
        except FileNotFoundError:
            navigation_running = False
        idle = (not navigation_running and data.get('state') in
                {'sleeping', 'idle_grace', 'waiting_devices'}
                and data.get('lease_kinds') == [] and data.get('motion_authorized') is False)
        devices = {}
        for key, name in (
            ('chassis', 'dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'),
            ('lidar', 'dev/wheeltec_lidar'),
        ):
            device = system / name
            connected = device.is_char_device()
            devices[key] = dict(connected=connected,
                                accessible=connected and os.access(device, os.R_OK | os.W_OK))
        return dict(idle=idle, sampled_at=now, devices=devices)
    except (OSError, ValueError, TypeError, AttributeError):
        return None


def evaluate(snapshot, observer, history=(), now=None, runtime=None):
    now = time.time() if now is None else now
    rules = dict(RULES)
    cpu = snapshot.get('cpu') or {}
    temperatures = [number(row.get('celsius')) for row in cpu.get('temperatures', []) if ('cluster' in str(row.get('type', '')) and 'thermal' in str(row.get('type', '')))]
    temperatures = [v for v in temperatures if v is not None]
    average = round(sum(temperatures) / len(temperatures), 1) if temperatures else number(cpu.get('temperature_celsius'))
    hottest = max(temperatures) if temperatures else average
    usage = number(cpu.get('usage_percent'))
    sustained = bool(history) and now - history[0][0] >= rules['cpu_duration_s'] and all(value >= rules['cpu_percent'] for _, value in history)
    metrics = []
    for key, label, value, bad, reason, advice, unit in [
        ('temperature', '处理器温度', average, hottest is not None and hottest >= rules['temperature_celsius'], f'核心最高温度达到 {hottest:g} °C' if hottest is not None else '', '检查散热风扇、通风和当前任务负载。', '°C'),
        ('cpu', '处理器占用', usage, sustained, '处理器占用连续 30 秒达到 90% 以上', '检查持续占用处理器的程序，避免同时运行额外计算任务。', '%'),
        ('memory', '内存占用', ratio(snapshot.get('memory') or {}), ratio(snapshot.get('memory') or {}) is not None and ratio(snapshot.get('memory') or {}) >= rules['memory_percent'], '可用内存不足，内存占用达到 90% 以上', '检查额外程序的内存使用情况。', '%'),
        ('storage', '存储占用', ratio(snapshot.get('storage') or {}), ratio(snapshot.get('storage') or {}) is not None and ratio(snapshot.get('storage') or {}) >= rules['storage_percent'], '存储空间不足，已用容量达到 90% 以上', '先导出需要保留的记录，再清理无用文件。', '%'),
    ]:
        state = 'unknown' if value is None else 'error' if bad else 'ok'
        metrics.append(status(key, label, state, '未获取到有效采样' if state == 'unknown' else reason if bad else '当前采样处于监测范围内', advice if state != 'ok' else '', value=value, unit=unit))

    fresh = isinstance(observer, dict) and 0 <= now - observer.get('sampled_at', 0) <= rules['observer_timeout_s']
    warming = fresh and now - observer.get('started_at', 0) < 10
    readings = observer.get('readings', {}) if fresh else {}
    def live(key):
        row = readings.get(key) or {}
        return row if 0 <= now - row.get('received_at', 0) <= rules['sensor_timeout_s'] else {}
    localization = live('localization')
    transport, feedback, control = live('transport'), live('feedback'), live('controller')
    hardware = [status('board', '主控采集', 'ok', '主控温度与系统资源接口已响应', source='主控系统')]
    if transport.get('value') is False or localization.get('hardware_transport_attached') is False:
        chassis_state, chassis_reason = 'error', '底盘通信未连接，未建立硬件传输链路'
    elif feedback.get('value') is False or localization.get('hardware_connected') is False:
        chassis_state, chassis_reason = 'error', '底盘反馈中断，未收到有效状态数据'
    elif live('odometry') and 0 <= now - live('odometry').get('source_at', 0) <= rules['sensor_timeout_s'] and feedback.get('value') is True:
        chassis_state, chassis_reason = 'ok', '底盘通信和里程计反馈正常'
    else:
        chassis_state, chassis_reason = ('unknown', '正在确认底盘反馈') if warming or not fresh else ('error', '未收到底盘实时反馈')
    hardware.append(status('chassis', '底盘与里程计', chassis_state, chassis_reason, '检查底盘供电、通信线和驱动状态。' if chassis_state != 'ok' else '', source='底盘状态与里程计'))
    for key, label, advice in [('lidar', '激光雷达', '检查雷达供电、数据线与雷达驱动。'), ('imu', '惯性传感器', '检查底盘供电与惯导数据反馈。')]:
        row = live(key)
        source_age = now - row.get('source_at', 0)
        online = bool(row) and 0 <= source_age <= rules['sensor_timeout_s']
        state = 'ok' if online else 'unknown' if warming or not fresh else 'error'
        reason = '实时反馈正常' if online else '数据已过期，未收到新的采样' if row else '正在确认实时反馈' if state == 'unknown' else '未收到实时反馈'
        hardware.append(status(key, label, state, reason, advice if state != 'ok' else '', age_s=round(source_age, 1) if row and number(source_age) is not None else None, source='实时传感器数据'))
    camera = snapshot.get('camera') or {}
    age = number((snapshot.get('stability') or {}).get('frame_age_ms'))
    camera_ok = camera.get('online') is True and age is not None and age <= 5000
    hardware.append(status('camera', '巡检相机', 'ok' if camera_ok else 'unknown' if camera.get('switching') else 'error', '相机画面持续更新' if camera_ok else '相机模式切换中' if camera.get('switching') else '相机离线或画面已停止更新', '' if camera_ok else '检查相机供电、连接线和当前相机模式。', source='相机采集链路'))
    valid = localization.get('valid') is True
    motion_observed = (localization.get('state') == 'CONTROLLED_MOTION'
                       and localization.get('lidar_fresh') is True
                       and localization.get('hardware_connected') is True
                       and bool(live('odometry')))
    stationary = (control.get('state') == 'DISARMED' and localization.get('armed') is False)
    settled = (stationary and localization.get('pose_reusable') is True
               and localization.get('state') in {'HOT_READY','WARM_VALIDATION_REQUIRED'}
               and localization.get('lidar_fresh') is True and localization.get('hardware_connected') is True)
    pose_state = 'ok' if valid or motion_observed else 'idle' if settled else 'unknown'
    pose_reason = ('运动中，导航持续校验定位' if motion_observed else '定位反馈正常' if valid else '小车静止，已验证定位保持中' if settled
                   else '定位正在初始化或重新核验' if localization else '正在确认定位状态')
    hardware.append(status('localization', '定位状态', pose_state, pose_reason,
                           source='定位状态反馈'))
    # Lifecycle diagnostics run more slowly than sensors; their five-second
    # absence is not a disconnected lidar or failed navigation process.
    nav = readings.get('navigation') or {}
    if not 0 <= now-nav.get('received_at', 0) <= 30:
        nav = {}
    nav_state = 'ok' if nav.get('level') == 0 else 'error' if nav.get('level') in {2,3} else 'unknown'
    awaiting_activation = (nav.get('message') == 'Nav2 is inactive'
        and (control.get('state') == 'DISARMED' or
             (control.get('state') == 'ESTOP_LATCHED'
              and live('transition').get('reason') == 'startup_estop')))
    if awaiting_activation:
        nav_state = 'unknown'
    nav_reason = ('导航已就绪' if nav_state == 'ok' else nav.get('message') or '导航报告异常'
                  if nav_state == 'error' else '等待定位核验，随后自动激活导航' if awaiting_activation
                  else '导航正在准备或自动恢复' if nav else '正在确认导航状态')
    hardware.append(status('navigation', '巡检导航', nav_state, nav_reason,
                           source='导航诊断'))
    if not fresh:
        hardware.append(status('observer', '硬件监测链路', 'error' if observer else 'unknown', '硬件监测数据已过期' if observer else '等待硬件监测数据', '检查网页服务的硬件监测组件。', source='监测心跳'))
    if control.get('state') in {'ESTOP_LATCHED', 'WATCHDOG_LATCHED'}:
        initial_stop = (control.get('state') == 'ESTOP_LATCHED'
                        and live('transition').get('reason') == 'startup_estop'
                        and localization.get('armed') is False)
        hardware.append(status('protection', '运行保护', 'idle' if initial_stop else 'error',
            '启动时保持停止，等待任务就绪' if initial_stop else
            '急停保护已触发' if control['state'] == 'ESTOP_LATCHED' else '通信看门狗保护已触发',
            '' if initial_stop else '小车已停止，系统正在检查恢复条件；真实急停需排除现场危险。',
            source='底盘保护状态'))
    runtime_stamp = number(runtime.get('sampled_at')) if isinstance(runtime, dict) else None
    if (runtime_stamp is not None and 0 <= now - runtime_stamp <= 15
            and runtime.get('idle') is True):
        for row in hardware:
            key = row['id']
            if key not in {'chassis', 'lidar', 'imu', 'localization', 'navigation'}:
                continue
            connection = runtime.get('devices', {}).get('chassis' if key == 'imu' else key)
            if key in {'chassis', 'lidar', 'imu'}:
                if not isinstance(connection, dict):
                    continue
                if connection.get('connected') is False:
                    row.update(state='error', status='异常', reason='未检测到设备 USB 连接',
                               advice='检查设备供电和 USB 连接线。')
                    continue
                if connection.get('connected') is not True:
                    continue
                if connection.get('accessible') is not True:
                    row.update(state='error', status='异常', reason='设备已连接，但当前账户无串口读写权限',
                               advice='检查设备访问权限。')
                    continue
            row.update(state='idle', status='待机', advice='', age_s=None,
                       reason='设备已连接，采集服务待机，任务启动后自动采集' if connection
                       else '当前无巡检任务，服务按需待机')
    issues = [row for row in hardware + metrics if row['state'] == 'error']
    unknown = sum(row['state'] == 'unknown' for row in hardware + metrics)
    idle = any(row['state'] == 'idle' for row in hardware)
    return {'state': 'error' if issues else 'unknown' if unknown else 'idle' if idle else 'ok', 'status': '监测异常' if issues else '正在确认' if unknown else '设备已连接，待机中' if idle else '监测正常', 'sampled_at': snapshot.get('sampled_at'), 'observer_age_s': round(now - observer.get('sampled_at', 0), 1) if observer else None, 'hardware': hardware, 'metrics': metrics, 'issues': issues, 'rules': rules}


class HealthMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.observer = {}
        self.process = None
        self.history = deque(maxlen=90)
        self.last_sample = None
        self.started_at = time.time()

    def start(self):
        if self.process is not None:
            return
        observer = Path(__file__).with_name('hardware_observer.py')
        command = ['nice', '-n', '15', 'bash', '-c', 'source /opt/ros/humble/setup.bash >/dev/null && exec /usr/bin/python3 "$1"', 'rvclaw-health', str(observer)]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True, name='hardware-health-reader').start()

    def _read(self):
        for line in self.process.stdout:
            try:
                value = json.loads(line)
                if isinstance(value, dict) and len(line) <= 32768:
                    with self.lock:
                        self.observer = value
            except (ValueError, TypeError):
                continue

    def attach(self, snapshot):
        now = time.time()
        with self.lock:
            sample = snapshot.get('sampled_at')
            value = number((snapshot.get('cpu') or {}).get('usage_percent'))
            if sample != self.last_sample:
                if value is None or value < RULES['cpu_percent'] or self.history and now - self.history[-1][0] > 15:
                    self.history.clear()
                if value is not None and value >= RULES['cpu_percent']:
                    self.history.append((now, value))
                self.last_sample = sample
            while len(self.history) > 1 and now - self.history[1][0] > RULES['cpu_duration_s']:
                self.history.popleft()
            observer = self.observer or ({'sampled_at': self.started_at, 'readings': {}} if now - self.started_at >= 10 else {})
            snapshot['health'] = evaluate(snapshot, observer, list(self.history), now,
                                          runtime=sample_runtime(now=now))
        return snapshot

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
