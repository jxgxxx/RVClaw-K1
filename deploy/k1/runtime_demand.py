#!/usr/bin/env python3
"""Own navigation demand without publishing ROS messages or arming hardware."""
import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

DEFAULT_STATE = Path('/data/RVClaw/runtime/k1/resource-scheduler')
NAV_UNIT = 'rvclaw-nav2-persistent.service'
NAV_DEVICES = ('/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00',
               '/dev/wheeltec_lidar')
ACTIVE_SCRIPTS = {'nav2_persistent_route.sh', 'rvclaw_waypoint_route_client.py',
                  'slam_mapping_session.sh', 'mapping_entry.sh',
                  'stm32_control_client.py', 'nav2_global_localization_scan.py',
                  'nav2_heading_spin_left_0350.py'}


def pid_identity(pid, proc=Path('/proc')):
    try:
        fields = (proc / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return fields[19]  # starttime, field 22 of proc_pid_stat
    except (OSError, ValueError, IndexError):
        return None


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def write_json(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value), encoding='utf-8')
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def locked(state):
    state.mkdir(parents=True, exist_ok=True)
    with (state / 'scheduler.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def device_ready(path):
    try:
        return Path(path).is_char_device() and os.access(path, os.R_OK | os.W_OK)
    except OSError:
        return False


def devices_ready():
    return all(device_ready(path) for path in NAV_DEVICES)


def boot_identity():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def live_leases(state, *, identity=pid_identity, boot=None):
    boot = boot or boot_identity()
    kinds = set()
    for path in state.glob('lease-*.json'):
        value = read_json(path)
        if (isinstance(value, dict) and value.get('boot') == boot
                and value.get('start') is not None
                and identity(value.get('pid')) == value['start']):
            kinds.add(value.get('kind'))
        else:
            path.unlink(missing_ok=True)
    return kinds


def external_demand(proc=Path('/proc')):
    """Protect older task runners, including mapping and manual diagnostics."""
    for path in proc.glob('[0-9]*/cmdline'):
        try:
            args = [item.decode(errors='replace') for item in path.read_bytes().split(b'\0') if item]
        except OSError:
            continue
        if 'rviz2' in set(map(os.path.basename, args)):
            return True
        if not set(map(os.path.basename, args)) & ACTIVE_SCRIPTS:
            continue
        if any(flag in args for flag in ('--run', '--voice-run', '--arm-token')):
            return True
    return False


def systemctl(action):
    return subprocess.run(['systemctl', '--user', action, '--no-block', NAV_UNIT],
                          check=True, capture_output=True, text=True, timeout=5)


def acquire(state, kind, *, ready=devices_ready, control=systemctl):
    with locked(state):
        if kind == 'inspection' and not ready():
            raise RuntimeError('navigation devices missing or inaccessible; runtime was not started')
        owner = os.getpid()
        path = state / ('lease-' + uuid.uuid4().hex + '.json')
        write_json(path, dict(pid=owner, start=pid_identity(owner), kind=kind,
                              boot=boot_identity()))
        try:
            if kind == 'inspection':
                control('start')
        except BaseException:
            path.unlink(missing_ok=True)
            raise
        return path


def request_warmup(state, seconds, *, ready=devices_ready, control=systemctl):
    # A wake alone may warm resources but never admits AUTO or publishes motion.
    with locked(state):
        if not ready() or live_leases(state) & {'mapping'}:
            return False
        old = read_json(state / 'warm.json') or {}
        now = time.monotonic()
        boot = boot_identity()
        try:
            old_until = float(old.get('until', 0))
        except (TypeError, ValueError):
            old_until = 0.
        # Monotonic deadlines belong to one boot. An old longer uptime must
        # not poison a new boot or create a deadline the supervisor rejects.
        if old.get('boot') != boot or not 0 < old_until - now <= 180.:
            old_until = now
        until = max(old_until, now + min(180., max(1., seconds)))
        write_json(state / 'warm.json', dict(until=until, boot=boot))
        control('start')
        return True


def should_stop(*, demanded, ready, idle_s, grace_s):
    return not demanded and (not ready or idle_s >= grace_s)


def supervise(state, idle_seconds):
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    last_demand = time.monotonic()
    last_status = None
    while not stopping:
        try:
            with locked(state):
                now = time.monotonic()
                leases = live_leases(state)
                warm = read_json(state / 'warm.json') or {}
                warming = (warm.get('boot') == boot_identity()
                           and 0 < float(warm.get('until', 0)) - now <= 181)
                demanded = bool(leases or warming or external_demand())
                if demanded:
                    last_demand = now
                ready = devices_ready()
                status = ('in_use' if demanded else 'idle_grace' if ready and
                          now-last_demand < idle_seconds else 'waiting_devices' if not ready else 'sleeping')
                if should_stop(demanded=demanded, ready=ready,
                               idle_s=now-last_demand, grace_s=idle_seconds):
                    # Start requests acquire this same lock, so a concurrent
                    # task cannot lose its resource to an idle-stop decision.
                    systemctl('stop')
                if status != last_status:
                    print('resource_scheduler ' + status, flush=True)
                    last_status = status
                write_json(state / 'status.json', dict(state=status, devices_ready=ready,
                           lease_kinds=sorted(leases), sampled_at=time.time(),
                           motion_authorized=False))
        except Exception as error:
            # Unknown state never causes an idle shutdown.
            print('resource_scheduler inspection_failed: ' + str(error), flush=True)
        for _ in range(50):
            if stopping:
                break
            time.sleep(.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, default=DEFAULT_STATE)
    commands = parser.add_subparsers(dest='action', required=True)
    run = commands.add_parser('run')
    run.add_argument('--kind', choices=('inspection', 'mapping'), required=True)
    run.add_argument('command', nargs=argparse.REMAINDER)
    warm = commands.add_parser('warm')
    warm.add_argument('--seconds', type=float, default=60)
    daemon = commands.add_parser('supervise')
    daemon.add_argument('--idle-seconds', type=float, default=60)
    args = parser.parse_args()
    if args.action == 'run':
        command = args.command
        if command[:1] == ['--']:
            command = command[1:]
        if not command:
            parser.error('missing task executable')
        path = acquire(args.state, args.kind)
        env = dict(os.environ, RVCLAW_RUNTIME_LEASE=str(path))
        try:
            os.execvpe(command[0], command, env)
        except BaseException:
            path.unlink(missing_ok=True)
            raise
    elif args.action == 'warm':
        print('prepared=' + str(request_warmup(args.state, args.seconds)).lower())
    else:
        if not 5 <= args.idle_seconds <= 600:
            parser.error('idle interval must be in [5, 600] seconds')
        supervise(args.state, args.idle_seconds)


if __name__ == '__main__':
    main()
