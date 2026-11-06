#!/usr/bin/env python3
"""Bound cold-start service readiness; no ROS, device access or service writes."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from uuid import uuid4


def write_json(path, payload):
    if not path:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=target.parent,
                                     delete=False) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        temporary = stream.name
    os.replace(temporary, target)


def service_state(unit, timeout):
    result = subprocess.run(
        ['systemctl', '--user', 'show', unit, '--no-pager',
         '--property=LoadState,ActiveState,SubState,MainPID'],
        check=True, capture_output=True, text=True, timeout=timeout)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def wait_ready(unit, timeout, *, query=service_state, clock=time.monotonic,
               sleep=time.sleep, cancelled=lambda: False, report=lambda value: None,
               progress=lambda elapsed: None):
    if not math.isfinite(timeout) or not 0 < timeout <= 30:
        raise ValueError('navigation readiness timeout must be within (0, 30] seconds')
    started = clock()
    announced = False
    state = {}

    def finish(status, code, error=None):
        payload = dict(stage='services', status=status, return_code=code,
                       elapsed_s=max(0., clock()-started), timestamp=time.time(),
                       service=unit, service_state=state, motion_authorized=False)
        if error:
            payload['error'] = error
        report(payload)
        return code

    while True:
        if cancelled():
            return finish('CANCELLED', 130, 'navigation readiness cancelled')
        remaining = timeout-(clock()-started)
        if remaining <= 0:
            return finish('TIMEOUT', 124, 'navigation service initialization timed out')
        try:
            state = query(unit, min(2., remaining))
        except (OSError, subprocess.SubprocessError) as error:
            if cancelled():
                return finish('CANCELLED', 130, 'navigation readiness cancelled')
            if clock()-started >= timeout:
                return finish('TIMEOUT', 124, 'navigation service query exceeded deadline')
            return finish('FAIL', 6, 'cannot query navigation service: %s' % error)
        if cancelled():
            return finish('CANCELLED', 130, 'navigation readiness cancelled')
        # A query returning ready after the deadline cannot revive the command.
        if clock()-started >= timeout:
            return finish('TIMEOUT', 124, 'navigation service initialization timed out')
        if state.get('LoadState') != 'loaded':
            return finish('FAIL', 6, 'navigation service is not loaded')
        if state.get('ActiveState') == 'active' and state.get('SubState') == 'running':
            try:
                running = int(state.get('MainPID', '0')) > 0
            except (ValueError, TypeError):
                running = False
            if running:
                return finish('PASS', 0)
            return finish('FAIL', 6, 'navigation service has no running process')
        # A task can arrive just after the idle scheduler queued a stop. Its
        # serialized start job follows that shutdown; keep the same deadline.
        if state.get('ActiveState') not in {'activating', 'deactivating'}:
            return finish('FAIL', 6, 'navigation service is not running: %s/%s' %
                          (state.get('ActiveState'), state.get('SubState')))
        if not announced:
            report(dict(stage='services', status='STARTED', timestamp=time.time(),
                        service=unit, service_state=state, motion_authorized=False))
            progress(max(0., clock()-started))
            announced = True
        sleep(min(.25, max(0., timeout-(clock()-started))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--unit', default='rvclaw-nav2-persistent.service')
    args = parser.parse_args()
    cancelled = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda signum, _frame: cancelled.append(signum))

    def report(value):
        write_json(os.getenv('RVCLAW_STARTUP_STATE_FILE'), value)
        print('navigation_service_readiness '+json.dumps(value), flush=True)

    def progress(elapsed):
        directory = os.getenv('RVCLAW_ARRIVAL_EVENT_DIR')
        if directory:
            event_id = str(uuid4())
            write_json(Path(directory)/('100-startup-'+event_id+'.json'), dict(
                event_id=event_id, event='navigation_startup_progress', status='STARTED',
                timestamp=time.time(), speech_key='navigation_startup_services',
                elapsed_s=elapsed, motion_authorized=False))

    return wait_ready(args.unit, args.timeout, cancelled=lambda: bool(cancelled),
                      report=report, progress=progress)


if __name__ == '__main__':
    raise SystemExit(main())
