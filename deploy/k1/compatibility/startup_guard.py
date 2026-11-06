"""Bound a stationary startup stage and emit validated progress events."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import tempfile
import time
from uuid import uuid4


def write_state(path, payload):
    if not path:
        return
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                    dir=destination.parent, delete=False) as stream:
        json.dump(payload, stream, ensure_ascii=False)
        temporary = stream.name
    os.replace(temporary, destination)


def progress(stage, elapsed):
    directory = os.environ.get('RVCLAW_ARRIVAL_EVENT_DIR', '')
    if not directory:
        return
    event_id = str(uuid4())
    write_state(Path(directory) / ('100-startup-' + event_id + '.json'), {
        'event_id': event_id, 'event': 'navigation_startup_progress',
        'status': 'STARTED', 'timestamp': time.time(),
        'speech_key': 'navigation_startup_' + stage,
        'elapsed_s': elapsed, 'motion_authorized': False,
    })


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=('map', 'localization'), required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--progress-after', type=float, default=8.)
    parser.add_argument('--handoff-ready', action='store_true',
                        help='release the stationary deadline only after a child readiness byte')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('a positive finite timeout and command are required')
    canceled = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda signum, _frame: canceled.append(signum))
    started = time.monotonic()
    state_path = os.environ.get('RVCLAW_STARTUP_STATE_FILE')
    write_state(state_path, {'stage': args.stage, 'status': 'STARTED', 'timestamp': time.time()})
    if canceled:
        return 130
    # Own the child's group so a timeout also terminates any descendants.
    ready_read = ready_write = None
    environment = os.environ.copy()
    if args.handoff_ready:
        ready_read, ready_write = os.pipe()
        environment['RVCLAW_STARTUP_READY_FD'] = str(ready_write)
    try:
        child = subprocess.Popen(command, start_new_session=True, env=environment,
                                 pass_fds=() if ready_write is None else (ready_write,))
    finally:
        if ready_write is not None:
            os.close(ready_write)
    next_progress = args.progress_after
    timed_out = False
    handed_off = False
    protocol_error = False
    def consume_ready(elapsed):
        nonlocal ready_read, handed_off, protocol_error
        if ready_read is None or not select.select([ready_read], [], [], 0)[0]:
            return
        message = os.read(ready_read, 2)
        os.close(ready_read)
        ready_read = None
        if message == b'R' and elapsed < args.timeout:
            handed_off = True
            write_state(state_path, {'stage': args.stage, 'status': 'PASS',
                        'return_code': 0, 'elapsed_s': elapsed, 'timestamp': time.time()})
        elif message:
            protocol_error = True
    try:
        while child.poll() is None:
            elapsed = time.monotonic() - started
            consume_ready(elapsed)
            if protocol_error:
                break
            if canceled or (not handed_off and elapsed >= args.timeout):
                timed_out = not canceled
                break
            if not handed_off and elapsed >= next_progress:
                progress(args.stage, elapsed)
                next_progress += 20.
            try:
                child.wait(timeout=.2 if handed_off else min(.2, max(.01, args.timeout-elapsed)))
            except subprocess.TimeoutExpired:
                pass
        # A fast child may complete between two polls; its readiness byte still
        # has to be consumed, and success without that byte is a protocol error.
        consume_ready(time.monotonic()-started)
        if args.handoff_ready and child.poll() == 0 and not handed_off:
            protocol_error = True
    finally:
        if ready_read is not None:
            os.close(ready_read)
        if child.poll() is None:
            for signum, grace in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
                try:
                    os.killpg(child.pid, signum)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=grace)
                    break
                except subprocess.TimeoutExpired:
                    continue
        # A child can exit while leaving a worker behind. Revoke that group
        # on every cancellation, including this race.
        if timed_out or canceled or protocol_error:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    code = 124 if timed_out else 130 if canceled else 1 if protocol_error else int(child.returncode)
    if not handed_off:
        write_state(state_path, {'stage': args.stage,
        'status': 'TIMEOUT' if timed_out else 'CANCELLED' if canceled else 'PASS' if code == 0 else 'FAIL',
            'return_code': code, 'elapsed_s': time.monotonic()-started, 'timestamp': time.time()})
    if timed_out:
        print('stationary startup %s timed out; child group terminated; refusing navigation' % args.stage, flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
