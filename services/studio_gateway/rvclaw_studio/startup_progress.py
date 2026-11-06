"""Bounded file-only startup timing for Studio; never grants motion permissions."""
import re
import math
from pathlib import Path
import time


def startup_status(raw, robot, *, now=None):
    now = time.time() if now is None else now
    try:
        started = float(raw['started_at'])
        if not math.isfinite(started) or not math.isfinite(now):
            return None
        directory = Path(raw['run_dir']).resolve()
        allowed = (Path(robot) / 'runtime/k1/mission/runs/inspection').resolve()
        if not directory.is_relative_to(allowed):
            return None
        with (directory / 'command.log').open('rb') as stream:
            data = stream.read(65536).decode('utf-8', errors='replace')
        stages = re.findall(r'^startup_timing stage=([a-z_]+)', data, re.M)
        motion = re.search(r'^startup_timing stage=first_motion timestamp=([0-9.]+)', data, re.M)
        at = float(motion[1]) if motion else None
        if at is not None and (not math.isfinite(at) or not started <= at <= now):
            at = None
        finished = float(raw.get('finished_at') or now)
        if not math.isfinite(finished):
            return None
        return dict(stage='first_motion' if at else stages[-1] if stages else 'accepted',
                    elapsed_s=round(max(0., (at or finished)-started), 1),
                    first_motion_at=at)
    except (OSError, ValueError, KeyError, TypeError):
        return None
