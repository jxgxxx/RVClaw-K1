"""Bounded extraction of actual saved map poses, independent of UI polling."""
import hashlib
import json
import math
import os
import re
import tempfile
from functools import lru_cache
from pathlib import Path

EVENT = re.compile(r'(?<!\\)"name"\s*:\s*"route_pose_observation"\s*,\s*"value"\s*:\s*')
DECODER = json.JSONDecoder()
MAX_POINTS = 4096
MAX_EVENT = 262144


def _extract(path):
    points = {}; buffer = ''; stride = 1
    with path.open(encoding='utf-8') as stream:
        while True:
            chunk = stream.read(65536); buffer += chunk; consumed = 0; pending = None
            for match in EVENT.finditer(buffer):
                try:
                    value, end = DECODER.raw_decode(buffer, match.end())
                except json.JSONDecodeError:
                    if chunk and len(buffer)-match.start() < MAX_EVENT:
                        pending = match.start(); break
                    consumed = match.end(); continue
                consumed = end
                pose = value.get('pose') if isinstance(value, dict) else None
                stamp = value.get('pose_stamp_ns') if isinstance(value, dict) else None
                if (not isinstance(pose, dict) or not isinstance(stamp, (int, float))
                        or isinstance(stamp, bool) or not math.isfinite(stamp) or stamp <= 0
                        or not all(isinstance(pose.get(k), (int, float))
                                   and not isinstance(pose[k], bool) and math.isfinite(pose[k])
                                   for k in ('x', 'y', 'yaw'))):
                    continue
                # Uniform time buckets bound both memory and response size.
                # Retaining gaps is preferable to drawing a fictional segment.
                bucket = int(stamp/1e9/stride)
                points.setdefault(bucket, dict(x=pose['x'], y=pose['y'], yaw=pose['yaw'],
                                               sampled_at=stamp/1e9))
                if len(points) > MAX_POINTS:
                    stride *= 2
                    reduced = {}
                    for p in sorted(points.values(), key=lambda p:p['sampled_at']):
                        reduced.setdefault(int(p['sampled_at']/stride), p)
                    points = reduced
            if not chunk: break
            buffer = buffer[pending:] if pending is not None else buffer[max(consumed,len(buffer)-128):]
    result = sorted(points.values(), key=lambda p:p['sampled_at'])
    previous = None
    for p in result:
        dt = p['sampled_at']-previous['sampled_at'] if previous else 0
        p['breakBefore'] = (previous is None or dt > 4.0 or dt <= 0
            or math.hypot(p['x']-previous['x'],p['y']-previous['y']) > dt+0.15)
        previous = p
    return result


@lru_cache(maxsize=64)
def _read(source, modified, size, cache_root):
    path = Path(source)
    key = dict(source=source, mtime_ns=modified, size=size, version=1)
    cache = Path(cache_root)/(hashlib.sha256(source.encode()).hexdigest()+'.trajectory.json') if cache_root else None
    if cache:
        try:
            saved = json.loads(cache.read_text(encoding='utf-8'))
            if saved.get('key') == key and isinstance(saved.get('points'), list):
                return saved['points']
        except (OSError, ValueError): pass
    points = _extract(path)
    if cache:
        temporary = None
        try:
            cache.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=cache.parent,
                    prefix='.trajectory-', delete=False) as stream:
                temporary = stream.name
                json.dump(dict(key=key, points=points), stream)
            os.replace(temporary, cache)
        except OSError: pass
        finally:
            if temporary: Path(temporary).unlink(missing_ok=True)
    return points


def read_trace_trajectory(path):
    path = Path(path).resolve(); stat = path.stat()
    cache = os.getenv('RVCLAW_TRACE_CACHE_ROOT', '')
    if not cache and path.is_relative_to(Path('/data')):
        cache = '/data/RVClaw/runtime/inspection-progress/trace-cache'
    return _read(str(path),stat.st_mtime_ns,stat.st_size,cache)
