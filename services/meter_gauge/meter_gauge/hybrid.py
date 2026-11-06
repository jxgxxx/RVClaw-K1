"""Bounded cloud assistance after a failed local read. No camera or motion API.

Local consensus stays in the route owner. Only independently captured frames
may corroborate a cloud candidate; model confidence alone never proves a result.
"""
from __future__ import annotations

from datetime import datetime
import io
import json
import math
import os
from pathlib import Path
import queue
from statistics import median
import tempfile
import threading
import time

from . import cloud_meter

_SLOT = threading.BoundedSemaphore(1)
_CIRCUIT_UNTIL = 0.0


def prepare_image(trial, photo_root):
    """Use only current saved scene pixels and a positively located target box."""
    source = Path(trial['evidence']['scene_path']).resolve()
    source.relative_to(Path(photo_root).resolve())
    if not source.is_file() or not 4 <= source.stat().st_size <= cloud_meter.MAX_IMAGE_BYTES:
        raise ValueError('invalid saved scene')
    raw = source.read_bytes()
    if not raw.startswith(b'\xff\xd8\xff'):
        raise ValueError('saved scene is not JPEG')
    box = (trial.get('display_region') or {}).get('bbox') or trial.get('gauge_box')
    if box is None:
        box = (trial.get('reference_registration') or {}).get('bbox')
    metadata = {'scene_path': str(source), 'crop_used': False}
    if box is not None:
        try:
            from PIL import Image
            with Image.open(io.BytesIO(raw)) as image:
                x0, y0, x1, y1 = map(float, box)
                if not all(map(math.isfinite, (x0, y0, x1, y1))) or x1 <= x0 or y1 <= y0:
                    raise ValueError('invalid current target box')
                px, py = max(16, (x1-x0)*.15), max(16, (y1-y0)*.15)
                rect = (max(0, int(x0-px)), max(0, int(y0-py)),
                        min(image.width, int(x1+px)), min(image.height, int(y1+py)))
                if rect[2] <= rect[0] or rect[3] <= rect[1]:
                    raise ValueError('target is outside scene')
                crop = image.crop(rect).convert('RGB')
                buffer = io.BytesIO()
                crop.save(buffer, format='JPEG', quality=92)
                raw = buffer.getvalue()
                metadata.update(crop_used=True, crop_bbox=list(rect), image_size=list(crop.size))
        except ImportError:
            pass  # A missing optional image helper must not disable local reading.
    metadata['bytes'] = len(raw)
    return raw, metadata


def _number(name, default, maximum):
    value = float(os.getenv(name, default))
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError('invalid hybrid time budget')
    return value


def _persist(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                     prefix='.cloud-', delete=False) as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        temporary = stream.name
    os.replace(temporary, path)


class HybridInspection:
    def __init__(self, point, request_id, deadline, accepted_trial, *,
                 clock=time.monotonic, sleeper=time.sleep, wall_clock=time.time,
                 photo_root=None, result_root=None, reader=None, evidence_session=None):
        self.point, self.request_id, self.deadline = point, request_id, deadline
        self.accepted_trial, self.clock, self.sleeper = accepted_trial, clock, sleeper
        self.wall_clock = wall_clock
        self.photo_root = Path(photo_root or os.getenv('RVCLAW_INSPECTION_PHOTO_ROOT', '/data/inspection_photos'))
        self.result_root = Path(result_root or cloud_meter.PROJECT_ROOT / 'runtime/k1/vision/cloud_meter')
        self.reader = reader or cloud_meter.read_jpeg
        self.budget = _number('RVCLAW_METER_HYBRID_BUDGET_S', '30', 45)
        self.final_wait = _number('RVCLAW_METER_HYBRID_FINAL_WAIT_S', '8', 12)
        self.request_timeout = _number('RVCLAW_METER_HYBRID_REQUEST_TIMEOUT_S', '20', 30)
        self.limit = None
        self.closed = threading.Event()
        self.inbox = queue.SimpleQueue()
        self.pending = False
        self.sources = []
        self.used = set()
        self.cloud_trials = []
        self.events = []
        self.attempts = 0
        self.context = None
        self.evidence_session = evidence_session

    def _identity(self, trial):
        evidence = trial.get('evidence') or {}
        identity = (evidence.get('session_id'), evidence.get('waypoint'), trial.get('frame_sequence'))
        if not all(value is not None for value in identity):
            return None
        if (str(identity[1]).upper() != str(self.point.waypoint).upper()
                or (self.evidence_session is not None and identity[0] != self.evidence_session)):
            return None
        if (trial.get('meter_id') != self.point.meter_id
                or trial.get('meter_type') != self.point.meter_type
                or trial.get('camera_mode') != 'astra_raw'):
            return None
        try:
            stamp = datetime.fromisoformat(trial['captured_at']).timestamp()
        except (KeyError, ValueError, TypeError):
            return None
        if not 0 <= self.wall_clock()-stamp <= 60:
            return None
        if self.context is not None and identity[:2] != self.context:
            return None
        return identity

    def observe(self, trial, accepted):
        if self.closed.is_set() or trial.get('status') == 'alignment_failed':
            return
        identity = self._identity(trial)
        if identity is None:
            return
        self.context = identity[:2]
        inconsistent = False
        if accepted and self.limit is None:
            for previous_id, previous in self.sources:
                previous_key = self.accepted_trial(previous)
                if previous_id == identity or previous_key is None:
                    continue
                if self.point.meter_type == 'pointer':
                    inconsistent = self.point.classify_value(previous['value']) != self.point.classify_value(trial['value'])
                else:
                    inconsistent = previous_key != self.accepted_trial(trial)
                if inconsistent:
                    break
        if not any(item[0] == identity for item in self.sources):
            self.sources.append((identity, dict(trial)))
        if (not accepted or inconsistent) and self.limit is None:
            self.limit = min(self.deadline, self.clock()+self.budget)
            self.events.append({'event': 'first_local_failure', 'frame': list(identity),
                                'reason': 'inconsistent_local_values' if inconsistent else 'rejected_local_read'})
            self._start(identity, trial)

    def _start(self, identity, trial):
        global _CIRCUIT_UNTIL
        now = self.clock()
        if (self.closed.is_set() or self.pending or self.limit is None
                or self.attempts >= 2 or self.limit-now < .5 or time.monotonic() < _CIRCUIT_UNTIL):
            return
        slot = _SLOT
        if not slot.acquire(blocking=False):
            self.events.append({'event': 'cloud_slot_busy'})
            return
        self.attempts += 1
        number = self.attempts
        self.pending = True
        self.used.add(identity)
        timeout = min(self.request_timeout, self.limit-now)
        self.events.append({'event': 'cloud_submitted', 'attempt': number, 'frame': list(identity)})
        def work():
            global _CIRCUIT_UNTIL
            try:
                raw, image_info = prepare_image(trial, self.photo_root)
                calibration = {key: value for key, value in dict(self.point.calibration).items()
                               if key in ('min_value', 'max_value', 'unit')}
                if trial.get('unit'):
                    calibration.setdefault('unit', trial['unit'])
                candidate = self.reader(raw, self.point.meter_type, calibration or None, timeout_s=timeout)
                candidate.update(meter_id=self.point.meter_id, meter_type=self.point.meter_type,
                                 frame_sequence=identity[2], captured_at=trial['captured_at'],
                                 camera_mode='astra_raw', evidence=dict(trial['evidence']),
                                 inference_source='cloud', cloud_attempt=number,
                                 cloud_request_id=self.request_id, cloud_input=image_info)
                if candidate.get('status') in ('inference_error', 'inference_offline'):
                    _CIRCUIT_UNTIL = time.monotonic()+30
            except Exception as error:
                candidate = {'status': 'inference_error', 'value': None, 'confidence': 0,
                             'reason': 'cloud assistance unavailable: '+type(error).__name__}
            finally:
                slot.release()
            finished = self.clock()
            late = self.closed.is_set() or finished > self.limit
            try:
                _persist(self.result_root / self.request_id / f'attempt_{number}.json',
                         {'frame': list(identity), 'trial': candidate, 'late': late})
            except (OSError, ValueError, TypeError):
                pass
            self.inbox.put((candidate, finished, late))
        try:
            threading.Thread(target=work, name='meter-cloud-io', daemon=True).start()
        except RuntimeError:
            self.pending = False
            slot.release()
            self.events.append({'event': 'cloud_worker_unavailable'})

    def _valid_cloud(self, trial):
        if self._identity(trial) is None or self.accepted_trial(trial) is None:
            return False
        if float(trial.get('confidence', 0)) < cloud_meter._min_confidence():
            return False
        try:
            value = float(trial['value'])
            calibration = dict(self.point.calibration)
            if (('min_value' in calibration and value < float(calibration['min_value']))
                    or ('max_value' in calibration and value > float(calibration['max_value']))):
                return False
        except (TypeError, ValueError, KeyError):
            return False
        expected = next((item.get('unit') for _, item in self.sources if item.get('unit')), None)
        unit = str(trial.get('unit') or '').replace('℃', '°C').lower()
        if expected and unit and unit != str(expected).replace('℃', '°C').lower():
            return False
        return True

    def _agree(self, first, second):
        if self._identity(first) == self._identity(second):
            return False
        if self.point.meter_type == 'pointer':
            if abs(float(first['value'])-float(second['value'])) > self.point.numeric_tolerance:
                return False
            return self.point.classify_value(first['value']) == self.point.classify_value(second['value'])
        return self.accepted_trial(first) == self.accepted_trial(second)

    def poll(self, local_trials):
        if self.closed.is_set() or self.limit is None or self.clock() >= self.limit:
            return None
        while not self.inbox.empty():
            trial, finished, late = self.inbox.get_nowait()
            self.pending = False
            self.events.append({'event': 'cloud_completed', 'status': trial.get('status'), 'late': late})
            if not late and finished <= self.limit and self._valid_cloud(trial):
                self.cloud_trials.append(trial)
        # A cloud candidate must agree with another independently captured
        # frame, never a local re-read of the very same uploaded JPEG.
        local = [trial for trial in local_trials if self._identity(trial) is not None
                 and self.accepted_trial(trial) is not None]
        for first in self.cloud_trials:
            for second in local + self.cloud_trials:
                if first is second or not self._agree(first, second):
                    continue
                required = max(2, self.point.min_consistent_reads)
                if required != 2:
                    return None  # Current site uses two; never relax another site's contract.
                values = [float(first['value']), float(second['value'])]
                value = median(values) if self.point.meter_type == 'pointer' else second['value']
                condition, anomaly = self.point.classify_value(value)
                reading = dict(second, value=value)
                return {'status': 'PASS', 'task_succeeded': True, 'error': None,
                        'reading': reading, 'value': value, 'unit': second.get('unit') or first.get('unit'),
                        'confidence': min(float(first['confidence']), float(second['confidence'])),
                        'condition': condition, 'is_anomaly': anomaly,
                        'speech_key': 'meter_abnormal' if anomaly else 'meter_normal',
                        'reading_consensus': {'mode': 'hybrid_independent_frames', 'count': 2,
                                              'required': required, 'values': values,
                                              'sources': [item.get('inference_source', 'local') for item in (first, second)],
                                              'frames': [list(self._identity(item)) for item in (first, second)]}}
        if not self.pending and self.cloud_trials and self.attempts == 1:
            other = next(((identity, item) for identity, item in reversed(self.sources) if identity not in self.used), None)
            if other:
                self._start(*other)  # One bounded confirmation, not an unbounded retry loop.
        return None

    def finish(self, local_trials):
        if self.limit is None:
            return None
        stop = min(self.deadline, self.limit, self.clock()+self.final_wait)
        while self.clock() < stop:
            result = self.poll(local_trials)
            if result is not None:
                return result
            if not self.pending:
                break
            self.sleeper(min(.05, max(0, stop-self.clock())))
        return None

    def close(self):
        self.closed.set()
        return {'enabled': True, 'attempts': self.attempts, 'pending_ignored': self.pending,
                'budget_s': self.budget, 'final_wait_s': self.final_wait,
                'events': list(self.events), 'cloud_trials': list(self.cloud_trials)}
