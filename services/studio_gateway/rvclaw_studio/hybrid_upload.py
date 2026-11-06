"""Local-first web uploads using the robot's existing bounded cloud client.

No camera/ROS/model ownership. Provider credentials stay in the board-local
configuration. Physical inspections keep their independent multi-frame policy.
"""
from __future__ import annotations

import base64
import http.client
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


# Manual uploads have a different deadline from a moving inspection task.
# The configured provider sometimes needs over 20 seconds even for valid input.
SOCKET_TIMEOUT = 45
WALL_TIMEOUT = 55
MIN_CONFIDENCE = .70
MAX_IMAGE_BYTES = 10 * 1024 * 1024
MAX_OUTPUT_BYTES = 1024 * 1024
ACCEPTED = {'stable', 'recognized', 'ok', 'success', 'normal', 'abnormal', 'out_of_range'}


def accepted(value):
    if not isinstance(value, dict) or value.get('status') not in ACCEPTED:
        return False
    number = value.get('value')
    if isinstance(number, bool):
        return False
    try:
        confidence = value.get('confidence')
        return (math.isfinite(float(number)) and
                (confidence is None or MIN_CONFIDENCE <= float(confidence) <= 1))
    except (ValueError, TypeError, OverflowError):
        return False


def summary(value):
    """Keep only bounded, diagnostic fields; never store request/credential data."""
    result = {}
    for key in ('status', 'value', 'raw_text', 'confidence', 'model', 'reason', 'elapsed_ms'):
        field = value.get(key)
        if isinstance(field, str):
            result[key] = field[:500]
        elif field is None or isinstance(field, (int, float)) and not isinstance(field, bool) and math.isfinite(field):
            result[key] = field
    return result


def unavailable(reason='云端辅助暂时不可用，请稍后重试。'):
    return {'status': 'inference_error', 'value': None, 'confidence': 0., 'reason': reason}


def failure_message(cloud):
    """Explain provider failures without returning raw transport errors."""
    reason = str(cloud.get('reason') or '')
    if '小车' in reason:
        return '小车已开始执行任务，本次网页云端辅助已停止。'
    if any(term in reason.lower() for term in ('timeout', 'timed out', '超时')):
        return '本地未能可靠读数，云端识别等待超时，请稍后重试。'
    if 'HTTP 401' in reason or 'HTTP 403' in reason:
        return '本地未能可靠读数，云端服务授权校验失败，请检查服务配置。'
    if 'HTTP 429' in reason:
        return '本地未能可靠读数，云端服务繁忙，请稍后重试。'
    if cloud.get('status') in {'inference_error', 'inference_offline'}:
        return '本地未能可靠读数，云端连接失败，请稍后重试。'
    return '本地和云端均未能给出可信读数，请更换清晰图片重试。'


class HybridUploadReader:
    def __init__(self, root, local_read, persist, task_active, *, cloud_read=None):
        self.root = Path(root)
        self.local_read, self.persist, self.task_active = local_read, persist, task_active
        self.cloud_read = cloud_read or self._cloud_read

    @staticmethod
    def _input(payload):
        if payload.get('meter_type') not in {'digital', 'pointer'}:
            raise ValueError('请选择有效的仪表类型。')
        encoded = payload.get('image')
        if not isinstance(encoded, str) or not encoded:
            raise ValueError('请先上传仪表图片。')
        try:
            raw = base64.b64decode(encoded.split(',', 1)[-1], validate=True)
        except ValueError:
            raise ValueError('图片编码无效。') from None
        if not 0 < len(raw) <= MAX_IMAGE_BYTES:
            raise ValueError('图片不能超过 10 MB。')
        # Decode once before any inference, including when a local service fails.
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.size == 0:
            raise ValueError('无法读取图片，请选择有效的仪表图片。')
        if image.shape[0] * image.shape[1] > 24_000_000:
            raise ValueError('图片尺寸过大，请缩小后重试。')
        return raw, image

    def read(self, payload):
        raw, image = self._input(payload)
        started = time.monotonic()
        try:
            local = self.local_read(payload)
        except (OSError, ValueError, http.client.HTTPException):
            local = unavailable('本地识别暂时不可用。')
        if not isinstance(local, dict):
            local = unavailable('本地识别未返回有效结果。')
        if accepted(local):
            return local

        if self.task_active():
            cloud = unavailable('小车已开始执行任务，本次网页云端辅助已停止。')
            attempted = False
        else:
            import cv2
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise ValueError('图片编码失败，请重新选择。')
            cloud_started = time.monotonic()
            try:
                cloud = self.cloud_read(jpeg.tobytes(), payload['meter_type'])
            except TimeoutError:
                cloud = unavailable('云端识别等待超时。')
            except (OSError, ValueError, subprocess.SubprocessError):
                cloud = unavailable()
            if not isinstance(cloud, dict):
                cloud = unavailable()
            cloud = {**cloud, 'elapsed_ms': round((time.monotonic() - cloud_started) * 1000, 1)}
            attempted = True

        # The shared client already enforces confidence. Check again at the
        # website boundary; a malformed candidate must never become a reading.
        cloud_ok = accepted(cloud) and cloud.get('confidence') is not None
        if cloud_ok:
            value = {key: cloud[key] for key in ('status', 'value', 'raw_text', 'confidence', 'model', 'value_method', 'angle_deg') if key in cloud}
            value.update(reason='', reason_zh='')
        else:
            reason = failure_message(cloud)
            if not attempted:
                reason = cloud['reason']
            value = {'status': local.get('status') if local.get('status') not in ACCEPTED else 'unreadable',
                     'value': None, 'confidence': 0., 'reason': reason, 'reason_zh': reason,
                     'model': str(local.get('model') or 'Local + cloud assistance')[:200]}
        value.update(recognition_policy='local_then_cloud',
                     inference_source='cloud' if cloud_ok else 'local',
                     local_result=summary(local),
                     cloud_assistance={'attempted': attempted, **summary(cloud)})
        previous = local.get('evidence_id')
        if previous:
            value['supersedes_evidence_id'] = previous
        return self.persist(value, payload, raw, started)

    def _cloud_read(self, jpeg, meter_type):
        helper = self.root / 'services/meter_gauge/run_cloud_meter.py'
        config = self.root / 'configs/device/meter/cloud_meter.env'
        if not helper.is_file() or not config.is_file():
            return unavailable()
        env = os.environ.copy()
        env.update(RVCLAW_METER_CLOUD_TIMEOUT=str(SOCKET_TIMEOUT),
                   OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
        with tempfile.TemporaryDirectory(prefix='rvclaw-web-meter-') as directory:
            source = Path(directory) / 'upload.jpg'
            source.write_bytes(jpeg)
            command = [sys.executable, str(helper), '--config', str(config),
                       '--image', str(source), '--meter-type', meter_type]
            process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, env=env)
            deadline = time.monotonic() + WALL_TIMEOUT
            try:
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return unavailable('云端识别等待超时。')
                    if self.task_active():
                        return unavailable('小车已开始执行任务，本次网页云端辅助已停止。')
                    try:
                        output, _ = process.communicate(timeout=min(.5, remaining))
                        break
                    except subprocess.TimeoutExpired:
                        continue
                if len(output) > MAX_OUTPUT_BYTES:
                    return unavailable()
                candidate = json.loads(output)
                return candidate if isinstance(candidate, dict) else unavailable()
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.communicate(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.communicate()
