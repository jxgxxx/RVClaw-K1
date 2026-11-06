"""Order existing local readers without owning cameras or inference models."""
import base64
import http.client
import re
import time

import cv2
import numpy as np

from .hybrid_upload import HybridUploadReader, accepted, summary, unavailable


def has_emissive_row(image):
    """A routing hint only: coloured segments on a dark instrument window.

    Whole-photo display detection needs surrounding context. A registered tight
    crop can make that detector treat the digits as an implausibly large row.
    Keep its full context for LED displays. OCR still makes every value decision.
    """
    scale = min(1., 720 / max(image.shape[:2]))
    if scale < 1:
        image = cv2.resize(image, None, fx=scale, fy=scale)
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    masks = [cv2.bitwise_or(cv2.inRange(hsv, (0, 90, 80), (18, 255, 255)),
                           cv2.inRange(hsv, (165, 90, 80), (179, 255, 255))),
             cv2.inRange(hsv, (30, 80, 70), (100, 255, 255)),
             cv2.inRange(hsv, (100, 80, 70), (145, 255, 255))]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                      (max(9, int(width * .09)) | 1,
                                       max(3, int(height * .006)) | 1))
    for mask in masks:
        joined = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, _, stats, _ = cv2.connectedComponentsWithStats(joined, 8)
        for x, y, w, h, area in stats[1:count]:
            if not (w >= width * .12 and height * .025 <= h <= height * .30
                    and 1.5 <= w / max(1, h) <= 10):
                continue
            if float(np.median(gray[y:y+h, x:x+w])) > 120:
                continue
            n, _, parts, _ = cv2.connectedComponentsWithStats(mask[y:y+h, x:x+w], 8)
            if sum(p[cv2.CC_STAT_AREA] >= 8 and p[cv2.CC_STAT_HEIGHT] >= h * .18
                   for p in parts[1:n]) >= 3:
                return True
    return False


class LocalUploadReader:
    def __init__(self, registered, full_photo, task_active):
        self.registered, self.full_photo = registered, full_photo
        self.task_active = task_active

    def read(self, payload):
        # Pointer validation and geometry retain their established behaviour.
        if payload.get('meter_type') != 'digital':
            value = self.registered.read(payload)
            return value if value is not None else self.full_photo(payload)

        started = time.monotonic()
        raw, image = HybridUploadReader._input(payload)
        if raw.startswith(b'\xff\xd8\xff'):
            jpeg, preprocessing = raw, 'jpeg_original'
        else:
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise ValueError('图片编码失败，请重新选择。')
            jpeg, preprocessing = encoded.tobytes(), 'jpeg92'
        normalized = {**payload, 'image': base64.b64encode(jpeg).decode()}
        first = has_emissive_row(image)
        paths = [('full_photo', self.full_photo), ('registered_display', self.registered.read)]
        if not first:
            paths.reverse()
        attempts, evidence = [], set()
        value = unavailable('本地未能可靠读取仪表。')
        for path, reader in paths:
            if self.task_active():
                value = unavailable('小车已开始执行任务，本次网页识别已停止。')
                break
            began = time.monotonic()
            try:
                candidate = reader(normalized)
            except (OSError, ValueError, http.client.HTTPException, cv2.error):
                candidate = unavailable('本地识别暂时不可用。')
            if not isinstance(candidate, dict):
                continue
            attempts.append({'path': path, **summary(candidate),
                             'elapsed_ms': round((time.monotonic() - began) * 1000, 1)})
            for identifier in [candidate.get('evidence_id'), candidate.get('supersedes_evidence_id'),
                               *(candidate.get('supersedes_evidence_ids') or [])]:
                if isinstance(identifier, str) and re.fullmatch('[a-f0-9]{32}', identifier):
                    evidence.add(identifier)
            value = candidate
            if accepted(value):
                break
        if not accepted(value):
            value = {**value, 'value': None, 'raw_text': None, 'confidence': 0.}
        value = {**value, 'inference_source': 'local', 'local_preprocessing': preprocessing,
                 'local_attempts': attempts, 'supersedes_evidence_ids': sorted(evidence)}
        if evidence:
            value['supersedes_evidence_id'] = sorted(evidence)[0]
        return self.registered.persist(value, payload, raw, started)
