"""Safety recognition: existing local model, fresh-frame evidence and bounded cloud jobs.

Cloud points are independent map-coordinate regions. Never uses next navigation
target as arrival evidence, changes a route, or sends motion commands.
"""
from __future__ import annotations
import base64
import io
from datetime import datetime, timezone
import json
import logging
import math
from pathlib import Path
import re
import sqlite3
import subprocess
import sys
import threading
import time
import warnings
from uuid import uuid4
from .safety_cloud import GEAR, settings

LOG = logging.getLogger('safety')
INTERVAL = 120


def iso(): return datetime.now(timezone.utc).isoformat()


def upload_image(payload):
    """Validate real pixels, bound decoding and strip metadata before inference."""
    from PIL import Image, ImageOps
    encoded = payload.get('image')
    if not isinstance(encoded, str) or not encoded or len(encoded) > 14 * 1024 * 1024:
        raise ValueError('请选择不超过 10 兆字节的图片。')
    try:
        raw = base64.b64decode(encoded.split(',', 1)[-1], validate=True)
        if not 4 <= len(raw) <= 10 * 1024 * 1024: raise ValueError()
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            with Image.open(io.BytesIO(raw)) as original:
                if original.format not in {'JPEG', 'PNG', 'WEBP'} or original.width * original.height > 20_000_000 or getattr(original, 'n_frames', 1) != 1:
                    raise ValueError()
                image = ImageOps.exif_transpose(original)
                image.thumbnail((1920, 1920), Image.Resampling.LANCZOS)
                if image.mode != 'RGB':
                    if 'A' in image.getbands():
                        background = Image.new('RGB', image.size, 'white')
                        background.paste(image, mask=image.getchannel('A'))
                        image = background
                    else: image = image.convert('RGB')
                output = io.BytesIO()
                image.save(output, 'JPEG', quality=90)
                return output.getvalue()
    except Exception:
        raise ValueError('图片无法读取，请选择清晰的静态图片，像素总数不超过 2000 万。') from None


class SafetyService:
    def __init__(self, root, camera_status, frame, context, switch):
        self.robot = Path(root)
        self.root = self.robot / 'state/studio/safety'
        self.root.mkdir(parents=True, exist_ok=True)
        self.camera_status, self.frame, self.context, self.switch = camera_status, frame, context, switch
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.job = None
        self.local_at = 0
        self.local_signature = None
        self.point = None
        self.candidate = None
        self.candidate_at = 0
        self.camera = {}
        self.error = ''
        self.provider_ready = False
        self.provider_check = 0
        self.config = self._read('config.json', {'cloud_enabled': False, 'required': ['helmet'], 'points': []})
        self.last_cloud_at = self._read('schedule.json', {}).get('last_cloud_at', 0)
        self.db = sqlite3.connect(self.root / 'records.sqlite3', check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS records (id TEXT PRIMARY KEY, created_at TEXT, body TEXT NOT NULL)')
        # A restart cannot turn interrupted inference into a passing inspection.
        for identifier, body in self.db.execute('SELECT id,body FROM records').fetchall():
            row = json.loads(body)
            if row.get('status') == 'pending':
                row.update(status='error', reason='服务重启中断了本次识别。')
                self.db.execute('UPDATE records SET body=? WHERE id=?', (json.dumps(row, ensure_ascii=False), identifier))
        self.db.commit()

    def _read(self, name, default):
        try: return json.loads((self.root / name).read_text(encoding='utf-8'))
        except (OSError, ValueError): return default

    def _write(self, name, value):
        target = self.root / name
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        temp.replace(target)

    def configure(self, payload):
        required, points = payload.get('required'), payload.get('points')
        if not isinstance(payload.get('cloud_enabled'), bool) or not isinstance(required, list) or not required or any(not isinstance(k, str) or k not in GEAR for k in required) or len(required) != len(set(required)):
            raise ValueError('请至少选择一项有效的防护装备。')
        if not isinstance(points, list) or len(points) > 50: raise ValueError('安全防护点位最多设置 50 个。')
        cleaned, identifiers = [], set()
        for point in points:
            if not isinstance(point, dict): raise ValueError('安全防护点位无效。')
            identifier, name, map_id = point.get('id'), point.get('name'), point.get('map_id')
            if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', identifier) or identifier.upper() in {'A', 'B', 'C', 'ORIGIN'} or identifier in identifiers:
                raise ValueError('点位标识需唯一，且不能使用已有仪表点位 A、B、C 或返航点。')
            if not isinstance(name, str) or not name.strip() or len(name) > 60 or any(ord(c) < 32 for c in name): raise ValueError('请填写有效的点位名称。')
            if not isinstance(map_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', map_id): raise ValueError('请选择点位所属地图。')
            for key, low, high in [('x', -10000, 10000), ('y', -10000, 10000), ('radius', .2, 5)]:
                value = point.get(key)
                if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high: raise ValueError('点位坐标或触发半径无效。')
            if not isinstance(point.get('enabled'), bool): raise ValueError('点位启用状态无效。')
            identifiers.add(identifier)
            cleaned.append({k: point[k] for k in ('id', 'name', 'map_id', 'x', 'y', 'radius', 'enabled')})
        with self.lock:
            config = {'cloud_enabled': payload['cloud_enabled'], 'required': required, 'points': cleaned}
            self._write('config.json', config)
            self.config = config
            self.point = self.candidate = None
            return self.status()

    def _record(self, row, image=None):
        with self.lock:
            image_path = self.root / (row['id'] + '.jpg')
            if image is not None:
                image_path.with_suffix('.tmp').write_bytes(image)
                image_path.with_suffix('.tmp').replace(image_path)
                row['image_url'] = '/api/safety/evidence/' + row['id'] + '.jpg'
            if row.get('status') in {'compliant', 'no_people'} and row.get('source') != 'upload':
                row.pop('image_url', None)
                image_path.unlink(missing_ok=True)
            self.db.execute('INSERT OR REPLACE INTO records VALUES (?,?,?)', (row['id'], row['created_at'], json.dumps(row, ensure_ascii=False, allow_nan=False)))
            self.db.commit()
            return row

    def records(self, limit=1000):
        with self.lock:
            return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM records ORDER BY created_at DESC LIMIT ?', (limit,))]

    def status(self):
        with self.lock:
            camera = self.camera
            item = (camera.get('cameras') or [{}])[0]
            inf = item.get('inference') or {}
            active = camera.get('active_mode') == 'astra_hardhat'
            age = item.get('frame_age_ms')
            fresh = item.get('online') and not camera.get('switching') and type(age) in (int, float) and age < 2000
            return {'config': json.loads(json.dumps(self.config)), 'interval_seconds': INTERVAL,
                    'local': {'active': active, 'ready': bool(active and fresh and inf.get('ready') and inf.get('ppe_ready')),
                              'counts': {k: int(inf.get(k) or 0) if active and fresh else 0 for k in ('persons', 'helmet', 'no_helmet', 'checking')}},
                    'camera': camera, 'cloud': {'ready': self.provider_ready, 'busy': self.job is not None,
                        'point': self.point, 'next_capture_at': max(time.time(), self.last_cloud_at + INTERVAL) if self.point and self.config['cloud_enabled'] else None,
                        'last_capture_at': self.last_cloud_at or None}, 'error': self.error}

    def local_control(self, enabled):
        self.switch('astra_hardhat' if enabled else 'astra_raw', enabled)
        with self.lock: self.camera = self.camera_status()
        return self.status()

    def _located(self, now):
        context = self.context()
        pose = context.get('pose')
        if not context.get('stationary') or not pose or type(pose.get('sampled_at')) not in (int, float) or not 0 <= now - pose['sampled_at'] <= 4:
            self.candidate = None
            self.point = None
            return None
        eligible = [p for p in self.config['points'] if p['enabled'] and p['map_id'] == pose.get('map_id') and
                    math.hypot(p['x'] - pose['x'], p['y'] - pose['y']) <= p['radius'] + (.25 if self.point and self.point['id'] == p['id'] else 0)]
        point = min(eligible, key=lambda p: math.hypot(p['x'] - pose['x'], p['y'] - pose['y']), default=None)
        key = (pose.get('task_id'), pose.get('map_id'), point['id']) if point else None
        if key != self.candidate:
            self.candidate, self.candidate_at, self.point = key, now, None
        if key and now - self.candidate_at >= 2:
            self.point = {**point, 'task_id': pose.get('task_id')}
        return self.point

    def _capture(self):
        value = self.frame()
        image = base64.b64decode(value['image'], validate=True)
        if not image.startswith(b'\xff\xd8\xff') or not 4 <= len(image) <= 10 * 1024 * 1024: raise ValueError('相机照片无效。')
        if not 0 <= time.time() - value['captured_at'] <= 4: raise ValueError('相机照片已过期。')
        return image, value

    def _local(self, now, camera):
        item = (camera.get('cameras') or [{}])[0]
        inf = item.get('inference') or {}
        if camera.get('active_mode') != 'astra_hardhat' or camera.get('switching') or not item.get('online') or not inf.get('ready') or not inf.get('ppe_ready'):
            self.local_signature = None
            return
        counts = {k: int(inf.get(k) or 0) for k in ('persons', 'helmet', 'no_helmet', 'checking')}
        if not counts['persons']: return
        signature = tuple(counts.values())
        first_violation = counts['no_helmet'] > 0 and (self.local_signature is None or self.local_signature[2] == 0)
        if (now - self.local_at < 30 and not first_violation) or (signature == self.local_signature and now - self.local_at < INTERVAL): return
        image, frame = self._capture()
        if frame['mode'] != 'astra_hardhat' or not frame.get('inference', {}).get('ppe_ready'): return
        counts = {k: int(frame.get('inference', {}).get(k) or 0) for k in counts}
        if not counts['persons']: return
        status = 'violation' if counts['no_helmet'] else 'unknown' if counts['checking'] or counts['helmet'] < counts['persons'] else 'compliant'
        self._record({'id': uuid4().hex, 'created_at': iso(), 'captured_at': frame['captured_at'], 'mode': 'local', 'status': status,
                      'required': ['helmet'], 'missing': ['helmet'] if counts['no_helmet'] else [], 'counts': counts,
                      'persons': [], 'point': None, 'reason': '头部可见性不足或检测尚未完成。' if status == 'unknown' else ''}, image)
        self.local_at, self.local_signature = now, tuple(counts.values())

    def request_cloud(self, manual=False, image=None):
        with self.lock:
            now = time.time()
            if self.job: raise ValueError('云端识别正在进行，请等待本次完成。')
            if not self.provider_ready: raise ValueError('云端识别服务尚未配置就绪。')
            if now < self.last_cloud_at + INTERVAL: raise ValueError('距离上次采集不足两分钟，请稍后再试。')
            if not manual and image is None and (not self.config['cloud_enabled'] or not self.point): return
            if image is None:
                camera = self.camera_status()
                item = (camera.get('cameras') or [{}])[0]
                if camera.get('switching') or not item.get('online'):
                    raise ValueError('相机正在切换或画面尚未就绪，请稍后拍照。')
            # Reserve globally before capture; also survives restart and manual requests.
            self._write('schedule.json', {'last_cloud_at': now})
            self.last_cloud_at = now
            row = {'id': uuid4().hex, 'created_at': iso(), 'mode': 'cloud', 'status': 'pending',
                   'required': list(self.config['required']), 'point': dict(self.point) if self.point and not manual and image is None else None,
                   'source': 'upload' if image is not None else 'manual' if manual else 'arrival', 'missing': [], 'persons': [], 'reason': ''}
            self._record(row)
            self.job = row['id']
            threading.Thread(target=self._cloud_job, args=(row,image), daemon=True, name='safety-cloud').start()
            return dict(row)

    def _cloud_job(self, row, image=None):
        try:
            if image is None:
                image, frame = self._capture()
                if frame['mode'] not in {'astra_raw', 'astra_hardhat', 'c70_general', 'astra_meter', 'astra_meter_digital', 'astra_meter_pointer'}:
                    raise ValueError('相机正在进行仪表检测，请稍后再试。')
                row['captured_at'] = frame['captured_at']
                with self.lock:
                    if row['source'] == 'arrival':
                        current = self._located(time.time())
                        if not self.config['cloud_enabled'] or not current or any(current.get(k) != row['point'].get(k) for k in ('id', 'map_id', 'task_id')):
                            raise ValueError('已离开检测点或定位失效，本次采集已取消。')
                    self.last_cloud_at = max(self.last_cloud_at, frame['captured_at'])
                    self._write('schedule.json', {'last_cloud_at': self.last_cloud_at})
            else: row['uploaded_at'] = row['created_at']
            self._record(row, image)
            run = subprocess.run([sys.executable, '-m', 'rvclaw_studio.safety_cloud', str(self.robot), str(self.root / (row['id'] + '.jpg')), json.dumps(row['required'])],
                                 capture_output=True, text=True, timeout=65)
            result = json.loads(run.stdout) if run.returncode == 0 else None
            if not isinstance(result, dict) or result.get('status') not in {'compliant', 'violation', 'unknown', 'no_people', 'error'}:
                raise ValueError('云端识别未完成，请稍后再试。')
            row.update(result)
        except Exception as error:
            row.update(status='error', reason=str(error) if isinstance(error, ValueError) else '照片采集失败或云端服务暂时不可用，请稍后重试。')
        finally:
            row['finished_at'] = iso()
            try: self._record(row)
            finally:
                with self.lock: self.job = None

    def tick(self):
        now = time.time()
        camera = self.camera_status()
        with self.lock:
            self.camera = camera
            self._located(now)
            if now - self.provider_check > 30:
                self.provider_ready, self.provider_check = settings(self.robot)['ready'], now
            item = (camera.get('cameras') or [{}])[0]
            due = self.config['cloud_enabled'] and self.point and not self.job and self.provider_ready and now >= self.last_cloud_at + INTERVAL and item.get('online') and not camera.get('switching')
        if due: self.request_cloud()
        self._local(now, camera)
        with self.lock: self.error = ''

    def _run(self):
        while not self.stop_event.is_set():
            try: self.tick()
            except Exception:
                with self.lock:
                    self.error = '安全识别状态暂时不可用，正在重连。'
                    self.camera = {}
                    self.point = None
                LOG.warning('Safety observation unavailable')
            self.stop_event.wait(2)

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True, name='safety-observer')
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread: self.thread.join(timeout=5)

    def events(self):
        result = []
        for row in self.records():
            if row['status'] != 'violation': continue
            missing = '、'.join(GEAR[k] for k in row['missing'])
            result.append({'id': row['id'], 'title': '发现人员未佩戴' + missing, 'type': '安全告警', 'time': row['created_at'],
                           'image': row.get('image_url'), 'alert': True, 'demo': False, 'description': '未佩戴：' + missing + '。请查看现场照片并复核。',
                           'value': missing, 'source_label': '图片上传' if row.get('source') == 'upload' else '本地识别' if row['mode'] == 'local' else '云端识别',
                           'category': 'safety', 'result_state': 'failure', 'related': {'page': 'safety', 'target': {'record': row['id']}}})
        return result
