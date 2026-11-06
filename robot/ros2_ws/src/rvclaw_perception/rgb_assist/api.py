"""Add local RGB observation endpoints without changing camera ownership."""
import json
import os
from pathlib import Path
import time
import uuid
from urllib.parse import urlparse
from .observation import trial_observation


def state_dir():
    return Path(os.environ.get('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}'))/'rvclaw-rgb-assist'


def read_status():
    try:
        payload=json.loads((state_dir()/'observation.json').read_text())
        age=time.time()-float(payload['captured_at'])
        if 0<=age<=3.0:
            return payload
    except (OSError,ValueError,KeyError,TypeError): pass
    return dict(version=1,captured_at=time.time(),status='unavailable',
                reason='no_fresh_rgb_observation',motion_authorized=False)


def install(server):
    """Called once from the K1 entrypoint; no changes to shared Studio server."""
    original=server._read_configured_meter_frame
    def read_with_observation(image,meter_type,meter_id,calibration=None):
        result=original(image,meter_type,meter_id,calibration=calibration)
        try:
            observation=trial_observation(result,image.shape)
            observation.update(version=1,status='observed',timestamp_source='parent_trial.captured_at',
                               expected_meter_id=meter_id,source='same_inspection_image')
            result=dict(result,rgb_assist=observation)
        except Exception:
            # Optional evidence must never discard a successful reading.
            server.logging.exception('RGB trial observation unavailable') if hasattr(server,'logging') else None
        return result
    server._read_configured_meter_frame=read_with_observation
    parent=server.RVClawRequestHandler
    generation=uuid.uuid4().hex
    class Handler(parent):
        def do_GET(self):
            path=urlparse(self.path).path
            if path not in ('/api/rgb-assist/frame','/api/rgb-assist/status'):
                return super().do_GET()
            if self.client_address[0] not in ('127.0.0.1','::1'):
                self.send_error(403); return
            if path.endswith('/status'):
                self._write_json(read_status()); return
            if server.METER_MUTATION_LOCK.locked():
                self._write_json({'error':'inspection_busy'},status=503); return
            camera=server.CAMERA_CONTROLLER.active_camera('astra',required_mode='astra_raw')
            if camera is None:
                self._write_json({'error':'raw_camera_unavailable'},status=503); return
            # A small atomic copy of the existing JPEG; never wait for hardware,
            # start VideoCapture, change mode, or trigger inference here.
            with camera._condition:
                frame=camera._frame; stamp=camera._last_frame_at; sequence=camera._sequence
            if frame is None or stamp is None or not 0<=time.time()-stamp<=.4:
                self._write_json({'error':'stale_frame'},status=503); return
            self.connection.settimeout(.5)
            self.send_response(200)
            self.send_header('Content-Type','image/jpeg')
            self.send_header('Content-Length',str(len(frame)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-RGB-Captured-At',str(stamp))
            self.send_header('X-RGB-Sequence',str(sequence))
            self.send_header('X-RGB-Generation',generation)
            self.end_headers(); self.wfile.write(frame)
    server.RVClawRequestHandler=Handler
