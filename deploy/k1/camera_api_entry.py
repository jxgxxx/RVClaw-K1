#!/usr/bin/env python3
"""Load the unchanged inspection API with OCR models ready before requests."""
import sys
import logging
from pathlib import Path
import cv2
cv2.setNumThreads(2)
from rvclaw_api import server

engine = server._ppocr_engine()
if engine is None:
    raise RuntimeError('K1 in-process OCR engine is unavailable')
for profile in (server.PPOCR_CONFIG, server.PPOCR_UPLOAD_CONFIG):
    engine.warmup(str(profile))
from rvclaw_api.meter_reference import prepare_display_reference
for record in server.METER_STORE.list():
    if record.get('display_region') and record.get('reference_image') and not record.get('pointer_scale'):
        try:
            reference = cv2.imread(str(server.METER_STORE.reference_image_dir / record['reference_image']))
            if reference is not None:
                prepare_display_reference(reference, record['display_region'])
        except (ValueError, cv2.error):
            logging.exception('Reference prewarm refused for %s', record.get('meter_id'))
print('K1 inspection OCR models ready', flush=True)
try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot/ros2_ws/src/rvclaw_perception'))
    from rgb_assist.api import install as install_rgb_assist
    install_rgb_assist(server)
except Exception:
    logging.exception('Optional RGB observation extension unavailable')
if sys.argv[1:] != ['--prepare-only']:
    server.main()
