"""Studio HTTP boundary. Reuses the robot's single existing vision process.

No model loading, camera ownership, ROS commands, or motion on startup.
"""
from __future__ import annotations
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import base64
import http.client
import json
import logging
import mimetypes
import os
from pathlib import Path
import re
import socket
import sqlite3
import subprocess
import threading
import time
from urllib.parse import parse_qs, unquote, urlsplit

from . import inspection_assets, inspection_maps
from .startup_progress import startup_status
from .live_progress import live_pose
from .health_monitor import HealthMonitor
from .helmet_evidence import HelmetEvidenceStore
from .meter_config import MeterConfigStore
from .studio_data import StudioData, Telemetry
from .upload_reader import UploadReader
from .local_upload import LocalUploadReader
from .hybrid_upload import HybridUploadReader
from .safety import SafetyService, upload_image

LOG = logging.getLogger('studio-gateway')
UPLOAD_LIMIT = 16 * 1024 * 1024
JSON_GETS = {'/api/cameras','/api/health','/api/meter-upload-records','/api/helmet-evidence'}
VISION_POSTS = {'/api/camera-mode','/api/meter-image-read','/api/meter-current-read'}
RECORD_IMAGE = re.compile(r'^/api/(meter-upload-records|helmet-evidence)/[a-f0-9]{32}/(image|crop|original)\.jpg$')
TRIAL_IMAGE = re.compile(r'^/api/meter-trials/[a-f0-9]{32}/scene\.jpg$')


def read_json(path, default=None):
    try: return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError): return {} if default is None else default


def robot_task_active(progress,task_db):
    if read_json(progress).get('state') in {'running','active','in_progress','localizing','finishing'}:return True
    if not task_db or not Path(task_db).exists():return False
    try:
        with sqlite3.connect(Path(task_db).as_uri()+'?mode=ro',uri=True,timeout=.2) as db:
            return db.execute("SELECT 1 FROM tasks WHERE state IN ('created','accepted','running','active','in_progress') LIMIT 1").fetchone() is not None
    except (OSError,sqlite3.Error):return True


class Application:
    def __init__(self):
        self.robot = Path(os.getenv('RVCLAW_ROBOT_ROOT','/data/RVClaw'))
        self.repo = Path(os.environ['RVCLAW_VISION_REPO_ROOT'])
        self.web = Path(os.environ['RVCLAW_STUDIO_ROOT']).resolve()
        self.photos = Path(os.getenv('RVCLAW_INSPECTION_PHOTO_ROOT','/data/inspection_photos')).resolve()
        self.progress = Path(os.environ['RVCLAW_INSPECTION_PROGRESS_OUT'])
        self.upstream = urlsplit(os.getenv('RVCLAW_VISION_UPSTREAM','http://127.0.0.1:8017'))
        assert self.upstream.hostname == '127.0.0.1' and self.upstream.port == 8017
        self.origins = set(os.environ['RVCLAW_STUDIO_ORIGINS'].split(','))
        self.meters = MeterConfigStore(self.repo/'configs/meter/meters.json',self.repo/'configs/meter/reference-images')
        self.telemetry = Telemetry()
        self.health = HealthMonitor()
        self.data = StudioData(self.repo,self.photos,self.progress.parent,self.uploads,self.logs,HelmetEvidenceStore(self.repo/'runs/helmet-evidence'))
        self.snapshot_lock = threading.Lock()
        self.snapshot_at, self.cached_snapshot = 0, None
        self.inference_lock = threading.Lock()
        self.upload_reader=UploadReader(self.repo,self.meters,self.read_upload)
        self.local_upload_reader=LocalUploadReader(self.upload_reader,self.read_upload,
            lambda: robot_task_active(self.progress,os.getenv('RVCLAW_TASK_DB')))
        self.hybrid_upload=HybridUploadReader(self.robot,self.local_upload,
            self.upload_reader.persist,
            lambda: robot_task_active(self.progress,os.getenv('RVCLAW_TASK_DB')))
        self.safety=SafetyService(self.robot,lambda:self.upstream_json('/api/cameras'),
            lambda:self.upstream_json('/api/safety-frame'),self.safety_context,self.safety_switch)
        self.data.safety_events=self.safety.events

    def safety_context(self):
        task=read_json(self.progress)
        with self.health.lock:
            observer=self.health.observer
        return {'pose':live_pose(task,observer,self.robot),
                'stationary':observer.get('readings',{}).get('localization',{}).get('valid') is True}

    def safety_switch(self,mode,enabled):
        if robot_task_active(self.progress,os.getenv('RVCLAW_TASK_DB')):
            raise ValueError('小车正在巡检，请在任务结束后切换本地识别。')
        if not self.inference_lock.acquire(blocking=False):raise ValueError('相机操作正在进行，请稍后再试。')
        try:
            current=self.upstream_json('/api/cameras')
            if not enabled and current.get('active_mode')!='astra_hardhat':return
            connection=self.connection(30)
            try:
                connection.request('POST','/api/camera-mode',body=json.dumps({'mode':mode}).encode(),headers={'Content-Type':'application/json'})
                response=connection.getresponse();response.read(1024*1024)
                if response.status!=200:raise ValueError('本地识别切换失败，请稍后重试。')
            finally:connection.close()
        finally:self.inference_lock.release()

    def local_upload(self,payload):
        return self.local_upload_reader.read(payload)

    def read_upload(self,payload):
        connection=self.connection(85)
        try:
            connection.request('POST','/api/meter-image-read',body=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
            response=connection.getresponse();value=json.loads(response.read(4*1024*1024))
            if response.status!=200:raise ValueError('本地识别未能完成，请检查图片后重试。')
            return value
        finally:connection.close()

    def connection(self, timeout=15):
        return http.client.HTTPConnection('127.0.0.1',8017,timeout=timeout)

    def upstream_json(self, path):
        connection = self.connection()
        try:
            connection.request('GET',path)
            response=connection.getresponse()
            if response.status != 200: raise RuntimeError('vision status unavailable')
            return json.loads(response.read(2*1024*1024))
        finally: connection.close()

    def snapshot(self):
        with self.snapshot_lock:
            if self.cached_snapshot and time.monotonic()-self.snapshot_at<2:
                return self.cached_snapshot
            value=self.upstream_json('/api/device/snapshot')
            sample=self.telemetry.sample()
            value['cpu']={**value.get('cpu',{}),'usage_percent':sample['usage_percent']}
            value['memory'],value['storage']=sample['memory'],sample['storage']
            temperatures=[r['celsius'] for r in value['cpu'].get('temperatures',[]) if 'cluster' in r.get('type','') and isinstance(r.get('celsius'),(int,float))]
            if temperatures:value['cpu']['temperature_celsius']=round(sum(temperatures)/len(temperatures),1)
            value['model']={**value.get('model',{}),'target':os.getenv('RVCLAW_BOARD_LABEL','Firefly K1')}
            value['network']={**value.get('network',{}),'online':True,'tailscale_ipv4':os.getenv('RVCLAW_STUDIO_BIND','')}
            value['sampled_at']=datetime.now(timezone.utc).isoformat()
            value=self.health.attach(value)
            self.cached_snapshot,self.snapshot_at=value,time.monotonic()
            return value

    def uploads(self):
        return read_json(self.repo/'runs/meter-uploads/index.json',[])+self.upload_reader.records()

    def logs(self):
        # Read a bounded journal sample. StudioData turns these into Chinese
        # descriptions and never forwards raw HTTP paths or Python tracebacks.
        try:
            run=subprocess.run(['journalctl','--user','-u','rvclaw-camera-api.service','-u','rvclaw-studio.service','-u','rvclaw-vendor-command.service','-n','120','-o','json','--no-pager'],capture_output=True,text=True,timeout=3)
            rows=[]
            for line in run.stdout.splitlines():
                try:
                    item=json.loads(line)
                    rows.append({'time':datetime.fromtimestamp(int(item.get('__REALTIME_TIMESTAMP','0'))/1e6,timezone.utc).isoformat(),'message':str(item.get('MESSAGE',''))[:2000],'level':'ERROR' if int(item.get('PRIORITY','6'))<=3 else 'INFO'})
                except (ValueError,TypeError):pass
            return {'logs':rows}
        except (OSError,subprocess.SubprocessError):return {'logs':[]}

    def public_progress(self, *, latest=False):
        value=read_json(self.progress)
        result=inspection_assets.public_task(value,{}) or {}
        result.update({key:value[key] for key in ('generated_at','progress','host') if key in value})
        if not latest:
            result['history']=[inspection_assets.public_task(row,{}) for row in value.get('history',[])]
        count=value.get('evidence',{}).get('photo_count')
        if isinstance(count,int) and not isinstance(count,bool) and count>=0:
            result['evidence']={'photo_count':count}
        revision=value.get('watcher',{}).get('observed_events')
        if isinstance(revision,int) and not isinstance(revision,bool) and revision>=0:
            result['event_revision']=revision
        result['startup'] = startup_status(value, self.robot)
        with self.health.lock:
            observer = self.health.observer
        result['live_pose'] = live_pose(value, observer, self.robot)
        result['live_track'] = live_pose(value, observer, self.robot, history=True) or []
        return result


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'

        def log_message(self,*_):pass

        def reply(self,status,payload,content_type='application/json; charset=utf-8',cache='no-store'):
            raw=json.dumps(payload,ensure_ascii=False,allow_nan=False).encode() if isinstance(payload,(dict,list)) else payload
            self.send_response(status)
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control',cache)
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Referrer-Policy','same-origin')
            self.end_headers()
            if self.command!='HEAD':self.wfile.write(raw)

        def file(self,root,name,allowed=None,immutable=False):
            target=(root/name).resolve()
            if not target.is_relative_to(root.resolve()) or not target.is_file() or (allowed and target.suffix.lower() not in allowed):
                return self.reply(404,{'error':'文件不存在。'})
            return self.reply(200,target.read_bytes(),mimetypes.guess_type(target.name)[0] or 'application/octet-stream','public, max-age=31536000, immutable' if immutable else 'no-store')

        def proxy(self,body=None):
            connection=app.connection(90 if self.command=='POST' else 12)
            try:
                headers={'Content-Type':'application/json'} if body is not None else {}
                connection.request(self.command,self.path,body=body,headers=headers)
                response=connection.getresponse()
                content_type=response.getheader('Content-Type','application/octet-stream')
                if self.path.startswith('/stream/'):
                    self.send_response(response.status)
                    self.send_header('Content-Type',content_type)
                    self.send_header('Cache-Control','no-store')
                    self.send_header('Connection','close');self.end_headers()
                    self.close_connection=True
                    while True:
                        data=response.read1(65536)
                        if not data:break
                        self.wfile.write(data);self.wfile.flush()
                else:
                    raw=response.read(24*1024*1024)
                    if self.command=='POST':app.data.invalidate()
                    self.reply(response.status,raw,content_type)
            finally:connection.close()

        def do_GET(self):
            parsed=urlsplit(self.path);path=unquote(parsed.path)
            try:
                if path=='/api/safety/status':return self.reply(200,app.safety.status())
                if path=='/api/safety/records':
                    rows=app.safety.records()
                    return self.reply(200,{'records':rows,'total':len(rows)})
                evidence=re.fullmatch(r'/api/safety/evidence/([a-f0-9]{32})\.jpg',path)
                if evidence:return self.file(app.safety.root,evidence[1]+'.jpg',{'.jpg'})
                if path=='/api/device/snapshot':return self.reply(200,app.snapshot())
                if path=='/api/inspection-maps':return self.reply(200,inspection_maps.catalog())
                if path=='/api/inspection-meter-settings':return self.reply(200,inspection_assets.inspection_settings(app.photos,app.progress.parent,app.meters,(parse_qs(parsed.query).get('map_id') or [None])[0]))
                if path=='/api/inspection-assets':return self.reply(200,inspection_assets.assets(app.photos,app.progress.parent,parsed.query))
                if path=='/api/inspection-map/image.png':return self.reply(200,inspection_assets.map_image(app.photos,app.progress.parent,parsed.query),'image/png')
                if path=='/voice-health.json':return self.file(app.web,'voice-health.json',{'.json'})
                if path=='/inspection-progress.json':return self.reply(200,app.public_progress(latest=parse_qs(parsed.query).get('latest')==['1']))
                if path=='/favicon.ico':return self.reply(204,b'','image/x-icon')
                if path in {'/api/studio/meter-history','/api/studio/events','/api/studio/logs'}:
                    rows=app.data.meter_records() if path.endswith('meter-history') else app.data.events() if path.endswith('events') else app.data.logs(limit=None if parse_qs(parsed.query).get('export')==['1'] else 1000)
                    result={'records':rows,'total':len(rows)}
                    if path.endswith('events'):result['video_notes']={k:v for k,v in app.data.notes().items() if k.startswith('video-demo-')}
                    return self.reply(200,result)
                if path.startswith('/inspection-photos/'):return self.file(app.photos,path.removeprefix('/inspection-photos/'),{'.jpg'})
                uploaded=re.fullmatch(r'/api/studio/uploads/([a-f0-9]{32})/image\.jpg',path)
                if uploaded:return self.file(app.upload_reader.root,uploaded[1]+'.jpg',{'.jpg'})
                if path.startswith('/inspection-evidence/'):
                    # Original evidence links may contain photos and JSON only.
                    return self.file(app.photos,path.removeprefix('/inspection-evidence/'),{'.jpg','.json'})
                if path in JSON_GETS or RECORD_IMAGE.fullmatch(path) or TRIAL_IMAGE.fullmatch(path) or path in {'/stream/astra.mjpg','/stream/c70.mjpg','/stream/camera.mjpg'}:return self.proxy()
                if path.startswith('/assets/'):return self.file(app.web,path.lstrip('/'),immutable=True)
                if path in {'/','/index.html'}:return self.file(app.web,'index.html')
                return self.reply(404,{'error':'页面或接口不存在。'})
            except (ValueError,KeyError,FileNotFoundError) as error:
                message=str(error)
                return self.reply(400,{'error':message if not re.search(r'/data/|Traceback',message) else '资料不完整，请刷新后重试。'})
            except (BrokenPipeError,ConnectionResetError,socket.timeout):pass
            except Exception:
                LOG.exception('GET failed for %s',path)
                return self.reply(503,{'error':'设备数据暂时不可用，请稍后刷新。'})

        def do_POST(self):
            path=urlsplit(self.path).path
            if self.headers.get('Origin') not in app.origins:
                self.close_connection=True;return self.reply(403,{'error':'请求来源不受支持。'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=UPLOAD_LIMIT:raise ValueError('请求内容为空或过大。')
                if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('请求格式无效。')
                body=self.rfile.read(size);payload=json.loads(body)
                if not isinstance(payload,dict):raise ValueError('请求内容无效。')
                if path=='/api/safety/config':return self.reply(200,app.safety.configure(payload))
                if path=='/api/safety/local':
                    if not isinstance(payload.get('enabled'),bool):raise ValueError('识别开关无效。')
                    return self.reply(200,app.safety.local_control(payload['enabled']))
                if path=='/api/safety/capture':return self.reply(202,app.safety.request_cloud(manual=True))
                if path=='/api/safety/upload':return self.reply(202,app.safety.request_cloud(manual=True,image=upload_image(payload)))
                if path=='/api/inspection-meter-settings':
                    # Shared lock also serializes different browser clients.
                    with inspection_assets.SETTINGS_LOCK:
                        result=inspection_assets.save_inspection_settings(app.photos,app.progress.parent,app.meters,payload)
                    app.data.invalidate();return self.reply(200,result)
                if path=='/api/studio/event-note':return self.reply(200,app.data.save_note(payload))
                if path in VISION_POSTS:
                    if path=='/api/meter-image-read':
                        if payload.get('meter_type') not in {'digital','pointer'}:raise ValueError('请选择有效的仪表类型。')
                        encoded=payload.get('image')
                        if not isinstance(encoded,str) or not encoded:raise ValueError('请先上传仪表图片。')
                        try:raw=base64.b64decode(encoded.split(',',1)[-1],validate=True)
                        except ValueError:raise ValueError('图片内容无效。')
                        if not 0<len(raw)<=10*1024*1024:raise ValueError('图片不能超过 10 MB。')
                    # A real route has priority. Reject optional uploads while
                    # driving instead of competing with inspection inference.
                    if robot_task_active(app.progress,os.getenv('RVCLAW_TASK_DB')):
                        return self.reply(409,{'error':'小车正在巡检，请在任务结束后使用网页识别或切换相机。'})
                    if not app.inference_lock.acquire(blocking=False):return self.reply(409,{'error':'识别正在进行，请稍候重试。'})
                    try:
                        if path=='/api/meter-image-read':
                            value=app.hybrid_upload.read(payload)
                            app.data.invalidate();return self.reply(200,value)
                        return self.proxy(body)
                    finally:app.inference_lock.release()
                return self.reply(404,{'error':'操作不存在。'})
            except (ValueError,KeyError) as error:
                self.close_connection=True
                return self.reply(400,{'error':str(error) if re.search(r'[\u4e00-\u9fff]',str(error)) and '/data/' not in str(error) else '请求内容无效，请检查输入。'})
            except (BrokenPipeError,ConnectionResetError,socket.timeout):pass
            except Exception:
                LOG.exception('POST failed for %s',path)
                return self.reply(503,{'error':'操作未能完成，请检查设备连接后重试。'})

    return Handler


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default=os.environ.get('RVCLAW_STUDIO_BIND','127.0.0.1'))
    parser.add_argument('--port',type=int,default=8017)
    args=parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    app=Application();app.health.start();app.safety.start()
    server=ThreadingHTTPServer((args.host,args.port),handler(app));server.daemon_threads=True
    try:server.serve_forever(poll_interval=1)
    finally:app.safety.stop();app.health.stop();server.server_close()


if __name__=='__main__':main()
