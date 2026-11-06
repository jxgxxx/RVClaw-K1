#!/usr/bin/env python3
"""Small Tailscale control endpoint using the existing guarded route executor.

No camera/ROS imports, polling or motion on startup. All movement remains behind
the reviewed route wrapper and its localization/arming/deadman checks.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from uuid import uuid4

ACTIVE = ('created', 'accepted', 'running', 'active', 'in_progress', 'paused')
ROUTE_INTENTS = {'start_inspection', 'resume_inspection', 'resume_inspection_relocalize'}
LOG = logging.getLogger('inspection_control')


class ControlError(Exception):
    def __init__(self, message, status=409):
        self.status = status
        super().__init__(message)


def process_routes(proc=Path('/proc')):
    """Recognize exact runner argv; never use pgrep substring signal targets."""
    result = []
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            args = [v.decode(errors='replace') for v in (path/'cmdline').read_bytes().split(b'\0') if v]
            names = {Path(v).name for v in args}
            route = ('nav2_persistent_route.sh' in names and '--run' in args) or ('rvclaw_waypoint_route_client.py' in names and '--arm-token' in args)
            mapping = bool(names & {'slam_mapping_session.sh', 'mapping_entry.sh'}) and bool(set(args) & {'--voice-run', '--run'})
            if not route and not mapping:
                continue
            stat = (path/'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            env = dict(v.decode(errors='replace').split('=', 1) for v in (path/'environ').read_bytes().split(b'\0') if b'=' in v)
            trace = env.get('RVCLAW_ROUTE_TRACE_FILE', '')
            if '--trace-file' in args:
                trace = args[args.index('--trace-file')+1]
            result.append({'pid': int(path.name), 'start_ticks': stat[19], 'pgid': int(stat[2]), 'kind': 'route' if route else 'mapping', 'trace_file': trace})
        except (OSError, ValueError, IndexError, ProcessLookupError):
            continue
    # A shell preflight and its child may share a group: signal the leader only.
    return list({row['pgid']: row for row in sorted(result, key=lambda r: r['pid'], reverse=True)}.values())


class Controller:
    def __init__(self, database, profile_reader, route, gateway, scanner=process_routes):
        self.database, self.profile_reader = database, profile_reader
        self.route, self.gateway, self.scanner = route, gateway, scanner
        self.lock = threading.RLock()
        self.stopping = set()
        self.owned_task = None
        self.started = threading.Event()
        self.cancel_announced = set()
        route.task_recorder = self.record_terminal

    def active_tasks(self):
        conn = sqlite3.connect('file:' + str(self.database) + '?mode=ro', uri=True, timeout=2)
        try:
            rows = conn.execute('SELECT task_id,state,payload FROM tasks WHERE state IN (' + ','.join('?' for _ in ACTIVE) + ')', ACTIVE).fetchall()
            return [dict(task_id=row[0], state=row[1], payload=json.loads(row[2])) for row in rows]
        finally:
            conn.close()

    def status(self):
        with self.lock:
            profile = self.profile_reader()
            tasks, runners = self.active_tasks(), self.scanner()
            active = [row for row in tasks if row['payload'].get('intent') in ROUTE_INTENTS]
            own = self.route.status()
            running = bool(active or any(p['kind'] == 'route' for p in runners) or own.get('state') in {'running', 'finishing'})
            stopping = bool(self.stopping)
            return {'state': 'stopping' if stopping else 'running' if running else 'idle',
                    'can_start': bool(profile) and not tasks and not runners and not running and not stopping,
                    'can_cancel': running and not stopping,
                    'map_id': profile.get('map_id') if profile else None,
                    'task_id': active[0]['task_id'] if active else self.owned_task if running else None}

    def start(self, map_id):
        with self.lock:
            profile = self.profile_reader()
            if not profile or map_id != profile['map_id']:
                raise ControlError('地图已改变，请刷新页面并确认已保存的巡检设置。')
            if not self.status()['can_start']:
                raise ControlError('小车已有任务或正在停车，请等待当前任务结束。')
            now = time.time()
            key = 'web-' + uuid4().hex
            # Serialize against other task writers until our accepted row is durable.
            self.gateway.db.execute('BEGIN IMMEDIATE')
            try:
                if self.active_tasks() or self.scanner():
                    raise ControlError('小车已有任务，请等待当前任务结束。')
                accepted = self.gateway.submit({'schema_version': 1, 'proposal_id': key, 'request_id': key,
                    'source': 'local', 'intent': 'start_inspection', 'parameters': {'map_id': map_id},
                    'confidence': 1.0, 'created_at': now, 'expires_at': now+30,
                    'owner_epoch': self.gateway.owner_epoch, 'evidence': {'interface': 'studio'}})
            except Exception:
                self.gateway.db.rollback()
                raise
            if accepted.get('status') != 'accepted':
                raise ControlError('任务未被小车接收，请刷新后重试。')
            self.owned_task = accepted['task_id']
            self.started.clear()
            try:
                run = self.route.start()
                self.gateway.record_event(self.owned_task, 'TASK_STARTED', {**run, 'map_id': map_id, 'source': 'studio'})
                try:
                    self.route._publish_terminal_event('inspection_start_accepted', 'PASS', 0,
                        {'task_id': self.owned_task, 'source': 'studio'})
                except Exception:
                    LOG.exception('startup speech event could not be published')
            except Exception:
                self.gateway.fail_task(self.owned_task, {'state': 'failed', 'reason': 'web_start_failed'})
                self.owned_task = None
                raise ControlError('巡检启动失败，请到 RVBench 检查设备状态。')
            finally:
                self.started.set()
            return self.status()

    def prepare(self, map_id):
        with self.lock:
            profile = self.profile_reader()
            if not profile or map_id != profile['map_id']:
                raise ControlError('地图已改变，请刷新后重试。')
            if not self.status()['can_start']:
                return {'prepared': False, 'motion_authorized': False}
            # A short resource-only head start overlaps reading the page. The
            # actual start request still runs every localization/arming gate.
            completed = subprocess.run(['/usr/bin/python3',
                '/data/RVClaw/deploy/k1/runtime_demand.py', 'warm', '--seconds', '90'],
                capture_output=True, text=True, check=True, timeout=5.)
            return {'prepared': completed.stdout.strip() == 'prepared=true',
                    'motion_authorized': False}

    def record_terminal(self, state, run):
        self.started.wait(10)
        with self.lock:
            if not self.owned_task:
                return
            event = {'completed': 'TASK_COMPLETED', 'stopped': 'TASK_ABORTED', 'failed': 'TASK_FAILED'}.get(state)
            if event:
                self.gateway.record_event(self.owned_task, event, {**run, 'state': 'aborted' if state == 'stopped' else state})
                self.owned_task = None

    def cancel(self, expected_task=None):
        with self.lock:
            status = self.status()
            if not status['can_cancel']:
                raise ControlError('当前没有可取消的巡检任务。')
            if expected_task and status['task_id'] and expected_task != status['task_id']:
                raise ControlError('当前任务已经改变，请刷新后重试。')
            runners = [row for row in self.scanner() if row['kind'] == 'route']
            own = self.route.status()
            if own.get('state') in {'running', 'finishing'} and own.get('source') != 'external':
                self.stopping.add('owned')
                threading.Thread(target=self._stop_owned, args=(status['task_id'],), daemon=True).start()
            elif runners:
                for runner in runners:
                    if runner['pgid'] != runner['pid']:
                        raise ControlError('巡检进程归属异常，未执行停止；请在现场检查。')
                    marker = self._marker(runner)
                    marker.parent.mkdir(parents=True, exist_ok=True)
                    temporary = marker.with_suffix('.tmp')
                    temporary.write_text(json.dumps({'pid': runner['pid'], 'start_ticks': runner['start_ticks'], 'requested_at': time.time()}))
                    temporary.replace(marker)
                    self.stopping.add(runner['pid'])
                    threading.Thread(target=self._stop_external, args=(runner, status['task_id']), daemon=True).start()
            else:
                # A durable row without a live process is not proof of a running chassis.
                raise ControlError('任务进程暂时无法确认，未执行停止；请刷新后重试。')
            return self.status()

    def _marker(self, runner):
        # Shared with the voice executor. PID plus kernel start time avoids PID reuse.
        return Path(os.getenv('RVCLAW_INSPECTION_CANCEL_ROOT', '/data/RVClaw/runtime/inspection-cancel')) / (str(runner['pid']) + '-' + runner['start_ticks'] + '.json')

    def _announce_cancelled(self, task_id):
        with self.lock:
            if not task_id or task_id in self.cancel_announced:
                return
            if any(row['kind']=='route' for row in self.scanner()):
                return
            self.route._publish_terminal_event('inspection_cancelled', 'CANCELLED', 0,
                {'task_id': task_id, 'source': 'studio'})
            self.cancel_announced.add(task_id)

    def _stop_owned(self, task_id=None):
        try:
            result = self.route.stop()
            if result.get('state') == 'stopped':
                self._announce_cancelled(task_id)
        except Exception:
            LOG.exception('safe route stop failed')
        finally:
            with self.lock:
                self.stopping.discard('owned')

    def _same_process(self, runner):
        return any(row['pid'] == runner['pid'] and row['start_ticks'] == runner['start_ticks'] for row in self.scanner())

    def _stop_external(self, runner, task_id):
        try:
            for signum, grace in ((signal.SIGINT, 60), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                if not self._same_process(runner):
                    break
                os.killpg(runner['pgid'], signum)
                deadline = time.monotonic() + grace
                while self._same_process(runner) and time.monotonic() < deadline:
                    time.sleep(.2)
            if self._same_process(runner):
                raise RuntimeError('route process still alive after stop')
            # Existing voice owners may also close the row. Last semantic event
            # is explicitly an operator cancellation, not a recognition failure.
            if task_id:
                with self.lock:
                    self.gateway.record_event(task_id, 'TASK_ABORTED', {'state': 'aborted', 'reason': 'operator_cancelled', 'source': 'studio'})
                self._announce_cancelled(task_id)
        except Exception:
            LOG.exception('external safe stop failed')
        finally:
            with self.lock:
                self.stopping.discard(runner['pid'])


def handler(controller, origins):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, status, value):
            raw = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            if self.headers.get('Origin') in origins:
                self.send_header('Access-Control-Allow-Origin', self.headers['Origin'])
                self.send_header('Vary', 'Origin')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_OPTIONS(self):
            if self.headers.get('Origin') not in origins:
                return self.reply(403, {'error': '来源不受支持。'})
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', self.headers['Origin'])
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Access-Control-Max-Age', '600')
            self.send_header('Vary', 'Origin')
            self.end_headers()

        def do_GET(self):
            if self.path != '/api/inspection-control':
                return self.reply(404, {'error': '地址不存在。'})
            try:
                self.reply(200, controller.status())
            except Exception:
                LOG.exception('status unavailable')
                self.reply(503, {'error': '小车任务状态暂不可用，请刷新重试。', 'can_start': False, 'can_cancel': False})

        def do_POST(self):
            if self.path != '/api/inspection-control':
                return self.reply(404, {'error': '地址不存在。'})
            if self.headers.get('Origin') not in origins:
                return self.reply(403, {'error': '来源不受支持。'})
            try:
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ControlError('请使用正确的请求格式。', 415)
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ControlError('请求内容无效。', 400)
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict) or body.get('action') not in {'start', 'cancel', 'prepare'}:
                    raise ControlError('操作无效。', 400)
                action = body['action']
                if action == 'prepare':
                    return self.reply(200, controller.prepare(body.get('map_id')))
                self.reply(200, controller.start(body.get('map_id')) if action == 'start' else controller.cancel(body.get('task_id')))
            except ControlError as error:
                self.reply(error.status, {'error': str(error)})
            except (ValueError, TypeError):
                self.reply(400, {'error': '请求内容无效。'})
            except Exception:
                LOG.exception('control request failed')
                self.reply(503, {'error': '小车暂时无法处理请求，请刷新后重试。'})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default=os.getenv('RVCLAW_STUDIO_BIND', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=8019)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    sys.path.insert(0, os.getenv('RVCLAW_VOICE_PACKAGE_ROOT', '/data/RVClaw/services/rvclaw_voice'))
    sys.path.insert(0, '/data/RVClaw/services/inspection_profiles')
    from rvclaw_voice.robot_client import ChassisRouteClient
    from rvclaw_voice.task_gateway import TaskGateway
    from profile_runtime import read_profile
    database = Path(os.environ['RVCLAW_TASK_DB'])
    controller = Controller(database, read_profile, ChassisRouteClient(), TaskGateway(str(database)))
    origins = {value.strip() for value in os.getenv('RVCLAW_STUDIO_ORIGINS', 'http://100.112.170.125:8017').split(',') if value.strip()}
    server = ThreadingHTTPServer((args.host, args.port), handler(controller, origins))
    server.daemon_threads = True
    server.serve_forever(poll_interval=1)


if __name__ == '__main__':
    main()
