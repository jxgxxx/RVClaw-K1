#!/usr/bin/env python3
"""RVClaw 巡检进度旁路 watcher —— 把"转瞬即逝"的巡检信号固化成页面能读的 JSON。

## 为什么需要这个进程

页面要显示「开始 → 到达第一个/第二个/第三个巡检点 → 结束」，但这三个信号在板端
**没有任何一处是既实时又持久的**：

| 信号 | 来源 | 问题 |
|---|---|---|
| 开始 / 结束 | `voice_tasks.sqlite3` 的 `tasks` + `events` | 持久，但**没有逐点粒度** |
| 到达 A/B/C | `runtime/waypoint-arrivals/*.json` | 实时，但语音服务播报成功后**立刻 `unlink`**，文件只活 2~4 秒 |
| 权威结果 | `<run_dir>/trace.json` 的 `completed_waypoints` | 全流程跑完才写一次 |

所以本进程做三件事：**高频抓**（spool）、**持久账**（自有 ledger）、**事后纠偏**（trace）。
它是纯旁路：只读现有文件，不改、不删、不重启任何现有服务；写出的文件只有它自己的
ledger 和一份给页面看的 JSON。

## 为什么输出到 studio/web/

`rvclaw_api/server.py` 的 `_serve_studio()` 会把 `studio/web/` 下的文件按请求现读并以
`no-store` 返回（`server.py:2361`）。把 JSON 放在那里，页面就能用同源相对路径直接轮询，
**完全不需要改 `server.py`、也就不需要重启 studio**（重启 studio 有已知的 TCM/DMA 风险）。
"""

from __future__ import annotations

import argparse
from inspection_feedback import annotate_runs
import errno
import json
import logging
import os
import re
import shutil
import signal
import sqlite3
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# `fcntl` 只有类 Unix 平台有。板端（Linux）必须有它来保证单实例；
# 开发机上（Windows）拿不到，则退化为"无锁运行"，这样 I/O 链路仍可离线自测。
try:
    import fcntl
except ImportError:  # pragma: no cover - 仅 Windows 开发机走到这里
    fcntl = None  # type: ignore[assignment]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'inspection_profiles'))
from trace_summary import read_trace_summary

import inspection_progress as ip  # noqa: E402

LOG = logging.getLogger("inspection-progress")

DEFAULT_TASK_DB = "/data/RVClaw/runtime/voice_tasks.sqlite3"
DEFAULT_SPOOL_DIR = "/data/RVClaw/runtime/waypoint-arrivals"
DEFAULT_OUTPUT = "/data/RVClaw/studio/web/inspection-progress.json"
DEFAULT_LEDGER = "/data/RVClaw/runtime/inspection-progress/ledger.jsonl"
DEFAULT_LOCK = "/data/RVClaw/runtime/inspection-progress/watcher.lock"
DEFAULT_ROUTE = "A,B,C,ORIGIN"

#: 巡检证据的持久根目录。与 studio 的 `RVCLAW_INSPECTION_PHOTO_ROOT` **同一个目录**：
#: 到点抓拍的照片先落在 `<根>/<task_id>/<点位>/`，巡检结束时由本进程打成一个
#: 自包含的证据包 `<根>/<时间>-<task_id 前 8 位>/`（含照片副本）。
DEFAULT_EVIDENCE_ROOT = "/data/inspection_photos"
#: 到点抓拍用 studio 的取帧口（不跑模型）。为空字符串则关闭抓拍。
DEFAULT_CAPTURE_URL = "http://127.0.0.1:8017/api/inspection-evidence/capture"
#: 归档时从 `run_dir` 一并拷走的原始记录。run_dir 在临时树里，不拷走就会丢。
EVIDENCE_RUN_FILES = ("trace.json", "command.log")
#: 单个归档文件上限：别让一次异常把证据盘写满（trace.json 正常 ~200 KB）。
EVIDENCE_MAX_FILE_BYTES = 8 * 1024 * 1024
#: 归档索引。既是"已归档过"的幂等凭据，也是页面拿"这次巡检的证据在哪"的来源。
EVIDENCE_INDEX = "index.json"

#: spool 扫描周期。到点文件寿命只有 2~4 秒，取 0.3 s 留足余量（单次扫描就是一个
#: glob + 少量小文件读取，开销可忽略）。
SPOOL_INTERVAL_S = 0.3
#: 任务库轮询周期。库只有几十 KB，1 s 足够。
DB_INTERVAL_S = 1.0
#: 即使内容没变也至少写一次输出的间隔 —— 页面靠 `generated_at` 判断板端是否在线。
HEARTBEAT_S = 5.0
#: 账本只回读最后 N 行，避免长期运行后内存随历史线性增长。
LEDGER_TAIL_LINES = 4000

#: 语音服务播报完会先写回执、再删原文件：`<序号>-waypoint-<点名>-<uuid>.json.ack`。
#: 回执带原始 `timestamp`，但路线可立即删除；新事件以 facts 为持久依据。
#: 原文件只活 2~4 秒，而语音服务的轮询是 0.2 秒 —— 到点事件是有可能输掉这场竞态的
#: （2026-09-20 复核：watcher 0.3 秒扫一次通常赢，但赢了是概率不是保证）。
#: 兼容读取仍存在的旧回执，同一件事靠 `arrival_map` 取最早去重。
#: ⚠️ 回执的 payload 里没有 `waypoint` 字段（只有 event_id/event/status/played/timestamp/error），
#: 点名只能从**文件名**里取。
ARRIVAL_ACK_RE = re.compile(r"^\d+-(?:waypoint|meter)-([A-Za-z])-[0-9a-fA-F-]+\.json\.ack$")

#: 同一次到达从"原文件"和"回执"两条路各记一笔时，时间戳差在这个范围内就算重复。
ACK_DUPLICATE_WINDOW_S = 3.0


# ======================================================================================
# 配置
# ======================================================================================


class Config:
    def __init__(self, args: argparse.Namespace) -> None:
        env = os.environ.get
        self.task_db = Path(args.task_db or env("RVCLAW_TASK_DB", DEFAULT_TASK_DB))
        self.spool_dir = Path(
            args.spool_dir or env("RVCLAW_ARRIVAL_EVENT_DIR", DEFAULT_SPOOL_DIR)
        )
        self.output = Path(args.output or env("RVCLAW_INSPECTION_PROGRESS_OUT", DEFAULT_OUTPUT))
        self.ledger = Path(
            args.ledger or env("RVCLAW_INSPECTION_PROGRESS_LEDGER", DEFAULT_LEDGER)
        )
        self.lock_path = Path(
            args.lock or env("RVCLAW_INSPECTION_PROGRESS_LOCK", DEFAULT_LOCK)
        )
        self.default_route = env("RVCLAW_INSPECTION_DEFAULT_ROUTE", DEFAULT_ROUTE)
        self.camera_stream_url = env("RVCLAW_CAMERA_STREAM_URL", "/stream/astra.mjpg")
        self.host = env("RVCLAW_INSPECTION_HOST_LABEL", "rvclaw-k3")
        self.history_limit = int(env("RVCLAW_INSPECTION_HISTORY_LIMIT", "6"))
        self.spool_interval = float(env("RVCLAW_INSPECTION_SPOOL_INTERVAL", SPOOL_INTERVAL_S))
        self.db_interval = float(env("RVCLAW_INSPECTION_DB_INTERVAL", DB_INTERVAL_S))
        self.evidence_root = Path(
            env("RVCLAW_INSPECTION_EVIDENCE_ROOT", DEFAULT_EVIDENCE_ROOT)
        )
        self.capture_url = env("RVCLAW_INSPECTION_CAPTURE_URL", DEFAULT_CAPTURE_URL).strip()
        self.capture_enabled = env("RVCLAW_INSPECTION_CAPTURE", "1").strip().lower() not in {
            "0",
            "false",
            "no",
            "off",
        }
        self.capture_timeout_s = float(env("RVCLAW_INSPECTION_CAPTURE_TIMEOUT", "6.0"))


# ======================================================================================
# 任务库读取（只读；失败时退化为"拷贝三件套再读"）
# ======================================================================================


class TaskDBReader:
    """读 `voice_tasks.sqlite3` 的 `tasks` / `events`。

    库是 WAL 模式、由语音服务持续写入。默认用 `mode=ro` 直连；若只读连接打不开
    （例如 `-shm` 不可用、拿不到读锁），退化为把 **主文件 + -wal + -shm 三件套** 拷到
    临时目录再读 —— 少任何一件都会读到过期快照，表现为"表里数据比实际少"。
    """

    def __init__(self, db_path: Path) -> None:
        self.db_path = db_path
        self._snapshot_dir: Optional[Path] = None
        self._warned = False
        self._diagnostic: Optional[str] = None

    # -- 内部 -------------------------------------------------------------------------

    def _connect_ro(self) -> sqlite3.Connection:
        uri = "file:%s?mode=ro" % self.db_path.as_posix()
        conn = sqlite3.connect(uri, uri=True, timeout=2.0)
        conn.execute("PRAGMA query_only=1")
        return conn

    def _snapshot_copy(self) -> Path:
        if self._snapshot_dir is None:
            self._snapshot_dir = Path(tempfile.mkdtemp(prefix="rvclaw-taskdb-"))
        for suffix in ("", "-wal", "-shm"):
            source = Path(str(self.db_path) + suffix)
            if source.exists():
                try:
                    shutil.copy2(source, self._snapshot_dir / (self.db_path.name + suffix))
                except OSError:
                    pass
        return self._snapshot_dir / self.db_path.name

    def _connect_snapshot(self) -> sqlite3.Connection:
        target = self._snapshot_copy()
        conn = sqlite3.connect(str(target), timeout=2.0)
        conn.execute("PRAGMA query_only=1")
        return conn

    def _connect(self) -> sqlite3.Connection:
        if self._snapshot_dir is not None:
            return self._connect_snapshot()
        try:
            return self._connect_ro()
        except sqlite3.Error as error:
            self._diagnostic = "read-only connect failed, using snapshot copy: %s" % error
            if not self._warned:
                LOG.warning("%s", self._diagnostic)
                self._warned = True
            return self._connect_snapshot()

    # -- 对外 -------------------------------------------------------------------------

    def read(self) -> Tuple[List[Dict[str, Any]], Dict[str, List[Dict[str, Any]]]]:
        """返回 `(tasks, events_by_task)`；任何失败都返回空结果而不是抛异常。

        页面宁可显示"暂时读不到任务库"，也不能因为一次读库失败就把整个进度页打崩。
        """

        if not self.db_path.exists():
            self._diagnostic = "task db missing: %s" % self.db_path
            return [], {}

        try:
            conn = self._connect()
        except (sqlite3.Error, OSError) as error:
            self._diagnostic = "open failed: %s" % error
            return [], {}

        try:
            conn.row_factory = sqlite3.Row
            task_rows = [
                dict(row)
                for row in conn.execute(
                    "SELECT task_id, state, updated, payload FROM tasks ORDER BY updated DESC"
                )
            ]
            event_rows = [
                dict(row)
                for row in conn.execute(
                    "SELECT task_id, type, payload, timestamp FROM events ORDER BY seq"
                )
            ]
        except sqlite3.Error as error:
            self._diagnostic = "query failed: %s" % error
            return [], {}
        finally:
            conn.close()

        events_by_task: Dict[str, List[Dict[str, Any]]] = {}
        for row in event_rows:
            events_by_task.setdefault(str(row.get("task_id")), []).append(row)
        return task_rows, events_by_task

    @property
    def diagnostic(self) -> Optional[str]:
        return self._diagnostic


# ======================================================================================
# 把 DB 行折叠成"本次巡检"
# ======================================================================================

_START_EVENTS = {"TASK_STARTED"}
#: ⚠️ `TASK_FAILED` **必须**在这里 —— 2026-09-20 复核的根因就是它被漏了：
#: 原先只认 COMPLETED/ABORTED，于是**失败任务永远拿不到 `finished_at`**，页面上的
#: "已用时"只能靠本地钟外推（失败态也在走表），到点窗口的右端也永远合不拢。
#: 任务库其实一直在写这个事件，而且 `state`/`return_code`/`route` 都带在里面
#: （实测 `e5a6496e` 的 TASK_FAILED：`state=failed`、`return_code=4`）。
_END_EVENTS = {"TASK_COMPLETED", "TASK_ABORTED", "TASK_FAILED", "TASK_CANCELLED"}

#: 结束事件没带 `state` 字段时的兜底映射（别再用"非 COMPLETED 即 aborted"那种猜法）。
_END_EVENT_STATES = {
    "TASK_COMPLETED": "completed",
    "TASK_ABORTED": "aborted",
    "TASK_CANCELLED": "aborted",
    "TASK_FAILED": "failed",
}


def _payload_dict(raw: Any) -> Dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    try:
        loaded = json.loads(raw)
    except (TypeError, ValueError):
        return {}
    return loaded if isinstance(loaded, dict) else {}


def fold_runs(
    tasks: List[Dict[str, Any]],
    events_by_task: Dict[str, List[Dict[str, Any]]],
) -> List[Dict[str, Any]]:
    """把任务行与事件折成 `run` dict 列表（按开始时间新的在前）。"""

    runs: List[Dict[str, Any]] = []
    for task in tasks:
        task_id = str(task.get("task_id") or "")
        proposal = _payload_dict(task.get("payload"))
        if proposal.get("intent") and proposal["intent"] not in {"start_inspection", "resume_inspection", "resume_inspection_relocalize"}:
            continue
        parameters = proposal.get("parameters") or {}
        context = {"map_id": parameters.get("map_id") or proposal.get("map_id")}
        events = events_by_task.get(task_id, [])

        started_at: Optional[float] = None
        finished_at: Optional[float] = None
        route: Any = None
        run_dir: Optional[str] = None
        trace_file: Optional[str] = None
        return_code: Optional[int] = None
        terminal_state: Optional[str] = None

        for event in events:
            etype = str(event.get("type") or "")
            payload = _payload_dict(event.get("payload"))
            for key in ("map_id", "fast_localization_trace_file", "timed_out"):
                if payload.get(key) is not None:
                    context[key] = payload[key]
            timestamp = event.get("timestamp")
            if etype in _START_EVENTS:
                if started_at is None or (timestamp or 0) < started_at:
                    started_at = timestamp
                route = payload.get("route", route)
                run_dir = payload.get("run_dir", run_dir)
                trace_file = payload.get("trace_file", trace_file)
            elif etype in _END_EVENTS:
                finished_at = timestamp
                terminal_state = payload.get("state") or _END_EVENT_STATES.get(etype, "aborted")
                if payload.get("return_code") is not None:
                    return_code = payload.get("return_code")
                route = payload.get("route", route)
                run_dir = payload.get("run_dir", run_dir)
                trace_file = payload.get("trace_file", trace_file)

        state = terminal_state or str(task.get("state") or "idle").strip().lower()
        if state in {"stopped", "cancelled"}:
            state = "aborted"
        runs.append(
            {
                **context,
                "task_id": task_id,
                "state": state,
                "route": route,
                "started_at": started_at,
                "finished_at": finished_at,
                "run_dir": run_dir,
                "trace_file": trace_file,
                "return_code": return_code,
                "updated": task.get("updated"),
            }
        )

    runs.sort(key=lambda run: (run.get("started_at") or run.get("updated") or 0.0), reverse=True)
    return runs


# ======================================================================================
# trace.json 回填
# ======================================================================================


class TraceCache:
    """按 (路径, mtime) 缓存 `trace.json` 的解析结果。

    trace 只在整条路线跑完时写一次，所以缓存命中率极高；用 mtime 作 key 是为了防止
    同名文件被下一轮巡检覆盖后拿到旧结果（跑车目录名带 uuid，但别赌这个）。
    """

    def __init__(self) -> None:
        self._cache: Dict[str, Tuple[float, Dict[str, Any]]] = {}
        self.diagnostic: Optional[str] = None

    def get(self, path: Optional[str]) -> Dict[str, Any]:
        if not path:
            return {}
        target = Path(path)
        if not target.exists():
            return {}
        try:
            mtime = target.stat().st_mtime
        except OSError as error:
            self.diagnostic = "stat failed for %s: %s" % (path, error)
            return {}
        cached = self._cache.get(str(target))
        if cached and cached[0] == mtime:
            return cached[1]
        try:
            data = read_trace_summary(target)
        except (OSError, ValueError) as error:
            self.diagnostic = "parse failed for %s: %s" % (path, error)
            return {}
        if not isinstance(data, dict):
            return {}
        if len(self._cache)>=256:self._cache.pop(next(iter(self._cache)))
        self._cache[str(target)] = (mtime, data)
        return data


# ======================================================================================
# 观测账本
# ======================================================================================


class Ledger:
    """追加式 JSONL 账本。

    ⚠️ 单实例锁是必须的：两个进程同时 append 同一个文件会**静默写坏**
    （只在最后校验时才会暴露）。所以这里用 `flock` 兜住。
    """

    def __init__(self, path: Path, lock_path: Path) -> None:
        self.path = path
        self.lock_path = lock_path
        self._handle = None
        self._lock_handle = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        if fcntl is None:
            LOG.warning(
                "fcntl unavailable (non-Unix host): 单实例锁被跳过，只应在开发机离线自测时出现"
            )
            self._handle = open(self.path, "a", encoding="utf-8")
            return
        self._lock_handle = open(self.lock_path, "w", encoding="utf-8")
        try:
            fcntl.flock(self._lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            if error.errno in (errno.EACCES, errno.EAGAIN):
                raise SystemExit(
                    "另一个 inspection-progress watcher 正在运行（锁文件 %s）" % self.lock_path
                )
            raise
        self._lock_handle.write("%d\n" % os.getpid())
        self._lock_handle.flush()
        self._handle = open(self.path, "a", encoding="utf-8")

    def append(self, record: Dict[str, Any]) -> None:
        if self._handle is None:
            return
        try:
            self._handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            self._handle.flush()
        except OSError:
            LOG.exception("ledger append failed")

    def tail(self, limit: int = LEDGER_TAIL_LINES) -> List[Dict[str, Any]]:
        if not self.path.exists():
            return []
        try:
            # 账本可能长到 MB 级，但只需最后若干行：按块从尾部读，避免整文件载入。
            with open(self.path, "rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                block = 64 * 1024
                data = b""
                while size > 0 and data.count(b"\n") <= limit:
                    step = min(block, size)
                    size -= step
                    stream.seek(size)
                    data = stream.read(step) + data
            lines = data.decode("utf-8", "replace").strip().splitlines()[-limit:]
        except OSError:
            LOG.exception("ledger read failed")
            return []
        records = []
        for line in lines:
            try:
                parsed = json.loads(line)
            except ValueError:
                continue
            if isinstance(parsed, dict):
                records.append(parsed)
        return records

    def close(self) -> None:
        for handle in (self._handle, self._lock_handle):
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass
        self._handle = None


# ======================================================================================
# 证据归档
# ======================================================================================


class EvidenceArchive:
    """把"一次巡检到底发生了什么"从临时树里救出来，存成一份自包含的证据包。

    每份证据包是一个目录 `<证据根>/<YYYYMMDD-HHMMSS>-<task_id 前 8 位>/`：

    | 文件 | 内容 |
    |---|---|
    | `index.html` | 证据包目录页（可直接在浏览器里点开看照片、下 JSON） |
    | `summary.json` | 一句话结论 + 关键字段（状态/路线/起止/耗时/完成点/return_code） |
    | `events.jsonl` | 任务库里属于本 task_id 的**全部事件**（含原始 payload） |
    | `arrivals.jsonl` | 本时间窗内的到点事件（含来源文件名，可回溯到具体 spool 记录） |
    | `progress.json` | 巡检结束那一刻页面看到的快照（含步骤明细；只有最新一次有） |
    | `trace.json` / `command.log` | 从 `run_dir` 拷来的原始跑车记录（拷得到才拷） |
    | `photos/<点位>/` | 本次到点抓拍的现场照片（自包含副本） |

    幂等：索引 `index.json` 里记过 task_id 就直接返回既有条目，watcher 重启不会
    重复归档、也不会把同一份证据写两遍。
    """

    def __init__(self, root: Path) -> None:
        self.root = root
        self.entries: Dict[str, Dict[str, Any]] = {}
        self.load()

    @property
    def index_path(self) -> Path:
        return self.root / EVIDENCE_INDEX

    def load(self) -> None:
        try:
            raw = json.loads(self.index_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        runs = raw.get("runs") if isinstance(raw, dict) else None
        if isinstance(runs, dict):
            self.entries = {str(key): value for key, value in runs.items() if isinstance(value, dict)}

    def save(self) -> None:
        payload = {"schema_version": 1, "updated_at": time.time(), "runs": self.entries}
        temp = self.root / (EVIDENCE_INDEX + ".tmp")
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            os.replace(temp, self.index_path)
        except OSError as error:
            LOG.warning("evidence index write failed: %s", error)

    def get(self, task_id: Any) -> Optional[Dict[str, Any]]:
        return self.entries.get(str(task_id or ""))

    # -- 抓拍照片（到点抓拍先落在 `<证据根>/<task_id>/<点位>/`） ------------------------

    def photos_for(self, task_id: str) -> List[Dict[str, Any]]:
        base = self.root / task_id
        found: List[Dict[str, Any]] = []
        if not base.is_dir():
            return found
        try:
            candidates = sorted(base.rglob("*.jpg"))
        except OSError:
            return found
        for path in candidates:
            try:
                stat = path.stat()
                relative = path.relative_to(self.root).as_posix()
            except (OSError, ValueError):
                continue
            found.append(
                {
                    "name": path.name,
                    "waypoint": path.parent.name,
                    "path": str(path),
                    "url": "/inspection-evidence/" + relative,
                    "bytes": stat.st_size,
                    "at": stat.st_mtime,
                }
            )
        return found

    # -- 归档 -------------------------------------------------------------------------

    @staticmethod
    def _jsonl(records: List[Dict[str, Any]]) -> str:
        return "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in records)

    @staticmethod
    def _directory_index(summary: Dict[str, Any],
                         photos: List[Dict[str, Any]], copied: List[str]) -> str:
        """证据包的目录页。

        为什么自己生成一个 index.html：`INSPECTION_PHOTO_ROOT` 不在 studio/web 下，
        静态路由只会按文件返回，**不会列目录**。写一个静态页就省掉了"服务端要不要
        支持目录浏览"这个改动，页面和人都能直接点开。
        """

        def esc(value: Any) -> str:
            return (
                str(value)
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
                .replace('"', "&quot;")
            )

        # 链接一律指向**证据包内的相对路径**（photos/... / trace.json ...）：
        # 这样整包拷到别处、或换个挂载点，页面里的链接依然有效。
        thumbs = "".join(
            '<figure><a href="%(rel)s"><img src="%(rel)s" alt="%(wp)s" loading="lazy"></a>'
            "<figcaption>%(wp)s · %(name)s</figcaption></figure>"
            % {
                "rel": esc(photo.get("rel")),
                "wp": esc(photo.get("waypoint") or ""),
                "name": esc(photo.get("name") or ""),
            }
            for photo in photos
        ) or "<p>本次巡检没有抓拍到照片。</p>"
        files = "".join(
            '<li><a href="%s">%s</a></li>' % (esc(name), esc(name))
            for name in sorted(summary.get("bundle_files") or [])
        )
        return """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>巡检证据 %(task)s</title>
<style>
 body{background:#0f1620;color:#e6edf5;font:14px/1.6 system-ui,"Microsoft YaHei",sans-serif;margin:0;padding:24px}
 h1{font-size:18px;margin:0 0 4px} .meta{color:#93a4b8;margin-bottom:16px}
 a{color:#7cc4ff} figure{margin:0} img{max-width:320px;border-radius:10px;border:1px solid #22303f}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:14px}
 figcaption{color:#93a4b8;font-size:12px;margin-top:6px}
 ul{padding-left:18px} h2{font-size:15px;margin:20px 0 8px}
</style></head><body>
<h1>巡检证据 · %(task)s</h1>
<div class="meta">%(state)s · %(route)s · 开始 %(started)s · 结束 %(finished)s · 耗时 %(duration)s · 照片 %(photos)d 张</div>
<h2>现场照片</h2><div class="grid">%(thumbs)s</div>
<h2>归档文件</h2><ul>%(files)s</ul>
<p class="meta">原始跑车记录：%(copied)s</p>
</body></html>
""" % {
            "task": esc(summary.get("task_id")),
            "state": esc(summary.get("state_text")),
            "route": esc(" → ".join(summary.get("route") or []) or "—"),
            "started": esc(_clock(summary.get("started_at"))),
            "finished": esc(_clock(summary.get("finished_at"))),
            "duration": esc(_duration(summary.get("duration_s"))),
            "photos": len(photos),
            "thumbs": thumbs,
            "files": files,
            "copied": esc("；".join(copied) or "（没能取到）"),
        }

    def archive(
        self,
        run: Dict[str, Any],
        events: List[Dict[str, Any]],
        observations: List[Dict[str, Any]],
        snapshot: Optional[Dict[str, Any]] = None,
    ) -> Optional[Dict[str, Any]]:
        task_id = str(run.get("task_id") or "")
        if not task_id:
            return None
        cached = self.entries.get(task_id)
        if cached:
            return cached
        if str(run.get("state") or "") not in ip.TERMINAL_STATES:
            return None

        stamp = run.get("finished_at") or run.get("started_at") or time.time()
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            stamp = time.time()
        directory = self.root / (
            time.strftime("%Y%m%d-%H%M%S", time.localtime(stamp)) + "-" + task_id[:8]
        )
        try:
            directory.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            LOG.warning("evidence dir create failed task=%s: %s", task_id[:8], error)
            return None

        # 1) 从临时树里把原始跑车记录抢出来（拷不到就如实记下来）
        copied: List[str] = []
        run_dir = run.get("run_dir")
        missing: List[str] = []
        if run_dir:
            base = Path(str(run_dir))
            for name in EVIDENCE_RUN_FILES:
                source = base / name
                try:
                    if not source.is_file():
                        missing.append(name)
                        continue
                    if source.stat().st_size > EVIDENCE_MAX_FILE_BYTES:
                        missing.append(name + "(过大)")
                        continue
                    shutil.copyfile(source, directory / name)
                    copied.append(name)
                except OSError as error:
                    missing.append(name)
                    LOG.warning("evidence copy failed task=%s file=%s: %s", task_id[:8], name, error)

        # 2) 到点事件（只留 arrival，且带来源文件名，便于回溯）
        arrivals = [
            {
                "waypoint": item.get("waypoint"),
                "at": item.get("at"),
                "source": item.get("source"),
                "file": item.get("file"),
            }
            for item in observations
            if isinstance(item, dict) and item.get("kind") == "arrival"
        ]

        # 3) 现场照片：自包含地拷进证据包（原件仍留在 <根>/<task_id>/<点位>/）
        photos = self.photos_for(task_id)
        photos_dir = directory / "photos"
        kept: List[Dict[str, Any]] = []
        for photo in photos:
            point_dir = photos_dir / str(photo.get("waypoint") or "UNKNOWN")
            try:
                point_dir.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(photo["path"], point_dir / photo["name"])
            except OSError as error:
                LOG.warning("evidence photo copy failed %s: %s", photo.get("path"), error)
                continue
            kept.append(
                {
                    "waypoint": photo.get("waypoint"),
                    "name": photo["name"],
                    "rel": "photos/%s/%s" % (point_dir.name, photo["name"]),
                    "url": photo["url"],
                    "bytes": photo.get("bytes"),
                }
            )

        steps = (snapshot or {}).get("steps") or []
        completed = [
            str(step.get("id"))
            for step in steps
            if step.get("status") == "done" and step.get("kind") == "waypoint"
        ]
        started_at = run.get("started_at")
        finished_at = run.get("finished_at")
        duration = None
        if isinstance(started_at, (int, float)) and isinstance(finished_at, (int, float)):
            duration = round(float(finished_at) - float(started_at), 3)

        bundle_files: List[str] = []
        summary: Dict[str, Any] = {
            "schema_version": 1,
            "task_id": task_id,
            "state": run.get("state"),
            "state_text": ip.STATE_TEXT.get(str(run.get("state")), run.get("state")),
            "route": ip.parse_route(run.get("route")),
            "started_at": started_at,
            "finished_at": finished_at,
            "duration_s": duration,
            "completed_waypoints": completed,
            "return_code": run.get("return_code"),
            "run_dir": run.get("run_dir"),
            "trace_file": run.get("trace_file"),
            "arrival_count": len(arrivals),
            "photo_count": len(kept),
            "copied_run_files": copied,
            "missing_run_files": missing,
            "archived_at": time.time(),
            "archived_by": "inspection-progress-watcher",
        }

        # 4) 落盘（先写正文，最后写 summary 与 index.html —— 它们引用文件名列表）
        try:
            (directory / "events.jsonl").write_text(
                self._jsonl(
                    [
                        {
                            "type": item.get("type"),
                            "timestamp": item.get("timestamp"),
                            "payload": _payload_dict(item.get("payload")),
                        }
                        for item in events
                    ]
                ),
                encoding="utf-8",
            )
            bundle_files.append("events.jsonl")
            (directory / "arrivals.jsonl").write_text(self._jsonl(arrivals), encoding="utf-8")
            bundle_files.append("arrivals.jsonl")
            if snapshot is not None:
                (directory / "progress.json").write_text(
                    json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
                bundle_files.append("progress.json")
            bundle_files.extend(copied)
            if kept:
                bundle_files.append("photos/")
            summary["bundle_files"] = sorted(set(bundle_files))
            (directory / "summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            (directory / "index.html").write_text(
                self._directory_index(summary, kept, copied), encoding="utf-8"
            )
        except OSError as error:
            LOG.warning("evidence write failed task=%s: %s", task_id[:8], error)
            return None

        entry = {
            "task_id": task_id,
            "state": summary["state"],
            "state_text": summary["state_text"],
            "dir": str(directory),
            "url": "/inspection-evidence/%s/index.html" % directory.name,
            "saved_at": summary["archived_at"],
            "started_at": started_at,
            "finished_at": finished_at,
            "duration_s": duration,
            "completed_waypoints": completed,
            "photo_count": len(kept),
            "photos": kept,
            "files": sorted(set(bundle_files + ["summary.json", "index.html"])),
            "copied_run_files": copied,
            "missing_run_files": missing,
        }
        self.entries[task_id] = entry
        self.save()
        LOG.info(
            "evidence archived task=%s dir=%s files=%d photos=%d copied=%s",
            task_id[:8],
            directory.name,
            len(entry["files"]),
            len(kept),
            ",".join(copied) or "-",
        )
        return entry


def _clock(unix_seconds: Any) -> str:
    if not isinstance(unix_seconds, (int, float)) or unix_seconds <= 0:
        return "—"
    return time.strftime("%H:%M:%S", time.localtime(unix_seconds))


def _duration(seconds: Any) -> str:
    if not isinstance(seconds, (int, float)) or seconds < 0:
        return "—"
    total = int(seconds)
    return "%d:%02d:%02d" % (total // 3600, (total % 3600) // 60, total % 60)


# ======================================================================================
# 主循环
# ======================================================================================


class Watcher:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.db = TaskDBReader(config.task_db)
        self.traces = TraceCache()
        self.ledger = Ledger(config.ledger, config.lock_path)
        self.seen_files: Dict[str, float] = {}
        self.observations: List[Dict[str, Any]] = []
        self.notes: List[str] = []
        self._last_write_signature: Optional[str] = None
        self._last_write_at = 0.0
        self._last_heartbeat_log = 0.0
        self._stop = False
        self._started_at = time.time()
        self.evidence = EvidenceArchive(config.evidence_root)
        #: 最近一次 DB 折叠出来的 run 列表（新的在前）。到点抓拍要用它反查
        #: "这个到点事件属于哪一次巡检" —— 到点事件本身**不含 task_id**。
        self._runs: List[Dict[str, Any]] = []

    # -- 生命周期 ---------------------------------------------------------------------

    def stop(self, *_args: Any) -> None:
        self._stop = True

    def load_ledger(self) -> None:
        records = self.ledger.tail()
        for record in records:
            event = record.get("event")
            if isinstance(event, dict):
                self.observations.append(event)
            # Restore names of persisted facts and any surviving legacy ack.
            name = record.get("file")
            if isinstance(name, str) and name:
                self.seen_files[name] = record.get("observed_at") or time.time()
        if records:
            LOG.info("loaded %d ledger record(s)", len(records))
        # 账本里可能积累了多次巡检的到点记录；按时间排序便于后续按窗口归并。
        self.observations.sort(key=lambda item: item.get("at") or 0.0)

    # -- spool -----------------------------------------------------------------------

    def scan_spool(self) -> int:
        """扫到点事件目录，把还没见过的文件读进账本。**只读，绝不删。**

        三路都读：
        * `facts/*.json` —— 到点前先原子落盘，不受语音消费和回执清理影响；
        * `*.json` —— 路线客户端 `os.replace` 出来的原始事件（**权威时刻**，但只活 2~4 秒）；
        * `*.json.ack` —— 兼容旧回执；路线可能立即清理，不能依赖它的寿命。
        """

        directory = self.config.spool_dir
        if not directory.exists():
            return 0
        try:
            facts = [(path, False) for path in sorted((directory / "facts").glob("*.json"))]
            primary = [(path, False) for path in sorted(directory.glob("*.json"))]
            acks = [(path, True) for path in sorted(directory.glob("*.json.ack"))]
        except OSError:
            return 0

        present = set()
        captured = 0
        for path, from_ack in facts + primary + acks:
            name = path.relative_to(directory).as_posix()
            present.add(name)
            if name in self.seen_files:
                continue
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                # 半截/损坏的文件：标记已见，别每 0.3 s 重试一次刷日志。
                self.seen_files[name] = time.time()
                continue
            self.seen_files[name] = time.time()
            event = ip.parse_spool_event(self._with_waypoint_from_name(payload, name) if from_ack else payload)
            if event is None:
                continue
            source = "fact" if path.parent.name == "facts" else "ack" if from_ack else "spool"
            event = dict(event, file=name, source=source)
            if self._already_recorded(event):
                continue
            self.observations.append(event)
            self.ledger.append(
                {
                    "observed_at": time.time(),
                    "source": event["source"],
                    "file": name,
                    "event": event,
                }
            )
            captured += 1
            if event.get("kind") == "arrival":
                self._spawn_capture(event)

        # 已被语音服务 unlink 掉的文件名可以从内存里放掉（文件名带 uuid，不可能复用）。
        for name in [key for key in self.seen_files if key not in present]:
            self.seen_files.pop(name, None)
        return captured

    # -- 到点抓拍（best-effort；失败只记日志，绝不影响进度链路） ----------------------

    def _run_id_for_event(self, at: Any) -> Optional[str]:
        """反查到点事件属于哪一次巡检。

        到点事件**不带 task_id**（spool 里只有 event/waypoint/timestamp），所以只能按
        时间窗归属，判据与 `_arrivals_for()` 保持一致（前后各放宽 5 秒）。
        """

        if not isinstance(at, (int, float)):
            return None
        for run in self._runs:
            started = run.get("started_at")
            if not isinstance(started, (int, float)) or at < started - 5.0:
                continue
            finished = run.get("finished_at")
            if isinstance(finished, (int, float)) and at > finished + 5.0:
                continue
            task_id = str(run.get("task_id") or "")
            if task_id:
                return task_id
        return None

    def _spawn_capture(self, event: Dict[str, Any]) -> None:
        """把抓拍丢到后台线程：主循环 0.3 s 一轮，不能被一次 HTTP 阻塞。"""

        if not self.config.capture_enabled or not self.config.capture_url:
            return
        task_id = self._run_id_for_event(event.get("at"))
        if not task_id:
            LOG.info(
                "evidence capture skipped waypoint=%s: 时间窗内没有活跃巡检",
                event.get("waypoint"),
            )
            return
        threading.Thread(
            target=self._capture_waypoint_photo,
            args=(task_id, str(event.get("waypoint") or "").upper()),
            name="rvclaw-evidence-capture",
            daemon=True,
        ).start()

    def _capture_waypoint_photo(self, task_id: str, waypoint: str) -> None:
        if not waypoint:
            return
        body = json.dumps({"evidence_session": task_id, "waypoint": waypoint}).encode("utf-8")
        request = urllib.request.Request(
            self.config.capture_url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.config.capture_timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8") or "{}")
        except (urllib.error.URLError, OSError, ValueError) as error:
            LOG.warning(
                "evidence capture failed waypoint=%s task=%s: %s", waypoint, task_id[:8], error
            )
            return
        scene = (payload.get("evidence") or {}).get("scene_path")
        LOG.info(
            "evidence captured waypoint=%s task=%s scene=%s", waypoint, task_id[:8], scene
        )

    @staticmethod
    def _with_waypoint_from_name(payload: Any, name: str) -> Any:
        """回执里没有 `waypoint` 字段，点名只能从文件名里取回来。"""

        if not isinstance(payload, dict) or payload.get("waypoint"):
            return payload
        match = ARRIVAL_ACK_RE.match(name)
        if match is None:
            return payload
        return dict(payload, waypoint=match.group(1).upper())

    def _already_recorded(self, event: Dict[str, Any]) -> bool:
        """同一件事从两条路进来时不重复记账（回执比原文件晚到，靠时间近邻判定）。"""

        at = event.get("at")
        if not isinstance(at, (int, float)):
            return False
        for known in reversed(self.observations):
            if known.get("kind") != event.get("kind"):
                continue
            if known.get("waypoint") != event.get("waypoint"):
                continue
            known_at = known.get("at")
            if isinstance(known_at, (int, float)) and abs(at - known_at) <= ACK_DUPLICATE_WINDOW_S:
                return True
        return False

    # -- 快照 ------------------------------------------------------------------------

    def _arrivals_for(self, run: Optional[Dict[str, Any]]) -> Dict[str, float]:
        """只取落在本次巡检时间窗内的到点事件。

        到点事件**不含 task_id**，所以只能按时间窗口归属：早于开始/晚于结束的都不算，
        否则上一次巡检的到点会被算进这一次。
        """

        if not run:
            return {}
        started_at = run.get("started_at")
        finished_at = run.get("finished_at")
        window_start = (started_at - 5.0) if isinstance(started_at, (int, float)) else None
        window_end = (
            (finished_at + 5.0) if isinstance(finished_at, (int, float)) else None
        )
        selected = []
        for event in self.observations:
            at = event.get("at")
            if not isinstance(at, (int, float)):
                continue
            if window_start is not None and at < window_start:
                continue
            if window_end is not None and at > window_end:
                continue
            selected.append(event)
        return ip.arrival_map(selected)

    def _spool_events_for(self, run: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if not run:
            return []
        started_at = run.get("started_at")
        window_start = (started_at - 5.0) if isinstance(started_at, (int, float)) else None
        selected = []
        for event in self.observations:
            at = event.get("at")
            if at is None:
                continue
            if window_start is not None and at < window_start:
                continue
            selected.append(event)
        return selected

    # -- 终态时间兜底 ------------------------------------------------------------------

    def _backfill_finish(self, run: Dict[str, Any], upper_bound: Optional[float] = None) -> None:
        """终态却没写结束时间时，用 spool 的结束事件补上（就地改）。

        这是"失败态永远没有 `finished_at`"的直接解药：补上之后，`elapsed_s` 变成真实区间
        （页面上的时钟停住），`_arrivals_for()` 的窗口右端也随之合拢
        （后来别的活动的到点不会再被算进这次已死的巡检）。
        """

        if run.get("state") not in ip.TERMINAL_STATES or run.get("finished_at") is not None:
            return
        terminal = ip.first_terminal_event(
            self.observations, run.get("started_at"), upper_bound=upper_bound
        )
        if terminal is None:
            return
        run["finished_at"] = terminal.get("at")
        if run.get("return_code") is None:
            run["return_code"] = terminal.get("return_code")
        LOG.info(
            "finished_at backfilled task=%s state=%s at=%.3f return_code=%s",
            str(run.get("task_id") or "")[:8],
            run.get("state"),
            float(terminal["at"]),
            terminal.get("return_code"),
        )

    def _backfill_history(self, history: List[Dict[str, Any]]) -> None:
        """历史行同样补：下一行（更新的一次巡检）的开始时间就是本次的上界。"""

        for index, item in enumerate(history):
            newer_start = history[index - 1].get("started_at") if index > 0 else None
            had_finish = item.get("finished_at") is not None
            self._backfill_finish(item, upper_bound=newer_start)
            if not had_finish and item.get("finished_at") is not None:
                item["duration_s"] = ip.elapsed_seconds(
                    item.get("started_at"), item.get("finished_at"), item.get("finished_at")
                )

    def build_snapshot(self, now: Optional[float] = None) -> Dict[str, Any]:
        now = time.time() if now is None else now
        tasks, events_by_task = self.db.read()
        runs = fold_runs(tasks, events_by_task)
        self._runs = runs
        annotate_runs(runs[: self.config.history_limit], self.traces)

        notes: List[str] = []
        if self.db.diagnostic:
            notes.append("任务库：%s" % self.db.diagnostic)
        if self.traces.diagnostic:
            notes.append("跑车归档：%s" % self.traces.diagnostic)

        run = runs[0] if runs else None
        trace = self.traces.get(run.get("trace_file")) if run else {}

        if run:
            spool_events = self._spool_events_for(run)
            run = dict(run)
            run["state"] = ip.resolve_state(run.get("state"), spool_events)
            self._backfill_finish(run)
            arrivals = self._arrivals_for(run)
        else:
            arrivals = {}

        # 历史列表用 trace 的权威完成点，而不是可能漏采的到点事件。
        traces_by_task: Dict[str, Dict[str, Any]] = {}
        for item in runs[: self.config.history_limit]:
            loaded = self.traces.get(item.get("trace_file"))
            if loaded:
                traces_by_task[str(item.get("task_id"))] = loaded

        history = ip.summarise_history(runs, traces_by_task, limit=self.config.history_limit)
        self._backfill_history(history)

        snapshot = ip.summarise_run(
            run=run,
            arrivals=arrivals,
            trace=trace,
            history=history,
            notes=notes,
            now=now,
            default_route=self.config.default_route,
            camera_stream_url=self.config.camera_stream_url,
            host=self.config.host,
        )
        # Preserve identity even when startup failed before any photo/route trace.
        if run:
            for key in ("map_id", "failure_reason", "failure_phase"):
                if run.get(key) is not None:
                    snapshot[key] = run[key]
            if run.get('failure_phase') == 'return':
                for step in snapshot.get('steps', []):
                    if step.get('id') == 'ORIGIN' and step.get('status') != 'done':
                        step.update(status='failed', status_text='返航失败')
        contexts = {item["task_id"]: item for item in runs}
        for row in snapshot.get("history", []):
            context = contexts.get(row.get("task_id"), {})
            for key in ("map_id", "failure_reason", "failure_phase"):
                if context.get(key) is not None:
                    row[key] = context[key]
        snapshot["watcher"] = {
            "pid": os.getpid(),
            "uptime_s": round(now - self._started_at, 1),
            "spool_dir": str(self.config.spool_dir),
            "task_db": str(self.config.task_db),
            "observed_events": len(self.observations),
        }

        # -- 证据归档 ------------------------------------------------------------------
        # 对**最近几次**终态巡检都做一遍（不只是最新那次）：watcher 重启、或某次巡检
        # 结束时它正好没在跑，历史那几次也能补上。归档是幂等的，索引里有的直接返回。
        fresh_entry = None
        for candidate in runs[: self.config.history_limit]:
            candidate_id = str(candidate.get("task_id") or "")
            is_current = bool(run) and candidate_id == str(run.get("task_id") or "")
            entry = self.evidence.archive(
                candidate,
                events_by_task.get(candidate_id, []),
                self._spool_events_for(candidate),
                snapshot if is_current else None,
            )
            if entry is None:
                continue
            # `runs` 新的在前 ⇒ 第一次成功的那份就是"最近一次巡检"的证据。
            # 先当兜底，等轮到当前 run 再覆盖它（两者通常是同一条）。
            if fresh_entry is None or is_current:
                fresh_entry = entry
        snapshot["evidence"] = fresh_entry
        # ⚠️ `summarise_run` 里写的是 `"history": [dict(item) for item in history]`，
        # 是**浅拷贝**。所以必须改快照里那份 `snapshot["history"]`，改调用前的
        # 本地 `history` 列表是改给空气看的——那一份根本不会被序列化出去。
        for row in snapshot.get("history") or []:
            entry = self.evidence.get(row.get("task_id"))
            if entry:
                row["evidence_url"] = entry.get("url")
                row["evidence_photo_count"] = entry.get("photo_count")
        return snapshot

    # -- 输出 ------------------------------------------------------------------------

    @staticmethod
    def _signature(snapshot: Dict[str, Any]) -> str:
        comparable = {key: value for key, value in snapshot.items() if key != "generated_at"}
        comparable.pop("watcher", None)
        return json.dumps(comparable, ensure_ascii=False, sort_keys=True)

    def write_snapshot(self, snapshot: Dict[str, Any], force: bool = False) -> bool:
        signature = self._signature(snapshot)
        now = time.time()
        if (
            not force
            and signature == self._last_write_signature
            and now - self._last_write_at < HEARTBEAT_S
        ):
            return False

        self.config.output.parent.mkdir(parents=True, exist_ok=True)
        body = (json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        # 原子替换：studio 是"按请求现读"这个文件，非原子写会让页面读到半截 JSON。
        temp = self.config.output.with_suffix(self.config.output.suffix + ".tmp")
        temp.write_bytes(body)
        os.replace(temp, self.config.output)
        self._last_write_signature = signature
        self._last_write_at = now
        return True

    # -- 主循环 ----------------------------------------------------------------------

    def run(self, once: bool = False) -> int:
        self.ledger.open()
        self.load_ledger()
        LOG.info(
            "inspection-progress watcher started pid=%d spool=%s db=%s out=%s",
            os.getpid(),
            self.config.spool_dir,
            self.config.task_db,
            self.config.output,
        )

        # 先把 run 列表折叠出来再扫 spool：到点抓拍要靠它反查 task_id，
        # 而第一次 scan_spool 可能早于第一次 build_snapshot（开机后立刻有车到点）。
        try:
            self._runs = fold_runs(*self.db.read())
        except Exception:  # noqa: BLE001 - 任务库读不到不该拦住 watcher 起来
            LOG.exception("initial task db read failed (capture will start one cycle later)")
        captured = self.scan_spool()
        snapshot = self.build_snapshot()
        self.write_snapshot(snapshot, force=True)
        LOG.info(
            "initial snapshot state=%s steps=%d captured=%d",
            snapshot["state"],
            len(snapshot["steps"]),
            captured,
        )
        if once:
            self.ledger.close()
            return 0

        last_db = time.time()
        previous_state = snapshot["state"]
        try:
            while not self._stop:
                self.scan_spool()
                now = time.time()
                if now - last_db >= self.config.db_interval:
                    last_db = now
                    snapshot = self.build_snapshot(now)
                    if self.write_snapshot(snapshot):
                        if snapshot["state"] != previous_state:
                            LOG.info(
                                "state %s -> %s (progress %d%%)",
                                previous_state,
                                snapshot["state"],
                                snapshot["progress"]["percent"],
                            )
                            previous_state = snapshot["state"]
                    if now - self._last_heartbeat_log >= 300:
                        self._last_heartbeat_log = now
                        LOG.info(
                            "heartbeat state=%s progress=%d%% observed=%d",
                            snapshot["state"],
                            snapshot["progress"]["percent"],
                            len(self.observations),
                        )
                time.sleep(self.config.spool_interval)
        finally:
            self.ledger.close()
            LOG.info("inspection-progress watcher stopped")
        return 0


# ======================================================================================
# 入口
# ======================================================================================


def _parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RVClaw inspection progress watcher")
    parser.add_argument("--task-db", default=None, help="任务库路径")
    parser.add_argument("--spool-dir", default=None, help="到点事件目录")
    parser.add_argument("--output", default=None, help="页面轮询的 JSON 输出路径")
    parser.add_argument("--ledger", default=None, help="自有账本路径")
    parser.add_argument("--lock", default=None, help="单实例锁文件")
    parser.add_argument(
        "--once",
        action="store_true",
        help="只扫一次、写一份快照就退出（部署校验/离线自检用）",
    )
    parser.add_argument("--dump", action="store_true", help="把快照打到 stdout")
    parser.add_argument("--verbose", action="store_true", help="DEBUG 日志")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stdout,
    )
    config = Config(args)
    watcher = Watcher(config)
    signal.signal(signal.SIGTERM, watcher.stop)
    signal.signal(signal.SIGINT, watcher.stop)
    if args.dump and args.once:
        watcher.ledger = Ledger(config.ledger, config.lock_path)
        watcher.load_ledger()
        print(json.dumps(watcher.build_snapshot(), ensure_ascii=False, indent=2))
        return 0
    return watcher.run(once=args.once)


if __name__ == "__main__":
    raise SystemExit(main())
