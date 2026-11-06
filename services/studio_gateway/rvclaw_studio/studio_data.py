"""Small request-driven telemetry and durable user-facing inspection records.

No worker, model, camera or ROS dependency. File scans are bounded and cached.
"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import threading
import time


def _json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {} if default is None else default


def _iso(value):
    if isinstance(value, (int, float)):
        return datetime.fromtimestamp(value, timezone.utc).isoformat()
    return str(value or "")


def _epoch(value):
    try:
        return float(value) if isinstance(value, (float, int)) else datetime.fromisoformat(str(value).replace("Z", "+00:00")).timestamp()
    except (ValueError, TypeError, OSError):
        return 0


def successful(row):
    try:
        numeric = not isinstance(row.get("value"), bool) and math.isfinite(float(row.get("value")))
    except (TypeError, ValueError):
        numeric = False
    return numeric and str(row.get("status", "")).lower() in {"stable", "recognized", "ok", "success", "normal", "abnormal", "out_of_range"}


def failure_reason(reason, status=""):
    text = str(reason or "")
    if "403" in text or "401" in text:
        return "识别服务拒绝了请求，请稍后重试；持续失败时请检查服务授权。"
    if "429" in text:
        return "识别服务繁忙，请稍后重试。"
    if re.search(r"timeout|unavailable|offline|HTTP 5", text, re.I) or status in {"inference_error", "inference_offline"}:
        return "识别服务暂时不可用，请稍后重试。"
    if status == "low_confidence":
        return "读数可信度不足，请更换清晰、正对仪表的图片。"
    if status in {"invalid_request", "unsupported"}:
        return "当前图片或仪表类型无法识别，请重新选择。"
    if text and re.search(r"[\u4e00-\u9fff]", text) and not re.search(r"/data/|Traceback|api[_-]?key", text, re.I):
        return text[:250]
    return "未能可靠读取仪表，请减少反光并确保数字或刻度清晰可见。"


class Telemetry:
    def __init__(self, proc=Path("/proc"), disk=Path("/data")):
        self.proc, self.disk = Path(proc), Path(disk)
        self.lock = threading.Lock()
        self.previous = self._cpu()
        self.updated = 0.0
        self.cached = {}
        self.history = deque(maxlen=120)

    def _cpu(self):
        try:
            values = [int(v) for v in (self.proc / "stat").read_text().splitlines()[0].split()[1:9]]
            return sum(values), values[3] + values[4]
        except (OSError, ValueError, IndexError):
            return None

    def sample(self):
        with self.lock:
            now = time.monotonic()
            if now - self.updated < 1 and self.cached:
                return self.cached
            current, usage = self._cpu(), None
            if current and self.previous:
                total, idle = current[0] - self.previous[0], current[1] - self.previous[1]
                if total > 0:
                    usage = round(max(0.0, min(100.0, 100.0 * (total - idle) / total)), 1)
            self.previous = current
            memory, storage = {}, {}
            try:
                fields = {}
                for line in (self.proc / "meminfo").read_text().splitlines():
                    key, rest = line.split(":", 1)
                    fields[key] = int(rest.split()[0]) * 1024
                total = fields["MemTotal"]
                available = fields.get("MemAvailable", fields.get("MemFree", 0) + fields.get("Buffers", 0) + fields.get("Cached", 0))
                memory = {"total_bytes": total, "used_bytes": max(0, total - available), "available_bytes": available}
            except (OSError, ValueError, KeyError, IndexError):
                pass
            try:
                disk = shutil.disk_usage(self.disk if self.disk.exists() else Path("/"))
                storage = {"total_bytes": disk.total, "used_bytes": disk.used, "available_bytes": disk.free}
            except OSError:
                pass
            sampled = datetime.now(timezone.utc).isoformat()
            if usage is not None:
                self.history.append({"time": sampled, "cpu_percent": usage})
            self.cached = {"usage_percent": usage, "memory": memory, "storage": storage, "history": list(self.history)}
            self.updated = now
            return self.cached


class StudioData:
    def __init__(self, repo, photos, studio, read_uploads, read_logs, helmets):
        self.repo, self.photos, self.studio = Path(repo), Path(photos), Path(studio)
        self.read_uploads, self.read_logs, self.helmets = read_uploads, read_logs, helmets
        self.state = Path(os.getenv("RVCLAW_STUDIO_STATE_ROOT", "/data/RVClaw/state/studio"))
        self.lock = threading.RLock()
        self.cache = None
        self.cached_at = 0.0
        self.safety_events = lambda: []

    def invalidate(self):
        with self.lock:
            self.cache = None

    def meter_records(self):
        with self.lock:
            if self.cache is not None and time.monotonic() - self.cached_at < 30:
                return self.cache
            from .inspection_assets import photo_rows, sessions, session_associations
            uploads = self.repo / "runs" / "meter-uploads"
            rows = list(self.read_uploads())
            superseded={row['supersedes_evidence_id'] for row in rows if row.get('supersedes_evidence_id')}
            superseded.update(identifier for row in rows
                              for identifier in (row.get('supersedes_evidence_ids') or [])
                              if isinstance(identifier,str) and re.fullmatch(r'[a-f0-9]{32}',identifier))
            rows=[row for row in rows if row.get('evidence_id') not in superseded]
            known = {str(row.get("evidence_id")) for row in rows} | superseded
            if uploads.is_dir():
                for path in sorted(uploads.glob("*.json"), reverse=True)[:2000]:
                    if not re.fullmatch(r"[a-f0-9]{32}", path.stem) or path.stem in known:
                        continue
                    row = _json(path)
                    if isinstance(row, dict) and "status" in row:
                        rows.append({**row, "evidence_id": path.stem, "image_url": "/api/meter-upload-records/" + path.stem + "/image.jpg"})
            result = [self._meter(row, "相机拍照" if row.get("source") == "camera_capture" else "上传图片") for row in rows]
            associations = session_associations(self.photos)
            for session in sessions(self.photos)[:120]:
                for row in photo_rows(self.photos, session):
                    result.append(self._meter({**row, "session": session, "map_id": associations.get(session, {}).get("map_id")}, "巡检采集"))
            trials = self.repo / "runs" / "meter-trials"
            if trials.is_dir():
                for path in sorted(trials.glob("*-result.json"), reverse=True)[:300]:
                    trial_id = path.name.removesuffix("-result.json")
                    if not re.fullmatch(r"[a-f0-9]{32}", trial_id):
                        continue
                    data = _json(path)
                    row = data.get("trial", data.get("result", data))
                    if isinstance(row, dict) and "status" in row:
                        result.append(self._meter({**row, "evidence_id": "trial-" + trial_id, "image_url": "/api/meter-trials/" + trial_id + "/scene.jpg"}, "历史试读"))
            unique = {}
            for row in result:
                unique[row["id"]] = row
            self.cache = sorted(unique.values(), key=lambda row: _epoch(row.get("created_at")), reverse=True)[:1500]
            self.cached_at = time.monotonic()
            return self.cache

    def _meter(self, row, source):
        status = str(row.get("status") or "unreadable")
        kind = row.get("kind") or row.get("meter_type") or ("pointer" if row.get("waypoint") == "C" else "digital")
        if kind in {"digital_meter", "pointer_meter"}:
            kind = kind.removesuffix("_meter")
        image = row.get("image_url") or row.get("scene_url") or row.get("evidence_url")
        identifier = row.get("evidence_id") or hashlib.sha256(str(image or json.dumps(row, sort_keys=True)).encode()).hexdigest()[:32]
        ok = successful(row)
        return {"id": str(identifier), "evidence_id": str(identifier), "created_at": _iso(row.get("created_at") or row.get("captured_at")),
                "kind": kind, "status": status, "value": row.get("value") if ok else None, "unit": row.get("unit") or "",
                "result_state": "success" if ok else "failure", "result_text": "识别成功" if ok else "识别失败",
                "reason_zh": "" if ok else failure_reason(row.get("reason"), status), "image_url": image,
                "source_label": source, "waypoint": row.get("waypoint"), "session": row.get("session"), "map_id": row.get("map_id"),
                "out_of_range": status.lower() in {"out_of_range", "abnormal"}, "confidence": row.get("confidence")}

    def notes(self):
        value = _json(self.state / "event-notes.json", {})
        return value if isinstance(value, dict) else {}

    def save_note(self, payload):
        identifier = str(payload.get("event_id") or "")
        text = payload.get("text", "")
        if not identifier or len(identifier) > 500 or any(ord(c) < 32 for c in identifier):
            raise ValueError("记录标识无效")
        if not isinstance(text, str) or len(text) > 2000:
            raise ValueError("事件说明最多可输入 2000 个字")
        with self.lock:
            # Accept only records supplied by this service or the bundled demo clips.
            valid = {r["id"] for r in self.events(include_notes=False)} | {"video-demo-people", "video-demo-factory", "video-demo-corridor"}
            if identifier not in valid:
                raise ValueError("记录不存在，请刷新后重试")
            notes = self.notes()
            previous = notes.get(identifier, {})
            record = {"text": text, "handled": bool(payload.get("handled", previous.get("handled", False))), "updated_at": datetime.now(timezone.utc).isoformat()}
            notes[identifier] = record
            self.state.mkdir(parents=True, exist_ok=True)
            target = self.state / "event-notes.json"
            temp = target.with_suffix(".tmp")
            temp.write_text(json.dumps(notes, ensure_ascii=False, indent=2), encoding="utf-8")
            os.replace(temp, target)
            return {"saved": True, "event_id": identifier, "note": record}

    def events(self, include_notes=True):
        from .inspection_maps import MAP_NAMES
        result = []
        for row in self.meter_records():
            failed, abnormal = row["result_state"] == "failure", row["out_of_range"]
            kind = "指针仪表" if row["kind"] == "pointer" else "数字仪表"
            result.append({"id": row["id"], "title": kind + ("识别失败" if failed else "读数超出范围" if abnormal else "识别成功"),
                           "type": "识别异常" if failed else "读数异常" if abnormal else "仪表记录", "time": row["created_at"],
                           "image": row["image_url"], "alert": failed or abnormal, "description": row["reason_zh"] or kind + "读数完成。",
                           "value": "未记录" if row["value"] is None else str(row["value"]),
                           "source_label": row["source_label"], "category": "meter", "result_state": row["result_state"],
                           "related": ({"page": "inspection", "target": {"session": row["session"], "map_id": row.get("map_id") if row.get("map_id") in MAP_NAMES else None, "photo": row["image_url"], "waypoint": row.get("waypoint")}} if row.get("session") else {"page": "meter", "target": {"record": row["id"]}})})
        for row in self.helmets.list_records():
            demo = row.get("source") == "debug_upload"
            count = int(row.get("no_helmet") or 0)
            result.append({"id": row.get("evidence_id") or str(row.get("created_at")), "title": "发现人员未佩戴安全帽" if count else "安全帽检测完成",
                           "type": "上传测试" if demo else "安全告警" if count else "安全记录", "time": _iso(row.get("created_at")),
                           "image": row.get("image_url") or row.get("original_image_url"), "alert": bool(count and not demo), "demo": demo,
                           "description": ("上传测试图片。" if demo else "") + ("发现 %d 人未佩戴安全帽。" % count if count else "未发现未佩戴安全帽的人员。"),
                           "value": "未记录", "source_label": "上传测试" if demo else "现场相机", "category": "safety", "result_state": "failure" if count else "success"})
        result.extend(self.safety_events())
        notes = self.notes() if include_notes else {}
        for row in result:
            row["note"] = notes.get(row["id"], {})
        return sorted(result, key=lambda row: _epoch(row.get("time")), reverse=True)

    def logs(self, limit=1000):
        result = []
        for event in self.events():
            result.append({"id": "event-" + event["id"], "time": event["time"], "category": "仪表识别" if event["category"] == "meter" else "安全检测",
                           "summary": event["title"], "description": event["description"], "failed": bool(event["alert"] or event["result_state"] == "failure"),
                           "source_label": event["source_label"], "image_url": event["image"], "value": event["value"], "event_id": event["id"], "snapshot": None})
        for index, row in enumerate(self.read_logs().get("logs", [])):
            text = str(row.get("message") or "")
            match = re.search(r'"(GET|POST|PUT|DELETE|PATCH)\s+([^?\s"]+)(?:\?[^\s"]*)?\s+HTTP/[\d.]+"\s+(\d{3})', text)
            failed = bool(re.search(r"error|exception|failed", text, re.I)) or row.get("level") in {"ERROR", "WARN"}
            category, summary = "系统运行", "服务运行记录"
            if match:
                method, path, code = match.groups()
                failed = int(code) >= 400
                # Background polling is not a user operation. Keep actual
                # recognition/task records and unsuccessful service requests.
                if method in {"GET", "HEAD"} and not failed:
                    continue
                category, summary = "网页服务", "页面数据读取"
                for prefix, group, label in [("/api/device", "设备状态", "设备状态更新"), ("/api/camera", "设备连接", "相机状态读取"),
                    ("/api/inspection-meter-settings", "点位设置", "巡检点位设置保存" if method == "POST" else "巡检点位设置读取"),
                    ("/inspection-progress", "巡检任务", "巡检进度更新"), ("/api/meter", "仪表识别", "仪表识别记录读取"),
                    ("/api/inspection", "巡检任务", "巡检资料读取"), ("/stream", "设备连接", "相机画面连接")]:
                    if path.startswith(prefix):
                        category, summary = group, label
                        break
                summary += "失败" if failed else "完成"
            elif failed:
                summary = "网页服务出现异常"
            result.append({"id": "system-" + hashlib.sha256((str(row.get("time")) + text + str(index)).encode()).hexdigest()[:24],
                           "time": row.get("time"), "category": category, "summary": summary, "failed": failed,
                           "description": "操作未能完成，请检查设备连接后重试。" if failed else "设备服务已完成本次操作。",
                           "source_label": "系统服务", "snapshot": None})
        ordered = sorted(result, key=lambda row: _epoch(row.get("time")), reverse=True)
        return ordered if limit is None else ordered[:limit]
