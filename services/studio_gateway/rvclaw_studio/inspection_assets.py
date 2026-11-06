"""Read-only inspection photos and the map referenced by real route evidence."""
import json
import math
import os
import re
import shutil
import sqlite3
import threading
import textwrap
import sys
from datetime import datetime
from contextlib import closing
from functools import lru_cache
from pathlib import Path
from urllib.parse import parse_qs, quote

import cv2
import yaml
from . import inspection_maps
sys.path.insert(0,str(Path(__file__).resolve().parents[2] / 'inspection_profiles'))
from trace_summary import read_trace_summary
from trace_trajectory import read_trace_trajectory


TOKEN = re.compile(r"^[A-Za-z0-9_-]{1,96}$")
EDITABLE_POINTS = ("A", "B", "C")
SETTINGS_LOCK = threading.RLock()


class InspectionSettingsError(ValueError):
    """A validation error safe to return to the settings page."""


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def values(value, key):
    if isinstance(value, dict):
        if key in value:
            yield value[key]
        for item in value.values():
            yield from values(item, key)
    elif isinstance(value, list):
        for item in value:
            yield from values(item, key)


@lru_cache(maxsize=128)
def read_trace(path, modified):
    try:return read_trace_summary(path)
    except (OSError,ValueError):return {}


def trace_path(photo_root, run, index):
    entry = index.get("runs", {}).get(run.get("task_id"), {})
    bundle = photo_root / Path(entry.get("dir") or run.get("evidence_url") or "").name
    if run.get("evidence_url"):
        bundle = photo_root / run["evidence_url"].split("/")[-2]
    candidates = [bundle / "trace.json"]
    if run.get("run_dir"):
        candidates.append(Path(run["run_dir"]) / "trace.json")
    for path in candidates:
        if path.is_file():
            return path
    return None


def task_trace(photo_root, run, index):
    path = trace_path(photo_root, run, index)
    if path:
        return read_trace(str(path), path.stat().st_mtime_ns)
    return {}


def sessions(photo_root):
    if not photo_root.is_dir():
        return []
    return sorted((p.name for p in photo_root.iterdir()
                   if p.is_dir() and TOKEN.fullmatch(p.name)
                   and any(child.is_dir() and TOKEN.fullmatch(child.name)
                           for child in p.iterdir())), reverse=True)


def photo_rows(photo_root, session):
    directory=photo_root/session
    # Captures use unique filenames. Child directory mtimes change whenever a
    # photo/result is added, while immutable historical JSON need not be parsed
    # again after every unrelated upload or every progress poll.
    version=tuple(sorted((p.name,p.stat().st_mtime_ns) for p in directory.iterdir() if p.is_dir())) if directory.is_dir() else ()
    return _photo_rows(str(photo_root),session,version)


@lru_cache(maxsize=256)
def _photo_rows(root, session, version):
    photo_root=Path(root)
    rows = []
    for path in sorted((photo_root / session).glob("*/*-result.json")):
        saved = read_json(path)
        result = saved.get("result", {})
        stem = path.name.removesuffix("-result.json")
        scene = path.with_name(stem + "-scene.jpg")
        crop = path.with_name(stem + "-meter.jpg")
        if not scene.is_file():
            continue
        url = lambda p: "/inspection-photos/" + quote(p.relative_to(photo_root).as_posix(), safe="/")
        rows.append({"waypoint": path.parent.name,
                     "capture_index": saved.get("evidence", {}).get("capture_index"),
                     "captured_at": result.get("captured_at") or saved.get("saved_at"),
                     "value": result.get("value"), "unit": result.get("unit", ""),
                     "status": result.get("status"), "reason": result.get("reason"),
                     "meter_type": result.get("meter_type") or result.get("kind") or saved.get("meter_type") or saved.get("kind"),
                     "scene_url": url(scene), "meter_url": url(crop) if crop.is_file() else None})
    return rows


def map_file(photo_root, runs, index):
    for run in runs:
        for name in values(task_trace(photo_root, run, index), "map_file"):
            path = Path(str(name)).resolve()
            if path.is_relative_to(Path("/data")) and path.is_file():
                return path
    return None


@lru_cache(maxsize=12)
def _map_metadata(path, display_name, modified):
    if path is None:
        return {"available": False, "message": "暂无巡检扫描地图"}
    config = yaml.load(path.read_text(encoding="utf-8"),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))
    image = (path.parent / config["image"]).resolve()
    if not image.is_relative_to(path.parent.resolve()):
        raise ValueError("地图图片路径无效")
    pixels = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise ValueError("地图图片无法读取")
    catalog_path = path.parent / "mission_catalog.yaml"
    catalog = yaml.load(catalog_path.read_text(encoding="utf-8"),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader)) if catalog_path.is_file() else {}
    points = [{"name": name, "display_name": point.get("display_name", ""), "x": point["x_m"], "y": point["y_m"],
               "yaw": math.radians(point.get("yaw_deg", 0))}
              for name, point in (catalog or {}).get("locations", {}).items()]
    version = str(image.stat().st_mtime_ns)
    return {"available": True, "id": inspection_maps.identifier_for_directory(path.parent), "resolution": config["resolution"],
            "origin": config["origin"], "width": pixels.shape[1], "height": pixels.shape[0],
            "negate": config.get("negate", 0), "points": points,
            "updated_at": image.stat().st_mtime,
            "name": display_name or (inspection_maps.bundles().get(inspection_maps.identifier_for_directory(path.parent)) or {}).get("name", path.parent.name),
            "image_url": "/api/inspection-map/image.png?map_id=" + quote(inspection_maps.identifier_for_directory(path.parent)) + "&version=" + version}


def map_metadata(path, display_name=None):
    if path is None:
        return {"available": False, "message": "暂无巡检扫描地图"}
    path = Path(path)
    version = tuple((path.parent/name).stat().st_mtime_ns for name in ("map.yaml","waypoints.yaml","mission_catalog.yaml"))
    return dict(_map_metadata(path, display_name, version))


def _current_map_file(photo_root, studio_root, map_id=None):
    try:
        return inspection_maps.bundle(map_id)["directory"] / "map.yaml"
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        if map_id or inspection_maps.bundles():
            raise InspectionSettingsError(str(error)) from error
    snapshot = read_json(studio_root / "inspection-progress.json")
    runs = [snapshot, *snapshot.get("history", [])]
    return map_file(photo_root, runs, read_json(photo_root / "index.json"))


def _configured_path(name, fallback):
    value = os.getenv(name, "").strip()
    path = Path(value) if value else fallback
    if not path.is_file():
        raise InspectionSettingsError(f"配置文件不存在：{path}")
    return path


def _unique_files(*paths):
    result = []
    for path in paths:
        path = Path(path)
        if path.is_file() and path.resolve() not in [item.resolve() for item in result]:
            result.append(path)
    return result


def _settings_files(map_path):
    map_dir = map_path.parent
    same_map = Path(os.getenv("RVCLAW_NAV2_WAYPOINTS_FILE", str(map_dir / "waypoints.yaml"))).parent.name == map_dir.name
    active_waypoints = _configured_path("RVCLAW_NAV2_WAYPOINTS_FILE", map_dir / "waypoints.yaml") if same_map else map_dir / "waypoints.yaml"
    active_tasks = _configured_path("RVCLAW_INSPECTION_CONFIG", map_dir / "inspection_route_tasks.yaml") if same_map else map_dir / "inspection_route_tasks.yaml"
    source_waypoints = map_dir / "waypoints.yaml"
    source_tasks = map_dir / "inspection_route_tasks.yaml"
    active_catalog = active_waypoints.parent / "mission_catalog.yaml"
    return {
        "waypoints": _unique_files(active_waypoints, source_waypoints),
        "tasks": _unique_files(active_tasks, source_tasks),
        "catalogs": _unique_files(map_dir / "mission_catalog.yaml", active_catalog),
        "primary_waypoints": active_waypoints,
        "primary_tasks": active_tasks,
    }


def _yaml(path):
    try:
        value = yaml.load(path.read_text(encoding="utf-8"),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))
    except (OSError, yaml.YAMLError) as error:
        raise InspectionSettingsError(f"无法读取配置：{path.name}") from error
    if not isinstance(value, dict):
        raise InspectionSettingsError(f"配置格式无效：{path.name}")
    return value


def inspection_settings(photo_root, studio_root, meter_store, map_id=None):
    map_path = _current_map_file(photo_root, studio_root, map_id)
    if map_path is None:
        raise InspectionSettingsError("暂无可配置的巡检地图")
    files = _settings_files(map_path)
    waypoints = _yaml(files["primary_waypoints"]).get("waypoints", {})
    tasks = _yaml(files["primary_tasks"]).get("points", {})
    meters = {item["meter_id"]: item for item in meter_store.list()}
    points = []
    for name in (*EDITABLE_POINTS, "ORIGIN"):
        waypoint, task = waypoints.get(name), tasks.get(name, {})
        if not isinstance(waypoint, dict) or (name != "ORIGIN" and not isinstance(task, dict)):
            continue
        meter_id = str(task.get("meter_id", ""))
        meter = meters.get(meter_id, {})
        points.append({
            "point": name,
            "x_m": waypoint.get("x_m"),
            "y_m": waypoint.get("y_m"),
            "yaw_deg": waypoint.get("yaw_deg", task.get("inspection_yaw_deg", 0)),
            "meter_id": meter_id,
            "meter_name": "返航点" if name == "ORIGIN" else task.get("display_name") or meter.get("name") or meter_id,
            "meter_kind": "origin" if name == "ORIGIN" else task.get("kind", "digital_meter"),
            "unit": meter.get("unit", ""),
            "normal_min": task.get("normal_min"),
            "normal_max": task.get("normal_max"),
        })
    environment = map_metadata(map_path)
    environment["points"] = [
        {"name": item["point"], "x": item["x_m"], "y": item["y_m"],
         "yaw": math.radians(float(item["yaw_deg"] or 0))}
        for item in points
    ]
    return {"map": environment, "points": points, "editable_points": [*EDITABLE_POINTS, "ORIGIN"], **inspection_maps.catalog()}


def _number(value, label, minimum=None, maximum=None, optional=False):
    if optional and (value is None or value == ""):
        return None
    if isinstance(value, bool):
        raise InspectionSettingsError(f"{label}必须是数字")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise InspectionSettingsError(f"{label}必须是数字") from error
    if not math.isfinite(result):
        raise InspectionSettingsError(f"{label}必须是有限数字")
    if minimum is not None and result < minimum:
        raise InspectionSettingsError(f"{label}不能小于 {minimum}")
    if maximum is not None and result > maximum:
        raise InspectionSettingsError(f"{label}不能大于 {maximum}")
    return result


def _point_block(text, section, point):
    lines = text.splitlines(keepends=True)
    section_index = next((i for i, line in enumerate(lines) if re.fullmatch(fr"{re.escape(section)}:\s*\n?", line)), None)
    if section_index is None:
        raise InspectionSettingsError(f"配置缺少 {section}")
    point_index = next((i for i in range(section_index + 1, len(lines))
                        if re.fullmatch(fr"  {re.escape(point)}:\s*\n?", lines[i])), None)
    if point_index is None:
        raise InspectionSettingsError(f"配置缺少巡检点 {point}")
    end = len(lines)
    for i in range(point_index + 1, len(lines)):
        if re.match(r"^\S", lines[i]) or re.match(r"^  [A-Za-z0-9_-]+:\s*(?:#.*)?$", lines[i].rstrip("\r\n")):
            end = i
            break
    return lines, point_index, end


def _patch_point(path, section, point, replacements):
    lines, start, end = _point_block(path.read_text(encoding="utf-8"), section, point)
    missing = []
    for key, value in replacements.items():
        match = next((i for i in range(start + 1, end) if re.match(fr"^    {re.escape(key)}\s*:", lines[i])), None)
        if match is None:
            if value is not None:
                missing.append(f"    {key}: {value}\n")
        elif value is None:
            del lines[match]
            end -= 1
        else:
            lines[match] = f"    {key}: {value}\n"
    if missing:
        lines[start + 1:start + 1] = missing
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("".join(lines), encoding="utf-8")
    temporary.replace(path)


def _backup(paths):
    root = Path(os.getenv("RVCLAW_INSPECTION_SETTINGS_BACKUP_ROOT", "/data/RVClaw/backups/inspection-meter-settings"))
    target = root / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    target.mkdir(parents=True, exist_ok=False)
    for index, path in enumerate(paths, start=1):
        shutil.copy2(path, target / f"{index:02d}-{path.parent.name}-{path.name}")
    return target


def _write_point(path, section, point, mapping):
    lines, start, end = _point_block(path.read_text(encoding="utf-8"), section, point)
    replacement = "  " + point + ":\n" + textwrap.indent(yaml.safe_dump(mapping, allow_unicode=True, sort_keys=False), "    ")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("".join(lines[:start]) + replacement + "".join(lines[end:]), encoding="utf-8")
    temporary.replace(path)


def save_inspection_settings(photo_root, studio_root, meter_store, payload):
    with SETTINGS_LOCK:
        return _save_settings(photo_root, studio_root, meter_store, payload)


def _save_settings(photo_root, studio_root, meter_store, payload):
    if not isinstance(payload, dict):
        raise InspectionSettingsError("请求必须是对象")
    if read_json(studio_root / "inspection-progress.json").get("state") in {"running", "active", "in_progress"}:
        raise InspectionSettingsError("巡检任务正在运行，请在任务结束后修改点位设置")
    database = Path(os.getenv("RVCLAW_TASK_DB", "/data/RVClaw/runtime/voice_tasks.sqlite3"))
    if database.is_file():
        try:
            with closing(sqlite3.connect(database.as_uri() + "?mode=ro", uri=True, timeout=.2)) as connection:
                if connection.execute("SELECT 1 FROM tasks WHERE state IN ('created','accepted','running','active','in_progress') LIMIT 1").fetchone():
                    raise InspectionSettingsError("小车任务正在启动或运行，请在任务结束后修改巡检设置")
        except sqlite3.Error as error:
            raise InspectionSettingsError("暂时无法确认小车任务状态，请稍后重试") from error
    # Watcher updates may lag the task process by a few seconds.
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            arguments = (process / "cmdline").read_bytes().split(b"\0")
            names = {Path(argument.decode(errors="replace")).name for argument in arguments if argument}
            if b"--run" in arguments and names.intersection({"nav2_persistent_route.sh", "mapping_entry.sh"}):
                raise InspectionSettingsError("小车任务正在运行，请在任务结束后修改巡检设置")
        except (OSError, ProcessLookupError):
            pass
    map_id = payload.get("map_id") or inspection_maps.selected_id()
    try:
        selected_map = inspection_maps.bundle(map_id)
        map_name = inspection_maps.validate_name(payload.get("map_name", selected_map["name"]))
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        raise InspectionSettingsError(str(error)) from error
    if payload.get("save_point") is False:
        backup = _backup(_unique_files(*inspection_maps.state_files()))
        inspection_maps.save_selection(map_id, map_name)
        return {**inspection_settings(photo_root, studio_root, meter_store, map_id), "saved": True, "backup": str(backup)}
    point = str(payload.get("point", "")).strip().upper()
    if point not in (*EDITABLE_POINTS, "ORIGIN"):
        raise InspectionSettingsError("请选择有效的巡检点或返航点")
    origin = point == "ORIGIN"
    x_m = _number(payload.get("x_m"), "横坐标")
    y_m = _number(payload.get("y_m"), "纵坐标")
    yaw_deg = _number(payload.get("yaw_deg", 0), "朝向角度", -180, 180)
    normal_min = _number(payload.get("normal_min"), "正常最小值", optional=True)
    normal_max = _number(payload.get("normal_max"), "正常最大值", optional=True)
    if not origin and normal_min is None and normal_max is None:
        raise InspectionSettingsError("至少填写一个读数范围边界")
    if normal_min is not None and normal_max is not None and normal_min > normal_max:
        raise InspectionSettingsError("正常最小值不能大于正常最大值")

    map_path = _current_map_file(photo_root, studio_root, map_id)
    if map_path is None:
        raise InspectionSettingsError("暂无可配置的巡检地图")
    map_config = _yaml(map_path)
    image_path = (map_path.parent / map_config["image"]).resolve()
    pixels = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise InspectionSettingsError("地图图片无法读取")
    ox, oy, map_yaw = map_config["origin"]
    c, s, resolution = math.cos(map_yaw), math.sin(map_yaw), float(map_config["resolution"])
    dx, dy = x_m - ox, y_m - oy
    u = (c * dx + s * dy) / resolution
    v = (-s * dx + c * dy) / resolution
    px, py = round(u), round(pixels.shape[0] - v)
    if not (0 <= px < pixels.shape[1] and 0 <= py < pixels.shape[0]):
        raise InspectionSettingsError("所选巡检点超出地图范围")
    if int(pixels[py, px]) < 250:
        raise InspectionSettingsError("所选位置不是白色可通行区域，请重新点击")

    files = _settings_files(map_path)
    waypoint_document = _yaml(files["primary_waypoints"])
    task_document = _yaml(files["primary_tasks"])
    old_waypoint = waypoint_document.get("waypoints", {}).get(point)
    task = task_document.get("points", {}).get(point, {})
    if not isinstance(old_waypoint, dict) or not isinstance(task, dict):
        raise InspectionSettingsError(f"巡检点 {point} 的配置不完整")
    meter_id = str(task.get("meter_id", "")).strip()
    meter = meter_store.get(meter_id)
    if not origin and not meter_id:
        raise InspectionSettingsError(f"巡检点 {point} 未关联仪表任务")

    name = str(payload.get("meter_name", task.get("display_name") or (meter or {}).get("name") or meter_id)).strip()
    kind = str(payload.get("meter_kind", task.get("kind", "digital_meter")))
    if not origin and (not name or len(name) > 120 or any(ord(c) < 32 for c in name)):
        raise InspectionSettingsError("仪表名称需填写 1 至 120 个有效字符")
    if not origin and kind not in {"digital_meter", "pointer_meter"}:
        raise InspectionSettingsError("请选择数字仪表或指针仪表")

    target = task.get("target")
    distance = (math.hypot(float(target["x_m"]) - float(old_waypoint["x_m"]),
                           float(target["y_m"]) - float(old_waypoint["y_m"]))
                if isinstance(target, dict) else None)
    angle = math.radians(yaw_deg)
    updated_task = {**task, "display_name": name, "kind": kind}
    for key, value in [("normal_min", normal_min), ("normal_max", normal_max)]:
        if value is None:
            updated_task.pop(key, None)
        else:
            updated_task[key] = value
    if distance is not None:
        updated_task["target"] = {"x_m": round(x_m + distance * math.cos(angle), 3), "y_m": round(y_m + distance * math.sin(angle), 3)}
        updated_task.pop("inspection_yaw_deg", None)
    else:
        updated_task["inspection_yaw_deg"] = yaw_deg
        updated_task.pop("target", None)

    if kind != task.get("kind"):
        # A regex or scale belonging to the old type must not reject the new type.
        updated_task.pop("raw_text_pattern", None)
        if kind == "digital_meter":
            if updated_task.get("calibration"):
                updated_task["saved_pointer_calibration"] = updated_task.pop("calibration")
            updated_task.pop("auto_scale", None)
        else:
            calibration = updated_task.pop("saved_pointer_calibration", None)
            if calibration:
                updated_task["calibration"] = calibration
                updated_task.pop("auto_scale", None)
            elif not updated_task.get("calibration"):
                updated_task["auto_scale"] = True

    all_files = _unique_files(*files["waypoints"], *([] if origin else files["tasks"]), *files["catalogs"], *([meter_store.config_path] if not origin and meter else []))
    all_files = [*all_files, *inspection_maps.state_files()]
    backup = _backup(_unique_files(*all_files))
    originals = {path: path.read_bytes() if path.exists() else None for path in all_files}
    try:
        for path in files["waypoints"]:
            document = _yaml(path)
            updated = {**document["waypoints"][point], "x_m": x_m, "y_m": y_m, "yaw_deg": yaw_deg, "yaw_rad": angle,
                       "map_validation": "operator_selected_web", "field_validation": "pending"}
            _write_point(path, "waypoints", point, updated)
        for path in files["catalogs"]:
            updated = {**_yaml(path)["locations"][point], "x_m": x_m, "y_m": y_m, "yaw_deg": yaw_deg}
            if not origin:
                updated["display_name"] = name
            _write_point(path, "locations", point, updated)
        if not origin:
            for path in files["tasks"]:
                _write_point(path, "points", point, updated_task)
            if meter:
                updated_meter = {**meter, "name": name, "normal_min": normal_min, "normal_max": normal_max}
                meter_store.upsert(updated_meter)
        if payload.get("map_id"):
            inspection_maps.save_selection(map_id, map_name)
        result = inspection_settings(photo_root, studio_root, meter_store, map_id)
    except Exception:
        for path, contents in originals.items():
            if contents is None:
                path.unlink(missing_ok=True)
                continue
            temporary = path.with_suffix(path.suffix + ".rollback")
            temporary.write_bytes(contents)
            temporary.replace(path)
        raise
    result.update({"saved": True, "point": point, "backup": str(backup)})
    return result


def trace_map_id(trace):
    identifiers = {inspection_maps.identifier_for_directory(Path(str(value)).parent) for value in values(trace, "map_file")}
    # The aligned 09-23 bundle is a different coordinate frame. Never silently
    # relabel older unaligned captures as belonging to that map.
    return next(iter(identifiers)) if len(identifiers) == 1 else None


@lru_cache(maxsize=256)
def trace_summary(path, modified):
    trace = read_trace(path, modified)
    return trace_map_id(trace), tuple(set(str(s) for s in values(trace, "evidence_session") if isinstance(s, str) and TOKEN.fullmatch(s)))


def run_summary(photo_root, run, index):
    path = trace_path(photo_root, run, index)
    return trace_summary(str(path), path.stat().st_mtime_ns) if path else (None, ())


@lru_cache(maxsize=4)
def _archived_runs(root, index_version, interval):
    photo_root = Path(root)
    index = read_json(photo_root / "index.json")
    result = []
    seen = set()
    for task_id, entry in index.get("runs", {}).items():
        # Large K1 diagnostic traces stay at their original location. The small
        # archive summary preserves that location without copying the trace.
        directory=photo_root / Path(entry.get('dir') or '').name
        summary=read_json(directory/'summary.json')
        run = {**summary,**entry, "task_id": task_id, "evidence_url": entry.get("url")}
        identifier, related = run_summary(photo_root, run, index)
        result.append((run, identifier, related))
        seen.add(Path(entry.get("dir") or "").name)
    # Some failed older runs were never inserted into index.json.
    for path in photo_root.glob("*/trace.json"):
        if path.parent.name not in seen:
            summary = read_json(path.parent / "progress.json") or read_json(path.parent / "summary.json")
            identifier, related = trace_summary(str(path), path.stat().st_mtime_ns)
            result.append(({**summary, "run_dir": str(path.parent)}, identifier, related))
    return result


def session_associations(photo_root, snapshot=None, index=None):
    import time
    photo_root = Path(photo_root)
    snapshot, index = snapshot or {}, index or read_json(photo_root / "index.json")
    index_path = photo_root / "index.json"
    version = index_path.stat().st_mtime_ns if index_path.is_file() else 0
    candidates = list(_archived_runs(str(photo_root), version, int(time.monotonic() // 15)))
    extra = [run for run in snapshot.get("history", []) if run.get("task_id") not in index.get("runs", {})]
    for run in reversed([snapshot, *extra]):
        if run.get("task_id"):
            identifier, related = run_summary(photo_root, run, index)
            candidates.append((run, identifier, related))
    associations = {}
    for run, identifier, related in candidates:
        for session in related:
            if isinstance(session, str) and TOKEN.fullmatch(session):
                previous = associations.get(session)
                # Preserve evidence rather than guessing when metadata conflicts.
                if previous and previous["map_id"] != identifier:
                    associations[session] = {"map_id": None, "run": None}
                else:
                    associations[session] = {"map_id": identifier, "run": run}
    # A currently running task may not have written its final trace yet.
    # Use its explicit map metadata only, never the currently selected UI map.
    active_map = snapshot.get("map_id") or run_summary(photo_root, snapshot, index)[0]
    if active_map and snapshot.get("state") in {"running", "active", "in_progress"}:
        from zoneinfo import ZoneInfo
        for session in sessions(photo_root):
            if session in associations or not re.fullmatch(r"\d{8}-\d{6}", session):
                continue
            at = datetime.strptime(session, "%Y%m%d-%H%M%S").replace(tzinfo=ZoneInfo("Asia/Shanghai")).timestamp()
            if snapshot.get("started_at", float("inf")) <= at <= snapshot.get("generated_at", 0):
                associations[session] = {"map_id": active_map, "run": snapshot}
    return associations


def public_task(run, trace):
    if not run:
        return None
    raw = run.get("route") or trace.get("route") or trace.get("route_requested") or ["A", "B", "C", "ORIGIN"]
    def identifier(item):
        if isinstance(item, str):
            return item
        waypoint = item.get("waypoint", item.get("id", item.get("name"))) if isinstance(item, dict) else None
        return waypoint.get("name") if isinstance(waypoint, dict) else waypoint
    route = list(dict.fromkeys(name for item in raw if (name := identifier(item)) in {"A", "B", "C", "ORIGIN"}))
    completed = set(trace.get("completed_waypoints", run.get("completed_waypoints", [])))
    completed.update(identifier(item) for item in raw if isinstance(item, dict) and item.get("status") in {"PASS", "done", "completed"})
    steps = run.get("steps")
    if not steps:
        steps = [{"id": name, "kind": "origin" if name == "ORIGIN" else "waypoint", "status": "done" if name in completed else "pending"} for name in route]
    if run.get('failure_phase') == 'return':
        steps = [{**step, 'status': 'failed', 'status_text': '返航失败'} if step.get('id') == 'ORIGIN' and step.get('status') != 'done' else step for step in steps]
    fields = ("task_id", "state", "state_text", "started_at", "finished_at", "duration_s", "elapsed_s", "current_step_id", "return_code", "map_id", "failure_reason", "failure_phase")
    return {**{key: run[key] for key in fields if key in run}, "route": route, "steps": steps}


def assets(photo_root, studio_root, query):
    params = parse_qs(query)
    snapshot = read_json(studio_root / "inspection-progress.json")
    index = read_json(photo_root / "index.json")
    associations = session_associations(photo_root, snapshot, index)
    requested_map = (params.get("map_id") or [""])[0]
    selected = (params.get("session") or [""])[0]
    known = sessions(photo_root)
    if selected and selected not in known:
        raise ValueError("照片批次不存在")
    # Explicit source links may show an older photo without inventing its map.
    source = associations.get(selected, {})
    identifier = requested_map or (source.get("map_id") if selected else None) or inspection_maps.selected_id()
    available = inspection_maps.bundles()
    if requested_map and requested_map not in available:
        raise ValueError("地图不存在，请刷新后重试")
    if selected and requested_map and source.get("map_id") != requested_map:
        raise ValueError("所选任务不属于当前地图")
    filtered = [session for session in known if associations.get(session, {}).get("map_id") == identifier]
    run = source.get("run") if selected else None
    current_map = snapshot.get("map_id") or run_summary(photo_root, snapshot, index)[0]
    if not selected:
        if current_map == identifier and snapshot.get("task_id"):
            run = snapshot
            selected = next((session for session in filtered if associations[session]["run"] and associations[session]["run"].get("task_id") == run.get("task_id")), "")
        if not run and filtered:
            selected = filtered[0]
            run = associations[selected]["run"]
    task_id = (params.get("task_id") or [""])[0]
    if task_id:
        match = next(((session, item["run"]) for session, item in associations.items() if item["map_id"] == identifier and item["run"] and item["run"].get("task_id") == task_id), None)
        if not match:
            raise ValueError("所选任务不属于当前地图")
        selected, run = match
    if selected and (source.get("map_id") if params.get("session") else identifier) not in available:
        environment = {"available": False, "id": source.get("map_id"), "name": "历史采集记录", "message": "该历史记录的原始地图暂不可用，仍可查看原始采集图片。"}
    else:
        entry = available[identifier]
        environment = map_metadata(entry["directory"] / "map.yaml", entry["name"])
    trajectory = None
    if run and environment.get('available', True) and environment.get('id') == identifier:
        path = trace_path(photo_root, run, index)
        if path:
            try:
                trajectory = dict(task_id=run.get('task_id'), map_id=identifier,
                                  points=read_trace_trajectory(path))
            except (OSError, ValueError): pass
    run = public_task(run, task_trace(photo_root, run, index) if run else {})
    return {"task_id": run.get("task_id") if run else None, "task": run, "session": selected,
            "trajectory": trajectory,
            "association": "trace" if selected in associations else "unassigned", "sessions": filtered,
            "photos": photo_rows(photo_root, selected) if selected else [], "map": environment,
            "active_map_id": inspection_maps.selected_id()}


@lru_cache(maxsize=2)
def encode_map(path, modified):
    pixels = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise ValueError("地图图片无法读取")
    ok, encoded = cv2.imencode(".png", pixels)
    if not ok:
        raise ValueError("地图图片无法转换")
    return encoded.tobytes()


def map_image(photo_root, studio_root, query):
    snapshot = read_json(studio_root / "inspection-progress.json")
    runs = [snapshot, *snapshot.get("history", [])]
    identifier = (parse_qs(query).get("map_id") or [""])[0]
    path = inspection_maps.bundle(identifier, require_ready=False)["directory"] / "map.yaml" if identifier else _current_map_file(photo_root, studio_root)
    if path is None:
        raise FileNotFoundError("暂无地图")
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    image = (path.parent / config["image"]).resolve()
    if not image.is_relative_to(path.parent.resolve()):
        raise ValueError("地图图片路径无效")
    version = (parse_qs(query).get("version") or [""])[0]
    if version and version != str(image.stat().st_mtime_ns):
        raise ValueError("地图已更新，请刷新")
    return encode_map(str(image), image.stat().st_mtime_ns)
