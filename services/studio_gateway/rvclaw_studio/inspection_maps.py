"""Validated map bundles and a durable selection consumed at task startup."""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
from functools import lru_cache

import yaml

TOKEN = re.compile(r"^[A-Za-z0-9_-]{1,96}$")
DEFAULT_ROOTS = (
    "/data/robot-inspection-stable-20261012/RVClaw/configs/scenes",
    "/data/rvclaw-k3-lab/tmp/newmap-aligned-20261004/stage/maps",
)
MAP_NAMES = {
    "inspection-eb72268b158a": "走廊巡检",
    "slam-20260827-105711": "办公室巡检",
    "voice-slam-20261006-110834-66940a96": "走廊巡检",
    "voice-slam-20260923-152736-b400c9b9-aligned-20261004": "巡检地图 · 09-23",
}


def state_root():
    return Path(os.getenv("RVCLAW_INSPECTION_PROFILE_ROOT", "/data/RVClaw/state"))


def state_files():
    return [state_root() / name for name in ("inspection-profile.json", "inspection-map-names.json")]


def read_json(path):
    try:
        value = path.read_text(encoding="utf-8")
        return json.loads(value)
    except (OSError, ValueError):
        return {}


def roots():
    value = os.getenv("RVCLAW_INSPECTION_MAP_ROOTS", "")
    return [Path(p).resolve() for p in value.split(os.pathsep) if p] if value else [Path(p).resolve() for p in DEFAULT_ROOTS]


def _yaml(path):
    path=Path(path)
    return _cached_yaml(str(path),path.stat().st_mtime_ns)


@lru_cache(maxsize=64)
def _cached_yaml(path,modified):
    value = yaml.load(Path(path).read_text(encoding="utf-8"),Loader=getattr(yaml,'CSafeLoader',yaml.SafeLoader))
    if not isinstance(value, dict):
        raise ValueError("配置格式不完整")
    return value


def validate_bundle(directory):
    """Validate identities and finite point data before exposing a selectable map."""
    map_path = directory / "map.yaml"
    data = _yaml(map_path)
    image = (directory / str(data.get("image", ""))).resolve()
    if not image.is_relative_to(directory.resolve()) or not image.is_file():
        raise ValueError("地图图片不完整")
    resolution = float(data.get("resolution", 0))
    origin = data.get("origin", [])
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError("地图坐标配置无效")
    if not all((directory / name).is_file() for name in ("waypoints.yaml", "inspection_route_tasks.yaml", "mission_catalog.yaml")):
        raise ValueError("尚未配置完整的巡检点位")
    waypoint = _yaml(directory / "waypoints.yaml")
    catalog = _yaml(directory / "mission_catalog.yaml")
    task = _yaml(directory / "inspection_route_tasks.yaml")
    for document in (waypoint, catalog):
        if document.get("map_id") != identifier_for_directory(directory) or (directory / str(document.get("map_yaml", "map.yaml"))).resolve() != map_path.resolve():
            raise ValueError("点位与地图未正确关联")
    for name in ("A", "B", "C", "ORIGIN"):
        point = waypoint.get("waypoints", {}).get(name)
        location = catalog.get("locations", {}).get(name)
        if not isinstance(point, dict) or not isinstance(location, dict):
            raise ValueError("巡检点位或返航点不完整")
        if not all(math.isfinite(float(point[key])) for key in ("x_m", "y_m", "yaw_deg")):
            raise ValueError("点位坐标无效")
        if name != "ORIGIN" and not isinstance(task.get("points", {}).get(name), dict):
            raise ValueError("仪表任务配置不完整")
    return map_path


def default_id():
    path = os.getenv("RVCLAW_NAV2_WAYPOINTS_FILE", "")
    if path:
        try: return _yaml(Path(path)).get("map_id") or Path(path).parent.name
        except (OSError,ValueError,yaml.YAMLError): pass
    return "voice-slam-20261006-110834-66940a96"


def default_name(identifier):
    if identifier in MAP_NAMES:
        return MAP_NAMES[identifier]
    date = re.search(r"(20\d{6})", identifier)
    return "巡检地图 · " + (date[1][4:6] + "-" + date[1][6:8] if date else identifier)


def registry():
    path = os.getenv("RVCLAW_STUDIO_MAP_REGISTRY", "")
    value = read_json(Path(path)) if path else {}
    return value.get("maps", [])


def identifier_for_directory(directory):
    directory = Path(directory).resolve()
    for entry in registry():
        if Path(entry["directory"]).resolve() == directory:
            return entry["id"]
    try:
        return _yaml(directory / "waypoints.yaml").get("map_id") or directory.name
    except (OSError, ValueError, yaml.YAMLError):
        return directory.name


def bundles():
    registered = registry()
    if registered:
        result = {}
        names = read_json(state_files()[1])
        for entry in registered:
            directory = Path(entry["directory"]).resolve()
            identifier = entry["id"]
            if not TOKEN.fullmatch(identifier) or not any(directory.is_relative_to(root) for root in roots()):
                continue
            try:
                validate_bundle(directory)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                continue
            result[identifier] = {"id":identifier,"name":names.get(identifier) or entry["name"],"ready":True,"reason":"","directory":directory}
        return result

    result = {}
    names = read_json(state_files()[1])
    for root in roots():
        if not root.is_dir():
            continue
        directories = [p for p in root.iterdir() if p.is_dir()]
        historical = root / "historical"
        if historical.is_dir():
            directories.extend(p for p in historical.iterdir() if p.is_dir())
        for directory in sorted(directories):
            identifier = directory.name
            if identifier not in MAP_NAMES or identifier in result or not TOKEN.fullmatch(identifier) or not (directory / "map.yaml").is_file() or not directory.resolve().is_relative_to(root):
                continue
            ready, reason = True, ""
            try:
                validate_bundle(directory)
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                ready, reason = False, "尚未配置完整的巡检点位"
            if ready:
                result[identifier] = {"id": identifier, "name": names.get(identifier) or default_name(identifier), "ready": True, "reason": "", "directory": directory.resolve()}
    return result


def selected_id():
    profile = read_json(state_files()[0])
    return profile.get("map_id") or default_id()


def bundle(identifier=None, require_ready=True):
    identifier = identifier or selected_id()
    if not isinstance(identifier, str) or not TOKEN.fullmatch(identifier):
        raise ValueError("地图选项无效")
    item = bundles().get(identifier)
    if not item:
        raise ValueError("地图不存在，请刷新后重试")
    if require_ready:
        validate_bundle(item["directory"])
    return item


def catalog():
    return {"active_map_id": selected_id(), "maps": [{key: value for key, value in item.items() if key != "directory"} for item in bundles().values()]}


def validate_name(value):
    if not isinstance(value, str) or not value.strip() or len(value.strip()) > 80 or any(ord(c) < 32 for c in value):
        raise ValueError("地图名称需填写 1 至 80 个有效字符")
    return value.strip()


def save_selection(identifier, name):
    item = bundle(identifier)
    name = validate_name(name)
    directory = item["directory"]
    profile_path, names_path = state_files()
    documents = {names_path: {**read_json(names_path), identifier: name}, profile_path: {
        "schema_version": 1, "map_id": identifier, "name": name,
        "map_file": str(directory / "map.yaml"), "waypoints_file": str(directory / "waypoints.yaml"),
        "inspection_config": str(directory / "inspection_route_tasks.yaml"), "catalog_file": str(directory / "mission_catalog.yaml"),
        "localization_cache": next((e.get("localization_cache") for e in registry() if e["id"] == identifier), None),
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }}
    originals = {path: path.read_bytes() if path.exists() else None for path in documents}
    try:
        for path, data in documents.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(path)
    except Exception:
        for path, contents in originals.items():
            if contents is None:
                path.unlink(missing_ok=True)
            else:
                path.write_bytes(contents)
        raise
    return catalog()
