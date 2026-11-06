#!/usr/bin/env python3
"""Consume a Studio map profile at startup; never move or arm the chassis.

Only a user-started route may reload an idle navigation stack when its live map
differs. HTTP saves and --shell/--describe never restart services or access ROS.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

DEFAULT_ROOTS = (
    "/data/RVClaw/configs/scenes",
    "/data/RVClaw/configs/studio-maps",
)
SERVICE = "rvclaw-nav2-persistent.service"


def profile_path():
    return Path(os.getenv("RVCLAW_INSPECTION_PROFILE_ROOT", "/data/RVClaw/state")) / "inspection-profile.json"


def read_profile():
    path = profile_path()
    if not path.exists():
        return None
    profile = json.loads(path.read_text(encoding="utf-8"))
    identifier = profile.get("map_id", "")
    if profile.get("schema_version") != 1 or not re.fullmatch(r"[A-Za-z0-9_-]{1,96}", identifier):
        raise ValueError("巡检地图设置无效，请重新保存")
    value = os.getenv("RVCLAW_INSPECTION_MAP_ROOTS", "")
    roots = [Path(p).resolve() for p in value.split(os.pathsep) if p] if value else [Path(p).resolve() for p in DEFAULT_ROOTS]
    directory = Path(profile["map_file"]).resolve().parent
    if not any(directory.is_relative_to(root) for root in roots):
        raise ValueError("巡检地图路径不在已登记的地图目录内")
    for key, filename in (("map_file", "map.yaml"), ("waypoints_file", "waypoints.yaml"), ("inspection_config", "inspection_route_tasks.yaml"), ("catalog_file", "mission_catalog.yaml")):
        target = Path(profile[key]).resolve()
        if target != directory / filename or not target.is_file():
            raise ValueError("巡检地图与点位配置不完整")
    import yaml
    for filename in ("waypoints.yaml", "mission_catalog.yaml"):
        if yaml.safe_load((directory/filename).read_text())["map_id"] != identifier:
            raise ValueError("巡检地图与实际任务坐标系不一致")
    return profile


def environment(profile):
    if not profile:
        return {}
    navigation = Path(os.getenv("RVCLAW_PROFILE_NAVIGATION_ROOT", "/data/RVClaw/runtime/k1/navigation"))
    return {
        "RVCLAW_SITE_DIR": str(Path(profile["map_file"]).parent),
        "RVCLAW_MAP_FILE": profile["map_file"],
        "RVCLAW_NAV2_WAYPOINTS_FILE": profile["waypoints_file"],
        "RVCLAW_INSPECTION_CONFIG": profile["inspection_config"],
        "RVCLAW_LOCALIZATION_CACHE": str(profile.get("localization_cache") or navigation / "state" / ("localization-pose-" + profile["map_id"] + ".json")),
    }


def verify_map(verifier, map_file):
    result = subprocess.run(["python3", str(verifier), "--map-file", str(map_file)], text=True, capture_output=True, timeout=12)
    return result.returncode == 0, result.stdout + result.stderr


def conflicting_tasks():
    """Exclude our own parent route, and refuse another route or mapping task."""
    parent = os.getppid()
    ancestors = {os.getpid(), parent}
    cursor = parent
    for _ in range(12):
        try:
            cursor = int(Path(f"/proc/{cursor}/stat").read_text().rsplit(")",1)[1].split()[1])
            if cursor <= 1 or cursor in ancestors: break
            ancestors.add(cursor)
        except (OSError,ValueError,IndexError): break
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) in ancestors:
            continue
        try:
            arguments = (process / "cmdline").read_bytes().split(b"\0")
            names = {Path(arg.decode(errors="replace")).name for arg in arguments if arg}
            if b"--run" in arguments and names.intersection({"nav2_persistent_route.sh", "mapping_entry.sh"}):
                return True
        except (OSError, ProcessLookupError):
            pass
    return False


def assert_stationary(timeout=5.0):
    """Read-only, fail-closed check before reloading an idle navigation stack."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    state = {"armed": None, "controller": None, "connected": None, "stable": 0}
    rclpy.init()
    node = rclpy.create_node("rvclaw_inspection_map_reload_guard", enable_rosout=False)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, "/rvclaw/control/armed", lambda msg: state.update(armed=msg.data), qos)
    node.create_subscription(String, "/rvclaw/control/state", lambda msg: state.update(controller=msg.data), qos)
    node.create_subscription(Bool, "/rvclaw/control/feedback_connected", lambda msg: state.update(connected=msg.data), qos)

    def on_odometry(message):
        twist = message.twist.twist
        values = [twist.linear.x, twist.linear.y, twist.angular.z]
        stopped = all(math.isfinite(v) for v in values) and max(abs(values[0]), abs(values[1])) < .02 and abs(values[2]) < .05
        safe = state["armed"] is False and str(state["controller"]).upper() == "DISARMED" and state["connected"] is True
        state["stable"] = state["stable"] + 1 if stopped and safe else 0

    node.create_subscription(Odometry, "/odom", on_odometry, 10)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if state["stable"] >= 5:
                return
        raise RuntimeError("无法确认小车已停稳并解除使能，已取消地图切换")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_resume(profile):
    context = os.getenv("RVCLAW_ROUTE_RESUME_CONTEXT", "")
    binding = profile_path().with_name("inspection-last-route-profile.json")
    if context:
        # Legacy checkpoints have no map identity. Never resume them on a
        # different scene or silently carry completed points across maps.
        previous = json.loads(binding.read_text()) if binding.exists() else {}
        previous_id = previous.get("map_id", "voice-slam-20261006-110834-66940a96")
        if previous_id != profile["map_id"]:
            raise RuntimeError("暂停记录属于其他地图，请启动新的巡检任务")


def bind_route(profile):
    target = profile_path().with_name("inspection-last-route-profile.json")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps({"map_id": profile["map_id"], "map_file": profile["map_file"], "started_at": time.time()}), encoding="utf-8")
    temporary.replace(target)


def ensure_runtime(profile, map_file, verifier):
    if not profile or Path(map_file).resolve() != Path(profile["map_file"]).resolve():
        raise ValueError("任务地图与已保存的巡检设置不一致")
    check_resume(profile)
    same_running_map = False
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            argv = (proc/'cmdline').read_bytes().split(b'\0')
            if not any(Path(a.decode(errors='replace')).name == 'map_server' for a in argv if a): continue
            pairs = (v.decode(errors='replace').split('=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v)
            current = dict(pairs).get('RVCLAW_MAP_FILE', '')
            if current and Path(current).resolve() == Path(map_file).resolve(): same_running_map = True
        except OSError: pass
    if same_running_map:
        bind_route(profile)
        return {'status':'PASS','map_id':profile['map_id'],'motion_started':False,'verification':'existing_K1_route_live_grid_gate'}
    matches, diagnostic = verify_map(verifier, map_file)
    if not matches:
        # Unavailable/multiple publishers indicate a runtime fault, not a
        # confirmed mismatch. Do not turn every ROS fault into a service reload.
        if not any(text in diagnostic for text in ("dimensions differ", "resolution differs", "origin differs", "cell count differs", "cell contents differ")):
            raise RuntimeError("无法核对当前导航地图，巡检已停止：" + diagnostic[-500:])
        if conflicting_tasks():
            raise RuntimeError("存在其他小车任务，已取消地图切换")
        assert_stationary()
        subprocess.run(["systemctl", "--user", "restart", SERVICE], check=True, timeout=45)
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            matches, diagnostic = verify_map(verifier, map_file)
            if matches:
                break
            time.sleep(1)
        if not matches:
            raise RuntimeError("新地图加载未通过核对，巡检已停止")
    bind_route(profile)
    return {"status": "PASS", "map_id": profile["map_id"], "motion_started": False}


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--shell", action="store_true")
    mode.add_argument("--describe", action="store_true")
    mode.add_argument("--ensure-runtime", action="store_true")
    parser.add_argument("--map-file")
    parser.add_argument("--verifier")
    args = parser.parse_args()
    try:
        profile = read_profile()
        if args.shell:
            for key, value in environment(profile).items():
                print("export " + key + "=" + shlex.quote(value))
        elif args.describe:
            print(json.dumps({"selected": bool(profile), "map_id": profile.get("map_id") if profile else None, "environment": environment(profile), "motion_started": False}, ensure_ascii=False))
        else:
            if not args.map_file or not args.verifier:
                parser.error("--ensure-runtime requires --map-file and --verifier")
            print(json.dumps(ensure_runtime(profile, args.map_file, args.verifier), ensure_ascii=False))
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error), "motion_started": False}, ensure_ascii=False), file=__import__("sys").stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
