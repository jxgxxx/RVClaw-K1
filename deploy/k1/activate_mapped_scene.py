#!/usr/bin/env python3
"""Switch deployment configuration to a reviewed scene; never send motion."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import yaml


def start_unit(unit, action):
    subprocess.run(['systemctl', '--user', action, '--no-block', unit],
                   check=True, timeout=15)
    deadline = time.monotonic() + 150
    while time.monotonic() < deadline:
        result = subprocess.run(['systemctl', '--user', 'is-active', unit],
                                capture_output=True, text=True, timeout=10)
        if result.stdout.strip() == 'active':
            return
        if result.stdout.strip() == 'failed':
            raise RuntimeError('Service failed: ' + unit)
        time.sleep(.5)
    raise RuntimeError('Service did not finish ' + action + ': ' + unit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--alignment-reviewed', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    scene = args.scene.resolve()
    if not args.check and (not scene.is_relative_to(root / 'configs/scenes')
                           or scene == root / 'configs/scenes/inspection'):
        raise RuntimeError('Use a prepared scene under configs/scenes, preserving inspection')
    report = json.loads((scene / 'scene-preparation.json').read_text())
    if report['static_route_status'] != 'PASS':
        raise RuntimeError('Prepared map did not pass fixed-point/corridor checks')
    reference = root / 'configs/scenes/inspection'
    for name in ('waypoints.yaml', 'mission_catalog.yaml'):
        candidate = yaml.safe_load((scene / name).read_text())
        baseline = yaml.safe_load((reference / name).read_text())
        candidate.pop('map_id')
        baseline.pop('map_id')
        if candidate != baseline:
            raise RuntimeError('Stable point/mission configuration differs: ' + name)
    if (scene / 'inspection_route_tasks.yaml').read_bytes() != (reference / 'inspection_route_tasks.yaml').read_bytes():
        raise RuntimeError('Inspection configuration or thresholds differ')
    sys.path.insert(0, str(root / 'robot/ros2_ws/src/rvclaw_nav2_bringup'))
    from rvclaw_nav2_bringup.site_bundle_contract import validate_map_waypoint_binding
    from rvclaw_nav2_bringup.map_acceptance import load_map, validate_mission_map
    from rvclaw_nav2_bringup.mission_contract import load_mission_catalog
    validate_map_waypoint_binding(scene / 'map.yaml', scene / 'waypoints.yaml')
    acceptance = validate_mission_map(load_mission_catalog(scene / 'mission_catalog.yaml'),
                                     'inspection_route', load_map(scene / 'map.yaml'))
    if acceptance['status'] != 'PASS':
        raise RuntimeError('Current candidate map no longer passes static checks')
    if args.check:
        print('MAPPED_SCENE_CONFIGURATION_PASS; scene not activated')
        return
    if not args.alignment_reviewed:
        raise RuntimeError('Review wall alignment and point coverage before activation')
    mapping_state = root / 'runtime/manual-mapping/session-state.json'
    if not mapping_state.is_file() or json.loads(mapping_state.read_text()).get('state') != 'COMPLETED':
        raise RuntimeError('Wait for voice mapping to complete and release hardware')
    latest = json.loads((root / 'runtime/k1/mapping/state/manual-mapping-latest.json').read_text())
    if Path(latest['map_yaml']).resolve() != Path(report['source_map']).resolve():
        raise RuntimeError('Candidate does not correspond to the latest voice map')
    source = Path(report['source_map'])
    metadata = yaml.safe_load(source.read_text())
    for path, expected in ((source, report['source_map_sha256']),
                           (source.parent / metadata['image'], report['source_image_sha256'])):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Source voice map changed after preparation')
    state = subprocess.run(['systemctl', '--user', 'is-active', 'rvclaw-nav2-persistent.service'],
                            capture_output=True, text=True, check=False).stdout.strip()
    if state not in ('inactive', 'failed'):
        raise RuntimeError('Navigation must be fully stopped, as after a successful mapping session')
    state_dir = root / 'runtime/k1/scenes' / scene.name
    if state_dir.exists():
        raise RuntimeError('Use a fresh scene state directory; never reuse localization/checkpoints')
    state_dir.mkdir(parents=True)
    cache, checkpoint = state_dir / 'localization-pose.json', state_dir / 'inspection-route-checkpoint.json'
    common = {'RVCLAW_MAP_FILE': scene / 'map.yaml', 'RVCLAW_LOCALIZATION_CACHE': cache}
    voice = {**common, 'RVCLAW_NAV2_WAYPOINTS_FILE': scene / 'waypoints.yaml',
             'RVCLAW_INSPECTION_CONFIG': scene / 'inspection_route_tasks.yaml',
             'RVCLAW_ROUTE_CHECKPOINT_FILE': checkpoint, 'RVCLAW_SITE_DIR': scene}
    original, replacements = {}, {}
    env_root = root / 'runtime/k1/environment'
    for unit, settings in (('rvclaw-nav2-persistent', common), ('rvclaw-vendor-command', voice)):
        path = env_root / (unit + '.service.env')
        original[str(path)] = path.read_text()
        text = '\n'.join(line for line in original[str(path)].splitlines()
                         if line.split('=', 1)[0] not in settings) + '\n'
        text += ''.join(key + '="' + str(value) + '"\n' for key, value in settings.items())
        replacements[path] = text
    (state_dir / 'previous-environment.json').write_text(json.dumps(original, indent=2) + '\n')
    try:
        for path, text in replacements.items():
            temporary = path.with_suffix('.env.pending')
            temporary.write_text(text)
            temporary.replace(path)
        start_unit('rvclaw-nav2-persistent.service', 'start')
        start_unit('rvclaw-vendor-command.service', 'restart')
    except Exception:
        for path, text in original.items():
            Path(path).write_text(text)
        subprocess.run(['systemctl', '--user', 'stop', 'rvclaw-nav2-persistent.service'],
                       check=False, timeout=60)
        start_unit('rvclaw-vendor-command.service', 'restart')
        raise
    (state_dir / 'active-scene.json').write_text(json.dumps(
        {'scene': str(scene), 'map_id': report['map_id'], 'motion_command_sent': False}, indent=2) + '\n')
    print('Mapped scene selected; stationary localization preflight still required. No motion command sent.')


if __name__ == '__main__':
    main()
