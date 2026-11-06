#!/usr/bin/env python3
"""Prepare a new map in the established inspection frame without activating it."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys

import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--x', type=float, required=True,
                        help='New SLAM origin X in the established inspection frame')
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw-deg', type=float, required=True)
    args = parser.parse_args()
    reference, source, output = args.reference.resolve(), args.map.resolve(), args.output.resolve()
    if output.exists() or output == reference or output == source.parent:
        raise RuntimeError('Require a new output directory; active/source maps must be preserved')
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw_deg)):
        raise RuntimeError('Map transform must be finite')
    old = yaml.safe_load((reference / 'map.yaml').read_text())
    new = yaml.safe_load(source.read_text())
    if any(int(meta.get('negate', 0)) != 0 or meta.get('mode', 'trinary') != 'trinary'
           or float(meta['resolution']) != 0.05 for meta in (old, new)):
        raise RuntimeError('Require the stable 5 cm trinary map format')
    if abs(float(old['origin'][2])) > 1e-8:
        raise RuntimeError('Reference inspection grid must remain axis aligned')
    old_pixels = np.asarray(Image.open(reference / old['image']).convert('L'))
    raw_pixels = np.asarray(Image.open(source.parent / new['image']).convert('L'))
    raw_cells = np.flipud(raw_pixels)
    height, width = old_pixels.shape
    cell_y, cell_x = np.indices((height, width))
    # Sample the raw map onto the exact existing grid. A rotated YAML origin
    # alone is unsuitable for navigation consumers that expect aligned grids.
    world_x = old['origin'][0] + (cell_x + 0.5) * old['resolution']
    world_y = old['origin'][1] + (cell_y + 0.5) * old['resolution']
    yaw = math.radians(args.yaw_deg)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    delta_x, delta_y = world_x - args.x, world_y - args.y
    raw_x = cosine * delta_x + sine * delta_y
    raw_y = -sine * delta_x + cosine * delta_y
    raw_yaw = float(new['origin'][2])
    cosine, sine = math.cos(raw_yaw), math.sin(raw_yaw)
    delta_x, delta_y = raw_x - new['origin'][0], raw_y - new['origin'][1]
    indices_x = np.floor((cosine * delta_x + sine * delta_y) / new['resolution']).astype(int)
    indices_y = np.floor((-sine * delta_x + cosine * delta_y) / new['resolution']).astype(int)
    inside = ((indices_x >= 0) & (indices_x < raw_cells.shape[1])
              & (indices_y >= 0) & (indices_y < raw_cells.shape[0]))
    pixels = np.full((height, width), 205, dtype=np.uint8)
    pixels[inside] = raw_cells[indices_y[inside], indices_x[inside]]
    pixels = np.flipud(pixels)
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / 'robot/ros2_ws/src/rvclaw_nav2_bringup'))
    from rvclaw_nav2_bringup.map_acceptance import load_map, validate_mission_map
    from rvclaw_nav2_bringup.mission_contract import load_mission_catalog
    from rvclaw_nav2_bringup.site_bundle_contract import validate_map_waypoint_binding

    output.mkdir(parents=True)
    Image.fromarray(pixels).save(output / 'map.pgm')
    metadata = dict(old, image='map.pgm')
    (output / 'map.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    map_id = 'inspection-' + hashlib.sha256((output / 'map.pgm').read_bytes()).hexdigest()[:12]
    for name in ('waypoints.yaml', 'mission_catalog.yaml'):
        text = (reference / name).read_text()
        text, changes = re.subn(r'^map_id:.*$', 'map_id: ' + map_id, text, flags=re.MULTILINE)
        if changes != 1:
            raise RuntimeError('Map identity missing or repeated: ' + name)
        (output / name).write_text(text)
    shutil.copy2(reference / 'inspection_route_tasks.yaml', output / 'inspection_route_tasks.yaml')
    binding = validate_map_waypoint_binding(output / 'map.yaml', output / 'waypoints.yaml')
    acceptance = validate_mission_map(load_mission_catalog(output / 'mission_catalog.yaml'),
                                      'inspection_route', load_map(output / 'map.yaml'))
    (output / 'map-acceptance.json').write_text(json.dumps(acceptance, indent=2) + '\n')
    occupied = lambda raster, meta: (255 - raster.astype(float)) / 255 >= meta['occupied_thresh']
    old_walls, new_walls = occupied(old_pixels, old), occupied(pixels, old)
    overlay = np.full((height, width, 3), 245, dtype=np.uint8)
    overlay[old_walls] = (0, 110, 220)
    overlay[new_walls] = (230, 70, 20)
    overlay[old_walls & new_walls] = (30, 30, 30)
    Image.fromarray(overlay).save(output / 'map-overlay.png')
    report = {'status': 'PREPARED', 'review_required': True, 'activated': False,
              'source_map': str(source), 'source_map_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'source_image_sha256': hashlib.sha256((source.parent / new['image']).read_bytes()).hexdigest(),
              'reference': str(reference), 'output': str(output), 'map_id': map_id,
              'transform_new_to_inspection': {'x': args.x, 'y': args.y, 'yaw_deg': args.yaw_deg},
              'waypoint_coordinates_changed': False, 'thresholds_changed': False,
              'binding': binding, 'static_route_status': acceptance['status'],
              'static_route_failures': acceptance['failures']}
    (output / 'scene-preparation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
