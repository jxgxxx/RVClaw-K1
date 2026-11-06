"""Scene-bound, stopped C observation verification. No ROS or motion APIs.

The operator's centered observation supplies a local geometric reference, not
a replacement occupancy map. Real-map score and free-space rays remain gates.
C recovery uses the strict checks. Stopped arrival may opt into an additional
local agreement rule; startup, recovery and A/B keep their existing gates.
"""
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np
from rvclaw_nav2_bringup import scan_map_consistency as sm


def map_identity(grid):
    geometry = [grid.resolution, grid.origin_x, grid.origin_y, grid.origin_yaw,
                grid.width, grid.height]
    return hashlib.sha256(json.dumps(geometry).encode() +
                          np.asarray(grid.cells, dtype=np.int8).tobytes()).hexdigest()


def endpoints(scan, maximum_range=8., count=240):
    return sm.scan_endpoints(scan.ranges, scan.angle_min, scan.angle_increment,
        max(.35, scan.range_min), scan.range_max,
        maximum_range_m=maximum_range, maximum_points=count)


def refinement_positions(grid, center_x, center_y):
    """Sample continuous poses inside one coarse peak, including sub-cell poses.

    The shared coarse search clamps its step to the occupancy cell size. Pose
    refinement must not use that clamp: a 5 cm occupancy raster still supports
    centimetre changes in sensor origin and ray intersections.
    """
    offsets = np.arange(-4, 5, dtype=float) * .0125
    dx, dy = np.meshgrid(offsets, offsets)
    positions = np.column_stack((center_x + dx.ravel(), center_y + dy.ravel()))
    cell_x, cell_y = sm._world_to_cells(grid, positions[:, 0], positions[:, 1])
    inside = ((cell_x >= 0) & (cell_y >= 0) &
              (cell_x < grid.width) & (cell_y < grid.height))
    safe_x, safe_y = np.clip(cell_x, 0, grid.width-1), np.clip(cell_y, 0, grid.height-1)
    return positions[inside & (grid.cells[safe_y, safe_x] == 0)]


class StationaryReference:
    def __init__(self, path, grid, *, allow_any_heading=False, arrival_target=None):
        document = json.loads(Path(path).read_text())
        if document.get('schema_version') != 1 or document.get('waypoint') != 'C':
            raise ValueError('unsupported stationary C reference')
        if document.get('map_identity') != map_identity(grid):
            raise ValueError('stationary reference belongs to another map')
        self.grid = grid
        self.allow_any_heading = bool(allow_any_heading)
        self.arrival_target = arrival_target
        self.anchor = document['anchor']
        if not all(math.isfinite(self.anchor.get(k, math.nan)) for k in ('x','y','yaw')):
            raise ValueError('invalid stationary reference anchor')
        occupied = np.asarray(document['occupied_cells'])
        if (occupied.ndim != 2 or occupied.shape[1] != 2 or len(occupied) < 100
                or occupied.dtype.kind not in 'iu'
                or np.any(occupied < 0) or np.any(occupied[:,0] >= grid.width)
                or np.any(occupied[:,1] >= grid.height)):
            raise ValueError('invalid stationary reference geometry')
        cells = np.zeros_like(grid.cells)
        cells[occupied[:,1], occupied[:,0]] = 100
        self.reference = sm.StaticMap(cells, sm._chamfer_distance(cells==100,grid.resolution),
            grid.resolution,grid.origin_x,grid.origin_y,grid.origin_yaw)
        self._positions = sm._local_world_positions(grid,self.anchor['x'],self.anchor['y'],1.75,.05)
        self._cells = np.ascontiguousarray(cells,dtype=np.int8)
        self._likelihood = np.ascontiguousarray(
            np.exp(-.5*(self.reference.occupied_distance_m/.10)**2),dtype=np.float32)
        self.deadline = math.inf
        self.cancel_event = None

    def _check_budget(self):
        if time.monotonic() >= self.deadline or (self.cancel_event is not None and self.cancel_event.is_set()):
            raise TimeoutError('C arrival geometry computation timed out')

    def _arrival_ray_check(self, points, candidate, center, score, gate, strict):
        """Bounded map residual allowance for an already-stopped arrival only.

        A local survey match and the live pose must independently agree with
        the unchanged arrival window. This does not authorize relocalization,
        movement, or acceptance of a new distant pose. Startup/recovery callers
        have no arrival_target and retain the original ray gate.
        """
        target = self.arrival_target
        if strict['status'] == 'PASS' or target is None:
            return strict
        def distance(a, b):
            return math.hypot(a['x']-b['x'], a['y']-b['y'])
        def angle(a, b):
            return abs(math.atan2(math.sin(a['yaw']-b['yaw']), math.cos(a['yaw']-b['yaw'])))
        if (not all(math.isfinite(p.get(k, math.nan)) for p in (candidate,center,target) for k in ('x','y','yaw'))
                or distance(candidate,target) > .12 or angle(candidate,target) > math.radians(10)
                or distance(candidate,center) > .10 or angle(candidate,center) > math.radians(5)
                or score['mean_likelihood'] < .90 or score['known_endpoint_ratio'] < .95
                or candidate['mean_likelihood'] < .80
                or gate['score_margin'] < .15 or gate['second_to_best_ratio'] > .75):
            return strict
        bounded = sm.evaluate_pose_ray_consistency(self.grid, points, candidate,
                                                  maximum_conflicting_ray_ratio=.30)
        if bounded['status'] != 'PASS':
            return strict
        return dict(bounded, method='stopped_arrival_local_agreement', strict_check=strict,
                    target_distance_m=distance(candidate,target),
                    amcl_distance_m=distance(candidate,center))

    def _score_table(self, points, positions, yaws):
        """Same kernel, lattice and tie ordering; materialize only selected dicts.

        Every candidate is retained, including global alternatives for the
        ambiguity gate. This is not a top-K or reduced-coverage search.
        """
        kernel=sm._native_map_kernel()
        points=np.asarray(points,dtype=np.float64)
        positions=np.ascontiguousarray(positions,dtype=np.float64)
        rows=[]
        work=np.empty(len(points),dtype=np.float32)
        for deg in yaws:
            self._check_budget()
            if kernel is None:
                candidates=sm._score_pose_candidates(self.reference,points,positions,[deg],.10,len(positions))
                rows.append(np.array([[p[k] for k in ('x','y','yaw','yaw_deg','mean_likelihood','known_endpoint_ratio')]
                                      for p in candidates],dtype=np.float64).reshape(-1,6))
                continue
            yaw=math.radians(float(deg));c=math.cos(yaw);s=math.sin(yaw)
            rx=c*points[:,0]-s*points[:,1];ry=s*points[:,0]+c*points[:,1]
            scores=np.zeros(len(positions),dtype=np.float64)
            known=np.zeros(len(positions),dtype=np.float64)
            grid=self.reference
            if kernel.rvclaw_score_poses(self._cells.ctypes.data,self._likelihood.ctypes.data,
                grid.height,grid.width,grid.resolution,grid.origin_x,grid.origin_y,grid.origin_yaw,
                positions.ctypes.data,len(positions),rx.ctypes.data,ry.ctypes.data,len(points),
                work.ctypes.data,scores.ctypes.data,known.ctypes.data)!=0:
                raise RuntimeError('static-map pose scoring kernel failed')
            if not len(scores):continue
            selected=np.argpartition(scores,-len(scores))
            rows.append(np.column_stack((positions[selected],np.full(len(scores),yaw),
                np.full(len(scores),deg),scores[selected],known[selected])))
        self._check_budget()
        return np.concatenate(rows) if rows else np.empty((0,6))

    @staticmethod
    def _candidate(row):
        return dict(zip(('x','y','yaw','yaw_deg','mean_likelihood','known_endpoint_ratio'),map(float,row)))

    @staticmethod
    def _alternate(table, pose, *, degrees=False):
        delta=table[:,2]-pose['yaw']
        angle=np.abs(np.arctan2(np.sin(delta),np.cos(delta)))
        yaw_distinct=(np.degrees(angle)>=15 if degrees else angle>=math.radians(15))
        indices=np.flatnonzero((np.hypot(table[:,0]-pose['x'],table[:,1]-pose['y'])>=.25)|yaw_distinct)
        return None if not len(indices) else StationaryReference._candidate(table[indices[0]])

    def search(self, scan, center):
        self._check_budget()
        failures = []
        points = endpoints(scan)
        if len(points) < 120:
            return dict(status='FAIL',failures=['insufficient C reference scan coverage'])
        if (math.hypot(center['x']-self.anchor['x'],center['y']-self.anchor['y']) > 1.0
                or (not self.allow_any_heading and
                    abs(math.atan2(math.sin(center['yaw']-self.anchor['yaw']),
                                   math.cos(center['yaw']-self.anchor['yaw']))) > math.radians(40))):
            return dict(status='FAIL',failures=['pose is outside stationary C reference region'])
        # Keep the search lattice fixed to the surveyed scene, not AMCL's
        # sub-cell phase. Otherwise the same stationary scan can lose its
        # valid free-ray candidate after a centimetre change in the seed.
        positions = self._positions
        positions = positions[(np.abs(positions[:,0]-center['x'])<=.75+1e-8)
                              & (np.abs(positions[:,1]-center['y'])<=.75+1e-8)]
        yaw_center=math.degrees(center['yaw'])
        anchor_yaw=math.degrees(self.anchor['yaw'])
        first=math.ceil((yaw_center-15-anchor_yaw)/2.5)
        last=math.floor((yaw_center+15-anchor_yaw)/2.5)
        yaws=anchor_yaw+np.arange(first,last+1)*2.5
        # Retain all scored candidates. A rejected peak must never suppress a
        # nearby valid pose before its real-map/free-ray tests have run.
        candidates = self._score_table(points,positions,yaws)
        candidates = candidates[np.argsort(-candidates[:,4],kind='stable')]
        distinct = []
        if len(candidates):
            distinct=[self._candidate(candidates[0])]
            alternate=self._alternate(candidates,distinct[0],degrees=True)
            if alternate is not None:distinct.append(alternate)
        gate = sm.evaluate_static_match_gate({'matches':distinct},
            minimum_best_score=.70,minimum_known_endpoint_ratio=.90,
            minimum_score_margin=.10,maximum_second_to_best_ratio=.85)
        if gate['status'] != 'PASS':
            return dict(status='FAIL',failures=gate['failures'],reference_gate=gate)
        peak = distinct[0]
        # Refine inside the unique peak before free-ray vetoes. A 5 cm map
        # search step is too coarse near walls: centimetre pose errors can
        # send many long rays through the adjacent occupied cell. The arrival
        # caller uses true sub-cell poses; other callers retain their lattice.
        fine_positions=(refinement_positions(self.grid,peak['x'],peak['y'])
                        if self.arrival_target is not None else
                        sm._local_world_positions(self.grid,peak['x'],peak['y'],.05,.0125))
        fine_yaws=np.arange(peak['yaw_deg']-2.5,peak['yaw_deg']+2.51,.5)
        refined=self._score_table(points,fine_positions,fine_yaws)
        candidates=np.concatenate((candidates,refined))
        candidates=candidates[np.argsort(-candidates[:,4],kind='stable')]
        # This local reference independently supplies uniqueness. The existing
        # final localization absolute map floor (.80) still applies, along with
        # .90 known endpoints and the original .25 free-ray conflict ceiling.
        # Only a strongly supported, already-arrived pose can use the bounded
        # stopped-arrival rule below; it cannot approve a new localization seed.
        map_points,rays_points = endpoints(scan,12),endpoints(scan,12,480)
        best = None
        eligible=((candidates[:,4]>=max(.70,peak['mean_likelihood']-.04)) &
            (np.hypot(candidates[:,0]-peak['x'],candidates[:,1]-peak['y'])<=.10) &
            (np.abs(candidates[:,2]-peak['yaw'])<=math.radians(2.5)+1e-8))
        for row in candidates[eligible]:
            self._check_budget()
            candidate=self._candidate(row)
            score = sm.score_pose(self.grid,map_points,candidate['x'],candidate['y'],candidate['yaw'])
            if score['mean_likelihood'] < .80 or score['known_endpoint_ratio'] < .90:
                continue
            rays = sm.evaluate_pose_ray_consistency(self.grid,rays_points,candidate)
            rays = self._arrival_ray_check(rays_points,candidate,center,score,gate,rays)
            if rays['status']=='PASS':
                best=candidate
                break
        if best is None:
            return dict(status='FAIL',failures=['no reference candidate passes real map and rays'],reference_gate=gate)
        alternate=self._alternate(candidates,best)
        gate=sm.evaluate_static_match_gate({'matches':[best]+([] if alternate is None else [alternate])},
            minimum_best_score=.70,minimum_known_endpoint_ratio=.90,
            minimum_score_margin=.10,maximum_second_to_best_ratio=.85)
        failures.extend(gate['failures'])
        if (rays.get('method') == 'stopped_arrival_local_agreement'
                and (gate['score_margin'] < .15 or gate['second_to_best_ratio'] > .75)):
            failures.append('stopped arrival lacks strong reference uniqueness')
        if (math.hypot(best['x']-center['x'],best['y']-center['y']) > .70
                or abs(math.atan2(math.sin(best['yaw']-center['yaw']),
                                 math.cos(best['yaw']-center['yaw']))) > math.radians(12.5)):
            failures.append('C reference winner lies outside search interior')
        return dict(status='PASS' if not failures else 'FAIL',failures=failures,
                    pose={k:best[k] for k in ('x','y','yaw')},reference_gate=gate,
                    map_score=score,ray_consistency=rays)

    def consensus(self, scans, center):
        checks = [self.search(scan,center) for scan in scans]
        failures = [f for check in checks for f in check['failures']]
        if len(checks) != 3:
            failures.append('three independent C scans are required')
        poses = [c['pose'] for c in checks if c['status']=='PASS']
        for first in poses:
            for second in poses:
                if (math.hypot(first['x']-second['x'],first['y']-second['y']) > .10
                        or abs(math.atan2(math.sin(first['yaw']-second['yaw']),
                                         math.cos(first['yaw']-second['yaw']))) > math.radians(3)):
                    failures.append('stationary C scans disagree')
        pose = None
        if poses and not failures:
            pose = dict(x=sum(p['x'] for p in poses)/len(poses),
                        y=sum(p['y'] for p in poses)/len(poses),
                        yaw=math.atan2(sum(math.sin(p['yaw']) for p in poses),
                                       sum(math.cos(p['yaw']) for p in poses)))
        return pose,dict(status='PASS' if pose else 'FAIL',failures=failures,
                         method='scene_bound_stationary_C_reference',checks=checks,pose=pose)

    def pose_gate(self, scan, pose):
        result = self.search(scan,pose)
        failures = list(result['failures'])
        candidate = sm.score_pose(self.grid,endpoints(scan,12),pose['x'],pose['y'],pose['yaw'])
        ray = sm.evaluate_pose_ray_consistency(self.grid,endpoints(scan,12,480),pose)
        best = result.get('pose')
        if (candidate['mean_likelihood'] < .80 or candidate['known_endpoint_ratio'] < .90):
            failures.append('converged C pose lacks static-map support')
        failures.extend(ray['failures'])
        if best is None or (math.hypot(best['x']-pose['x'],best['y']-pose['y']) > .10
                or abs(math.atan2(math.sin(best['yaw']-pose['yaw']),
                                 math.cos(best['yaw']-pose['yaw']))) > math.radians(5)):
            failures.append('AMCL did not converge to independently observed C pose')
        reference_score = sm.score_pose(self.reference,endpoints(scan),
            pose['x'],pose['y'],pose['yaw'],sigma_m=.10)['mean_likelihood']
        best_score = result.get('reference_gate',{}).get('best_score') or 0.
        comparison = dict(candidate_to_best_ratio=reference_score/best_score if best_score else 0.,
            candidate_to_best_yaw_deg=None if best is None else abs(math.degrees(
                math.atan2(math.sin(best['yaw']-pose['yaw']),math.cos(best['yaw']-pose['yaw'])))))
        return dict(status='PASS' if not failures else 'FAIL',failures=failures,
                    candidate=candidate,ray_consistency=ray,reference=result,
                    comparison=comparison,uniqueness=result.get('reference_gate',{'status':'FAIL'}))
