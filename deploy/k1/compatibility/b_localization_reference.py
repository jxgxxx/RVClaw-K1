"""Bounded B-area scan verification. This module has no ROS or motion APIs."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from rvclaw_nav2_bringup import scan_map_consistency as sm


def identity(grid):
    geometry = [grid.resolution, grid.origin_x, grid.origin_y, grid.origin_yaw,
                grid.width, grid.height]
    return hashlib.sha256(json.dumps(geometry).encode() +
                          np.asarray(grid.cells, dtype=np.int8).tobytes()).hexdigest()


def endpoints(scan, count=240):
    return sm.scan_endpoints(scan.ranges, scan.angle_min, scan.angle_increment,
        max(.35, scan.range_min), scan.range_max,
        maximum_range_m=12., maximum_points=count)


class BLocalizationReference:
    def __init__(self, path, grid, *, waypoint='B'):
        document = json.loads(Path(path).read_text())
        if (waypoint not in ('B', 'ORIGIN') or document.get('schema_version') != 1
                or document.get('waypoint') != waypoint
                or document.get('map_identity') != identity(grid)):
            raise ValueError('B reference does not belong to this scene')
        self.grid = grid
        self.waypoint = waypoint
        self.anchor = document['anchor']
        if not all(math.isfinite(self.anchor.get(k, math.nan)) for k in ('x','y','yaw')):
            raise ValueError('invalid B reference anchor')
        occupied = np.asarray(document['occupied_cells'])
        if (occupied.ndim != 2 or occupied.shape[1] != 2 or len(occupied) < 100
                or occupied.dtype.kind not in 'iu' or np.any(occupied < 0)
                or np.any(occupied[:,0] >= grid.width) or np.any(occupied[:,1] >= grid.height)):
            raise ValueError('invalid B reference geometry')
        cells = np.zeros_like(grid.cells)
        cells[occupied[:,1],occupied[:,0]] = 100
        self.reference = sm.StaticMap(cells, sm._chamfer_distance(cells==100,grid.resolution),
            grid.resolution,grid.origin_x,grid.origin_y,grid.origin_yaw)
        # This reference is immutable for the node's lifetime. All local
        # searches use sigma=.10, so its likelihood field need only be built
        # once. Keep the original float32 expression and native reduction.
        self._likelihood = np.ascontiguousarray(np.exp(
            -0.5 * (self.reference.occupied_distance_m / .10) ** 2),dtype=np.float32)

    def _search_reference(self, points, x, y, yaw, *, position_radius_m,
                          yaw_radius_deg, position_step_m, yaw_step_deg,
                          sigma_m, keep_per_yaw, maximum_matches):
        kernel = sm._native_map_kernel()
        if kernel is None:
            return sm.search_local_matches(self.reference,points,x,y,yaw,
                position_radius_m=position_radius_m,yaw_radius_deg=yaw_radius_deg,
                position_step_m=position_step_m,yaw_step_deg=yaw_step_deg,
                sigma_m=sigma_m,keep_per_yaw=keep_per_yaw,maximum_matches=maximum_matches)
        assert sigma_m == .10
        grid=self.reference
        points=np.asarray(points,dtype=np.float64)
        positions=np.ascontiguousarray(sm._local_world_positions(
            grid,x,y,position_radius_m,position_step_m),dtype=np.float64)
        center=math.degrees(float(yaw))
        yaws=np.arange(center-float(yaw_radius_deg),
            center+float(yaw_radius_deg)+float(yaw_step_deg)*.5,float(yaw_step_deg))
        cells=np.ascontiguousarray(grid.cells,dtype=np.int8)
        work=np.empty(len(points),dtype=np.float32)
        candidates=[]
        for degrees in yaws:
            angle=math.radians(float(degrees))
            cosine,sine=math.cos(angle),math.sin(angle)
            rx=cosine*points[:,0]-sine*points[:,1]
            ry=sine*points[:,0]+cosine*points[:,1]
            scores=np.zeros(len(positions),dtype=np.float64)
            known=np.zeros(len(positions),dtype=np.float64)
            if len(points) and kernel.rvclaw_score_poses(
                    cells.ctypes.data,self._likelihood.ctypes.data,grid.height,grid.width,
                    grid.resolution,grid.origin_x,grid.origin_y,grid.origin_yaw,
                    positions.ctypes.data,len(positions),rx.ctypes.data,ry.ctypes.data,
                    len(points),work.ctypes.data,scores.ctypes.data,known.ctypes.data)!=0:
                raise RuntimeError('B reference pose scoring kernel failed')
            count=min(int(keep_per_yaw),len(scores))
            if not count:continue
            selected=np.argpartition(scores,-count)[-count:]
            candidates.extend(dict(x=float(positions[i,0]),y=float(positions[i,1]),
                yaw=angle,yaw_deg=float(degrees),mean_likelihood=float(scores[i]),
                known_endpoint_ratio=float(known[i])) for i in selected)
        return dict(matches=sm._select_distinct_matches(candidates,
            minimum_position_separation_m=.25,minimum_yaw_separation_deg=15.,
            maximum_matches=maximum_matches))

    def search(self, scan, center, *, moving=False, global_report=None):
        if (not all(math.isfinite(center.get(k, math.nan)) for k in ('x','y','yaw'))
                or math.hypot(center['x']-self.anchor['x'],center['y']-self.anchor['y']) > 3.):
            return dict(status='FAIL',failures=['outside B reference region'])
        points = endpoints(scan,180 if moving else 240)
        if len(points) < 120:
            return dict(status='FAIL',failures=['insufficient B scan coverage'])
        matches = self._search_reference(points,
            center['x'],center['y'],center['yaw'],position_radius_m=1.5,
            yaw_radius_deg=10,position_step_m=.1 if moving else .05,
            yaw_step_deg=5 if moving else 2.5,sigma_m=.10,
            keep_per_yaw=128,maximum_matches=8)['matches']
        # The coarse grid is for finding peaks, not estimating the offset.
        # Its 10 cm quantization previously changed a pair of consistent
        # offsets into an apparent disagreement at the edge of the search.
        if moving and matches:
            refined=[]
            comparison_points=endpoints(scan)
            for peak in matches:
                refined.extend(self._search_reference(comparison_points,
                    peak['x'],peak['y'],peak['yaw'],position_radius_m=.15,
                    yaw_radius_deg=2.5,position_step_m=.05,yaw_step_deg=2.5,
                    sigma_m=.10,keep_per_yaw=16,maximum_matches=1)['matches'])
            # Refine competing peaks as well; improving only the winner can
            # incorrectly make an ambiguous corridor look unique.
            matches=sm._select_distinct_matches(refined,
                minimum_position_separation_m=.25,minimum_yaw_separation_deg=15.,
                maximum_matches=8)
        gate = sm.evaluate_static_match_gate({'matches':matches},minimum_best_score=.70,
            minimum_known_endpoint_ratio=.90,minimum_score_margin=.10,
            maximum_second_to_best_ratio=.85)
        rejected = []
        if (self.waypoint == 'B' and gate['status'] != 'PASS' and matches
                and all(f.startswith(('stationary scan match margin is too small',
                                      'stationary scan has an ambiguous second match'))
                        for f in gate['failures'])):
            # Reference endpoints alone can prefer a translated wall whose
            # free rays pass through the real box. Veto those competitors
            # with the unchanged actual-map ray test before judging ambiguity.
            # Never promote another peak if the original winner is invalid.
            rays = endpoints(scan,480)
            supported = []
            for index, candidate in enumerate(matches):
                ray = sm.evaluate_pose_ray_consistency(self.grid,rays,candidate)
                if index == 0 and ray['status'] != 'PASS':
                    break
                if ray['status'] == 'PASS':
                    supported.append(candidate)
                else:
                    rejected.append(dict(pose=candidate,ray_consistency=ray))
            else:
                gate = sm.evaluate_static_match_gate({'matches':supported},minimum_best_score=.70,
                    minimum_known_endpoint_ratio=.90,minimum_score_margin=.10,
                    maximum_second_to_best_ratio=.85)
                matches = supported
        if self.waypoint == 'ORIGIN' and gate['status'] != 'PASS' and global_report is not None:
            from origin_reference_policy import resolve_reference_tie
            resolution = resolve_reference_tie(self.grid,points,endpoints(scan,480),
                matches,gate,global_report,center)
            if resolution['status']=='PASS':
                gate=dict(gate,status='PASS',failures=[],sparse_gate=gate,
                          full_map_disambiguation=resolution)
        if gate['status'] != 'PASS':
            return dict(status='FAIL',failures=gate['failures'],reference_gate=gate,
                        ray_rejected_competitors=rejected)
        best=matches[0]
        score=sm.score_pose(self.grid,points,best['x'],best['y'],best['yaw'])
        rays=sm.evaluate_pose_ray_consistency(self.grid,endpoints(scan,480),best)
        failures=list(rays['failures'])
        if score['mean_likelihood'] < .80 or score['known_endpoint_ratio'] < .90:
            failures.append('B candidate lacks actual map support')
        if (math.hypot(best['x']-center['x'],best['y']-center['y']) > 1.35
                or abs(best['yaw']-center['yaw']) > math.radians(7.5)+1e-6):
            failures.append('B winner lies outside search interior')
        return dict(status='PASS' if not failures else 'FAIL',failures=failures,
            pose={k:best[k] for k in ('x','y','yaw')},reference_gate=gate,
            map_score=score,ray_consistency=rays,ray_rejected_competitors=rejected)

    def consensus(self, scans, center, *, global_report=None):
        checks=[self.search(s,center) if global_report is None else
                self.search(s,center,global_report=global_report) for s in scans]
        failures=[f for c in checks for f in c['failures']]
        poses=[c['pose'] for c in checks if c['status']=='PASS']
        if len(scans)!=3:failures.append('three independent B scans required')
        for a in poses:
            for b in poses:
                if (math.hypot(a['x']-b['x'],a['y']-b['y'])>.10+1e-6
                        or abs(math.atan2(math.sin(a['yaw']-b['yaw']),math.cos(a['yaw']-b['yaw'])))>math.radians(3)):
                    failures.append('B stationary scans disagree')
        pose=None
        if poses and not failures:
            pose=dict(x=sum(p['x'] for p in poses)/len(poses),y=sum(p['y'] for p in poses)/len(poses),
                yaw=math.atan2(sum(math.sin(p['yaw']) for p in poses),sum(math.cos(p['yaw']) for p in poses)))
        return pose,dict(status='PASS' if pose else 'FAIL',failures=failures,checks=checks,pose=pose,
                         method='scene_bound_stationary_B_reference')

    def pose_gate(self, scan, pose):
        result=self.search(scan,pose)
        score=sm.score_pose(self.grid,endpoints(scan),pose['x'],pose['y'],pose['yaw'])
        rays=sm.evaluate_pose_ray_consistency(self.grid,endpoints(scan,480),pose)
        failures=list(result['failures'])+rays['failures']
        best=result.get('pose')
        if score['mean_likelihood']<.80 or score['known_endpoint_ratio']<.90:
            failures.append('AMCL B pose lacks actual map support')
        if best is None or math.hypot(best['x']-pose['x'],best['y']-pose['y'])>.12:
            failures.append('AMCL B pose has not converged to measured geometry')
        ref_score=sm.score_pose(self.reference,endpoints(scan),pose['x'],pose['y'],pose['yaw'],sigma_m=.10)['mean_likelihood']
        best_score=result.get('reference_gate',{}).get('best_score') or 0.
        return dict(status='PASS' if not failures else 'FAIL',failures=failures,candidate=score,
            ray_consistency=rays,reference=result,uniqueness=result.get('reference_gate',{'status':'FAIL'}),
            comparison=dict(candidate_to_best_ratio=ref_score/best_score if best_score else 0.,
                candidate_to_best_yaw_deg=None if best is None else abs(math.degrees(
                    math.atan2(math.sin(best['yaw']-pose['yaw']),math.cos(best['yaw']-pose['yaw']))))))


def preserves_offset_observation(previous, result, center):
    """Keep, but never confirm, fresh strong evidence across an ambiguous frame."""
    if previous is None or result.get('status') != 'FAIL':
        return False
    gate = result.get('reference_gate') or {}
    best = gate.get('best') or {}
    failures = result.get('failures') or []
    if (not failures or any(not (f.startswith('stationary scan match margin is too small')
            or f.startswith('stationary scan has an ambiguous second match')) for f in failures)):
        return False
    try:
        values = [best[k] for k in ('x', 'y', 'mean_likelihood', 'known_endpoint_ratio')]
        if not all(math.isfinite(float(v)) for v in values):
            return False
        offset = (best['x'] - center['x'], best['y'] - center['y'])
        return (best['mean_likelihood'] >= .70 and best['known_endpoint_ratio'] >= .90
                and math.hypot(offset[0]-previous[0], offset[1]-previous[1]) <= .15)
    except (KeyError, TypeError, ValueError):
        return False


def offset_evidence(previous, result, center):
    """Two strong independent scans may request a stop, never a pose write."""
    if result.get('status') != 'PASS':return None,False
    p=result['pose'];offset=(p['x']-center['x'],p['y']-center['y'])
    distance=math.hypot(*offset)
    if distance <= .25:return None,False
    agrees=previous is not None and math.hypot(offset[0]-previous[0],offset[1]-previous[1])<=.15
    return offset,agrees


def approach_arrival_stop(previous, result, center, present, waypoint, speed, angular):
    """A measured arrival can stop navigation; fresh stopped proof still follows."""
    if result.get('status') != 'PASS' or result.get('pose') is None:
        return None,False
    p=result['pose'];offset=(p['x']-center['x'],p['y']-center['y'])
    agrees=previous is not None and math.hypot(offset[0]-previous[0],offset[1]-previous[1])<=.15
    if (present is None or not all(math.isfinite(v) for v in (speed,angular))
            or not .01<speed<=.35 or abs(angular)>.1):
        return offset,False
    dx,dy=waypoint['x']-present['x'],waypoint['y']-present['y']
    ahead=dx*math.cos(present['yaw'])+dy*math.sin(present['yaw'])
    lateral=abs(-dx*math.sin(present['yaw'])+dy*math.cos(present['yaw']))
    stopping_distance=speed*speed/(2*.20)+.03
    # This stop is only for a straight, centered final approach. A curved
    # controller trajectory may still need lateral travel; stopping at its
    # projected braking envelope prematurely interrupted the second field run.
    return offset,bool(agrees and math.hypot(dx,dy)<=.10
                       and lateral<=.05 and -.08<=ahead<=stopping_distance)
