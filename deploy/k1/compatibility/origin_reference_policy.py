"""Resolve a sparse ORIGIN template tie with independently unique full-map evidence.

Pure geometry only: no ROS, pose publication, reset or motion interfaces.
"""
import math
from rvclaw_nav2_bringup import scan_map_consistency as sm


def unique_global_reference_fallback(report, center, reference_report, fresh_pose_gates):
    """Keep the independently proven global pose if optional refinement is weak.

    No sparse pose is promoted. The live verifier still applies its independent
    final fresh AMCL/map/ray/continuity gates. This function cannot move a robot.
    """
    result=dict(status='FAIL',motion_authorized=False,
                method='unique_global_pose_with_fresh_full_map_confirmation')
    consensus=(report or {}).get('consensus') or {}
    try:
        pose=consensus['pose']
        stamps=[s['stamp_ns'] for s in report['input_scans']]
        if (report.get('status')!='PASS' or consensus.get('status')!='PASS'
                or consensus.get('require_global_uniqueness') is not True
                or consensus.get('scan_count')!=3 or consensus.get('unique_pass_count',0)<2
                or len(stamps)!=3 or len(set(stamps))!=3 or not all(s>0 for s in stamps)
                or any(not math.isfinite(float(p[k])) for p in (pose,center) for k in ('x','y','yaw'))
                or any(abs(float(pose[k])-float(center[k]))>1e-6 for k in ('x','y','yaw'))):
            raise ValueError('independent unique global proof is unavailable')
        checks=reference_report.get('checks') or []
        if len(checks)!=3 or len(fresh_pose_gates)!=3:
            raise ValueError('three fresh confirmations are required')
        for check in checks:
            failures=check.get('failures') or []
            if any(not f.startswith(('best stationary scan match is too weak',
                    'stationary scan match margin is too small',
                    'stationary scan has an ambiguous second match')) for f in failures):
                raise ValueError('reference has an actual map or coverage failure')
            best=(check.get('reference_gate') or {}).get('best') or {}
            if (not all(math.isfinite(float(best[k])) for k in ('x','y','yaw'))
                    or math.hypot(best['x']-pose['x'],best['y']-pose['y'])>.25
                    or abs(math.atan2(math.sin(best['yaw']-pose['yaw']),
                                     math.cos(best['yaw']-pose['yaw'])))>math.radians(5)):
                raise ValueError('reference best contradicts the global pose')
        if any(g.get('status')!='PASS' or (g.get('ray_consistency') or {}).get('status')!='PASS'
               for g in fresh_pose_gates):
            raise ValueError('fresh full map pose or rays reject the global pose')
    except (KeyError,TypeError,ValueError) as exc:
        return dict(result,reason=str(exc))
    return dict(result,status='PASS',pose=dict(pose),fresh_pose_gates=fresh_pose_gates)


def resolve_reference_tie(grid, points, rays, matches, sparse_gate, global_report, center):
    result=dict(status='FAIL',motion_authorized=False,
                method='unique_global_and_full_map_reference_candidates')
    failures=sparse_gate.get('failures') or []
    if (not failures or any(not f.startswith((
            'stationary scan match margin is too small',
            'stationary scan has an ambiguous second match')) for f in failures)):
        return dict(result,reason='sparse reference failure is not an ambiguity')
    report=global_report or {}
    consensus=report.get('consensus') or {}
    pose=consensus.get('pose') or {}
    try:
        if (report.get('status')!='PASS' or consensus.get('status')!='PASS'
                or consensus.get('require_global_uniqueness') is not True
                or consensus.get('scan_count')!=3 or consensus.get('unique_pass_count',0)<2
                or not matches):
            raise ValueError('independent unique global consensus is missing')
        stamps=[s['stamp_ns'] for s in report.get('input_scans',[])]
        if len(stamps)!=3 or len(set(stamps))!=3 or not all(s>0 for s in stamps):
            raise ValueError('global proof lacks three independent acquisitions')
        values=[float(p[k]) for p in (pose,center,matches[0]) for k in ('x','y','yaw')]
        values += [float(matches[0][k]) for k in ('mean_likelihood','known_endpoint_ratio')]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite geometry')
        if (math.hypot(center['x']-pose['x'],center['y']-pose['y'])>1e-6
                or abs(math.atan2(math.sin(center['yaw']-pose['yaw']),
                                  math.cos(center['yaw']-pose['yaw'])))>1e-6):
            raise ValueError('global proof does not belong to this search center')
        winner=matches[0]
        if (float(winner['mean_likelihood'])<.70
                or float(winner['known_endpoint_ratio'])<.90):
            raise ValueError('sparse winner lacks absolute support')
        if (math.hypot(winner['x']-pose['x'],winner['y']-pose['y'])>.25
                or abs(math.atan2(math.sin(winner['yaw']-pose['yaw']),
                                  math.cos(winner['yaw']-pose['yaw'])))>math.radians(5)):
            raise ValueError('reference winner contradicts globally verified geometry')
    except (KeyError,TypeError,ValueError) as exc:
        return dict(result,reason=str(exc))
    supported=[];checks=[]
    for index,candidate in enumerate(matches):
        score=sm.score_pose(grid,points,candidate['x'],candidate['y'],candidate['yaw'])
        ray=sm.evaluate_pose_ray_consistency(grid,rays,candidate)
        mapped=dict(candidate,mean_likelihood=score['mean_likelihood'],
                    known_endpoint_ratio=score['known_endpoint_ratio'])
        checks.append(dict(reference_index=index,pose=candidate,map_score=score,ray=ray))
        if index==0 and ray['status']!='PASS':
            return dict(result,reason='original reference winner crosses actual map obstacles',checks=checks)
        if ray['status']=='PASS':supported.append(mapped)
    # Never replace a bad reference winner with a convenient competitor.
    supported.sort(key=lambda p:p['mean_likelihood'],reverse=True)
    if not supported or any(supported[0][k]!=winner[k] for k in ('x','y','yaw')):
        return dict(result,reason='actual map contradicts original reference winner',checks=checks)
    gate=sm.evaluate_static_match_gate(dict(matches=supported),minimum_best_score=.80,
        minimum_known_endpoint_ratio=.90,minimum_score_margin=.05,
        maximum_second_to_best_ratio=.95)
    return dict(result,status=gate['status'],map_gate=gate,checks=checks)
