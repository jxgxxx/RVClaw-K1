"""B arrival evidence from fresh stationary scans; no motion publishers."""
import math
import time
from pathlib import Path


def arrival_decision(consensus, amcl_pose, waypoint, tolerance_m=.12, *, position_only=False):
    """Keep geometric arrival and the controller's AMCL agreement separate."""
    result = dict(status='FAIL', decision='STOP', consensus=consensus)
    if consensus.get('status') != 'PASS':
        return dict(result, error='B arrival has no unique stationary geometry')
    measured = consensus.get('pose')
    try:
        if not all(math.isfinite(float(p[k])) for p in (measured, amcl_pose, waypoint)
                   for k in ('x', 'y', 'yaw')):
            raise ValueError('nonfinite pose')
        distance = math.hypot(measured['x']-waypoint['x'], measured['y']-waypoint['y'])
        amcl_error = math.hypot(measured['x']-amcl_pose['x'], measured['y']-amcl_pose['y'])
        yaw_error = abs(math.atan2(math.sin(measured['yaw']-waypoint['yaw']),
                                   math.cos(measured['yaw']-waypoint['yaw'])))
        amcl_yaw_error = abs(math.atan2(math.sin(measured['yaw']-amcl_pose['yaw']),
                                        math.cos(measured['yaw']-amcl_pose['yaw'])))
    except (KeyError, TypeError, ValueError):
        return dict(result, error='B arrival contains invalid pose evidence')
    result.update(geometric_pose=measured, amcl_pose=amcl_pose, distance_m=distance,
                  amcl_distance_m=amcl_error, heading_error_rad=yaw_error,
                  amcl_heading_error_rad=amcl_yaw_error, tolerance_m=tolerance_m)
    if distance > .75:
        return dict(result, error='B measured position exceeds correction budget')
    if amcl_error > .12 or amcl_yaw_error > math.radians(10):
        return dict(result, status='REVIEW', decision='RELOCALIZE')
    if distance > tolerance_m:
        return dict(result, status='REVIEW', decision='REAPPROACH')
    if position_only:
        return dict(result, status='PASS', decision='POSITION_CONFIRMED')
    if yaw_error > math.radians(10):
        return dict(result, status='REVIEW', decision='ALIGN')
    return dict(result, status='PASS', decision='ARRIVED')


def review_stopped_arrival(map_file, center, waypoint, *, timeout_s=4.0,
                           reference_factory=None, decide=arrival_decision):
    """Subscribe only; require fresh scans and uninterrupted stationary feedback."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from b_localization_reference import BLocalizationReference
    from rvclaw_nav2_bringup.scan_map_consistency import load_static_map
    from rvclaw_nav2_bringup.fast_start_policy import build_scan_signature, compare_scan_signatures
    from concurrent.futures import ThreadPoolExecutor

    node = Node('rvclaw_b_stopped_arrival', enable_rosout=False)
    worker = None
    started = time.monotonic()
    deadline = started + min(4.0, timeout_s)
    scans = []
    state = dict(armed=None, state=None, armed_at=0., state_at=0., odom_at=0.,
                 unsafe=False, pose=dict(center), odom_count=0)
    def armed(m):
        state.update(armed=m.data, armed_at=time.monotonic())
        state['unsafe'] |= m.data
    def control_state(m):
        state.update(state=m.data, state_at=time.monotonic())
        state['unsafe'] |= m.data != 'DISARMED'
    def odom(m):
        t=m.twist.twist
        state.update(odom_at=time.monotonic(), odom_count=state['odom_count']+1)
        state['unsafe'] |= max(abs(t.linear.x),abs(t.linear.y))>.01 or abs(t.angular.z)>.03
    def pose(m):
        p=m.pose.pose;q=p.orientation;c=m.pose.covariance
        state['pose']=dict(x=p.position.x,y=p.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            cov_x=c[0],cov_y=c[7],cov_yaw=c[35])
    def scan(m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        if not 0<=age<=.5 or not m.header.frame_id or m.angle_increment<=0:
            return
        if (len(m.ranges)-1)*m.angle_increment < math.radians(330):
            return
        if scans and (stamp<=scans[-1][1] or (stamp-scans[-1][1])/1e9<.15):
            return
        scans.append((time.monotonic(),stamp,m));del scans[:-3]
    node.create_subscription(Bool,'/rvclaw/control/armed',armed,10)
    node.create_subscription(String,'/rvclaw/control/state',control_state,10)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan_localization',scan,qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose,10)
    def stationary():
        now=time.monotonic()
        return (not state['unsafe'] and state['armed'] is False
                and state['state']=='DISARMED' and state['odom_count']>=5
                and all(now-state[k]<.5 for k in ('armed_at','state_at','odom_at')))
    try:
        grid=load_static_map(map_file)
        reference=(BLocalizationReference(Path(map_file).parent/'b_localization_reference.json',grid)
                   if reference_factory is None else reference_factory(grid))
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.03)
            if state['unsafe']:
                raise RuntimeError('B arrival review observed motion or a latched controller')
            if len(scans)==3 and stationary():break
        if len(scans)!=3 or not stationary():
            raise RuntimeError('B arrival review lacks fresh stationary feedback and scans')
        captured=list(scans)
        worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='b-stopped-reference')
        pending=worker.submit(reference.consensus,[s[2] for s in captured],dict(state['pose']))
        while not pending.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if not stationary():
                raise RuntimeError('B arrival stationary feedback changed during matching')
        if not pending.done():raise RuntimeError('B arrival geometry computation timed out')
        _,consensus=pending.result()
        if (not stationary() or time.monotonic()>deadline
                or time.monotonic()-scans[-1][0]>.5):
            raise RuntimeError('B arrival review expired or stationary state changed '
                '(elapsed=%.3f, scan_age=%.3f, state_age=%.3f, odom_age=%.3f, unsafe=%s)' % (
                    time.monotonic()-started,time.monotonic()-captured[-1][0],
                    time.monotonic()-state['state_at'],time.monotonic()-state['odom_at'],state['unsafe']))
        def signature(scan):
            return build_scan_signature(scan.ranges,scan.angle_min,scan.angle_increment,
                                        scan.range_min,scan.range_max)
        continuity=compare_scan_signatures(signature(captured[-1][2]),signature(scans[-1][2]),
            maximum_median_delta_m=.05,maximum_p90_delta_m=.10,maximum_close_delta_m=.10)
        if continuity['status']!='PASS':
            raise RuntimeError('B arrival scan changed while stationary geometry was computed')
        result=decide(consensus,state['pose'],waypoint)
        result.update(final_armed=False,final_state='DISARMED',
                      checked_monotonic=time.monotonic(),duration_s=time.monotonic()-started,
                      scan_stamps_ns=[s[1] for s in captured],
                      confirmation_scan_stamp_ns=scans[-1][1],scan_continuity=continuity)
        # Keep exact acquisitions for replay after the robot is carried away.
        # They are diagnostic evidence only, never a cached arrival permit.
        result['input_scans'] = [dict(stamp_ns=s[1],angle_min=s[2].angle_min,
            angle_increment=s[2].angle_increment,range_min=s[2].range_min,
            range_max=s[2].range_max,ranges=[float(v) if math.isfinite(v) else None
                for v in s[2].ranges]) for s in captured]
        return result
    finally:
        if worker is not None:worker.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()


def prepare_arrival(waypoint, navigation, *, align, review, verify, navigate,
                    clock=time.monotonic, timeout_s=90., review_position=None):
    """Finish B before announcing arrival; one verified reapproach at most."""
    result=dict(navigation)
    evidence=dict(status='FAIL', attempts=[], corrections=0, verifications=0,heading_corrections=0)
    result['b_arrival']=evidence
    started=clock();deadline=started+min(90.,timeout_s)
    evidence['started_monotonic']=started
    current=navigation
    need_heading=True
    last_heading=None
    heading_target=waypoint['yaw']
    try:
        if (review_position is not None and current.get('status')=='PASS'
                and current.get('final_armed') is False and current.get('final_state')=='DISARMED'
                and clock()+48.<deadline):
            # Prove XY before turning toward the camera. An ambiguous approach
            # view cannot authorize a pose update or motion; retain the normal
            # observation-heading review in that case.
            observed=current.get('final_amcl_pose')
            check=review_position(observed,waypoint,min(4.,deadline-clock()))
            evidence['attempts'].append(dict(stage='approach_geometry',result=check))
            if check.get('final_armed') is not False or check.get('final_state')!='DISARMED':
                raise RuntimeError('B approach review lost stationary hardware confirmation')
            if check.get('status')=='REVIEW' and check.get('decision') in ('RELOCALIZE','REAPPROACH'):
                pose,verification=verify(observed,min(20.,deadline-clock()-24.))
                evidence['verifications']+=1
                evidence['attempts'].append(dict(stage='approach_localization',result=verification))
                if pose is None or verification.get('status')!='PASS':
                    raise RuntimeError('B approach localization could not be verified')
                # Verification already includes fresh independent final scans,
                # AMCL convergence and actual-map rays. Do not immediately
                # repeat the sparse reference that necessitated that proof.
                check=dict(arrival_decision(dict(status='PASS',pose=pose),pose,waypoint,
                    position_only=True),final_armed=False,final_state='DISARMED')
                evidence['attempts'].append(dict(stage='verified_approach_geometry',result=check))
                if check.get('final_armed') is not False or check.get('final_state')!='DISARMED':
                    raise RuntimeError('B verified approach lost stationary hardware confirmation')
                current=dict(status='PASS',final_armed=False,final_state='DISARMED',final_amcl_pose=pose)
                if check.get('status')=='REVIEW' and check.get('decision')=='REAPPROACH':
                    if clock()+20.>=deadline:
                        raise RuntimeError('B approach correction has insufficient remaining budget')
                    current=dict(navigate(waypoint,pose,min(30.,deadline-clock()-15.)))
                    evidence['corrections']+=1
                    evidence['attempts'].append(dict(stage='approach_reapproach',result=current))
                elif check.get('status')!='PASS' or check.get('decision')!='POSITION_CONFIRMED':
                    raise RuntimeError('B verified approach geometry remains inconsistent')
        for _ in range(3):
            if (current.get('status')!='PASS' or current.get('final_armed') is not False
                    or current.get('final_state')!='DISARMED'):
                raise RuntimeError(current.get('error') or 'B arrival requires a successful DISARMED stop')
            if clock()+4.>=deadline:
                raise RuntimeError('B arrival preparation deadline expired')
            if need_heading:
                current=dict(align(heading_target))
                last_heading=current
                evidence['attempts'].append(dict(stage='heading',result=current))
                if (current.get('status')!='PASS' or current.get('final_armed') is not False
                        or current.get('final_state')!='DISARMED'):
                    raise RuntimeError('B observation heading did not finish safely')
            observed=current.get('final_amcl_pose')
            check=review(observed,waypoint,min(4.,deadline-clock()))
            evidence['attempts'].append(dict(stage='geometry',result=check))
            if (check.get('final_armed') is not False or check.get('final_state')!='DISARMED'):
                raise RuntimeError('B arrival geometry has no stationary hardware confirmation')
            if check.get('status')=='PASS' and check.get('decision')=='ARRIVED':
                evidence.update(status='PASS',check=check,duration_s=clock()-started,
                    checked_monotonic=clock(),target_yaw=waypoint['yaw'],
                    heading=dict(last_heading,final_amcl_pose=check['amcl_pose']))
                result.update(final_amcl_pose=check['amcl_pose'],final_armed=False,
                              final_state='DISARMED',final_transition_reason='operator_disarm')
                return result
            if check.get('status')!='REVIEW':
                raise RuntimeError(check.get('error','B arrival geometry was rejected'))
            if check.get('decision')=='ALIGN':
                if evidence['heading_corrections'] or clock()+4.>=deadline:
                    raise RuntimeError('B measured heading remains outside its bounded correction')
                measured=check.get('geometric_pose')
                amcl=check.get('amcl_pose')
                # This review already proved three stationary scans, actual
                # map rays and AMCL agreement. Correct the small measured yaw
                # bias through the existing guarded heading action, without
                # publishing a pose or repeating full localization.
                if (measured is None or amcl is None
                        or check.get('consensus',{}).get('status')!='PASS'
                        or not 0<=clock()-check.get('checked_monotonic',-math.inf)<=2.):
                    raise RuntimeError('B measured heading proof is unavailable or expired')
                bias=math.atan2(math.sin(amcl['yaw']-measured['yaw']),
                                math.cos(amcl['yaw']-measured['yaw']))
                if not math.isfinite(bias) or abs(bias)>math.radians(10):
                    raise RuntimeError('B measured heading bias exceeds its correction bound')
                heading_target=math.atan2(math.sin(waypoint['yaw']+bias),
                                          math.cos(waypoint['yaw']+bias))
                delta=abs(math.atan2(math.sin(heading_target-amcl['yaw']),
                                    math.cos(heading_target-amcl['yaw'])))
                if delta>math.radians(25):
                    raise RuntimeError('B measured heading correction is too large')
                evidence['attempts'].append(dict(stage='measured_heading_correction',
                    target_yaw=heading_target,bias_rad=bias))
                evidence['heading_corrections']+=1
                need_heading=True
                continue
            if evidence['verifications'] or clock()+24.>=deadline:
                raise RuntimeError('B arrival remained inconsistent after independent verification')
            pose,verification=verify(observed,min(20.,deadline-clock()-4.))
            evidence['verifications']+=1
            evidence['attempts'].append(dict(stage='localization',result=verification))
            if pose is None or verification.get('status')!='PASS':
                raise RuntimeError('B stopped localization could not be verified')
            # Recheck fresh geometry AFTER a verified AMCL update. Never drive
            # using an unaccepted scan fit or the earlier ambiguous AMCL pose.
            check=review(pose,waypoint,min(4.,deadline-clock()))
            evidence['attempts'].append(dict(stage='verified_geometry',result=check))
            if (check.get('final_armed') is not False or check.get('final_state')!='DISARMED'):
                raise RuntimeError('B verification lost stationary hardware confirmation')
            current=dict(status='PASS',final_armed=False,final_state='DISARMED',final_amcl_pose=pose)
            if check.get('status')=='PASS' and check.get('decision')=='ARRIVED':
                evidence.update(status='PASS',check=check,duration_s=clock()-started,
                    checked_monotonic=clock(),target_yaw=waypoint['yaw'],
                    heading=dict(last_heading,final_amcl_pose=check['amcl_pose']))
                result.update(final_amcl_pose=check['amcl_pose'],final_armed=False,
                              final_state='DISARMED',final_transition_reason='operator_disarm')
                return result
            if (check.get('decision')!='REAPPROACH' or check.get('status')!='REVIEW'
                    or evidence['corrections'] or clock()+20.>=deadline):
                raise RuntimeError('B verified geometry does not authorize a reapproach')
            current=dict(navigate(waypoint,pose,min(30.,deadline-clock()-15.)))
            evidence['corrections']+=1
            evidence['attempts'].append(dict(stage='reapproach',result=current))
            need_heading=True
        raise RuntimeError('B arrival correction budget exhausted')
    except Exception as exc:
        evidence.update(error=str(exc),duration_s=clock()-started)
        result.update(status='FAIL',error=str(exc),automatic_recovery_forbidden=True,
                      final_armed=current.get('final_armed'),final_state=current.get('final_state'),
                      final_amcl_pose=current.get('final_amcl_pose'))
        return result
