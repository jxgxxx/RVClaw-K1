"""Bounded, subscribe-only C arrival review. Never arms or resets hardware."""
import math
import threading
import time
from pathlib import Path


def arrival_decision(consensus, amcl_pose, waypoint):
    # Preserve the exact established B/C geometry tolerances.
    from b_stopped_arrival import arrival_decision as decide
    result=decide(consensus,amcl_pose,waypoint)
    if 'error' in result:result['error']=result['error'].replace('B ','C ')
    return result


def review_stopped_arrival(map_file, center, waypoint, *, timeout_s=12.0):
    """Independent acquisition/compute budgets within one overall deadline.

    Continue servicing safety feedback during computation. A deadline cancels
    the pure geometry worker; it cannot publish or affect the stopped robot.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from concurrent.futures import ThreadPoolExecutor
    from c_stationary_reference import StationaryReference
    from rvclaw_nav2_bringup.scan_map_consistency import load_static_map
    from rvclaw_nav2_bringup.fast_start_policy import build_scan_signature, compare_scan_signatures

    started=time.monotonic();deadline=started+min(12.,max(0.,timeout_s))
    node=Node('rvclaw_c_stopped_arrival',enable_rosout=False)
    worker=None;cancel=threading.Event();scans=[];timings={};phase='setup'
    state=dict(armed=None,state=None,armed_at=0.,state_at=0.,odom_at=0.,unsafe=False,
               pose=dict(center),odom_count=0)
    result=dict(status='FAIL',decision='STOP',failure_code='ARRIVAL_VERIFICATION_INCOMPLETE')

    def armed(m):
        state.update(armed=m.data,armed_at=time.monotonic())
        state['unsafe'] |= m.data
    def control_state(m):
        state.update(state=m.data,state_at=time.monotonic())
        state['unsafe'] |= m.data!='DISARMED'
    def odom(m):
        t=m.twist.twist
        state.update(odom_at=time.monotonic(),odom_count=state['odom_count']+1)
        state['unsafe'] |= max(abs(t.linear.x),abs(t.linear.y))>.01 or abs(t.angular.z)>.03
    def pose(m):
        p=m.pose.pose;q=p.orientation;c=m.pose.covariance
        state['pose']=dict(x=p.position.x,y=p.position.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            cov_x=c[0],cov_y=c[7],cov_yaw=c[35])
    def scan(m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        age=(node.get_clock().now().nanoseconds-stamp)/1e9
        if not 0<=age<=.5 or not m.header.frame_id or m.angle_increment<=0:return
        if (len(m.ranges)-1)*m.angle_increment<math.radians(330):return
        if scans and m.header.frame_id!=scans[0][2].header.frame_id:
            state['unsafe']=True;return
        if scans and (stamp-scans[-1][1])/1e9<.15:return
        scans.append((time.monotonic(),stamp,m));del scans[:-3]
    node.create_subscription(Bool,'/rvclaw/control/armed',armed,10)
    node.create_subscription(String,'/rvclaw/control/state',control_state,10)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan_localization',scan,qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose,10)
    def stationary():
        now=time.monotonic()
        return (not state['unsafe'] and state['armed'] is False and state['state']=='DISARMED'
            and state['odom_count']>=5
            and all(now-state[k]<.5 for k in ('armed_at','state_at','odom_at')))
    try:
        grid=load_static_map(map_file)
        reference=StationaryReference(Path(map_file).parent/'c_stationary_reference.json',grid,
                                      arrival_target=dict(waypoint))
        timings['setup_s']=time.monotonic()-started
        phase='acquisition';phase_start=time.monotonic()
        acquisition_deadline=min(deadline,phase_start+3.)
        while time.monotonic()<acquisition_deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if state['unsafe']:raise RuntimeError('C arrival observed motion or a latched controller')
            if len(scans)==3 and stationary():break
        timings['acquisition_s']=time.monotonic()-phase_start
        if len(scans)!=3 or not stationary():
            raise RuntimeError('C arrival lacks fresh stationary feedback and scans')
        captured=list(scans)
        result['scan_stamps_ns']=[s[1] for s in captured]
        phase='computation';phase_start=time.monotonic()
        # This is pure computation on a confirmed stopped chassis. K1's
        # observed 4 s scheduling excursion is not missing geometry evidence.
        # Keep the overall deadline and continuously recheck fresh feedback.
        reference.deadline=min(deadline,phase_start+8.)
        reference.cancel_event=cancel
        worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='c-stopped-reference')
        pending=worker.submit(reference.consensus,[s[2] for s in captured],dict(state['pose']))
        while not pending.done() and time.monotonic()<reference.deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if not stationary():raise RuntimeError('C arrival stationary feedback changed during matching')
        timings['computation_s']=time.monotonic()-phase_start
        if not pending.done():raise TimeoutError('C arrival geometry computation timed out')
        _,consensus=pending.result()
        result['consensus']=consensus
        phase='confirmation';phase_start=time.monotonic()
        if (not stationary() or time.monotonic()>=deadline
                or time.monotonic()-scans[-1][0]>.5):
            raise RuntimeError('C arrival review expired or stationary state changed')
        def signature(s):
            return build_scan_signature(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max)
        continuity=compare_scan_signatures(signature(captured[-1][2]),signature(scans[-1][2]),
            maximum_median_delta_m=.05,maximum_p90_delta_m=.10,maximum_close_delta_m=.10)
        result.update(scan_continuity=continuity,confirmation_scan_stamp_ns=scans[-1][1])
        if continuity['status']!='PASS':
            raise RuntimeError('C arrival scan changed while stationary geometry was computed')
        result.update(arrival_decision(consensus,state['pose'],waypoint))
        timings['confirmation_s']=time.monotonic()-phase_start
        if result['status']=='PASS':result.pop('failure_code',None)
    except Exception as exc:
        result.update(status='FAIL',decision='STOP',error=str(exc),failed_phase=phase)
    finally:
        cancel.set()
        # Do not turn a bounded failure into an unbounded executor join. The
        # pure worker checks cancellation between every scoring yaw/ray batch.
        if worker is not None:worker.shutdown(wait=False,cancel_futures=True)
        stopped=stationary()
        result.update(final_armed=state['armed'],final_state=state['state'],
            stationary_confirmed=stopped,checked_monotonic=time.monotonic(),
            duration_s=time.monotonic()-started,timings=timings,
            feedback_ages_s={k:time.monotonic()-state[k] for k in ('armed_at','state_at','odom_at')})
        if result['status']=='PASS' and (not stopped or time.monotonic()>=deadline):
            result.update(status='FAIL',decision='STOP',failure_code='ARRIVAL_VERIFICATION_INCOMPLETE',
                error='C arrival evidence expired before completion',failed_phase='confirmation')
        node.destroy_node()
    return result
