"""Scoped C and return observation model; updates never authorize motion.

B's validated additive model remains the default. The probabilistic model
is retained for C's approach and observation, then restored for A/B and
at route exit.
ORIGIN uses the range/occlusion beam model so an endpoint match across a wall
cannot keep the return filter on a displaced corridor hypothesis. Recorded
returns reproduce that failure with the probabilistic endpoint-only model.
Return uses 60 sampled rays to retain more of the narrow corridor landmarks;
the measurement noise and all acceptance thresholds remain unchanged.
A/B/C retain their reviewed rates and beam counts. The matching K1
AMCL library changes recovery rates in place, preserving pose and particles.
No pose or map is rewritten by a profile change.
"""
import math
import time

DEFAULT = dict(laser_model_type='likelihood_field', max_beams=60,
               recovery_alpha_fast=0., recovery_alpha_slow=0.)
C_APPROACH = dict(laser_model_type='likelihood_field_prob', max_beams=90,
                  recovery_alpha_fast=0., recovery_alpha_slow=0.)
LEGACY_RETURN_TRACKING = dict(C_APPROACH,recovery_alpha_fast=.1,recovery_alpha_slow=.001)
LEGACY_BEAM_RETURN_TRACKING = dict(laser_model_type='beam', max_beams=30,
                       recovery_alpha_fast=.1,recovery_alpha_slow=.001)
RETURN_TRACKING = dict(LEGACY_BEAM_RETURN_TRACKING,max_beams=60)


class InspectionLocalizationProfile:
    def __init__(self):
        self.node = None
        self.events = []
        self.changed = False

    def _open(self):
        if self.node is not None:
            return
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool, String
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        self.node = Node('inspection_localization_profile', enable_rosout=False)
        self.state = dict(armed=None, state=None, armed_at=0., state_at=0.,
                          odom_at=0., speed=math.inf)
        def armed(m): self.state.update(armed=m.data, armed_at=time.monotonic())
        def state(m): self.state.update(state=m.data, state_at=time.monotonic())
        def odom(m):
            t=m.twist.twist
            stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
            age=(self.node.get_clock().now().nanoseconds-stamp)/1e9
            if not 0.<=age<=.3:
                return
            self.state.update(odom_at=time.monotonic(), speed=max(
                abs(t.linear.x)/.01, abs(t.linear.y)/.01, abs(t.angular.z)/.03))
        self.node.create_subscription(Bool,'/rvclaw/control/armed',armed,10)
        self.node.create_subscription(String,'/rvclaw/control/state',state,10)
        self.node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
        self.get = self.node.create_client(GetParameters,'/amcl/get_parameters')
        self.set = self.node.create_client(SetParametersAtomically,'/amcl/set_parameters_atomically')

    def _call(self, client, request):
        import rclpy
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('AMCL observation profile service unavailable')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(self.node,future,timeout_sec=3.)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('AMCL observation profile update timed out')
        return future.result()

    def _read(self):
        from rcl_interfaces.srv import GetParameters
        request=GetParameters.Request();request.names=list(DEFAULT)
        values=self._call(self.get,request).values
        if (len(values)!=4 or values[0].type!=4 or values[1].type!=2
                or values[2].type!=3 or values[3].type!=3):
            raise RuntimeError('AMCL observation profile types are invalid')
        return dict(laser_model_type=values[0].string_value,max_beams=values[1].integer_value,
                    recovery_alpha_fast=values[2].double_value,recovery_alpha_slow=values[3].double_value)

    def _stationary(self, restoring):
        import rclpy
        barrier=time.monotonic();deadline=barrier+3.
        # Updating an observation model does not reset a safety latch or arm
        # the chassis. A manual resume must be able to prepare localization
        # while the previous stop is still latched; navigation resets it only
        # after its normal pre-arm and path checks.
        allowed={'DISARMED','WATCHDOG_LATCHED','ESTOP_LATCHED'}
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.02)
            now=time.monotonic();s=self.state
            if (s['armed'] is False and s['state'] in allowed and s['speed']<=1.
                    and all(barrier<s[k]<=now and now-s[k]<.5
                            for k in ('armed_at','state_at','odom_at'))):
                return
        raise RuntimeError('AMCL observation profile requires fresh stopped hardware feedback')

    def _apply(self, desired, *, restoring=False):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        self._open()
        current=self._read()
        if current==desired:
            return
        if current not in (DEFAULT,C_APPROACH,LEGACY_RETURN_TRACKING,
                           LEGACY_BEAM_RETURN_TRACKING,RETURN_TRACKING):
            raise RuntimeError('unexpected AMCL observation profile; refusing to replace it')
        self._stationary(restoring)
        # Mark before sending: even a missing response may have applied it.
        self.changed=True
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in desired.items()]
        response=self._call(self.set,request)
        if not response.result.successful or self._read()!=desired:
            raise RuntimeError('AMCL observation profile did not commit: '+response.result.reason)
        self.events.append(dict(before=current,after=dict(desired),motion_authorized=False))

    def select(self, waypoint):
        # Also repairs the known C profile left by an interrupted prior route,
        # while refusing to overwrite any unrelated configuration.
        use_probabilistic = waypoint.get('name') in {'C', 'ORIGIN'}
        desired=(RETURN_TRACKING if waypoint.get('name')=='ORIGIN'
                 else C_APPROACH if use_probabilistic else DEFAULT)
        self._apply(desired,
                    restoring=not use_probabilistic)

    def close(self):
        try:
            if self.node is not None:
                self._apply(DEFAULT,restoring=True)
        finally:
            if self.node is not None:
                self.node.destroy_node();self.node=None
