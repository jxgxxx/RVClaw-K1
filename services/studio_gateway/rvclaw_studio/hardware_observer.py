"""Passive ROS 2 subscriber. Never creates command publishers or service clients."""
import json
import math
import signal
import time
from live_progress import append_display_track


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from diagnostic_msgs.msg import DiagnosticArray

    rclpy.init()
    node = Node('rvclaw_web_health_observer', enable_rosout=False, start_parameter_services=False)
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    readings, subscriptions = {}, []
    started = time.time()
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def sensor(msg, key):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        readings[key] = {'received_at': time.time(), 'source_at': stamp}

    for topic, kind, key in [('/scan', LaserScan, 'lidar'), ('/odom', Odometry, 'odometry'), ('/imu/data_raw', Imu, 'imu')]:
        subscriptions.append(node.create_subscription(kind, topic, lambda msg, key=key: sensor(msg, key), sensor_qos))

    def boolean(msg, key):
        readings[key] = {'received_at': time.time(), 'value': bool(msg.data)}

    for topic, key in [('/rvclaw/control/feedback_connected', 'feedback'), ('/rvclaw/control/hardware_transport_attached', 'transport')]:
        subscriptions.append(node.create_subscription(Bool, topic, lambda msg, key=key: boolean(msg, key), state_qos))

    def controller(msg):
        readings['controller'] = {'received_at': time.time(), 'state': str(msg.data)[:80]}

    def transition(msg):
        readings['transition'] = {'received_at': time.time(), 'reason': str(msg.data)[:120]}

    def localization(msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # Display-only pose rides the existing status stream, with its map
        # hash and source stamp. The gateway validates it before publishing.
        previous=readings.get('localization',{}).get('display_track',[])
        row={'received_at': time.time(), **{key: data.get(key) for key in ('state', 'valid', 'hardware_connected', 'hardware_transport_attached', 'lidar_fresh', 'pose_message_fresh', 'pose_reusable', 'armed', 'reason', 'published_at_s', 'map_sha256', 'display_pose')}}
        row['display_track']=append_display_track(previous,row)
        readings['localization']=row

    def diagnostics(msg):
        for item in msg.status:
            if item.name == 'lifecycle_manager_navigation: Nav2 Health':
                level = item.level[0] if isinstance(item.level, bytes) else int(item.level)
                readings['navigation'] = {'received_at': time.time(), 'level': level,
                                           'message': str(item.message)[:160]}

    subscriptions.extend([
        node.create_subscription(String, '/rvclaw/control/state', controller, state_qos),
        node.create_subscription(String, '/rvclaw/control/transition_reason', transition, state_qos),
        node.create_subscription(String, '/rvclaw/localization/status', localization, state_qos),
        node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics, state_qos),
    ])
    last_output = 0
    try:
        while not stopped and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.25)
            # spin_once waits when idle; a fixed sleep after every callback
            # starves localization/status behind high-rate IMU and odometry.
            # depth=1 already bounds pending work for each subscription.
            now = time.time()
            if now - last_output >= 1:
                print(json.dumps({'sampled_at': now, 'started_at': started, 'readings': readings}, allow_nan=False), flush=True)
                last_output = now
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
