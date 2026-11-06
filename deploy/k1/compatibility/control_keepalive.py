"""Reusable independent ROS deadman publisher for a route process."""
import atexit
import os
import select
import subprocess
import sys
import time


_PROCESS = None
_CONTROL_FD = None
_REPLY_FD = None
_PUBLISHING = False
_READY = False


def configure_keepalive_thread():
    """Change only this Linux thread; leave its parent and watchdog unchanged."""
    value = os.environ.get('RVCLAW_DEADMAN_CPU', '').strip()
    if not value:
        return None
    cpu = int(value)
    if cpu < 0 or cpu >= (os.cpu_count() or 0):
        raise RuntimeError('configured deadman CPU is unavailable')
    os.sched_setaffinity(0, {cpu})
    if os.sched_getaffinity(0) != {cpu}:
        raise RuntimeError('deadman thread CPU allocation did not apply')
    return cpu


def _reply(expected: bytes, timeout_s: float) -> None:
    readable, _, _ = select.select([_REPLY_FD], [], [], timeout_s)
    if not readable or os.read(_REPLY_FD, 1) != expected:
        raise RuntimeError('independent deadman publisher handshake failed')


def shutdown_independent_publisher() -> None:
    global _PROCESS, _CONTROL_FD, _REPLY_FD, _PUBLISHING, _READY
    process, control_fd, reply_fd = _PROCESS, _CONTROL_FD, _REPLY_FD
    _PROCESS = _CONTROL_FD = _REPLY_FD = None
    _PUBLISHING = False
    _READY = False
    for fd in (control_fd, reply_fd):
        if fd is not None:
            os.close(fd)
    if process is not None:
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)


atexit.register(shutdown_independent_publisher)


def prepare_independent_publisher() -> int:
    """Start discovery while disarmed; do not enable or publish the heartbeat."""
    global _PROCESS, _CONTROL_FD, _REPLY_FD
    if _PROCESS is not None and _PROCESS.poll() is not None:
        shutdown_independent_publisher()
    if _PROCESS is None:
        control_read, control_write = os.pipe()
        reply_read, reply_write = os.pipe()
        try:
            process = subprocess.Popen(
                [sys.executable, __file__, '--serve', str(control_read),
                 str(reply_write), str(os.getpid())],
                pass_fds=(control_read, reply_write),
            )
        except BaseException:
            for fd in (control_read, control_write, reply_read, reply_write):
                os.close(fd)
            raise
        os.close(control_read)
        os.close(reply_write)
        _PROCESS, _CONTROL_FD, _REPLY_FD = process, control_write, reply_read
    return _PROCESS.pid


def start_independent_publisher() -> int:
    global _PUBLISHING, _READY
    prepare_independent_publisher()
    if not _READY:
        try:
            _reply(b'R', 15.0)
            _READY = True
        except BaseException:
            shutdown_independent_publisher()
            raise
    if not _PUBLISHING:
        try:
            os.write(_CONTROL_FD, b'1')
            _reply(b'1', 3.0)
        except BaseException:
            shutdown_independent_publisher()
            raise
        _PUBLISHING = True
    return _PROCESS.pid


def pause_independent_publisher() -> None:
    global _PUBLISHING
    if _PROCESS is None or not _PUBLISHING:
        return
    try:
        os.write(_CONTROL_FD, b'0')
        _reply(b'0', 3.0)
        _PUBLISHING = False
    except BaseException:
        shutdown_independent_publisher()


def serve(control_fd: int, ready_fd: int, parent_pid: int) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    configure_keepalive_thread()
    rclpy.init()
    node = Node('rvclaw_independent_deadman_keepalive')
    publisher = node.create_publisher(
        Bool,
        '/rvclaw/control/deadman',
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    try:
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        if publisher.get_subscription_count() == 0:
            return 2
        os.write(ready_fd, b'R')
        poller = select.poll()
        poller.register(control_fd, select.POLLIN | select.POLLHUP | select.POLLERR)
        active = False
        last_sent = None
        max_gap_s = 0.0
        publish_count = 0
        while os.getppid() == parent_pid:
            if poller.poll(30 if active else 200):
                command = os.read(control_fd, 1)
                if not command:
                    break
                if command == b'1':
                    active = True
                    publisher.publish(Bool(data=True))
                    last_sent = time.monotonic()
                    max_gap_s = 0.0
                    publish_count = 1
                elif command == b'0':
                    active = False
                    publisher.publish(Bool(data=False))
                    print(f'independent_deadman count={publish_count} '
                          f'max_send_gap_s={max_gap_s:.4f}', file=sys.stderr,
                          flush=True)
                else:
                    break
                os.write(ready_fd, command)
            elif active:
                publisher.publish(Bool(data=True))
                now = time.monotonic()
                max_gap_s = max(max_gap_s, now - last_sent)
                last_sent = now
                publish_count += 1
        return 0
    finally:
        if ready_fd >= 0:
            os.close(ready_fd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 5 or sys.argv[1] != '--serve':
        raise SystemExit(2)
    raise SystemExit(serve(*(int(value) for value in sys.argv[2:])))
