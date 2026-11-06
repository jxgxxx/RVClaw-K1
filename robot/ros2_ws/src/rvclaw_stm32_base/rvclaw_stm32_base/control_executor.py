"""Bounded ready-batch scheduling for the single-owner hardware bridge.

Humble's default executor dispatches timers before subscriptions. When both
are ready, serial feedback/telemetry can consume the remaining deadman budget
before an already received heartbeat is handled. Process ready safety inputs
first, in the same thread; do not change deadlines or add concurrent writers.
"""
import time
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.qos_event import QoSEventHandler
from rclpy.timer import Timer


def input_priority(item):
    entity = item[1]
    topic = getattr(entity, 'topic_name', '')
    if topic.endswith('/rvclaw/control/emergency_stop'):
        return 0
    if topic.endswith('/rvclaw/control/deadman'):
        return 1
    if topic.endswith(('/rvclaw/cmd_vel/teleop', '/rvclaw/cmd_vel/autonomy')):
        return 2
    # An already-ready arm/reset request must run after fresh safety inputs
    # and before another feedback/telemetry timer can consume their budget.
    if isinstance(entity, Timer):
        return 4
    return 3


class ControlInputExecutor(SingleThreadedExecutor):
    def __init__(self, *, context=None):
        super().__init__(context=context)
        self._next_qos_warning_poll = 0.0
        self._poll_qos_warnings = True

    def can_execute(self, entity):
        # Humble creates an implicit incompatible-QoS warning event for every
        # publisher/subscriber. Rebuilding all these native wait-set entries
        # on every control callback is expensive on K1. Check only these
        # diagnostic events at 2 Hz; leave user callbacks, deadline/liveliness
        # events, subscriptions, services and timers at their original rate.
        if isinstance(entity, QoSEventHandler) and not self._poll_qos_warnings:
            callback = entity.callback
            if (getattr(callback, '__module__', '') == 'rclpy.qos_event'
                    and getattr(callback, '__name__', '') == '_default_incompatible_qos_callback'):
                return False
        return super().can_execute(entity)

    def _wait_for_ready_callbacks(self, timeout_sec=None, nodes=None,
                                  condition=lambda: False):
        # Completing one upstream batch also releases its wait set. A finite
        # wait matters for in-progress tasks which Humble yields before wait().
        seconds = getattr(timeout_sec, 'timeout', timeout_sec)
        seconds = .01 if seconds is None or seconds < 0 else min(.01, seconds)
        now = time.monotonic()
        self._poll_qos_warnings = now >= self._next_qos_warning_poll
        if self._poll_qos_warnings:
            self._next_qos_warning_poll = now + .5
        pending = []
        try:
            pending.extend(super()._wait_for_ready_callbacks(seconds, nodes, condition))
        except TimeoutException:
            if not pending:
                raise
        yield from sorted(pending, key=input_priority)
