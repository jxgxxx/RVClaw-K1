#!/usr/bin/env python3
"""Prepare navigation priorities at service startup, before any mission."""
from pathlib import Path
import os
import subprocess
import sys
import time

patterns = (
    '/rvclaw_stm32_base/hardware_node',
    '/rvclaw_nav2_bringup/cmd_vel_adapter',
    '/nav2_controller/controller_server',
    '/nav2_planner/planner_server',
    '/nav2_bt_navigator/bt_navigator',
    '/nav2_behaviors/behavior_server',
    '/nav2_velocity_smoother/velocity_smoother',
    '/nav2_collision_monitor/collision_monitor',
    '/rvclaw_nav2_bringup/rvclaw-scan-self-filter',
    '/rvclaw_nav2_bringup/rvclaw-odom-tf',
    '/nav2_amcl/amcl',
    '/rvclaw_nav2_bringup/rvclaw-localization-runtime',
    '/lslidar_driver/lslidar_driver_node',
    '/nav2_lifecycle_manager/lifecycle_manager',
    '/nav2_map_server/map_server',
    '/nav2_smoother/smoother_server',
    '/nav2_waypoint_follower/waypoint_follower',
)

# All three managers must remain schedulable alongside their managed nodes.
# Their unchanged four-second bond timeout cannot compensate for starvation.
expected_counts = {'/nav2_lifecycle_manager/lifecycle_manager': 3}

# Keep command admission and the sole serial owner most responsive. Sensors and
# the controller share the full CPU pool at a modest priority; orchestration,
# planning and lifecycle work do not compete at realtime-like nice -10.
def expected_nice(pattern):
    if pattern in ('/rvclaw_stm32_base/hardware_node',
                   '/rvclaw_nav2_bringup/cmd_vel_adapter'):
        return -10
    if pattern in ('/nav2_controller/controller_server',
                   '/nav2_collision_monitor/collision_monitor',
                   '/rvclaw_nav2_bringup/rvclaw-scan-self-filter',
                   '/rvclaw_nav2_bringup/rvclaw-odom-tf',
                   '/nav2_amcl/amcl',
                   '/lslidar_driver/lslidar_driver_node'):
        return -5
    return 0


def set_thread_priority(tasks, wanted, deadline):
    tids = [int(task.name) for task in tasks]
    privileged = [tid for tid in tids
                  if os.getpriority(os.PRIO_PROCESS, tid) != wanted
                  and os.getpriority(os.PRIO_PROCESS, tid) > -10]
    if wanted < 0 and privileged:
        subprocess.run(['sudo', '-n', '/usr/bin/renice', '-n', '-10', '-p',
                        *[str(tid) for tid in privileged]], check=True,
                       stdout=subprocess.DEVNULL,
                       timeout=max(.01, min(5., deadline-time.monotonic())))
    for tid in tids:
        if os.getpriority(os.PRIO_PROCESS, tid) != wanted:
            os.setpriority(os.PRIO_PROCESS, tid, wanted)

def expected_cpus(pattern):
    # Keep navigation, sensor, adapter, and chassis workers eligible for all
    # online cores; the independent mission heartbeat has its own CPU.
    return set(range(os.cpu_count() or 0))

def prepare(ready_only=False):
    remaining=set(patterns)
    # Mission admission can overlap late lifecycle-created threads. Retry a
    # transient preparation failure, but never admit unresolved priorities.
    deadline=time.monotonic()+(10. if ready_only else 60.)
    while remaining and time.monotonic()<deadline:
        found={pattern:[] for pattern in remaining}
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:arguments=(entry/'cmdline').read_bytes().split(b'\0')
            except OSError:continue
            for pattern in remaining:
                if any(argument.endswith(pattern.encode()) for argument in arguments[:2]):
                    found[pattern].append(entry)
        # Prepare the dependency threads in one privileged call. Launching
        # sudo/renice separately for every node serialized cold task startup.
        privileged=[]
        for pattern, processes in found.items():
            if len(processes)!=expected_counts.get(pattern,1) or expected_nice(pattern)>=0:
                continue
            try:
                for process in processes:
                    for task in (process/'task').iterdir():
                        tid=int(task.name)
                        if os.getpriority(os.PRIO_PROCESS,tid)>expected_nice(pattern):
                            privileged.append(tid)
            except OSError:continue
        if privileged:
            try:
                subprocess.run(['sudo','-n','/usr/bin/renice','-n','-10','-p',
                    *map(str,privileged)],check=True,stdout=subprocess.DEVNULL,
                    timeout=max(.01,min(5.,deadline-time.monotonic())))
            except subprocess.SubprocessError as error:
                print('bulk control priority preparation deferred:',str(error),flush=True)
        for pattern,processes in found.items():
            if len(processes)!=expected_counts.get(pattern, 1):continue
            try:
                tasks=[task for process in processes for task in (process/'task').iterdir()]
                cpus=expected_cpus(pattern)
                for entry in tasks:
                    if os.sched_getaffinity(int(entry.name)) != cpus:
                        os.sched_setaffinity(int(entry.name), cpus)
                wanted_nice = expected_nice(pattern)
                try:
                    set_thread_priority(tasks, wanted_nice, deadline)
                except subprocess.SubprocessError as error:
                    print('control priority helper incomplete:',pattern,str(error),flush=True)
                tasks=[task for process in processes for task in (process/'task').iterdir()]
                if not tasks or any(os.getpriority(os.PRIO_PROCESS, int(entry.name)) != wanted_nice for entry in tasks):
                    continue
                if any(os.sched_getaffinity(int(entry.name)) != cpus for entry in tasks):
                    continue
                print('control priority prepared:',pattern,[p.name for p in processes],
                      'nice='+str(wanted_nice),flush=True)
                remaining.remove(pattern)
            except (OSError,subprocess.SubprocessError) as error:
                print('control priority preparation deferred:',pattern,str(error),flush=True)
        if remaining:time.sleep(min(.25,max(0.,deadline-time.monotonic())))
    if remaining:
        raise RuntimeError('Navigation dependency priorities unresolved: %s' % sorted(remaining))

if __name__=='__main__':prepare(ready_only='--ready' in sys.argv)
