# Patrol live status

`GET /inspection-progress.json` preserves confirmed waypoint progress. Optional
`live_pose` contains `{task_id,map_id,x,y,yaw,sampled_at}` in the selected map's
metre/radian frame. It is display-only and never authorizes motion or confirms
arrival. Only finite, fresh poses from this task and the exact selected map
hash are exposed. The existing localization status carries a display-only
`display_pose` plus source time. The existing passive web observer forwards it
in memory; no extra process, subscriber, timer or pose file is created.
The browser interpolates between received coordinates only, freezes on stale,
paused or terminal data, and clears the marker on task/map changes. Reduced
motion disables interpolation. Confirmed progress remains waypoint-based.

After a Studio start is accepted and its route process is created, the control
service emits exactly one `inspection_start_accepted` speech event containing
the task ID, source `studio`, status PASS and timestamp. The existing voice
event watcher plays the same `inspection-launched.wav` reply as spoken starts.
Rejected starts/prepare requests do not emit the event or start another route.

Optional `startup` includes `{stage,elapsed_s,first_motion_at}` from this run's
bounded command log. `first_motion` requires fresh nonzero chassis odometry
while armed/ACTIVE. Service readiness and task acceptance never prove motion.

`POST :8019/api/inspection-control`, action `prepare`, map_id: validates the
current map and idle task state, warms navigation for 90 seconds and returns
`{prepared,motion_authorized:false}`. It does not create a task, arm, reset pose,
clear latches or publish velocity. It uses the existing origin restriction.
Opening the current inspection page may prepare once; no periodic warmup occurs.
Missing USB devices or active mapping prevent warming. Idle/absent hardware
still sleeps through the existing demand scheduler.

Cold route startup may import and validate Python configuration while the
demand scheduler initializes navigation. The same bounded service, selected
profile, live-map, stationary-localization and control-priority checks must
pass before ROS control publishers or motion admission. Import completion is
not service readiness or evidence of first motion.

Stationary `valid` is not a moving display verdict: a fresh `CONTROLLED_MOTION`
pose may be displayed when hardware is connected, transport attached and lidar
fresh. The exact map hash, task binding and source-time checks still apply;
this changes no localization or motion permission. The passive observer emits
one in-memory snapshot per second. Progress polling runs independently of
photo/history loading, with at most one request of each type in flight.

Navigation software errors and automatic recoveries remain quiet; their actual
cause and terminal task state remain visible in Studio and the event log.
Only an explicitly identified emergency/collision or critical battery condition
uses a navigation fault voice alert. A generic safety latch is not collision
evidence. Quiet events still receive acknowledgements without audio playback.
This changes neither audio gain nor emergency-stop enforcement.

`GET /inspection-progress.json?latest=1` omits task history for 1 Hz live polling;
the default endpoint retains its history contract. The latest response retains
confirmed steps, an evidence photo-count revision and `event_revision` from
the existing progress watcher's consumed-event count. Photo/history requests
run independently. Read-only polling has bounded timeouts, including response
body reads, so a stalled network response cannot block future refreshes.
Display trails split across missing intervals or physically impossible position
jumps; no line is drawn through a relocalization jump. Actual received positions
and confirmed progress are preserved, without substituting an animated route.

The K1 serial transport retains its 50 ms write/backpressure deadline. A single
complete, unblocked guarded-motion write with less than 50 ms thread CPU may
be observed within 150 ms of starting, accommodating measured host scheduling
delay. Beyond that bound it is a host scheduling stop, not a disconnected
transport. Partial writes, backpressure, I/O errors and AUTO admission retain
their existing failure behavior. Scheduling recovery still requires a terminal
old goal, fresh stopped feedback and normal admission; it cannot clear ESTOP.

Inspection alignment uses one in-place retry only for a verified software
deadman timeout: the exact prior Spin must be terminal and fresh hardware
feedback must confirm a connected, DISARMED, stationary chassis with the same
timeout reason. The next attempt repeats normal sensor, pose and collision
admission. It does not re-drive an arrived waypoint or clear emergency latches.
An exhausted alignment retry retains its actual transition reason and error.

The K1 control profile permits a 0.50 s heartbeat gap, a 0.60 s autonomous
command receive gap and 0.30 s autonomous command timestamp age. Teleoperation
command limits, explicit release/ESTOP, missing feedback and obstacle checks
are unchanged. These are bounded host scheduling allowances, not permission
to continue without sensors or to reuse commands after a stop/re-arm.
The controller/collision TF wait allowance is 0.35 s and behavior allowance
0.30 s. Collision and command-adapter scan freshness both allow 0.50 s;
missing scan data beyond that bound still forces zero output. Obstacle
polygons and speed/acceleration limits are unchanged. These matched bounds
accommodate measured transform delivery jitter without unlimited blind motion.

K1 keeps controller updates at 20 Hz and the existing trajectory collision
sampling resolution. Candidate velocity samples are 10 linear by 13 angular;
the BT dispatcher ticks every 50 ms. The velocity smoother's odometry window
is 0.10 s; the installed Humble controller has no `odom_duration` parameter.
Speed, acceleration, goal tolerances and obstacle geometry are unchanged.
Transient controller calculation/TF failures may recover within 0.80 s;
collision checks and the independent chassis stop timeouts remain authoritative.

`GET /api/inspection-assets` optionally returns `trajectory` containing
`{task_id,map_id,points:[{x,y,yaw,sampled_at,breakBefore}]}` for the selected
task. Points come only from saved `route_pose_observation` events. No planned
route is substituted when evidence is absent. Duplicate samples are removed;
missing intervals, relocalization jumps and decimated intervals split lines.
Extraction uses bounded memory and a source-versioned cache. Historical views
show only the selected task's trajectory on its associated map; stale requests
cannot replace a newer selection. The live poll payload is unchanged.

After an accepted Studio cancellation has terminated its exact route process,
the control service emits one `inspection_cancelled` event, status `CANCELLED`,
task ID and source `studio`. The existing speech watcher plays
`inspection-cancelled.wav`: “巡检任务已取消”, using the existing voice and gain.
Rejected requests and unsuccessful stops do not emit completion speech.

After navigation reports a terminal action, motion is disarmed before draining
callbacks and zero commands. Stationary feedback still owns final completion.
Independent turn stopping includes bounded braking coast. A disagreement between
the stopping scan and settled AMCL first compares three fresh, stationary scans
against the turn's original reference, including the completed coast. Insufficient
or conflicting evidence retains full stopped map verification. No motion or pose
reset is authorized by this measurement.

C arrival geometry has a bounded eight-second computation allowance within a
twelve-second total review. Hardware and fresh scan continuity remain required
throughout computation; all geometric acceptance thresholds are unchanged.

Combined startup prepares the existing runtime concurrently with Python imports,
then waits for the same admission before constructing control nodes. A freshly
independently verified startup pose becomes the first segment's continuity anchor;
current pose, sensor and path checks still apply. An already matching, fresh,
low-covariance live pose may avoid particle reinitialization, retaining five fresh
stationary updates and independent final map/ray/runtime acceptance.

Live progress also returns `live_track`, at most 64 recent measured map poses
from the existing passive observer. The window covers short network gaps; each
point retains its original source timestamp and must match the task/map and
follow task start. It is display-only and never grants navigation validity.
The browser merges timestamps, preserves the track across stationary pauses,
and draws no interpolation across a relocalization jump. Current motion uses
the live track; saved trajectories are for completed or historical tasks.

Verified A/B/C arrivals are atomically persisted under
`runtime/waypoint-arrivals/facts/` before publishing their voice events. These
facts retain the original event ID, waypoint, status and occurrence timestamp;
voice consumption and acknowledgement cleanup cannot remove them. The progress
watcher reads facts alongside legacy spool/ack files, deduplicates occurrences,
and assigns them to the existing task time window. No elapsed-time animation
or unverified motion is treated as a completed waypoint.

Meter-result speech is informational and queued for the existing voice owner;
navigation preparation does not wait for its playback acknowledgement. Arrival
announcements, verified inspection completion and route-summary speech retain
their existing ordering. `inspection_task.speech_ack.status=QUEUED` means
playback was requested, not that it was heard or completed.

During travel, a delayed AMCL publication requests one asynchronous filter
update at a time, at most once per second after 0.75 s without a callback.
The existing 1.50 s freshness and three-observation stop bounds remain;
missing pose publications and measured map disagreement use separate streaks.
This request never publishes a pose, blocks a control callback or clears a latch.
Interrupted ORIGIN transit uses the existing unique whole-map stopped proof;
the endpoint's local reference applies only to endpoint verification. Existing
pose-change, route-distance, scan/ray and normal re-arm gates remain required.

Turn fitting may wait at most 0.30 s while disarmed for a fresh reference.
The measured-turn stopping algorithm, stopped absolute heading and position
acceptance, correction budgets and motion limits are unchanged. Experimental
source-time turn prediction is excluded after comparative coast simulations
showed an increased undershoot risk in some high-gain scenarios.

An unattempted guarded frame delayed by host scheduling for 50–150 ms is
discarded, replaced by ordinary zero for that control tick, and counted in
the hardware log. The next tick reevaluates command/deadman/feedback freshness
and ramps from zero. No old frame is replayed. Partial/blocked writes, AUTO
admission, I/O errors and scheduling delays beyond 150 ms retain their stop.

K1 AMCL retains up to eight TF-waiting scans, separately from the one-scan
subscription backlog. Samples older than 0.50 s are discarded before filter
work. This covers measured delayed odometry delivery without allowing an
unbounded scan backlog or changing particle, likelihood or geometric gates.

A STALE_AMCL motion stop disarms and terminates the exact navigation goal;
it does not assert a synthetic emergency latch. One autonomous resumption per
leg requires connected, stationary, DISARMED hardware, fresh stopped independent
whole-map verification and a pose change within 0.75 m. Normal path, scan,
costmap and arming admission repeats. Real latches, unconfirmed stops, ambiguous
verification and a second freshness interruption remain stopped.

Dashboard hardware health distinguishes initialization, stationary localization
without a new AMCL publication, and bounded automatic recovery from a physical
device failure. These states use `unknown`/待确认 or `idle`/待机 with a specific
reason; they never grant motion validity. Explicit disconnection and emergency
stops remain errors. Observer/diagnostic age is not a sensor disconnection.
