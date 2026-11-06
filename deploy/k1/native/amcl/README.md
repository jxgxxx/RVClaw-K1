# K1 AMCL sensor-model switch repair

`amcl_node.cpp` is derived from ros-navigation/navigation2 tag `1.1.18`,
`nav2_amcl/src/amcl_node.cpp`. Its original LGPL-2.1-or-later notice is retained.
The installed package must be `ros-humble-nav2-amcl 1.1.18-1jammy`; its public
header SHA256 is `4cfa6e727e05e498444b1f35d659472bc34579f4000e22ec286c0c7207703bdb`.

The change separates rebuilding the laser observation model from rebuilding
the TF message filter. Beam/model updates preserve the live filter and scan
subscription; changes to the scan topic, odometry frame, or transform tolerance
still rebuild the filter. Recovery-rate changes update likelihood averaging
rates in place and restart only those averages. They preserve the particle
cloud, odometry reference and pose; other particle-filter settings retain the
upstream reinitialization path. Invalid recovery rates are rejected before any
update. Model scoring, map and motion safeguards are unchanged.

The laser subscription retains only one pending callback. Its TF filter has
eight waiting slots: K1 measurements showed odometry TF arriving up to 100 ms
behind 83 ms laser periods, so a one-slot TF queue could evict every pending
scan. Filter work rejects scans older than 500 ms. Slow model work therefore
cannot accumulate expired measurements or publish their transforms. Odometry
deltas cover the full interval between processed scans; particle counts, beam
count, sensor model, noise, movement thresholds and TF tolerance are unchanged.

Build on K1 with `bash build_amcl.sh /data/RVClaw/build/k1/amcl`. The result is a
replacement `libamcl_core.so`, linked against the matching installed ROS
libraries. From the repository root, install it with
`install -D -m 755 build/k1/amcl/libamcl_core.so runtime/k1/native/amcl/libamcl_core.so`;
the persistent navigation launcher adds that directory
after sourcing ROS. System ROS files are not overwritten.

Rollback restores the previous launcher/library from the deployment backup
while the robot is stopped, then restarts the persistent navigation service.
If no replacement library is present, the launcher uses the installed ROS one.
