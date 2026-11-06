# RGB assistance

K1 uses the existing Astra RGB process; the worker never opens USB/video devices.

- Target framing: same-image registered display geometry is appended to `trial.rgb_assist`; image side, size and clipping are available without another inference.
- Landmark review: bounded SIFT matches the fixed bodies of **all** configured instruments. Digits and needle are masked. Repeated observations can confirm appearance identity. Ambiguous, blurred, occluded and unsupported matches remain unknown.
- Motion consistency: sparse optical flow and bracketed odometry timestamps identify repeated gross disagreement. Camera timestamp is host JPEG receive time, not calibrated exposure time. Scene motion can mimic chassis movement; translation flow cannot determine metres or prove wheel slip.

All outputs are advisory. No `cmd_vel`, arming, map/initial pose or navigation goal publisher exists. No OCR reading, threshold, voice setting or arrival condition is changed. Instrument appearance alone is not a measured absolute robot pose. Autonomous image-based steering requires validated camera intrinsics/extrinsics and field validation; it is not enabled here.

The worker samples at up to 1 Hz, matches landmarks at most once per 5 seconds, skips camera fetch during OCR, uses one OpenCV thread, and is constrained to 30% of one CPU on cores 4–6. No backlog is queued. Latest JSON is under `$XDG_RUNTIME_DIR/rvclaw-rgb-assist`; bounded rotating JSONL is persisted under `runtime/k1/diagnostics/rgb-assist`. `/rvclaw/perception/rgb_assist` contains JSON `std_msgs/String`; local HTTP `/api/rgb-assist/status` exposes fresh status. The existing inspection evidence carries the synchronous target check.

Local fixed-body landmark crops under `configs/perception/rgb_landmarks` come from previously registered field photographs. The mutable readout is erased in the stored image. Common descriptors across different instruments are removed, and distant box/floor texture is excluded. Crops provide appearance identity only; no parking position is learned from them, and their original readings are never used. Repeated or insufficient texture remains unknown. Camera/scene changes require revalidation of these references.

Engineering entrypoints: `deploy/k1/run_rgb_assist.sh`, `deploy/k1/systemd-user/rvclaw-rgb-assist.service`. Disable with `systemctl --user disable --now rvclaw-rgb-assist.service`; the camera/voice/navigation pipelines continue normally. Camera API integration is isolated in `deploy/k1/camera_api_entry.py`, keeping Studio source unchanged.
