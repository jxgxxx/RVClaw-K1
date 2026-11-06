# Studio portable deployment contract

The Vue application uses the existing Studio JSON shapes. Map IDs refer to the
declared coordinate frame in `waypoints.yaml`; a deployment registry maps IDs to
directories. Directory names and machine IP addresses are not map identities.

* Website: TCP 8017 on the board's Tailscale IPv4, static assets and JSON APIs.
* Existing K1 vision: TCP 8017 on loopback, unchanged; only an explicit allowlist
  is forwarded. Uploaded images use this local model service first; rejected reads may use the bounded cloud assistance described in `studio-upload-recognition.md`.
* Inspection control: TCP 8019 on Tailscale; existing start/cancel contract,
  `map_id`, `task_id`, `can_start`, `can_cancel`, exact Origin validation.
* Coordinates, headings, meter type/range/name and return point are written to
  registered robot scene files transactionally. Active tasks prohibit changes.
* Saved selection is read by both mission and navigation startup wrappers.
  Saving never moves the robot or reloads hardware. A requested new-map mission
  must confirm stationary/disarmed state before an idle navigation map reload.
* Progress uses the board's real task database, arrival events, and traces.
  A failed task with no photos remains visible and never inherits old values.
  Large minified and pretty JSON traces have the same root metadata semantics.
  Explicit return failure uses `failure_phase=return`; completed waypoint
  steps remain done and the failed ORIGIN step is red, with Chinese feedback.
* Health: passive ROS subscriptions and cached /proc measurements. Missing or
  stale hardware is unknown/error, never a fabricated healthy state.
* Browser TXT and Excel use the same full record formatter. Historical device
  measurements are explicitly absent when no historical snapshot was saved.
* No robot commands or API keys are returned in user log descriptions. Secret
  files and environment files are excluded from the portable source archive.
* Web uploads use local models first, with one bounded cloud request after a rejected read. Registered known instruments are measured
  from current uploaded pixels; reference images provide static geometry only.
  An unverified local pointer result remains a local failure; any accepted cloud
  result is stored with its separate source and never described as locally verified. Optional uploads are rejected while the task database or progress
  reports an active inspection, including the gap before progress is published.
