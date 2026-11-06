# Studio uploaded-image recognition on K1

`POST /api/meter-image-read` keeps the existing request and response fields.
The uploaded image is processed by the existing local reader first. A finite,
validated local result returns immediately. A rejected or unavailable local
result can use one bounded cloud request through the robot's existing
`services/meter_gauge/run_cloud_meter.py` and board-local configuration.

This is a single-photo workflow. It does not claim the independent multi-frame
consensus required by a physical inspection task. It does not control cameras,
navigation, alarms or motion. Active robot tasks reject optional website reads.

Cloud assistance has a 45-second socket timeout and a 55-second wall-clock limit.
These are maximum waits, not delays applied to successful responses. The website
budget is independent of the shorter physical inspection deadlines. One cloud
request is made per upload; no automatic duplicate requests or cached readings
are used. User-facing failures distinguish timeout, task preemption, provider
authorization, rate limits, and connectivity failures without exposing secrets.
Only one website read runs at a time. A newly active robot task stops the cloud
worker. The worker loads no local inference model and starts no persistent service.
Credentials and provider addresses remain on the board; request payloads cannot
select a provider, credential, command, input file or configuration file.

For assisted results, saved evidence adds:

- `recognition_policy`: `local_then_cloud`.
- `inference_source`: `cloud` when an accepted cloud candidate supplies the
  result, or `local` when cloud assistance cannot recover the local failure.
- `local_result`: bounded original status/value/confidence/model/reason fields.
- `cloud_assistance`: attempted/status/value/confidence/model/reason/elapsed_ms.
- `supersedes_evidence_id`: the intermediate local evidence ID, if any.

Cloud candidates require an accepted status, a finite numeric value and at least
70% confidence. Failure never supplies a guessed value. The final result uses
the original uploaded photo and has no instrument unit. Intermediate evidence
is retained for diagnosis and replaced by the final record in user lists.

No-image requests still fail validation before recognition or evidence creation.
Existing frontend fields, routes, and inspection configuration remain compatible.

## Local uploaded-photo preprocessing

Digital uploads are decoded before local recognition. Existing JPEG bytes are
preserved; other formats are normalized to JPEG quality 92, matching the image
format used by the camera pipeline and avoiding repeated lossy compression. Evidence
retains the original uploaded bytes; no cached reading supplies a result.
An image with a substantial bright saturated display row uses the full-photo
reader first, so a premature registered crop cannot remove display context.
This cheap colour test only orders readers; it never supplies a numeric value
or lowers any OCR validation gate. Other digital photos retain registration
first, including the existing reflective LCD path. A rejected first pass can
try the other existing local path once, provided no robot task has begun.
Pointer recognition keeps its existing local path unchanged.

Final local digital evidence additionally records `inference_source: local`,
`local_preprocessing: jpeg92` (or `jpeg_original`), and bounded `local_attempts` diagnostics. The
existing `supersedes_evidence_id` is retained; `supersedes_evidence_ids` can
additionally list intermediate IDs from both local paths. History excludes
all superseded IDs, leaving one visible final record per upload. Separate
same-photo crop/path checks do not claim independent camera-frame consensus.
