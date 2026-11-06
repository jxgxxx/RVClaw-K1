# Voice monitoring contract

`GET /voice-health.json` returns a small, uncached, board-local snapshot.
Its producer samples every three seconds and only reads systemd state,
the configured serial device metadata, process descriptors and ALSA status.
It never opens the serial device, records audio, runs ASR, sends commands,
or restarts the voice service. Idle silence is a valid healthy state.

The response contains `version=1`, numeric Unix `sampled_at`, and one `voice`
hardware row with `id=voice`, `label=语音模块`, `state=ok|error|unknown`,
Chinese `status`, `reason`, `advice`, and `components` for serial, capture,
and service readiness. No transcripts, raw commands, device paths, process
IDs or environment variables are exposed to browsers.

Normal requires the configured control device and capture card, an active
voice service owning the current serial descriptor, and a running capture
owned by that service's cgroup. Startup may remain unknown for 30 seconds.
Unplugged hardware has the red `error` state, independently on each board.
Studio consistently displays every voice error badge as `异常`; the raw
snapshot may retain `未连接`, and the reason explains the actual missing
device or service problem. Healthy and unknown statuses remain distinct. A failed
sampler or a sample older than ten seconds is `待确认`, never cached normal.

Studio polls the voice snapshot along with the existing device snapshot.
The voice row joins the device information, hardware table, diagnosis and
overview warning without changing the existing robot monitoring payload.
