# Studio device health

`GET /api/device/snapshot` retains the existing `health` fields and adds the
optional health state `idle`. Individual hardware rows may also have state
`idle`, status `待机`, and a reason explaining that USB is connected while
acquisition is asleep. This confirms connection availability only, not live
sensor accuracy or readiness to move.

Idle requires a fresh scheduler report with no task leases, no motion
authorization, an intentional sleep/idle state, and no process in the managed
navigation cgroup. Device existence and current-account read/write access are
checked without opening the serial ports. Missing devices or denied access
remain errors. A stale scheduler report never authorizes idle classification.

While navigation runs, existing live-data, localization, diagnostic and
protective-latch rules remain in effect. Camera, voice, resource and monitoring
faults are never suppressed by navigation idle. The browser preserves idle
labels through voice-health aggregation and displays a standby summary.
No navigation process, hardware driver, command publisher or service client
is created by this status check.
