#!/usr/bin/env python3
"""Recover the verified vendor wake process once per host/USB connection."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path('/data/RVClaw')
STAGE = ROOT / 'runtime/k1/voice-module'
STATE = ROOT / 'runtime/k1/resource-scheduler/voice-module.json'


def instance():
    device = Path('/dev/rvclaw-m260c-control').resolve(strict=True).name
    port = (Path('/sys/class/tty') / device / 'device').resolve(strict=True)
    for parent in (port, *port.parents):
        if (parent / 'idVendor').is_file():
            identity = [str(parent)]
            for field in ('idVendor', 'idProduct', 'serial', 'busnum', 'devnum'):
                path = parent / field
                identity.append(path.read_text().strip() if path.exists() else '')
            identity.append(Path('/proc/sys/kernel/random/boot_id').read_text().strip())
            return hashlib.sha256('\n'.join(identity).encode()).hexdigest()
    raise RuntimeError('USB identity unavailable')


def main():
    # Diagnostic permission may be unavailable after unplugging. This optional
    # repair must never prevent the normal serial/audio service from starting.
    try:
        identity = instance()
        try:
            saved = json.loads(STATE.read_text())
        except (OSError, ValueError):
            saved = {}
        if saved.get('instance') == identity and saved.get('status') == 'verified':
            print('voice_module recovery already verified for this USB connection', flush=True)
            return
        adb = os.getenv('RVCLAW_ADB') or shutil.which('adb')
        if not adb:
            raise RuntimeError('install the K1 system adb package for optional module recovery')
        STAGE.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ,
                   ADB_SERVER_SOCKET='localfilesystem:' + str(STAGE / 'adb-server.sock'),
                   ANDROID_USER_HOME=str(STAGE / 'adb-home'))
        listing = subprocess.run([str(adb), 'devices'], env=env, capture_output=True,
                                 text=True, check=True, timeout=4)
        devices = [line.split()[0] for line in listing.stdout.splitlines()
                   if len(line.split()) == 2 and line.split()[1] == 'device']
        if len(devices) != 1:
            raise RuntimeError('one authorized diagnostic device is required')
        # Do not change firmware, wake threshold, beam configuration or gain.
        # The vendor watchdog restarts only the exact verified executable.
        script = r'''
set -eu
test -n "$(pidof watch)"
before=$(sha256sum /data/build/user_cfg.ini)
candidate=
for entry in /proc/[0-9]*/exe; do
  if test "$(readlink "$entry" 2>/dev/null || true)" = /data/build/demo_vtn; then
    test -z "$candidate"
    candidate=${entry#/proc/}
    candidate=${candidate%/exe}
  fi
done
test -n "$candidate"
kill -TERM "$candidate"
sleep 5
new=
for entry in /proc/[0-9]*/exe; do
  if test "$(readlink "$entry" 2>/dev/null || true)" = /data/build/demo_vtn; then
    new=${entry#/proc/}
    new=${new%/exe}
  fi
done
test -n "$new"
test "$new" != "$candidate"
test "$before" = "$(sha256sum /data/build/user_cfg.ini)"
grep -q 'start load library success' /data/build/demo.log
grep -q 'open record success' /data/build/demo.log
echo vendor_wake_and_capture_reloaded
'''
        result = subprocess.run([str(adb), '-s', devices[0], 'shell', script], env=env,
                                capture_output=True, text=True, timeout=10, check=True)
        if 'vendor_wake_and_capture_reloaded' not in result.stdout:
            raise RuntimeError('module recovery did not confirm wake and capture')
        STATE.parent.mkdir(parents=True, exist_ok=True)
        temporary = STATE.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(instance=identity, status='verified',
                                             timestamp=time.time())))
        os.replace(temporary, STATE)
        print('voice_module verified wake/capture recovery; acoustic settings preserved', flush=True)
    except Exception as error:
        print('voice_module optional recovery unavailable: ' + str(error), flush=True)


if __name__ == '__main__':
    main()
