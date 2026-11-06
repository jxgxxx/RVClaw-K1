#!/usr/bin/env python3
"""Install captured systemd units on a new K1; never enable or start services."""
import argparse
import ipaddress
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind-address',required=True,help='a local LAN/Tailscale IPv4 address of this K1')
    parser.add_argument('--install',action='store_true',help='without this flag only validate')
    args=parser.parse_args();address=ipaddress.IPv4Address(args.bind_address)
    if address.is_unspecified or str(address)=='127.0.0.1':
        raise ValueError('use a dedicated address: the camera upstream already owns 127.0.0.1:8017')
    root=Path(__file__).resolve().parents[2]
    if root!=Path('/data/RVClaw'):
        raise RuntimeError('the captured K1 launch paths require /data/RVClaw')
    instance=json.loads((root/'runtime/k1/environment/instance.json').read_text(encoding='utf-8'))
    if instance['source_only'] or instance['bind_address']!=str(address):
        raise RuntimeError('complete resource assembly with this same bind address before installing')
    source=root/'deploy/k1/snapshot/systemd-user';units=Path.home()/'.config/systemd/user'
    files=sorted(p for p in source.rglob('*') if p.is_file())
    for name in sorted({p.relative_to(source).parts[0] for p in files}):
        if (units/name).exists() or (units/name).is_symlink():
            raise RuntimeError('refusing to overwrite an existing unit/drop-in: '+name)
    for relative in ('runtime/k1/environment/rvclaw-vendor-command.service.env',
                     'runtime/k1/navigation/ros2_ws/install/setup.bash',
                     'build/yolov8_mjpeg','build/meter-paddleocr/meter_paddleocr',
                     'studio/web/index.html'):
        if not (root/relative).is_file():raise RuntimeError('missing build/configuration: '+relative)
    if not args.install:
        print('Preflight passed; no files written or services started.');return
    for p in files:
        destination=units/p.relative_to(source);destination.parent.mkdir(parents=True,exist_ok=True)
        text=(p.read_text(encoding='utf-8').replace('@ROBOT_ROOT@',str(root))
              .replace('@USER_HOME@',str(Path.home())).replace('@BOARD_ADDRESS@',str(address)))
        destination.write_text(text,encoding='utf-8');destination.chmod(0o644)
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    print('Units installed. No service was enabled or started; no motion command was sent.')


if __name__=='__main__':main()
