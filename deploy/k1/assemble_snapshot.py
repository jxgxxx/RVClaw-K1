#!/usr/bin/env python3
"""Assemble the captured K1 sources and configuration without starting services."""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def child(root, relative):
    destination=(root/relative).resolve()
    if not destination.is_relative_to(root.resolve()):
        raise ValueError('path escapes project: '+relative)
    return destination


def render(text, root, address):
    return (text.replace('@ROBOT_ROOT@',str(root))
            .replace('@USER_HOME@',str(Path.home()))
            .replace('@BOARD_ADDRESS@',address))


def copy_file(source,destination,mode=0o644):
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source,destination);destination.chmod(mode)


def workspace_link(source,destination):
    destination.parent.mkdir(parents=True,exist_ok=True)
    try:destination.symlink_to(os.path.relpath(source,destination.parent),target_is_directory=True)
    except OSError:
        if os.name!='nt':raise
        shutil.copytree(source,destination)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--bind-address',default='127.0.0.2')
    parser.add_argument('--source-only',action='store_true',help='skip model checks; not a runnable deployment')
    parser.add_argument('--vision-cache',type=Path,default=Path.home()/'.cache/models/vision')
    args=parser.parse_args();root=args.root.resolve();snapshot=root/'deploy/k1/snapshot'
    address=ipaddress.IPv4Address(args.bind_address)
    if address.is_unspecified or str(address)=='127.0.0.1':
        raise ValueError('use a dedicated local IPv4 address; camera upstream owns 127.0.0.1:8017')
    runtime=root/'runtime/k1'
    if runtime.exists():
        raise RuntimeError('fresh checkout required; existing runtimes are never overwritten')
    if not args.source_only:
        subprocess.run([sys.executable,str(root/'scripts/check_delivery.py'),'--root',str(root),
                        '--require-assets','--vision-cache',str(args.vision_cache)],check=True)
    layout=json.loads((snapshot/'source-layout.json').read_text(encoding='utf-8'))
    for row in layout['files']:
        p=child(root,row['source'])
        if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:
            raise RuntimeError('snapshot source differs: '+row['source'])
    common=json.loads((root/'deploy/k1/runtime_layout.json').read_text(encoding='utf-8'))
    for group in ('scripts','vision','contracts','algorithms'):
        for row in common[group]:
            if not child(root,row['source']).is_file():
                raise RuntimeError('missing assembly source: '+row['source'])
    runtime.mkdir(parents=True)
    # Profile workspaces are generated from common sources plus the captured overrides.
    for profile in ('navigation','mapping'):
        source=root/'robot/profiles'/profile/'overrides'
        destination=root/'robot/profiles'/profile/'ros2_ws/src'
        shutil.copytree(root/'robot/ros2_ws/src/rvclaw_nav2_bringup',
                        destination/'rvclaw_nav2_bringup')
        shutil.copytree(source,destination,dirs_exist_ok=True)
    for profile in ('base','mission'):
        workspace_link(root/'robot/ros2_ws',runtime/profile/'ros2_ws')
    for profile in ('navigation','mapping'):
        workspace_link(root/'robot/profiles'/profile/'ros2_ws',runtime/profile/'ros2_ws')
    for row in common['scripts']:
        copy_file(child(root,row['source']),child(runtime/row['profile'],row['destination']),row.get('mode',0o644))
    vision=runtime/'vision'
    for group,base in (('vision',vision/'services/rvclaw_api'),('contracts',vision),
                       ('algorithms',vision/'robot/ros2_ws/src/rvclaw_algo')):
        for row in common[group]:
            copy_file(child(root,row['source']),child(base,row['destination']),row.get('mode',0o644))
    for source,destination in (('configs/device/meter','configs/meter'),
                               ('configs/device/vision','configs/vision'),
                               ('configs/thresholds','configs/thresholds')):
        shutil.copytree(root/source,vision/destination)
    for row in layout['files']:
        source=child(root,row['source']);destination=child(runtime,row['target'])
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,destination);destination.chmod(row['mode'])
    for profile in ('base','navigation','mission','mapping'):
        link=runtime/profile/'env'
        workspace_link(root/'deploy/k1/env',link)
        (runtime/profile/'state').mkdir(parents=True,exist_ok=True)
        (runtime/profile/'runs').mkdir(parents=True,exist_ok=True)
    for path in (snapshot/'environment').glob('*.env'):
        destination=runtime/'environment'/path.name
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_text(render(path.read_text(encoding='utf-8'),root,args.bind_address),encoding='utf-8')
    studio=root/'deploy/k1/run_studio.sh'
    studio.write_text(render((snapshot/'run_studio.sh').read_text(encoding='utf-8'),root,args.bind_address),encoding='utf-8')
    studio.chmod(0o755)
    state=root/'state/inspection-profile.json'
    if state.exists():raise RuntimeError('existing map selection must not be overwritten')
    state.parent.mkdir(parents=True,exist_ok=True)
    selected=(snapshot/'inspection-profile.json').read_text(encoding='utf-8').replace('/data/RVClaw',str(root))
    state.write_text(selected,encoding='utf-8')
    for directory in ('runtime/waypoint-arrivals','runtime/inspection-progress','runtime/inspection-control',
                      'runtime/k1/secrets','runtime/k1/scenes/inspection-k1-reviewed'):
        child(root,directory).mkdir(parents=True,exist_ok=True)
    (runtime/'secrets').chmod(0o700)
    (runtime/'environment/instance.json').write_text(json.dumps(dict(bind_address=str(address),
        source_only=args.source_only,root=str(root)),indent=2),encoding='utf-8')
    print(json.dumps(dict(status='PASS',source_only=args.source_only,services_started=False,
                          motion_sent=False,root=str(root))))


if __name__=='__main__':main()
