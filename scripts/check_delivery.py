#!/usr/bin/env python3
"""Check a source delivery without importing robot code or accessing hardware."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--require-assets',action='store_true')
    parser.add_argument('--vision-cache',type=Path,default=Path.home()/'.cache/models/vision')
    args=parser.parse_args();root=args.root.resolve();errors=[];python_count=0
    for folder in ('services','deploy','scripts','robot/ros2_ws/src','robot/profiles'):
        for path in (root/folder).rglob('*.py'):
            if any(part in {'__pycache__','build','install','log'} for part in path.parts):continue
            try:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
            except (SyntaxError,UnicodeError) as error:errors.append(str(error))
            python_count+=1
    layout=json.loads((root/'deploy/k1/snapshot/source-layout.json').read_text(encoding='utf-8'))
    for row in layout['files']:
        path=(root/row['source']).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            errors.append('missing/outside snapshot source: '+row['source']);continue
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
            errors.append('snapshot hash differs: '+row['source'])
    common=json.loads((root/'deploy/k1/runtime_layout.json').read_text(encoding='utf-8'))
    for group in ('scripts','vision','contracts','algorithms'):
        for row in common[group]:
            if not (root/row['source']).is_file():errors.append('missing assembly source: '+row['source'])
    for folder in ('services','deploy','scripts','studio/web-vue'):
        for path in (root/folder).rglob('*'):
            if not path.is_file() or 'node_modules' in path.parts:continue
            if path.name.startswith(('test_','test-')) or 'verification' in path.parts:
                errors.append('temporary test source in delivery: '+str(path.relative_to(root)))
    if args.require_assets:
        manifest=json.loads((root/'models/k1-assets.json').read_text(encoding='utf-8'))
        for rows,base,prefix in ((manifest['portable'],root,''),
                (manifest['vision_cache'],args.vision_cache,'vision-model-cache/')):
            for row in rows:
                path=base/row['path'].removeprefix(prefix)
                if not path.is_file() or path.stat().st_size!=row['bytes']:
                    errors.append('missing/incompatible resource: '+row['path']);continue
                if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
                    errors.append('resource hash differs: '+row['path'])
    print(json.dumps(dict(status='FAIL' if errors else 'PASS',python_files=python_count,
                          snapshot_files=len(layout['files']),require_assets=args.require_assets,
                          errors=errors),ensure_ascii=False,indent=2))
    return bool(errors)


if __name__=='__main__':sys.exit(main())
