#!/usr/bin/env python3
"""Verify and import the supplied K1 resources; archives cannot escape their declared roots."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import tarfile
import tempfile


def scan(archive, rows, destination, write):
    expected={row['path']:row for row in rows};seen=set()
    with tarfile.open(archive,'r:gz') as bundle:
        for member in bundle:
            if not member.isfile() or member.name not in expected or member.name in seen:
                raise ValueError('unexpected or duplicate archive member: '+member.name)
            relative=PurePosixPath(member.name)
            if relative.is_absolute() or '..' in relative.parts:raise ValueError('unsafe member path')
            row=expected[member.name]
            if member.size!=row['bytes']:raise ValueError('asset size differs: '+member.name)
            target=destination(member.name).resolve()
            stream=bundle.extractfile(member);digest=hashlib.sha256()
            temporary=None
            if write:
                target.parent.mkdir(parents=True,exist_ok=True)
                fd,temporary=tempfile.mkstemp(prefix='.asset-',dir=target.parent)
                output=os.fdopen(fd,'wb')
            try:
                for chunk in iter(lambda:stream.read(1024*1024),b''):
                    digest.update(chunk)
                    if write:output.write(chunk)
                if write:output.close()
                if digest.hexdigest()!=row['sha256']:raise ValueError('asset hash differs: '+member.name)
                if write:
                    if target.exists():
                        if hashlib.sha256(target.read_bytes()).hexdigest()!=row['sha256']:
                            raise RuntimeError('existing asset differs; refusing overwrite: '+str(target))
                    else:os.replace(temporary,target)
            finally:
                if write and not output.closed:output.close()
                if temporary:Path(temporary).unlink(missing_ok=True)
            seen.add(member.name)
    if seen!=expected.keys():raise ValueError('archive has missing declared assets')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--portable-archive',type=Path,required=True)
    parser.add_argument('--vision-archive',type=Path,required=True)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--vision-cache',type=Path,default=Path.home()/'.cache/models/vision')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args();root=args.root.resolve();cache=args.vision_cache.resolve()
    manifest=json.loads((root/'models/k1-assets.json').read_text(encoding='utf-8'))
    def portable(name):
        p=(root/name).resolve()
        if not p.is_relative_to(root):raise ValueError('asset escapes project')
        return p
    def vision(name):
        p=(cache/PurePosixPath(name).relative_to('vision-model-cache')).resolve()
        if not p.is_relative_to(cache):raise ValueError('asset escapes vision cache')
        return p
    groups=[(args.portable_archive,manifest['portable'],portable),
            (args.vision_archive,manifest['vision_cache'],vision)]
    for group in groups:scan(*group,write=False)
    if not args.check_only:
        for group in groups:scan(*group,write=True)
    print(json.dumps(dict(status='PASS',assets=sum(len(g[1]) for g in groups),check_only=args.check_only)))


if __name__=='__main__':main()
