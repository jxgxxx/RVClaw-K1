#!/usr/bin/env python3
"""Reuse exact ROS setup exports while their inputs and hooks are unchanged."""
import argparse,fcntl,hashlib,json,os,shlex,subprocess,tempfile
from pathlib import Path

INPUTS=('PATH','PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH',
        'CMAKE_PREFIX_PATH','PKG_CONFIG_PATH','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION',
        'LUA_PATH','LUA_CPATH','COLCON_CURRENT_PREFIX','AMENT_CURRENT_PREFIX')

def guard(prefixes):
    result=[];seen=set()
    for prefix in prefixes:
        # Directory metadata also detects newly added/removed package hooks.
        for directory,dirs,files in os.walk(prefix,followlinks=True):
            resolved=Path(directory).resolve()
            if resolved in seen:
                dirs[:]=[];continue
            seen.add(resolved)
            dirs[:]=sorted(d for d in dirs if d not in ('__pycache__','lib','bin','include'))
            path=Path(directory);s=path.stat();result.append((str(path),s.st_mtime_ns))
            for name in sorted(files):
                if name.endswith(('.sh','.bash','.dsv')) or name.startswith('_local_setup_util_'):
                    p=path/name;s=p.stat();result.append((str(p),s.st_mtime_ns,s.st_size))
    return result

def exports(root,base_root,cache=None):
    install=root/'ros2_ws/install'
    base_install=base_root/'ros2_ws/install'
    scripts=[Path('/opt/ros/humble/setup.bash')]
    prefixes=[Path('/opt/ros/humble')]
    if base_install.resolve()!=install.resolve() and (base_install/'setup.bash').is_file():
        scripts.append(base_install/'local_setup.bash');prefixes.append(base_install)
    scripts.append(install/'local_setup.bash');prefixes.append(install)
    baseline=dict(os.environ)
    key=dict(version=1,scripts=list(map(str,scripts)),hooks=guard(prefixes),
             environment={k:baseline.get(k) for k in INPUTS})
    stamp=hashlib.sha256(json.dumps(key,sort_keys=True).encode()).hexdigest()
    cache=cache or root/'cache/ros-environment.json';cache.parent.mkdir(parents=True,exist_ok=True)
    with cache.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        try:
            saved=json.loads(cache.read_text())
            if saved['stamp']==stamp:return saved['exports']
        except (OSError,ValueError,KeyError):pass
        command='set -e; '+''.join('source '+shlex.quote(str(p))+' >/dev/null; ' for p in scripts)+'env -0'
        completed=subprocess.run(['bash','-c',command],capture_output=True,check=True,timeout=30,env=baseline)
        after=dict(item.decode().split('=',1) for item in completed.stdout.split(b'\0') if b'=' in item)
        ignored={'_','SHLVL','PWD','OLDPWD'}
        changed={k:v for k,v in after.items() if k not in ignored and baseline.get(k)!=v}
        removed=[k for k in baseline if k not in ignored and k not in after]
        output='\n'.join(['unset '+shlex.quote(k) for k in removed]
            +['export '+k+'='+shlex.quote(v) for k,v in sorted(changed.items())])
        fd,name=tempfile.mkstemp(prefix='.ros-env-',dir=cache.parent)
        try:
            with os.fdopen(fd,'w') as stream:json.dump(dict(stamp=stamp,exports=output),stream)
            os.replace(name,cache)
        finally:Path(name).unlink(missing_ok=True)
        return output

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--base-root',type=Path,required=True);args=parser.parse_args()
    print(exports(args.root,args.base_root))

if __name__=='__main__':main()
