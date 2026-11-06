"""Passive, bounded observation of the existing M260C voice service.

No serial opens, recording, inference, speech events, or robot commands.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

SERVICE='rvclaw-vendor-command.service'
ADVICE='检查语音模块供电、控制线和录音连接；连接后等待语音服务就绪。'


def verdict(facts):
    """Do not mistake a running process or an unrelated microphone for ready."""
    parts=[
        {'id':'serial','label':'控制串口','state':'ok' if facts['serial_present'] else 'error','status':'已连接' if facts['serial_present'] else '未连接'},
        {'id':'capture','label':'语音录音','state':'ok' if facts['capture_present'] else 'error','status':'已连接' if facts['capture_present'] else '未连接'},
        {'id':'service','label':'语音服务','state':'ok' if facts.get('service_running') else 'unknown' if facts.get('service_known') is False else 'error','status':'运行中' if facts.get('service_running') else '待确认' if facts.get('service_known') is False else '未就绪'},
    ]
    if not facts['serial_present'] or not facts['capture_present']:
        missing=[part['label'] for part in parts[:2] if part['state']=='error']
        state,status,reason='error','未连接','未检测到'+ '、'.join(missing)+'。'
    elif facts.get('service_known') is False:
        state,status,reason='unknown','待确认','暂时无法读取语音服务状态。'
    elif not facts.get('service_running'):
        state,status,reason='error','异常','语音设备已连接，但语音服务尚未运行。'
    elif facts.get('serial_owned') and facts.get('capture_running'):
        state,status,reason='ok','正常','控制串口已连接，录音采集正常，语音服务运行中。'
    elif facts.get('starting'):
        state,status,reason='unknown','准备中','语音服务正在准备控制串口与录音采集。'
    else:
        problems=[]
        if not facts.get('serial_owned'):problems.append('语音控制链路未就绪')
        if not facts.get('capture_running'):problems.append('语音录音未运行')
        state,status,reason='error','异常','；'.join(problems)+'。'
    if facts['capture_present']:
        parts[1].update(state='ok' if facts.get('capture_running') else 'unknown',status='采集中' if facts.get('capture_running') else '未采集')
    if facts['serial_present'] and facts.get('serial_owned'):
        parts[0]['status']='已就绪'
    return {'id':'voice','label':'语音模块','state':state,'status':status,
            'reason':reason,'advice':'' if state=='ok' else ADVICE,
            'source':'语音控制与录音链路','components':parts}


def service_state():
    try:
        result=subprocess.run(['systemctl','--user','show',SERVICE,'-p','ActiveState','-p','SubState','-p','MainPID','-p','ControlGroup','-p','ExecMainStartTimestampMonotonic'],capture_output=True,text=True,timeout=1.5,check=True)
        return dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    except (OSError,subprocess.SubprocessError):return None


def process_ids(state,cgroup_root):
    if not state:return set()
    values={int(state.get('MainPID') or 0)}-{0}
    group=state.get('ControlGroup','').lstrip('/')
    path=(cgroup_root/group/'cgroup.procs').resolve()
    if group and path.is_relative_to(cgroup_root.resolve()):
        try:values.update(int(row) for row in path.read_text().split() if row.isdigit())
        except OSError:pass
    return values


def sample(port,card,proc=Path('/proc'),cgroup=Path('/sys/fs/cgroup'),now=None):
    state=service_state();pids=process_ids(state,cgroup)
    facts={'serial_present':port.exists(),'capture_present':False,
           'service_known':state is not None,
           'service_running':bool(state and state.get('ActiveState')=='active' and state.get('SubState')=='running' and pids),
           'serial_owned':False,'capture_running':False,'starting':False}
    if state:
        try:
            uptime=float((proc/'uptime').read_text().split()[0])
            started=float(state.get('ExecMainStartTimestampMonotonic') or 0)/1e6
            facts['starting']=started>0 and 0<=uptime-started<30
        except (OSError,ValueError):pass
    for folder in (proc/'asound').glob('card[0-9]*'):
        try:
            if (folder/'id').read_text().strip()!=card:continue
            # Require a capture PCM for the configured card, not an unrelated
            # webcam microphone or a card offering playback only.
            captures=list(folder.glob('pcm*c/sub*/status'))
            facts['capture_present']=bool(captures)
            for path in captures:
                text=path.read_text();owner=re.search(r'owner_pid\s*:\s*(\d+)',text)
                if re.search(r'^state:\s*RUNNING\s*$',text,re.M) and owner and int(owner[1]) in pids:
                    facts['capture_running']=True
        except OSError:continue
    if facts['serial_present']:
        try:
            expected=port.stat()
            for pid in pids:
                for fd in (proc/str(pid)/'fd').glob('*'):
                    try:
                        actual=fd.stat()
                        if (actual.st_dev,actual.st_ino,getattr(actual,'st_rdev',0))==(expected.st_dev,expected.st_ino,getattr(expected,'st_rdev',0)):facts['serial_owned']=True
                    except OSError:pass
        except OSError:facts['serial_present']=False
    return {'version':1,'sampled_at':time.time() if now is None else now,'voice':verdict(facts)}


def publish(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name('.'+path.name+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False),encoding='utf-8')
    os.replace(temporary,path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/data/RVClaw/studio/web/voice-health.json'))
    parser.add_argument('--port-device',type=Path,default=Path('/dev/rvclaw-m260c-control'))
    parser.add_argument('--capture-card',default='XFMDPV0018')
    parser.add_argument('--once',action='store_true')
    args=parser.parse_args();stop=threading.Event()
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:stop.set())
    while not stop.is_set():
        try:result=sample(args.port_device,args.capture_card)
        except Exception:
            result={'version':1,'sampled_at':time.time(),'voice':{'id':'voice','label':'语音模块','state':'unknown','status':'待确认','reason':'语音监测暂时不可用，请稍后刷新。','advice':'检查语音监测服务。','components':[]}}
        publish(args.output,result)
        if args.once:break
        stop.wait(3)


if __name__=='__main__':main()
