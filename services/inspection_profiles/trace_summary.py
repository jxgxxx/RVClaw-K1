"""Read bounded UI metadata from large, pretty-printed robot traces.

Robot diagnostics can exceed 70 MB. Keep the original file intact; scan its
small metadata fields in chunks instead of materializing every diagnostic.
Small JSON documents retain normal, complete JSON parsing.
"""
import json,re,os,hashlib,tempfile,threading
from pathlib import Path

SMALL_LIMIT=512*1024
ROOT_KEYS='completed_waypoints|route_requested|duration_s|status|error|map_file|evidence_session|resume_plan'
PATTERN=re.compile(r'"(?P<key>'+ROOT_KEYS+r'|yaml_path)"\s*:')
STRUCTURAL=re.compile(rb'["{}\[\]\\]')
ROOT_PATTERN=re.compile(('"(?P<key>'+ROOT_KEYS+r'|final_runtime_state)"\s*:').encode('ascii'))
DECODER=json.JSONDecoder()
LOCK=threading.RLock()

def compact(value):
    if not isinstance(value,dict):return {}
    result={k:value[k] for k in ROOT_KEYS.split('|')+['final_runtime_state'] if k in value}
    refs=[];seen=set()
    def visit(item):
        if isinstance(item,dict):
            for key,v in item.items():
                if key in {'map_file','yaml_path','evidence_session'} and isinstance(v,str):
                    pair=('map_file' if key=='yaml_path' else key,v)
                    if pair not in seen and len(seen)<256:seen.add(pair);refs.append({pair[0]:v})
                elif isinstance(v,(dict,list)):visit(v)
        elif isinstance(item,list):
            for v in item:visit(v)
    visit(value);result['_references']=refs
    if not result.get('map_file'):
        source=next((r['map_file'] for r in refs if 'map_file' in r),None)
        if source:result['map_file']=source
    return result

def read_trace_summary(path):
    path=Path(path).resolve();stat=path.stat()
    configured=os.getenv('RVCLAW_TRACE_CACHE_ROOT')
    root=Path(configured) if configured else Path('/data/RVClaw/runtime/inspection-progress/trace-cache') if any(path.is_relative_to(Path(p)) for p in ['/data/RVClaw','/data/inspection_photos']) else None
    if root is None:return _read_uncached(path)
    key={'source':str(path),'mtime_ns':stat.st_mtime_ns,'size':stat.st_size,'version':3}
    cache=root/(hashlib.sha256(str(path).encode()).hexdigest()+'.json')
    with LOCK:
        try:
            saved=json.loads(cache.read_text(encoding='utf-8'))
            saved_key=saved.get('key') or {}
            same_source=all(saved_key.get(field)==key[field] for field in ('source','mtime_ns','size'))
            summary=saved.get('summary')
            if same_source and saved_key.get('version') in (3,4) and isinstance(summary,dict):
                # Successful traces do not need the newer recovery code. Keep
                # their existing cache hits and enrich only failed traces.
                if saved_key.get('version')==4 or summary.get('status')!='FAIL' or 'resume_plan' in summary:
                    return summary
        except (OSError,ValueError,KeyError):pass
        result=_read_uncached(path)
        # Version 4 marks summaries that have been checked for route recovery
        # metadata. This avoids invalidating the whole historical cache.
        key['version']=4 if result.get('status')=='FAIL' else 3
        try:
            root.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=root,prefix='.trace-',delete=False) as f:
                json.dump({'key':key,'summary':result},f,ensure_ascii=False);temporary=f.name
            os.replace(temporary,cache)
            files=list(root.glob('*.json'))
            if len(files)>512:
                for old in sorted(files,key=lambda p:p.stat().st_mtime)[:len(files)-512]:old.unlink(missing_ok=True)
        except OSError:pass
        return result

def _root_values(path):
    """Locate small root fields in either pretty or minified JSON, bounded RAM.

    Track strings and container depth instead of treating indentation as JSON
    structure. Escaped quotes/brackets in diagnostic text cannot become fields.
    Values are read separately, so the huge nested diagnostic arrays stay out
    of the Python object graph.
    """
    offsets={};depth=0;quoted=False;escaped_at=-1;offset=0;buffer=b''
    with path.open('rb') as stream:
        while True:
            chunk=stream.read(65536);buffer+=chunk
            limit=max(0,len(buffer)-512) if chunk else len(buffer)
            for token in STRUCTURAL.finditer(buffer,0,limit):
                position=offset+token.start();symbol=buffer[token.start()]
                if quoted:
                    if position==escaped_at:
                        escaped_at=-1;continue
                    if symbol==92:escaped_at=position+1
                    elif symbol==34:quoted=False
                elif symbol==34:
                    if depth==1:
                        match=ROOT_PATTERN.match(buffer,token.start())
                        if match:offsets.setdefault(match.group('key').decode('ascii'),offset+match.end())
                    quoted=True
                elif symbol in (123,91):depth+=1
                elif symbol in (125,93):depth-=1
            offset+=limit;buffer=buffer[limit:]
            if not chunk:break
    result={}
    with path.open('rb') as stream:
        for key,position in offsets.items():
            stream.seek(position)
            value=stream.read(262144).decode('utf-8',errors='ignore').lstrip()
            try:result[key]=DECODER.raw_decode(value)[0]
            except json.JSONDecodeError:pass
    return result

def _read_uncached(path):
    path=Path(path)
    if path.stat().st_size<=SMALL_LIMIT:
        value=json.loads(path.read_text(encoding='utf-8'))
        return compact(value)
    result=_root_values(path);references=[];seen=set();buffer=''
    with path.open(encoding='utf-8') as stream:
        while True:
            chunk=stream.read(65536);buffer+=chunk;start=0;pending=None
            for match in PATTERN.finditer(buffer):
                key=match.group('key')
                root=buffer[buffer.rfind('\n',0,match.start())+1:match.start()]=='  '
                if not root and key not in {'map_file','evidence_session','yaml_path'}:continue
                position=match.end()
                while position<len(buffer) and buffer[position].isspace():position+=1
                try:value,end=DECODER.raw_decode(buffer,position)
                except json.JSONDecodeError:
                    if chunk and len(buffer)-match.start()<262144:pending=match.start();break
                    continue
                if root and key not in result:result[key]=value
                if key in {'map_file','yaml_path','evidence_session'} and isinstance(value,str):
                    pair=('map_file' if key=='yaml_path' else key,value)
                    if pair not in seen and len(seen)<256:seen.add(pair);references.append({pair[0]:value})
                start=end
            if not chunk:break
            # Retain only a possible incomplete value or split metadata key.
            buffer=buffer[pending:] if pending is not None else buffer[max(start,len(buffer)-128):]
    result['_references']=references
    if not result.get('map_file'):
        source=next((r['map_file'] for r in references if 'map_file' in r),None)
        if source:result['map_file']=source
    return result
