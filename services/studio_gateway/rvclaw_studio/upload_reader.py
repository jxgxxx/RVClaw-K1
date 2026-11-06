"""Reuse K1's calibrated *geometry* and resident local models for known meters.

No models are loaded here. Static instrument features must register against the
uploaded current pixels; old display digits/pointer pixels never supply values.
Unregistered instruments retain the existing generic local upload reader.
"""
import base64,json,sys,time,os,types,re,math
from collections import Counter
from datetime import datetime,timezone
from pathlib import Path
from uuid import uuid4
import cv2
import numpy as np
from . import inspection_maps


def verified_display_digits(value,record):
    """Recover only agreeing OCR of current margins inside a registered LCD.

    The generic endpoint may reject its tiny detector box while its complete
    window checks all agree. Static registration supplies the missing evidence
    that this window is the actual display, rather than a printed serial row.
    """
    checks=value.get('detected_region_scan',{}).get('vertical_margin_verification',[])
    if value.get('status')=='stable' or len(checks)<2:return value
    places=int(record.get('decimal_places') or 0);count=record.get('digit_count')
    pattern=r'[+-]?\d+'+(r'\.\d{'+str(places)+'}' if places else '')
    candidates=[]
    for item in checks:
        text=str(item.get('text') or '').strip();score=float(item.get('score') or 0)
        if (math.isfinite(score) and score>=.8 and re.fullmatch(pattern,text)
                and (count is None or sum(c.isdigit() for c in text)==count)):
            candidates.append((text,score))
    votes=Counter(text for text,_ in candidates)
    if not votes:return value
    text,total=votes.most_common(1)[0]
    if total<2 or total*2<=len(checks):return value
    return {**value,'status':'stable','value':text,'raw_text':text,'reason':'',
            'confidence':min(score for candidate,score in candidates if candidate==text),
            'value_method':'paddleocr_registered_display_margin_consensus'}

class UploadReader:
    def __init__(self,repo,meters,upstream):
        self.repo,self.meters,self.upstream=repo,meters,upstream
        self.root=Path(os.getenv('RVCLAW_UPLOAD_EVIDENCE_ROOT','/data/RVClaw/runtime/inspection-progress/web-uploads'))
        # The API package initializer imports the robot perception pipeline.
        # Load only its pure geometry files through a private namespace.
        package=types.ModuleType('_rvclaw_studio_geometry')
        package.__path__=[str(repo/'services/rvclaw_api/rvclaw_api')]
        sys.modules.setdefault(package.__name__,package)
        self.refs={}
        self.body_features={}
        cv2.setNumThreads(1)

    def reference(self,record):
        name=record.get('reference_image','')
        if not name or Path(name).name!=name:return None
        path=self.meters.reference_image_dir/name
        if not path.is_file():return None
        key=(str(path),path.stat().st_mtime_ns)
        if key not in self.refs:
            self.refs[key]=cv2.imread(str(path))
            if len(self.refs)>8:self.refs.pop(next(iter(self.refs)))
        return self.refs[key]

    def ordered_candidates(self,records,image):
        """Cheap ordering only; the strict registration still decides identity."""
        if len(records)<2:return records
        detector=cv2.ORB_create(nfeatures=1200,edgeThreshold=12,fastThreshold=8)
        def small(image):
            scale=min(1.,720/max(image.shape[:2]))
            return cv2.resize(image,None,fx=scale,fy=scale) if scale<1 else image.copy()
        _,live=detector.detectAndCompute(cv2.cvtColor(small(image),cv2.COLOR_BGR2GRAY),None)
        if live is None:return records
        ranked=[]
        for record in records:
            ref=self.reference(record);score=0
            if ref is not None:
                path=self.meters.reference_image_dir/record['reference_image']
                key=(str(path),path.stat().st_mtime_ns,str(record.get('display_region')))
                if key not in self.body_features:
                    body=small(ref);height,width=body.shape[:2];region=record['display_region']
                    x0,y0=round(region['x']*width),round(region['y']*height)
                    x1,y1=round((region['x']+region['width'])*width),round((region['y']+region['height'])*height)
                    body[max(0,y0-3):y1+3,max(0,x0-3):x1+3]=128
                    self.body_features[key]=detector.detectAndCompute(cv2.cvtColor(body,cv2.COLOR_BGR2GRAY),None)[1]
                    if len(self.body_features)>8:self.body_features.pop(next(iter(self.body_features)))
                features=self.body_features[key]
                if features is not None:
                    pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(features,live,k=2)
                    score=sum(len(pair)==2 and pair[0].distance<.7*pair[1].distance for pair in pairs)
            ranked.append((score,record))
        return [record for _,record in sorted(ranked,key=lambda item:-item[0])]

    def read(self,payload):
        from _rvclaw_studio_geometry.meter_reference import register_display,ReferenceRegistrationError
        from _rvclaw_studio_geometry.pointer_reference import read_registered_pointer,_pose_current
        raw=base64.b64decode(payload['image'].split(',',1)[-1],validate=True)
        image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.size==0:raise ValueError('无法读取图片，请选择有效的仪表图片。')
        if image.shape[0]*image.shape[1]>24_000_000:raise ValueError('图片尺寸过大，请缩小后重试。')
        kind=payload['meter_type'];started=time.monotonic()
        pointer_pose=None
        if kind=='pointer':
            try:pointer_pose=_pose_current(image)
            except (ReferenceRegistrationError,cv2.error,TimeoutError):
                # The general reader previously returned a plausible but wrong
                # number for tightly cropped dials rejected by the live model.
                # Never promote that unverified fallback into a successful read.
                return self.persist({'status':'unreadable','value':None,'confidence':0.,
                    'reason':'未能可靠定位当前指针，请上传包含完整表盘与周围边缘的清晰图片。',
                    'model':'NanoDet + YOLOv8s-Pose (local pointer validation)'},payload,raw,started)
        registry=inspection_maps.bundle()['directory']
        plans=inspection_maps._yaml(registry/'inspection_route_tasks.yaml').get('points',{})
        compatible={p.get('meter_id'):p for p in plans.values() if p.get('kind')==kind+'_meter'}
        pointer_failure=None
        # Public list() deliberately removes the reference filename.
        records=[record for meter_id in compatible if (record:=self.meters.get(meter_id))]
        for record in self.ordered_candidates(records,image):
            reference=self.reference(record)
            if reference is None:continue
            if kind=='pointer' and not record.get('pointer_scale'):continue
            try:
                if kind=='pointer':
                    calibration=compatible[record['meter_id']].get('calibration')
                    if not calibration:continue
                    # Registration has a strict wall-clock budget in the car
                    # code. Under the website's lower CPU quota a cold static
                    # feature cache may use that budget. One bounded retry
                    # reuses only static descriptors and the same current pose.
                    for attempt in range(2):
                        try:value=read_registered_pointer(reference,image,record,calibration,pose_reader=lambda _:pointer_pose)
                        except TimeoutError:value={'status':'unreadable','value':None}
                        if value.get('status')=='stable':break
                    if value.get('status')!='stable':
                        pointer_failure=value
                        continue
                else:
                    current,box,diagnostics=register_display(reference,image,record['display_region'])
                    # Keep the small verified window plus its registration
                    # margin. The existing detector needs this boundary to
                    # locate digits; a tight crop alone can lose the LCD edge.
                    ok,jpeg=cv2.imencode('.jpg',current,[cv2.IMWRITE_JPEG_QUALITY,95])
                    if not ok:continue
                    request={**payload,'image':base64.b64encode(jpeg).decode(),'filename':payload.get('filename','uploaded')}
                    value=self.upstream(request)
                    value=verified_display_digits(value,record)
                    value['supersedes_evidence_id']=value.get('evidence_id')
                    # The configured digit count is a rejection gate only.
                    count=record.get('digit_count')
                    if value.get('value') is not None and count is not None and sum(c.isdigit() for c in str(value['value']))!=count:
                        value.update(status='unreadable',value=None,reason='当前显示区读数位数不一致，请更换清晰图片。')
                    value['reference_registration']=diagnostics
                    value['model']='PaddleOCR PP-OCRv5 (current registered display)'
                return self.persist({**value,'reference_meter_id':record['meter_id']},payload,raw,started)
            except (ReferenceRegistrationError,cv2.error,np.linalg.LinAlgError,TimeoutError):
                continue
        if kind=='pointer':
            return self.persist({**(pointer_failure or {}),'status':'unreadable','value':None,
                'reason':'当前指针与刻度未能通过一致性校验，请使用正对仪表、包含完整表盘的图片重试。',
                'model':'NanoDet + YOLOv8s-Pose (local pointer validation)'},payload,raw,started)
        return None

    def persist(self,value,payload,raw,started):
        kind=payload['meter_type']
        value={**value,'kind':kind,'meter_type':kind,'unit':'','inference_ms':round((time.monotonic()-started)*1000,1),'created_at':datetime.now(timezone.utc).isoformat(),'source':'web_upload','filename':str(payload.get('filename') or 'uploaded-image')}
        for key in ['evidence_id','image_url','crop_url','original_image_url']:value.pop(key,None)
        identifier=uuid4().hex;value.update(evidence_id=identifier,image_url='/api/studio/uploads/'+identifier+'/image.jpg')
        self.root.mkdir(parents=True,exist_ok=True)
        (self.root/(identifier+'.jpg')).write_bytes(raw)
        temporary=self.root/(identifier+'.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(temporary,self.root/(identifier+'.json'))
        return value

    def records(self):
        rows=[]
        for p in sorted(self.root.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:1500]:
            try:rows.append(json.loads(p.read_text(encoding='utf-8')))
            except (OSError,ValueError):pass
        return rows
