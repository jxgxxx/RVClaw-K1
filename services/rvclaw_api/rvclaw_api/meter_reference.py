"""Register configured display geometry; reference readings are never used."""
import math
import re
from collections import OrderedDict
import hashlib
import threading

import cv2
import numpy as np


class ReferenceRegistrationError(ValueError):
    pass


_BODY_FEATURES = OrderedDict()
_BODY_FEATURE_LOCK = threading.Lock()


def prepare_display_reference(reference, region):
    """Cache only fixed reference-body features, never a live frame or reading."""
    rh, rw = reference.shape[:2]
    x0, y0 = float(region['x'])*rw, float(region['y'])*rh
    x1, y1 = x0+float(region['width'])*rw, y0+float(region['height'])*rh
    if not all(map(math.isfinite, (x0,y0,x1,y1))):
        raise ReferenceRegistrationError('nonfinite display geometry')
    if not (0 <= x0 < x1 <= rw and 0 <= y0 < y1 <= rh):
        raise ReferenceRegistrationError('display outside reference')
    key = (reference.shape, reference.dtype.str,
           hashlib.sha256(reference.tobytes()).digest(), x0,y0,x1,y1)
    with _BODY_FEATURE_LOCK:
        if key in _BODY_FEATURES:
            _BODY_FEATURES.move_to_end(key)
            return _BODY_FEATURES[key]
        body = reference.copy()
        body[max(0,int(y0)):min(rh,math.ceil(y1)),
             max(0,int(x0)):min(rw,math.ceil(x1))] = 128
        mask = np.full((rh,rw),255,np.uint8)
        mask[max(0,int(y0)-3):min(rh,math.ceil(y1)+3),
             max(0,int(x0)-3):min(rw,math.ceil(x1)+3)] = 0
        detector = cv2.SIFT_create(nfeatures=2500, contrastThreshold=.02)
        features = detector.detectAndCompute(cv2.cvtColor(body,cv2.COLOR_BGR2GRAY),mask)
        if features[1] is not None:
            features[1].setflags(write=False)
        _BODY_FEATURES[key] = features
        while len(_BODY_FEATURES)>8:
            _BODY_FEATURES.popitem(last=False)
        return features


def register_display(reference, live, region):
    """Locate a display by body features outside its changing digits."""
    rh, rw = reference.shape[:2]
    lh, lw = live.shape[:2]
    x0 = float(region['x']) * rw
    y0 = float(region['y']) * rh
    x1 = x0 + float(region['width']) * rw
    y1 = y0 + float(region['height']) * rh
    if not all(map(math.isfinite, (x0,y0,x1,y1))):
        raise ReferenceRegistrationError('nonfinite display geometry')
    if not (0 <= x0 < x1 <= rw and 0 <= y0 < y1 <= rh):
        raise ReferenceRegistrationError('display outside reference')
    # SIFT descriptors sample neighbouring pixels even when their keypoint
    # centre is outside a mask. Replace the changing display before feature
    # extraction as well, so its digits cannot influence body descriptors.
    detector = cv2.SIFT_create(nfeatures=2500, contrastThreshold=.02)
    rk,rd = prepare_display_reference(reference,region)
    lk,ld = detector.detectAndCompute(cv2.cvtColor(live,cv2.COLOR_BGR2GRAY),None)
    if rd is None or ld is None:
        raise ReferenceRegistrationError('instrument body features unavailable')
    matches = cv2.BFMatcher().knnMatch(rd,ld,k=2)
    good = [pair[0] for pair in matches if len(pair)==2 and pair[0].distance < .70*pair[1].distance]
    # A repetitive label must not count several reference features as the same
    # live feature; keep only its strongest correspondence.
    good = list({m.trainIdx:m for m in sorted(good,key=lambda m:-m.distance)}.values())
    if len(good)<12:
        raise ReferenceRegistrationError('too few independent body matches')
    src = np.float32([rk[m.queryIdx].pt for m in good])
    dst = np.float32([lk[m.trainIdx].pt for m in good])
    H,keep = cv2.findHomography(src,dst,cv2.RANSAC,2.)
    if H is None or keep is None or not np.isfinite(H).all():
        raise ReferenceRegistrationError('no finite display homography')
    use = keep.ravel().astype(bool)
    inliers = int(use.sum())
    coverage = cv2.contourArea(cv2.convexHull(src[use])) / (rw*rh)
    if inliers<12 or inliers/len(good)<.35 or coverage<.05:
        raise ReferenceRegistrationError('body matches lack spatial support')
    source_quad = np.float32([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
    quad = cv2.perspectiveTransform(source_quad[None],H)[0]
    if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
        raise ReferenceRegistrationError('invalid projected display')
    if (cv2.contourArea(quad,oriented=True)<=0 or
            np.any(quad[:,0]<0) or np.any(quad[:,0]>=lw) or
            np.any(quad[:,1]<0) or np.any(quad[:,1]>=lh)):
        raise ReferenceRegistrationError('display mirrored or outside current frame')
    lengths = np.linalg.norm(quad-np.roll(quad,-1,axis=0),axis=1)
    if min(lengths)<12 or min(lengths[0],lengths[2])<30:
        raise ReferenceRegistrationError('display too small to read')
    if max(lengths[0],lengths[2])/min(lengths[0],lengths[2])>2 or max(lengths[1],lengths[3])/min(lengths[1],lengths[3])>2:
        raise ReferenceRegistrationError('display perspective is excessive')
    # Keep a local, bounded warp; no unsupported parts of the reference image
    # are needed, even when the instrument's lower body is outside the camera.
    pad = .24*(y1-y0)
    left,top = max(0,math.floor(x0-pad)),max(0,math.floor(y0-pad))
    right,bottom = min(rw,math.ceil(x1+pad)),min(rh,math.ceil(y1+pad))
    local_to_ref = np.array([[1,0,left],[0,1,top],[0,0,1]],float)
    inverse = np.linalg.inv(H @ local_to_ref)
    size = (right-left,bottom-top)
    rectified = cv2.warpPerspective(live,inverse,size,borderValue=(128,128,128))
    support = cv2.warpPerspective(np.full((lh,lw),255,np.uint8),inverse,size,
                                  flags=cv2.INTER_NEAREST,borderValue=0)
    if float(np.mean(support==255))<.995:
        raise ReferenceRegistrationError('display crop contains missing current pixels')
    return rectified,(x0-left,y0-top,x1-left,y1-top), {
        'matches':len(good),'inliers':inliers,'body_coverage':round(coverage,4),
        'display_quad':quad.tolist(),
        'bbox':[float(quad[:,0].min()),float(quad[:,1].min()),
                float(quad[:,0].max()),float(quad[:,1].max())],
        'reference_digits_used_for_registration':False,
    }


def read_registered_display(reference,live,record,recognise_crops,make_crop,normalise,
                            *,min_score=.8,min_agreement=2,unit=''):
    rectified,box,diagnostics = register_display(reference,live,record['display_region'])
    # Keep the complete registered window. A mild brightness variant preserves
    # the top stroke of a seven that local contrast enhancement can erase.
    tight=make_crop(rectified,box,.02,.02,.02,1.)[0]
    gray=cv2.cvtColor(tight,cv2.COLOR_BGR2GRAY)
    brighter=np.uint8(np.clip(255*(gray.astype(np.float32)/255)**.8,0,255))
    equal=cv2.createCLAHE(clipLimit=2.,tileGridSize=(4,4)).apply(gray)
    crops=[tight,cv2.cvtColor(brighter,cv2.COLOR_GRAY2BGR),
           cv2.cvtColor(equal,cv2.COLOR_GRAY2BGR)]
    results=recognise_crops(crops)
    tally={}
    count=record.get('digit_count')
    places=int(record.get('decimal_places',0))
    for result in results:
        if not result or result['score']<min_score:continue
        text=normalise(result['text'])
        pattern = r'[+-]?\d+' + (r'\.\d{'+str(places)+'}' if places else '')
        if not re.fullmatch(pattern,text):continue
        if count is not None and sum(c.isdigit() for c in text)!=int(count):continue
        tally.setdefault(text,[]).append(float(result['score']))
    diagnostics['tally']=[{'text':text,'votes':len(scores),'score':min(scores)}
                          for text,scores in tally.items()]
    best=max(tally,key=lambda text:len(tally[text]),default=None)
    total=sum(map(len,tally.values()))
    stable=best is not None and len(tally[best])>=min_agreement and len(tally[best])*2>total
    return {'kind':'digital','status':'stable' if stable else 'ambiguous',
            'value':best if stable else None,'raw_text':best if stable else None,
            'confidence':min(tally[best]) if stable else 0.,'unit':unit,
            'reason':'' if stable else 'registered current display crops disagree',
            'value_method':'paddleocr_registered_current_display',
            'model':'PaddleOCR PP-OCRv5 mobile_rec (registered display)',
            'display_region':{'bbox':diagnostics['bbox']},
            'reference_registration':diagnostics,'candidates':[]}
