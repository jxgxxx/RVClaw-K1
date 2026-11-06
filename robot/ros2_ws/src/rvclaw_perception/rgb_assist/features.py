"""Small fixed-body template matcher with explicit ambiguity rejection."""
import json
from pathlib import Path
import cv2
import numpy as np
from .observation import alignment


def gray_small(image, width=480):
    h,w = image.shape[:2]
    if w > width:
        image = cv2.resize(image, (width, round(h*width/w)), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image


class LandmarkMatcher:
    def __init__(self, config_path, scene_path=None):
        config_path = Path(config_path)
        self.detector = cv2.SIFT_create(nfeatures=700, contrastThreshold=.025)
        self.templates = []
        records=[(r,config_path.parent/"reference-images",False) for r in json.loads(config_path.read_text())["meters"]]
        if scene_path:
            scene_path=Path(scene_path)
            if scene_path.is_file():
                records.extend((r,scene_path.parent,True) for r in json.loads(scene_path.read_text())["meters"])
        scene_indices=[]
        for record,directory,is_scene in records:
            filename = record.get("reference_image", "")
            if Path(filename).name != filename or not filename:
                continue
            image = cv2.imread(str(directory/filename))
            if image is None: continue
            gray = gray_small(image); h,w = gray.shape
            r = record["display_region"]
            quad = np.float32([[r['x']*w,r['y']*h],[(r['x']+r['width'])*w,r['y']*h],
                               [(r['x']+r['width'])*w,(r['y']+r['height'])*h],
                               [r['x']*w,(r['y']+r['height'])*h]])
            mask = np.full_like(gray,255)
            # Remove changing digits/needle from pixels AND keypoint mask.
            mutable = record.get('pointer_scale',{}).get('mutable_region')
            if mutable:
                center = (round(mutable['cx']*w), round(mutable['cy']*h))
                axes = (round(mutable['rx']*w)+4, round(mutable['ry']*h)+4)
                cv2.ellipse(mask,center,axes,0,0,360,0,-1)
            else:
                cv2.rectangle(mask,tuple(np.floor(quad[0]-4).astype(int)),
                              tuple(np.ceil(quad[2]+4).astype(int)),0,-1)
            if is_scene:
                # Keep the local instrument surroundings, not distant repeated
                # fire-box text or floor/wall patterns that could fit elsewhere.
                support=np.zeros_like(mask)
                pad=.35*max(quad[2]-quad[0])
                cv2.rectangle(support,tuple(np.floor(quad[0]-pad).astype(int)),
                              tuple(np.ceil(quad[2]+pad).astype(int)),255,-1)
                mask=cv2.bitwise_and(mask,support)
            gray[mask==0] = 128
            key,desc = self.detector.detectAndCompute(gray,mask)
            if desc is not None:
                if is_scene:scene_indices.append(len(self.templates))
                self.templates.append((record['meter_id'],key,desc,quad,w,h))
        # Repeated boxes/signs cannot establish place identity. Remove scene
        # descriptors that also occur at another configured landmark. This is
        # reference-only preparation, not added work for every camera frame.
        original=list(self.templates)
        for index in scene_indices:
            name,key,desc,quad,w,h=original[index]
            others=[d for other,k,d,q,ow,oh in original if other!=name]
            if not others:continue
            matches=cv2.BFMatcher().match(desc,np.concatenate(others))
            keep=[m.queryIdx for m in matches if m.distance>=200.]
            self.templates[index]=(name,[key[i] for i in keep],desc[keep],quad,w,h)

    def observe(self, image):
        gray = gray_small(image); h,w = gray.shape
        unknown = {"status":"unknown", "absolute_pose_available":False}
        if float(cv2.Laplacian(gray,cv2.CV_32F).var()) < 10:
            return dict(unknown,reason="blur_or_low_texture"), {"status":"unknown"}
        lk,ld = self.detector.detectAndCompute(gray,None)
        if ld is None: return unknown, {"status":"unknown"}
        candidates=[]
        for name,rk,rd,quad,rw,rh in self.templates:
            if len(rd)<14:continue
            pairs=cv2.BFMatcher().knnMatch(rd,ld,k=2)
            good=[p[0] for p in pairs if len(p)==2 and p[0].distance < .68*p[1].distance]
            good=list({m.trainIdx:m for m in sorted(good,key=lambda m:-m.distance)}.values())
            if len(good)<14: continue
            src=np.float32([rk[m.queryIdx].pt for m in good])
            dst=np.float32([lk[m.trainIdx].pt for m in good])
            H,keep=cv2.findHomography(src,dst,cv2.RANSAC,2.)
            if H is None or keep is None or not np.isfinite(H).all(): continue
            use=keep.ravel().astype(bool); n=int(use.sum())
            coverage=cv2.contourArea(cv2.convexHull(src[use]))/(rw*rh)
            if n<14 or n/len(good)<.5 or coverage<.06: continue
            projected=cv2.perspectiveTransform(quad[None],H)[0]
            if not np.isfinite(projected).all() or not cv2.isContourConvex(projected): continue
            area=cv2.contourArea(projected,oriented=True)
            if area<150 or area>w*h*.75: continue
            lengths=np.linalg.norm(projected-np.roll(projected,-1,axis=0),axis=1)
            if min(lengths)<10 or max(lengths)/min(lengths)>12: continue
            if max(lengths[0],lengths[2])/min(lengths[0],lengths[2])>2: continue
            if max(lengths[1],lengths[3])/min(lengths[1],lengths[3])>2: continue
            bbox=[float(projected[:,0].min()),float(projected[:,1].min()),
                  float(projected[:,0].max()),float(projected[:,1].max())]
            if bbox[0]<0 or bbox[1]<0 or bbox[2]>=w or bbox[3]>=h: continue
            candidates.append(dict(meter_id=name,inliers=n,coverage=round(coverage,3),bbox=bbox))
        # Never choose solely because navigation expects this meter. Two valid
        # identities remain ambiguous even when one has a larger match count.
        by_id={}
        for candidate in candidates:
            name=candidate['meter_id']
            if name not in by_id or candidate['inliers']>by_id[name]['inliers']:
                by_id[name]=candidate
        candidates=list(by_id.values())
        if len(candidates)!=1:
            return dict(unknown,reason="ambiguous" if candidates else "no_supported_match",
                        candidates=candidates), {"status":"unknown"}
        found=candidates[0]
        return dict(found,status="matched",absolute_pose_available=False), alignment(found['bbox'],w,h)
