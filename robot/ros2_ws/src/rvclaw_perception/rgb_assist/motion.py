"""Time-bounded optical flow vs wheel odometry. No metric visual odometry."""
import math
from collections import deque
import cv2
import numpy as np
from .features import gray_small


def angle(value): return math.atan2(math.sin(value),math.cos(value))


class OdomHistory:
    def __init__(self): self.samples=deque(maxlen=400); self.frame=None

    def add(self, stamp, x, y, yaw, frame):
        if not all(math.isfinite(v) for v in (stamp,x,y,yaw)): return
        if frame!=self.frame or (self.samples and stamp<=self.samples[-1][0]):
            self.samples.clear(); self.frame=frame
        self.samples.append((stamp,x,y,yaw))

    def at(self, stamp):
        samples=list(self.samples)
        for a,b in zip(samples,samples[1:]):
            if a[0]<=stamp<=b[0] and 0<b[0]-a[0]<=.15:
                dt=b[0]-a[0]
                if math.hypot(b[1]-a[1],b[2]-a[2])>1.0*dt+.02 or abs(angle(b[3]-a[3]))>2*dt+.03:
                    return None
                f=(stamp-a[0])/dt
                return a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]),angle(a[3]+f*angle(b[3]-a[3]))
        return None


class MotionConsistency:
    def __init__(self): self.previous=None; self.suspect_count=0

    def observe(self,image,stamp,sequence,generation,odom):
        gray=gray_small(image,320)
        old=self.previous
        self.previous=(gray,stamp,sequence,generation,odom)
        unknown={"status":"unknown", "metric_scale_available":False}
        if old is None or odom is None or old[4] is None:
            self.suspect_count=0; return dict(unknown,reason="odometry_or_previous_frame_unavailable")
        prev,ts,seq,gen,op=old
        dt=stamp-ts
        if gen!=generation or sequence<=seq or not .25<=dt<=2.5 or prev.shape!=gray.shape:
            self.suspect_count=0; return dict(unknown,reason="frame_gap_or_camera_restart")
        points=cv2.goodFeaturesToTrack(prev,100,.02,10)
        if points is None or len(points)<25:
            self.suspect_count=0; return dict(unknown,reason="low_texture")
        nxt,ok,err=cv2.calcOpticalFlowPyrLK(prev,gray,points,None,winSize=(15,15),maxLevel=2)
        if nxt is None or ok is None: return dict(unknown,reason="flow_failed")
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(gray,prev,nxt,None,winSize=(15,15),maxLevel=2)
        if back is None: return dict(unknown,reason="flow_failed")
        valid=(ok.ravel()>0)&(back_ok.ravel()>0)&(err.ravel()<25)&(np.linalg.norm(back-points,axis=2).ravel()<1.)
        if valid.sum()<25 or valid.mean()<.55:
            self.suspect_count=0; return dict(unknown,reason="insufficient_flow_support")
        src=points[valid].reshape(-1,2); dst=nxt[valid].reshape(-1,2)
        coverage=cv2.contourArea(cv2.convexHull(src))/(gray.shape[0]*gray.shape[1])
        transform,mask=cv2.estimateAffinePartial2D(src,dst,method=cv2.RANSAC,ransacReprojThreshold=1.5)
        if transform is None or mask is None or mask.mean()<.65 or coverage<.2:
            self.suspect_count=0; return dict(unknown,reason="scene_not_globally_consistent")
        px=float(np.median(np.linalg.norm(dst-src,axis=1)))
        dist=math.hypot(odom[0]-op[0],odom[1]-op[1]); yaw=abs(angle(odom[2]-op[2]))
        # Cannot identify slip from translation flow alone: scene depth is unknown.
        # Report a suspicion only after three supported pairs with wheel rotation
        # but essentially fixed images, or a fixed base but broad scene motion.
        mismatch=(yaw>.12 and px<.2) or (dist<.003 and yaw<.005 and px>4.)
        self.suspect_count=self.suspect_count+1 if mismatch else 0
        return dict(unknown,status="suspect" if self.suspect_count>=3 else "observed",
                    optical_flow_px=round(px,3),odom_translation_m=round(dist,4),
                    odom_rotation_rad=round(yaw,4),support=int(valid.sum()),
                    consecutive_disagreements=self.suspect_count,
                    timestamp_basis="host_frame_receive_not_exposure",
                    reason="possible_camera_or_scene_motion" if mismatch else "no_repeated_gross_disagreement")
