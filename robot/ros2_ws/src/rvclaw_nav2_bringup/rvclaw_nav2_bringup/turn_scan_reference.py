"""Independent in-place rotation measurement; no ROS or motion interfaces.

The reference is the fresh full-view scan taken while disarmed. Matching is
about the base centre, including the laser offset, so AMCL translation aliases
cannot change the measured turn. A match may request an earlier safe stop;
it never authorizes arming, resets a latch, or bypasses collision checking.
"""
import math
import numpy as np
from scipy.spatial import cKDTree


def endpoints(scan, maximum=240):
    from rvclaw_nav2_bringup.scan_map_consistency import scan_endpoints
    if ((len(scan.ranges)-1)*scan.angle_increment < math.radians(330)
            or scan.angle_increment <= 0):
        raise ValueError('turn reference requires a full-view scan')
    points=scan_endpoints(scan.ranges,scan.angle_min,scan.angle_increment,
        max(.35,scan.range_min),scan.range_max,maximum_range_m=8.,maximum_points=maximum)
    if len(points)<120:raise ValueError('turn reference has insufficient endpoints')
    return np.asarray(points,dtype=float)


class TurnScanReference:
    def __init__(self, points):
        self.points=np.asarray(points,dtype=float)
        if self.points.ndim!=2 or self.points.shape[1]!=2 or len(self.points)<120 or not np.isfinite(self.points).all():
            raise ValueError('invalid turn reference points')
        self.tree=cKDTree(self.points)

    def match(self, points, expected):
        points=np.asarray(points,dtype=float)
        if (points.ndim!=2 or points.shape[1]!=2 or len(points)<120
                or not np.isfinite(points).all() or not math.isfinite(expected)):
            return dict(status='FAIL',reason='invalid turn observation')
        # Independent geometry selects the angle. Odometry only bounds the
        # search away from the opposite corridor mode; it does not score it.
        if len(points)>120:
            points=points[np.linspace(0,len(points)-1,120).astype(int)]
        def evaluate(angles):
            c,s=np.cos(angles)[:,None],np.sin(angles)[:,None]
            rotated=np.stack((c*points[:,0]-s*points[:,1],s*points[:,0]+c*points[:,1]),axis=-1)
            distances,_=self.tree.query(rotated.reshape(-1,2),workers=1)
            distances=distances.reshape(len(angles),len(points))
            return np.mean(np.exp(-.5*(distances/.10)**2),axis=1),distances
        coarse=expected+np.deg2rad(np.arange(-90.,90.01,5.))
        coarse_scores,_=evaluate(coarse)
        peak=int(np.argmax(coarse_scores))
        angles=coarse[peak]+np.deg2rad(np.arange(-5.,5.01,.5))
        scores,distances=evaluate(angles)
        # Occlusions change when the offset lidar turns about the base. Keep
        # broad support instead of allowing a handful of matching endpoints.
        index=int(np.argmax(scores));best=float(scores[index]);angle=float(angles[index])
        other=coarse_scores[np.abs(coarse-angle)>=math.radians(15)]
        second=float(np.max(other)) if len(other) else 1.
        inliers=float(np.mean(distances[index]<.15))
        valid=(best>=.70 and inliers>=.75 and best-second>=.10 and 0<peak<len(coarse)-1)
        return dict(status='PASS' if valid else 'FAIL',delta_yaw_rad=angle,
                    score=best,second_score=second,inlier_ratio=inliers)


def scan_turn_stop(target, progress, angular_velocity, age_s, consecutive):
    """Stop slightly before target to cover sensor latency and physical coast."""
    if (consecutive<2 or not all(math.isfinite(v) for v in (target,progress,angular_velocity,age_s))
            or not 0<=age_s<=.40 or abs(target)<.10 or target*progress<=0
            or abs(progress)>abs(target)+math.radians(8)):
        return False
    # Disarm is immediate; this is a stop request, not a velocity increase.
    # Include the physical deceleration after the triggering scan. This only
    # requests a stop; the stopped heading gates still decide completion.
    velocity=abs(angular_velocity)
    lead=min(math.radians(30),.40*abs(target),
             velocity*age_s+velocity*velocity/(2*.60)+math.radians(1.))
    return abs(progress)>=abs(target)-lead
