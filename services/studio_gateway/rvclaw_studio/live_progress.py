"""Validate the existing observer's pose without ROS work on HTTP requests."""
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import time
import yaml


def append_display_track(history, row, *, now=None):
    """Small source-stamped window on the existing observer, no extra ROS work."""
    now=time.time() if now is None else now
    pose=row.get('display_pose') or {}
    ready=row.get('valid') is True or (row.get('state')=='CONTROLLED_MOTION'
        and all(row.get(k) is True for k in ('hardware_connected','hardware_transport_attached','lidar_fresh')))
    if not ready or not row.get('map_sha256') or row.get('pose_message_fresh') is not True:
        return history
    values=[pose.get(k) for k in ('x','y','yaw','sampled_at')]
    if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in values):
        return history
    if not 0<=now-pose['sampled_at']<=4.:return history
    item={k:pose[k] for k in ('x','y','yaw','sampled_at')}
    if history and history[-1]['map_sha256']!=row['map_sha256']:history=[]
    if history and pose['sampled_at']<=history[-1]['sampled_at']:return history
    return [*history[-63:],dict(item,map_sha256=row['map_sha256'])]


@lru_cache(maxsize=8)
def _digest(yaml_path, metadata_stamp, image_path, image_stamp):
    digest=hashlib.sha256(b'rvclaw-map-v1\0')
    digest.update(Path(yaml_path).read_bytes())
    digest.update(b'\0')
    digest.update(Path(image_path).read_bytes())
    return digest.hexdigest()


@lru_cache(maxsize=8)
def _metadata(path, stamp):
    return yaml.safe_load(Path(path).read_text())


def live_pose(task, observer, root, *, now=None, history=False):
    now=time.time() if now is None else now
    try:
        if task.get('state') not in {'running','active','in_progress','localizing','finishing'}:
            return None
        if not task.get('task_id') or not 0<=now-observer['sampled_at']<=4.:
            return None
        row=observer['readings']['localization']
        pose=row['display_pose']
        # valid is the *stationary motion-admission* verdict. It is deliberately
        # false during controlled motion, when stationary verification is
        # deferred. Fresh moving poses remain useful for display only.
        display_ready = row.get('valid') is True or (
            row.get('state') == 'CONTROLLED_MOTION'
            and row.get('hardware_connected') is True
            and row.get('hardware_transport_attached') is True
            and row.get('lidar_fresh') is True)
        if (not 0<=now-row['received_at']<=4.
                or (not history and (not display_ready or row.get('pose_message_fresh') is not True))):
            return None
        if history:
            pose=None
        else:
            values=[pose[key] for key in ('x','y','yaw','sampled_at')]
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
                return None
            if not 0<=now-pose['sampled_at']<=4. or pose['sampled_at']<task['started_at']:
                return None
        source=Path(root)/'state/inspection-profile.json'
        stat=source.stat()
        profile=_metadata(str(source),(stat.st_mtime_ns,stat.st_size))
        if task.get('map_id')!=profile.get('map_id'):
            return None
        path=Path(profile['map_file']);stat=path.stat()
        metadata=_metadata(str(path),(stat.st_mtime_ns,stat.st_size))
        image=path.parent/metadata['image'];image_stat=image.stat()
        digest=_digest(str(path),(stat.st_mtime_ns,stat.st_size),str(image),
                       (image_stat.st_mtime_ns,image_stat.st_size))
        if digest!=row.get('map_sha256'):
            return None
        if history:
            result=[]
            for item in row.get('display_track',[])[-64:]:
                if item.get('map_sha256')!=digest:continue
                values=[item.get(k) for k in ('x','y','yaw','sampled_at')]
                if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):continue
                if not max(task['started_at'],now-90.)<=item['sampled_at']<=now:continue
                result.append({k:item[k] for k in ('x','y','yaw','sampled_at')})
            return sorted(result,key=lambda p:p['sampled_at'])
        return {key:pose[key] for key in ('x','y','yaw','sampled_at')} | {
            'task_id':task['task_id'],'map_id':task['map_id']}
    except (OSError,ValueError,KeyError,TypeError,yaml.YAMLError):
        return None
