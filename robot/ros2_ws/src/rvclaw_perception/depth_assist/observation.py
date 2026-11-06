"""Read optional same-host depth evidence; it never authorizes robot motion."""
import json
import math
from pathlib import Path
import time


def read_observation(path, *, now_ns=None, boot_id=None):
    unavailable = dict(available=False, motion_authority=False)
    try:
        with Path(path).open('rb') as stream:
            raw = stream.read(16385)
        if len(raw) > 16384:
            raise ValueError('oversized depth state')
        data = json.loads(raw)
        if (data['schema_version'] != 1 or data['status'] != 'OK'
                or data['motion_authority'] is not False
                or data['frame_id'] != 'astra_depth_optical_frame'
                or data['calibration_status'] != 'unverified'):
            raise ValueError('depth observation unavailable or unsupported')
        boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        timestamp = data['monotonic_ns']
        if type(timestamp) is not int or data['boot_id'] != boot_id:
            raise ValueError('depth clock mismatch')
        age = ((time.monotonic_ns() if now_ns is None else now_ns)-timestamp)/1e9
        if not 0 <= age <= 1.0:
            raise ValueError('depth observation stale')
        regions = data['regions']
        if not isinstance(regions,list) or len(regions)!=3:
            raise ValueError('incomplete depth regions')
        if {r['name'] for r in regions} != {'left','center','right'}:
            raise ValueError('invalid depth regions')
        valid = []
        for r in regions:
            fraction=r['valid_fraction']
            if type(fraction) not in (int,float) or not math.isfinite(fraction) or not 0 <= fraction <= 1:
                raise ValueError('invalid depth coverage')
            values=(r['median_z_m'],r['p10_z_m'])
            if values==(None,None): continue
            if fraction<.35 or any(type(v) not in (int,float) or not math.isfinite(v) or not .6<=v<=4. for v in values):
                raise ValueError('invalid depth distances')
            if values[1]>values[0]: raise ValueError('inconsistent depth quantiles')
            valid.append(r)
        if not valid: raise ValueError('no usable depth region')
        return dict(available=True, motion_authority=False, age_s=age,
                    frame_id=data['frame_id'], calibration_status='unverified', regions=regions)
    except (OSError,ValueError,TypeError,KeyError,OverflowError) as exc:
        return dict(unavailable,reason=str(exc))
