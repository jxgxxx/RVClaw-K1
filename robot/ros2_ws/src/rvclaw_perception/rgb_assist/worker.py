"""Low-priority subscriber and existing-JPEG consumer; no motion publishers."""
import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import threading
import time
from urllib.request import urlopen
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .api import state_dir
from .features import LandmarkMatcher
from .motion import OdomHistory,MotionConsistency


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--landmarks')
    parser.add_argument('--log-dir')
    parser.add_argument('--duration',type=float,default=0)
    args=parser.parse_args()
    cv2.setNumThreads(1)
    matcher=LandmarkMatcher(args.config,args.landmarks)
    history=OdomHistory(); motion=MotionConsistency(); lock=threading.Lock()
    rclpy.init(); node=Node('rvclaw_rgb_assist',enable_rosout=False)
    publisher=node.create_publisher(String,'/rvclaw/perception/rgb_assist',2)
    def odom(msg):
        p=msg.pose.pose;q=p.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with lock: history.add(stamp,p.position.x,p.position.y,yaw,(msg.header.frame_id,msg.child_frame_id))
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    spinner=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);spinner.start()
    directory=state_dir();directory.mkdir(mode=0o700,parents=True,exist_ok=True)
    log=logging.getLogger('rgb_assist');log.setLevel(logging.INFO)
    log_directory=Path(args.log_dir) if args.log_dir else directory
    log_directory.mkdir(mode=0o700,parents=True,exist_ok=True)
    handler=RotatingFileHandler(log_directory/'observations.jsonl',maxBytes=1048576,backupCount=2)
    log.addHandler(handler)
    begin=time.monotonic();next_landmark=0;last_key=None;last_log=0;consecutive=0;last_id=None
    previous_generation=None;last_match_time=0
    try:
        while rclpy.ok() and (not args.duration or time.monotonic()-begin<args.duration):
            started=time.monotonic();cpu=time.process_time()
            observation=dict(version=1,captured_at=time.time(),status='unavailable',motion_authorized=False)
            try:
                with urlopen('http://127.0.0.1:8017/api/rgb-assist/frame',timeout=.5) as response:
                    stamp=float(response.headers['X-RGB-Captured-At'])
                    sequence=int(response.headers['X-RGB-Sequence']);generation=response.headers['X-RGB-Generation']
                    raw=response.read(2*1024*1024+1)
                if len(raw)>2*1024*1024 or not 0<=time.time()-stamp<=.5:
                    raise ValueError('stale_or_oversized_frame')
                image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_REDUCED_COLOR_2)
                if image is None: raise ValueError('invalid_jpeg')
                # Allow the next odometry callback to bracket host receive time.
                with lock: pose=history.at(stamp)
                if pose is None:
                    time.sleep(.06)
                    with lock: pose=history.at(stamp)
                observation.update(captured_at=stamp,status='observed',frame_sequence=sequence,
                                   camera_generation=generation,motion=motion.observe(image,stamp,sequence,generation,pose))
                if started>=next_landmark:
                    landmark,align=matcher.observe(image)
                    found=landmark.get('meter_id')
                    if generation!=previous_generation or started-last_match_time>8:
                        consecutive=0;last_id=None
                    consecutive=consecutive+1 if found and found==last_id else (1 if found else 0)
                    last_id=found;previous_generation=generation;last_match_time=started
                    landmark['consecutive_observations']=consecutive
                    landmark['confirmed']=bool(found and consecutive>=2)
                    observation.update(landmark=landmark,alignment=align)
                    next_landmark=time.monotonic()+4.0
                else:
                    observation.update(landmark={'status':'unknown','reason':'rate_limited'},alignment={'status':'unknown'})
            except Exception as error:
                observation.update(reason=str(error)[:180]);motion.previous=None;motion.suspect_count=0
                consecutive=0;last_id=None
            observation['processing_ms']=round((time.monotonic()-started)*1000,2)
            observation['cpu_ms']=round((time.process_time()-cpu)*1000,2)
            if time.time()-observation['captured_at']>2.5:
                observation.update(status='unknown',reason='processing_over_budget',landmark={'status':'unknown'},
                                   alignment={'status':'unknown'},motion={'status':'unknown'})
            body=json.dumps(observation,allow_nan=False,separators=(',',':'))
            pending=directory/'observation.pending';pending.write_text(body);os.replace(pending,directory/'observation.json')
            publisher.publish(String(data=body))
            key=(observation['status'],observation.get('landmark',{}).get('meter_id'),observation.get('motion',{}).get('status'))
            if key!=last_key or time.monotonic()-last_log>=15:
                log.info(body);last_key=key;last_log=time.monotonic()
            time.sleep(max(.05,1.0-(time.monotonic()-started)))
    finally:
        rclpy.shutdown();spinner.join(timeout=2);node.destroy_node();handler.close()


if __name__=='__main__': main()
