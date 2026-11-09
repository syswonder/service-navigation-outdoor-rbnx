"""Atomic RTK/IMU pose history. Never modifies the existing robot TF tree."""
import json,math,time,threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from collections import deque
import numpy as np
from std_msgs.msg import String


class NavigationPose:
    def __init__(self,node,topic='/outdoor/localization/sample',max_age=.4):
        self.lock=threading.RLock()
        self.changed=threading.Condition(self.lock)
        self.callback_group=MutuallyExclusiveCallbackGroup()
        self.node=node;self.max_age=max_age;self.samples=deque(maxlen=100)
        self.latest=None;self.receipt=0.;self.epoch=None
        self.expected_contract=None
        self.sub=node.create_subscription(String,topic,self.receive,20,callback_group=self.callback_group)

    def receive(self,msg):
        with self.lock:
            try:
                s=json.loads(msg.data);stamp=float(s['stamp']);epoch=str(s['epoch'])
                if s.get('frame_id')!='rtk_map' or not math.isfinite(stamp):raise ValueError('Invalid frame/time')
                if self.expected_contract and any(s.get(k)!=self.expected_contract[k] for k in ('map_id','map_version','datum_id','frame_id','conversion_version')):
                    raise ValueError('Localization coordinate contract mismatch')
                if epoch!=self.epoch or not s.get('valid'):self.samples.clear()
                self.epoch=epoch;self.latest=s;self.receipt=time.monotonic()
                if s.get('valid'):
                    p=np.asarray(s['pose'],float)
                    if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Invalid pose')
                    if self.samples and stamp<=self.samples[-1][0]:self.samples.clear()
                    self.samples.append((stamp,p))
            except (KeyError,ValueError,TypeError):
                self.latest=None;self.samples.clear()
            self.changed.notify_all()

    def current(self):
        with self.lock:
            s=self.latest;now=self.node.get_clock().now().nanoseconds*1e-9
            if (not s or not s.get('valid') or time.monotonic()-self.receipt>self.max_age
                    or not -.05<=now-s['stamp']<=self.max_age):raise ValueError('NAVIGATION_POSE_INVALID_OR_STALE')
            return s

    def at(self,stamp):
        with self.lock:
            self.current()
            for t,p in reversed(self.samples):
                if abs(t-stamp)<1e-6:return p.copy()
            for (a,p),(b,q) in zip(self.samples,list(self.samples)[1:]):
                if a<=stamp<=b and b-a<=.3:
                    k=(stamp-a)/(b-a);d=(q[2]-p[2]+math.pi)%(2*math.pi)-math.pi
                    return np.array([p[0]+k*(q[0]-p[0]),p[1]+k*(q[1]-p[1]),p[2]+k*d])
            raise ValueError('NAVIGATION_POSE_TIME_NOT_BRACKETED')

    def wait_at(self,stamp,timeout=.12):
        """Wait only for a newer bracket, never extrapolate or cross a reset."""
        deadline=time.monotonic()+timeout
        with self.changed:
            self.current()
            epoch=self.epoch
            while True:
                if self.epoch!=epoch:
                    raise ValueError('NAVIGATION_POSE_EPOCH_CHANGED')
                try:
                    return self.at(stamp)
                except ValueError as exc:
                    remaining=deadline-time.monotonic()
                    if (str(exc)!='NAVIGATION_POSE_TIME_NOT_BRACKETED' or remaining<=0
                            or not self.samples or stamp<=self.samples[-1][0]
                            or stamp-self.samples[-1][0]>.3):
                        raise
                    self.changed.wait(remaining)

    def matrix(self,stamp,wait_timeout=0.):
        stamp=stamp.sec+stamp.nanosec*1e-9 if hasattr(stamp,'sec') else stamp
        x,y,yaw=self.wait_at(stamp,wait_timeout) if wait_timeout else self.at(stamp)
        c,s=math.cos(yaw),math.sin(yaw)
        return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1]],float)
