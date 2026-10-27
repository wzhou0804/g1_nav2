"""Hardware command adaptation; no motor publisher is created here."""
import threading
import time

import numpy as np

from navigation.student_deploy_core import DeployFault
from navigation.velocity_feedback import VelocityFeedback


class PostureTransition:
    """Smooth entry trajectory; duration follows displacement, not a fixed timer."""
    def __init__(self,start,target,speed=.5):
        self.start=np.array(start,dtype=float,copy=True)
        self.target=np.array(target,dtype=float,copy=True)
        if (self.start.shape!=(27,) or self.target.shape!=(27,) or
            not np.isfinite(self.start).all() or not np.isfinite(self.target).all() or
            not np.isfinite(speed) or speed<=0):
            raise DeployFault('Invalid posture transition or entry speed')
        # Cubic smoothstep has peak slope 1.5; speed bounds the reference's peak rate.
        self.duration=1.5*float(np.max(np.abs(self.target-self.start)))/speed

    def at(self,elapsed):
        if self.duration==0:return self.target.copy()
        ratio=float(np.clip(elapsed/self.duration,0.,1.))
        blend=ratio*ratio*(3.-2.*ratio)
        return self.start+(self.target-self.start)*blend


def yaw_velocity(message,frame):
    """Explicitly interpret the odometry velocity frame; never infer it from topic name."""
    if frame not in ('world','yaw'):raise DeployFault('Velocity frame must be world or yaw')
    values=np.array([*message.velocity[:2],message.yaw_speed],dtype=float)
    if message.error_code or not np.isfinite(values).all():
        raise DeployFault('Invalid odometry velocity/error_code')
    if frame=='world':
        q=np.asarray(message.imu_state.quaternion,dtype=float)
        norm=np.linalg.norm(q)
        if q.shape!=(4,) or not np.isfinite(q).all() or not np.isfinite(norm) or norm==0:
            raise DeployFault('Invalid odometry orientation for world-to-yaw velocity')
        w,x,y,z=q/norm
        yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        c,s=np.cos(yaw),np.sin(yaw)
        values[:2]=[c*values[0]+s*values[1],-s*values[0]+c*values[1]]
    return values


class OdomVelocityReader:
    def __init__(self,frame,topic='rt/odommodestate'):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        self.frame=frame;self.topic=topic;self.lock=threading.Lock()
        self.latest=None;self.last_source_ns=None;self.fault=None
        self.count=0;self.source_clock_available=False
        self.subscriber=ChannelSubscriber(topic,SportModeState_)
        self.subscriber.Init(self.receive,1)

    def receive(self,message):
        try:
            values=yaw_velocity(message,self.frame)
            stamp=int(message.stamp.sec)*1_000_000_000+int(message.stamp.nanosec)
            if stamp<0 or not 0<=message.stamp.nanosec<1_000_000_000:
                raise DeployFault('Invalid odometry timestamp')
            with self.lock:
                if stamp>0:
                    self.source_clock_available=True
                    if self.last_source_ns is not None:
                        if stamp==self.last_source_ns:return  # Do not refresh stale samples.
                        if stamp<self.last_source_ns:raise DeployFault('Odometry clock reset/out of order')
                    self.last_source_ns=stamp
                elif self.source_clock_available:
                    raise DeployFault('Odometry clock disappeared')
                self.latest=(time.monotonic(),values);self.count+=1
        except Exception as exc:
            with self.lock:self.fault=str(exc)

    def ready(self):
        with self.lock:return self.latest is not None or self.fault is not None

    def get(self):
        with self.lock:
            if self.fault:raise DeployFault(self.fault)
            if self.latest is None:raise DeployFault('No physical odometry velocity')
            stamp,values=self.latest
            if not 0<=time.monotonic()-stamp<=.15:
                raise DeployFault('Physical odometry velocity stale (>150ms)')
            return values.copy()


class HomieDeployCommand:
    """Same bounded 50 Hz outer loop as the tested simulation, with real measurements."""
    def __init__(self,closed_loop=False,yaw_feedforward=0.,limits=(.8,.5,.8),navigation=False):
        self.navigation=navigation
        self.feedback=VelocityFeedback() if closed_loop else None
        if self.feedback is not None:
            self.feedback.limit=np.array(limits,dtype=float)
            self.feedback.feedforward[2]=yaw_feedforward
            if navigation:
                self.feedback.acceleration=np.array([.2,.15,.4])
                self.feedback.smooth_stop=True
        self.reset()

    def reset(self):
        if self.feedback is not None:self.feedback.reset()
        self.desired=np.zeros(3);self.measured=None;self.command=np.zeros(3)
        self.last_update=None;self.command_dt=.02

    def update(self,desired,measured=None,now=None):
        self.desired=np.asarray(desired,dtype=float)
        if self.desired.shape!=(3,) or not np.isfinite(self.desired).all():
            raise DeployFault('Invalid desired velocity')
        self.measured=None if measured is None else np.asarray(measured,dtype=float)
        if self.feedback is not None:
            if self.measured is None and not np.any(self.desired):
                self.feedback.reset()
                self.command=np.zeros(3)
                self.last_update=None
                return self.command.copy()
            if self.measured is None or self.measured.shape!=(3,) or not np.isfinite(self.measured).all():
                raise DeployFault('Closed-loop deployment requires finite physical velocity')
            if self.navigation:
                now=time.monotonic() if now is None else now
                self.command_dt=.02 if self.last_update is None else now-self.last_update
                self.last_update=now
            self.command=self.feedback.update(self.desired,self.measured,dt=self.command_dt)
        else:self.command=self.desired.copy()
        return self.command.copy()
