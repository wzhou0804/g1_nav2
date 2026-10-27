#!/usr/bin/env python3
"""Locked G1 deployment: read-only by default; OpenHomie accepts explicit velocity inputs."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone, timedelta
import fcntl
import json
import ipaddress
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from navigation.student_deploy_core import (
    A, L1, START, ACTIVE_SLOTS, CHECKPOINT_SHA, Configuration, DeployFault, FreshButton,
    Policy, bounded_target, fill_command, pd_bounded_target, snapshot_from_message, validate_state)
from navigation.locomotion_commands import TerminalCommands,UnitreeJoystickCommands
from navigation.homie_deploy import HomieDeployCommand,OdomVelocityReader,PostureTransition
from navigation.homie_nav2_transport import HomieNavCommands,DEFAULT_PORT


def deployment_durations(policy,seconds=None,wait_seconds=None):
    seconds=(0. if policy=='homie' else 10.) if seconds is None else seconds
    wait_seconds=(0. if policy=='homie' else 180.) if wait_seconds is None else wait_seconds
    if policy=='homie':
        if not all(np.isfinite(x) and x>=0 for x in (seconds,wait_seconds)):
            raise ValueError('OpenHomie durations must be finite and nonnegative; 0 means unlimited')
    elif not 0<seconds<=60 or not 0<wait_seconds<=600:
        raise ValueError('HTD --seconds must be (0,60], --wait-seconds must be (0,600]')
    return seconds,wait_seconds


def within_duration(elapsed,limit):
    return limit==0 or elapsed<limit


class StateReader:
    def __init__(self,start_thread=True,domain_id=0):
        from navigation.lowstate_receiver import LowStateSubscription
        from unitree_sdk2py.utils.crc import CRC
        self.crc = CRC()
        self.lock = threading.Lock()
        self.latest = None
        self.fault = None
        self.count = self.bad_crc = self.duplicate_ticks = 0
        self.received_count=0;self.read_timeouts=0
        self.last_arrival=None;self.max_arrival_gap=0.
        self.stop=threading.Event()
        self.subscriber = LowStateSubscription(domain_id)
        # The SDK listener takes only one sample per notification. A dedicated
        # reader continuously drains DDS, including samples already queued.
        self.thread=threading.Thread(target=self.loop,name='g1-lowstate-receiver',daemon=True)
        if start_thread:self.thread.start()

    def loop(self):
        try:
            while not self.stop.is_set():
                message=self.subscriber.Read(timeout=.05)
                if message is None:
                    with self.lock:self.read_timeouts+=1
                else:self.receive(message)
        except Exception as exc:
            with self.lock:self.fault='LowState receive: '+str(exc)

    def diagnostics(self):
        with self.lock:
            now=time.monotonic()
            return {'receiver':'dedicated DDS reader','received_messages':self.received_count,
                'unique_ticks':self.count,'duplicate_ticks':self.duplicate_ticks,'read_timeouts':self.read_timeouts,
                'max_arrival_gap_ms':1000*self.max_arrival_gap,
                'latest_unique_tick':None if self.latest is None else self.latest.tick,
                'unique_tick_age_ms':None if self.latest is None else 1000*(now-self.latest.stamp),
                'last_message_age_ms':None if self.last_arrival is None else 1000*(now-self.last_arrival),
                'thread_alive':self.thread.is_alive(),'fault':self.fault}

    def close(self):
        self.stop.set()
        if self.thread.is_alive():self.thread.join(1.)
        self.subscriber.Close()

    def receive(self,message):
        try:
            now=time.monotonic()
            with self.lock:
                self.received_count+=1
                if self.last_arrival is not None:self.max_arrival_gap=max(self.max_arrival_gap,now-self.last_arrival)
                self.last_arrival=now
            if self.crc.Crc(message)!=message.crc:
                with self.lock:
                    self.bad_crc += 1
                    self.fault = 'LowState CRC failure'
                return
            s = snapshot_from_message(message,time.monotonic())
            with self.lock:
                if self.latest is not None:
                    delta = (s.tick-self.latest.tick) & 0xffffffff
                    if delta==0:
                        self.duplicate_ticks += 1
                        return
                    if delta>0x80000000:
                        self.fault = 'LowState tick reset/out of order'
                        return
                self.latest = s
                self.count += 1
        except Exception as exc:
            with self.lock: self.fault = 'LowState decode: '+str(exc)

    def get(self):
        with self.lock:
            if self.fault: raise DeployFault(self.fault)
            return self.latest


class Discovery:
    def __init__(self):
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        self.participant = DomainParticipant(0)
        self.reader = BuiltinDataReader(self.participant,BuiltinTopicDcpsPublication)
        self.publishers = {}

    def count_lowcmd(self):
        for endpoint in self.reader.take(200):
            key = str(endpoint.key)
            if endpoint.sample_info.valid_data:
                if endpoint.topic_name=='rt/lowcmd': self.publishers[key]=endpoint.type_name
            else: self.publishers.pop(key,None)
        return len(self.publishers)


def initialize_channel(address):
    # The SDK's stock name-only interface template selects the first IPv4 alias.
    # Bind the specific robot-facing address within this process; preserve the
    # installed SDK and the computer's interface configuration.
    from unitree_sdk2py.core import channel
    address=str(ipaddress.IPv4Address(address))
    channel.ChannelConfigHasInterface = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface address="$__IF_NAME__$"/>'
        '</Interfaces><AllowMulticast>true</AllowMulticast>'
        '</General></Domain></CycloneDDS>')
    channel.ChannelFactoryInitialize(0,address)


class MotorWriter:
    """The only owner of LowCmd. Latches faults and sends bounded passive damping before exit."""
    def __init__(self,reader,cfg):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        self.reader,self.cfg = reader,cfg
        self.crc = CRC()
        self.command = unitree_hg_msg_dds__LowCmd_()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.stopped = threading.Event()
        self.fault = None
        self.frame = None
        self.posture = None
        self.count = self.damping_count = self.overruns = 0
        self.max_gap = 0.
        self.target_age_max_ms = self.source_age_max_ms = 0.
        self.last_target = None
        self.publisher = ChannelPublisher('rt/lowcmd',LowCmd_)
        self.publisher.Init()
        self.thread = threading.Thread(target=self.loop,name='student-lowcmd-500hz',daemon=True)

    def update(self,target,source_stamp):
        with self.lock:
            if self.fault: raise DeployFault(self.fault)
            self.posture = None
            self.frame = (np.array(target,copy=True),time.monotonic(),source_stamp)

    def move_to_posture(self,start,target,speed=.5):
        plan=PostureTransition(start,target,speed)
        with self.lock:
            if self.fault:raise DeployFault(self.fault)
            self.posture=(plan,time.monotonic())
            self.frame=None
        return plan

    def hold_posture(self,target):
        self.move_to_posture(target,target)

    def fail(self,reason):
        with self.lock:
            if self.fault is None: self.fault=str(reason)
        self.stop.set()

    def check(self):
        with self.lock: fault=self.fault
        if fault: raise DeployFault(fault)
        if self.stopped.is_set(): raise DeployFault('LowCmd publisher exited')

    def loop(self):
        deadline = time.monotonic()
        previous = None
        damping_since = None
        try:
            while True:
                now = time.monotonic()
                gap=.002
                if previous is not None:
                    gap=now-previous;self.max_gap=max(self.max_gap,gap)
                    if gap>.05 and getattr(self.cfg,'policy_timing_checks',True):
                        self.fail(f'LowCmd scheduling gap {gap:.3f}s')
                    if gap>.004: self.overruns+=1
                previous=now
                target=self.cfg.default
                if not self.stop.is_set():
                    try:
                        s=self.reader.get()
                        checked_at=time.monotonic()
                        validate_state(s,checked_at,self.cfg)
                        if getattr(self.cfg,'require_l1',True) and not s.buttons & L1:
                            raise DeployFault('L1 deadman released')
                        with self.lock: frame,posture=self.frame,self.posture
                        if posture is not None:
                            # An explicit entry/hold trajectory has no old policy sample.
                            # LowState validity, L1 and motor-fault checks still apply.
                            requested=posture[0].at(checked_at-posture[1])
                        else:
                            if frame is None:raise DeployFault('Policy/control target missing')
                            update_age=checked_at-frame[1];source_age=checked_at-frame[2]
                            self.target_age_max_ms=max(self.target_age_max_ms,1000*update_age)
                            self.source_age_max_ms=max(self.source_age_max_ms,1000*source_age)
                            if getattr(self.cfg,'policy_timing_checks',True) and (update_age>.10 or source_age>.10):
                                raise DeployFault('Policy/control target stale')
                            requested=frame[0]
                        target=pd_bounded_target(requested,s,self.cfg,self.last_target,min(.01,max(.001,gap)))
                    except Exception as exc:
                        self.fail(exc)
                if self.stop.is_set():
                    if damping_since is None: damping_since=now
                    # Do not send mode=6 commands to a robot reporting another model.
                    with self.reader.lock: trusted=self.reader.latest
                    if trusted is not None and trusted.machine!=6: break
                    if now-damping_since>=1.: break
                    fill_command(self.command,target,self.cfg,damping=True)
                    self.damping_count+=1
                else:
                    fill_command(self.command,target,self.cfg)
                    self.last_target=target.copy()
                self.command.crc=self.crc.Crc(self.command)
                if not self.publisher.Write(self.command): raise DeployFault('LowCmd DDS write failed')
                self.count+=1
                deadline+=.002
                delay=deadline-time.monotonic()
                if delay>0: time.sleep(delay)
                else: deadline=time.monotonic()
        except Exception as exc:
            self.fail(exc)
        finally:
            self.stopped.set()

    def close(self):
        self.stop.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2.)
        self.publisher.Close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface',default='enx207bd298e838')
    parser.add_argument('--policy',choices=('htd','homie'),default='htd')
    parser.add_argument('--address',default='192.168.123.222')
    parser.add_argument('--arm',action='store_true',help='Enable takeover: navigation service in --nav2 mode, remote handshake otherwise')
    parser.add_argument('--support-confirmed',action='store_true')
    parser.add_argument('--operator-ready',action='store_true')
    parser.add_argument('--seconds',type=float,help='Shadow or active duration; OpenHomie defaults to 0 (unlimited), HTD to 10s')
    parser.add_argument('--wait-seconds',type=float,help='Remote gate timeout; OpenHomie defaults to 0 (unlimited), HTD to 180s')
    parser.add_argument('--posture-speed',type=float,help='OpenHomie entry reference peak speed in rad/s (default 0.5); not a policy output limit')
    parser.add_argument('--output',type=Path,default=ROOT/'.cache-g1/student-deploy/current')
    parser.add_argument('--policy-target-margin',type=float,default=.5,
                        help='HTD raw target rejection margin, 0..1 rad; ignored for OpenHomie')
    parser.add_argument('--interactive',action='store_true',help='OpenHomie: enter vx vy wz / stop / quit after L1+A')
    parser.add_argument('--command',type=float,nargs=3,metavar=('VX','VY','WZ'),help='OpenHomie: fixed velocity after L1+A')
    parser.add_argument('--joystick',action='store_true',help='OpenHomie: Unitree remote left stick vx/vy, right stick yaw')
    parser.add_argument('--nav2',action='store_true',help='OpenHomie: use the real-navigation enable_motion service and velocity input')
    parser.add_argument('--nav2-port',type=int,default=DEFAULT_PORT,help='Loopback velocity port shared with the navigation bridge')
    parser.add_argument('--joystick-scale',type=float,nargs=3,metavar=('VX','VY','WZ'),
                        help='Full-stick speeds (default 0.30 0.15 0.40); zero disables an axis')
    parser.add_argument('--joystick-deadzone',type=float,help='Rescaled stick center deadzone (default 0.08)')
    parser.add_argument('--feedback',choices=('raw','closed-loop'),default='raw',help='OpenHomie velocity command adaptation')
    parser.add_argument('--yaw-feedforward',type=float,default=0.,help='Closed-loop yaw feedforward in rad/s (navigation profile: 0.4)')
    parser.add_argument('--velocity-topic',default='rt/odommodestate',help='SportModeState_ source for closed-loop velocity')
    parser.add_argument('--velocity-frame',choices=('world','yaw'),help='Explicit measured linear velocity frame; required for closed-loop')
    args=parser.parse_args()
    print('[student-deploy] selected policy: '+args.policy,flush=True)
    if args.arm and not args.nav2 and not (args.support_confirmed and args.operator_ready):
        parser.error('--arm requires confirmed suspension and an on-site emergency-stop operator')
    if args.policy!='homie' and (args.interactive or args.command is not None or args.joystick or args.nav2 or
                               args.joystick_scale is not None or args.joystick_deadzone is not None or
                               args.feedback!='raw' or args.velocity_frame or args.yaw_feedforward):
        parser.error('Velocity command options require --policy homie (run_g1_homie.sh)')
    if sum((args.interactive,args.command is not None,args.joystick,args.nav2))>1:
        parser.error('Choose only one of --interactive, --command, --joystick, or --nav2')
    if not 1<=args.nav2_port<=65535:parser.error('--nav2-port must be a valid UDP port')
    if not args.joystick and (args.joystick_scale is not None or args.joystick_deadzone is not None):
        parser.error('--joystick-scale and --joystick-deadzone require --joystick')
    try:
        joystick=UnitreeJoystickCommands(
            (.30,.15,.40) if args.joystick_scale is None else args.joystick_scale,
            .08 if args.joystick_deadzone is None else args.joystick_deadzone) if args.joystick else None
    except ValueError as exc:parser.error(str(exc))
    if args.interactive and not sys.stdin.isatty():parser.error('--interactive requires a terminal')
    if not np.isfinite(args.yaw_feedforward) or args.yaw_feedforward<0:
        parser.error('--yaw-feedforward must be nonnegative and finite')
    if args.yaw_feedforward and args.feedback!='closed-loop':parser.error('--yaw-feedforward requires --feedback closed-loop')
    if args.nav2 and args.velocity_frame is not None:parser.error('--nav2 uses /dog_odom body twist from the ROS bridge')
    if args.feedback=='closed-loop' and not args.nav2 and args.velocity_frame is None:
        parser.error('--feedback closed-loop requires an explicitly verified --velocity-frame world or yaw')
    if args.feedback=='raw' and args.velocity_frame is not None:parser.error('--velocity-frame requires --feedback closed-loop')
    try:fixed_command=TerminalCommands(limits=None if args.policy=='homie' else (.45,.15,.6)).accept(' '.join(map(str,args.command or [0,0,0])))
    except ValueError as exc:parser.error(str(exc))
    try:args.seconds,args.wait_seconds=deployment_durations(args.policy,args.seconds,args.wait_seconds)
    except ValueError as exc:parser.error(str(exc))
    if args.policy=='homie':
        args.posture_speed=.5 if args.posture_speed is None else args.posture_speed
        if not np.isfinite(args.posture_speed) or args.posture_speed<=0:
            parser.error('--posture-speed must be positive and finite')
    elif args.posture_speed is not None:parser.error('--posture-speed requires --policy homie')
    if args.policy!='homie' and (not np.isfinite(args.policy_target_margin) or not 0<=args.policy_target_margin<=1.):
        parser.error('--policy-target-margin must be within [0,1] rad')
    if not (Path('/sys/class/net')/args.interface).exists(): parser.error('Network interface does not exist')
    try: args.address=str(ipaddress.IPv4Address(args.address))
    except ValueError: parser.error('Invalid local robot-facing IPv4 address')
    interface_info=json.loads(subprocess.check_output(['ip','-j','-4','address','show','dev',args.interface],text=True))
    local_addresses={a['local'] for item in interface_info for a in item.get('addr_info',[])}
    if args.address not in local_addresses: parser.error('--address must belong to --interface')
    args.output.mkdir(parents=True,exist_ok=True)
    # Cross-process lock also covers shadow: never start a second controller accidentally.
    lock_path=ROOT/'.cache-g1/student-deploy/controller.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock_file=lock_path.open('a+')
    try: fcntl.flock(lock_file,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('Another Student deployment process holds the lock')
    import os
    lock_file.seek(0);lock_file.truncate();lock_file.write(str(os.getpid()));lock_file.flush()
    pid_file=lock_path.with_name('controller.pid')
    pid_file.write_text(str(os.getpid())+'\n')
    config_class,policy_class=Configuration,Policy
    if args.policy=='homie':
        from navigation.homie import Configuration as config_class,Policy as policy_class
    cfg=config_class(ROOT,policy_target_margin=args.policy_target_margin)
    if args.nav2:cfg.require_l1=False
    sdk=ROOT/'.cache-g1/unitree_sdk2_python'
    revision=subprocess.check_output(['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip()
    if revision!='814556d15970dd2ecf1c9984e845ca02ab07e206': raise DeployFault('SDK commit mismatch')
    policy=policy_class(cfg)
    commands=HomieDeployCommand(args.feedback=='closed-loop',args.yaw_feedforward,
        limits=(.6,.6,.6) if args.nav2 else (.8,.5,.8),navigation=args.nav2) if args.policy=='homie' else None
    terminal=TerminalCommands(limits=None) if args.interactive else None
    nav2=HomieNavCommands(args.nav2_port,require_feedback=args.feedback=='closed-loop') if args.nav2 else None
    initialize_channel(args.address)
    reader=StateReader()
    velocity_reader=OdomVelocityReader(args.velocity_frame,args.velocity_topic) if args.feedback=='closed-loop' and not args.nav2 else None
    discovery=Discovery() if args.arm else None
    writer=None;released=False;release_attempted=False
    cancelled=threading.Event()
    def stop_signal(*_): cancelled.set()
    signal.signal(signal.SIGINT,stop_signal);signal.signal(signal.SIGTERM,stop_signal)
    status={'started_at':datetime.now(timezone(timedelta(hours=8))).isoformat(),
        'armed_request':args.arm,'phase':'initializing','hardware_command_count':0,
        'release_attempted':False,'original_controller_released':False,'finished':False,
        'sdk_commit':revision,'configuration':cfg.metadata,'policy':args.policy,
        'runtime':{'python':sys.version.split()[0],**(policy.runtime if args.policy=='homie' else
                   {'torch':policy.torch.__version__,'device':'cpu'})},'interface':args.interface,'local_address':args.address,
        'command':cfg.metadata['command'],'frames':0,'inference_ms':[],
        'timing':{'policy_seconds':args.seconds,'remote_wait_seconds':args.wait_seconds,'zero_means_unlimited':args.policy=='homie',
                  'inference_timeout_ms':None if args.policy=='homie' else 50.,
                  'target_timeout_ms':None if args.policy=='homie' else 100.,
                  'inference_input_timeout_ms':None if args.policy=='homie' else 100.,
                  'publisher_gap_timeout_ms':None if args.policy=='homie' else 50.,
                  'lowstate_timeout_ms':100.,'posture_speed_rad_s':args.posture_speed},
        'upper_body_target_rad':cfg.default[12:].tolist(),
        'velocity_input':{'mode':'nav2' if nav2 is not None else 'joystick' if joystick is not None else 'terminal' if args.interactive else 'constant',
                          'nav2_port':args.nav2_port if nav2 is not None else None,
                          'initial_command':fixed_command.tolist(),
                          'joystick':None if joystick is None else {'scale':joystick.scale.tolist(),
                              'deadzone':joystick.deadzone,'axes_order':['lx','ly','rx','ry'],
                              'mapping':['ly','-lx','-rx'],'source':'LowState.wireless_remote'},
                          'feedback':args.feedback,'topic':'/dog_odom' if nav2 and commands.feedback is not None else args.velocity_topic if velocity_reader else None,
                          'frame':'body -> heading' if nav2 else args.velocity_frame,
                          'yaw_feedforward_rad_s':args.yaw_feedforward,
                          'acceleration':commands.feedback.acceleration.tolist() if commands is not None and commands.feedback is not None else None,
                          'smooth_stop':commands.feedback.smooth_stop if commands is not None and commands.feedback is not None else False,
                          'feedback_limit':commands.feedback.limit.tolist() if commands is not None and commands.feedback is not None else None},
        'support_confirmed':args.support_confirmed,'operator_ready':args.operator_ready}
    inference_count=0;inference_sum_ms=0.;inference_max_ms=0.;inference_over_50ms=0
    def inference_stats():
        return {'mean':inference_sum_ms/inference_count if inference_count else None,
                'max':inference_max_ms if inference_count else None,'over_50ms_count':inference_over_50ms}
    def save(phase=None,**extra):
        if phase is not None:
            status['phase']=phase
            print('[student-deploy] '+phase,flush=True)
        status.update(extra)
        status['lowstate_samples']=reader.count;status['crc_invalid']=reader.bad_crc
        status['lowstate_receive']=reader.diagnostics()
        if velocity_reader is not None:
            status['odometry_samples']=velocity_reader.count
            status['odometry_source_clock_available']=velocity_reader.source_clock_available
        status['original_controller_released']=released;status['release_attempted']=release_attempted
        if writer is not None:
            status.update(hardware_command_count=writer.count,damping_commands=writer.damping_count,
                          publish_gap_max_ms=1000*writer.max_gap,publish_intervals_over_4ms=writer.overruns,
                          target_age_max_ms=writer.target_age_max_ms,source_age_max_ms=writer.source_age_max_ms,
                          publisher_fault=writer.fault)
        temporary=args.output/'status.json.partial'
        temporary.write_text(json.dumps(status,indent=2)+'\n');temporary.replace(args.output/'status.json')

    def state():
        if cancelled.is_set(): raise DeployFault('Operator process stop')
        s=reader.get();validate_state(s,time.monotonic(),cfg)
        if velocity_reader is not None:velocity_reader.get()
        if writer is not None: writer.check()
        if nav2 is not None:
            phase=status['phase']
            ready=args.arm and phase not in ('initializing','waiting_lowstate','preflight_shadow','exit_damping','finished')
            nav2.poll(active=phase=='homie_active',ready=ready,phase=phase)
        return s

    def wait_button(mask,label,hold_deadman=False):
        edge=FreshButton(mask);started=time.monotonic()
        last_save=0.
        save(label)
        while within_duration(time.monotonic()-started,args.wait_seconds):
            s=state()
            if hold_deadman and not s.buttons&L1: raise DeployFault('L1 released during posture hold')
            if edge.press(s.buttons) and s.buttons&L1: return s
            if writer is not None and commands is None: writer.update(current_target,s.stamp)
            record_state(s,label)
            if time.monotonic()-last_save>.5:
                save(remote_buttons=s.buttons);last_save=time.monotonic()
            time.sleep(.02)
        raise DeployFault('Remote startup button timeout')

    def mode_rpc(method):
        if nav2 is None:return method()
        # MotionSwitcher waits for an RPC response; keep the navigation link
        # updated throughout ownership transfer, as during posture transition.
        with ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(method)
            while not future.done():
                state()
                time.sleep(.02)
            return future.result()

    exit_code=0
    try:
        save('waiting_lowstate')
        start=time.monotonic()
        while reader.get() is None or (velocity_reader is not None and not velocity_reader.ready()):
            if cancelled.is_set() or time.monotonic()-start>10: raise DeployFault('Required LowState/odometry missing after 10 seconds')
            time.sleep(.02)
        # Healthy preflight includes warm inference, before any motion RPC or LowCmd.
        save('preflight_shadow')
        preflight=time.monotonic()
        current_target=state().q.copy()
        with (args.output/'telemetry.jsonl').open('w') as log:
            last_joystick_print=0.
            def desired_velocity(s,shadow=False):
                nonlocal last_joystick_print
                if nav2 is not None:
                    desired=np.asarray(nav2.command,dtype=float)
                elif joystick is not None:
                    desired=joystick.update(s.wireless_remote)
                    if not args.arm and time.monotonic()-last_joystick_print>=.5:
                        print('[joystick] lx ly rx ry='+str(joystick.axes.round(3).tolist())+
                              ' -> vx vy wz='+str(desired.round(3).tolist()),flush=True)
                        last_joystick_print=time.monotonic()
                else:desired=terminal.poll() if terminal is not None and not (shadow and args.arm) else fixed_command
                return np.zeros(3) if shadow and args.arm else desired

            def record_state(s,phase,**extra):
                log.write(json.dumps({'host_monotonic':time.monotonic(),'phase':phase,'tick':s.tick,
                    'q':s.q.tolist(),'dq':s.dq.tolist(),'tau_est':s.tau.tolist(),
                    'temperature':s.temperature.tolist(),'imu_quaternion':s.quaternion.tolist(),
                    'lowstate_age_ms':1000*(time.monotonic()-s.stamp),
                    'sent_target':None if writer is None or writer.last_target is None else writer.last_target.tolist(),
                    **extra})+'\n')
                status['telemetry_frames']=status.get('telemetry_frames',0)+1
            def infer_record(s,phase,desired=None):
                nonlocal inference_count,inference_sum_ms,inference_max_ms,inference_over_50ms
                command_fields={}
                t0=time.monotonic()
                if commands is not None:
                    measured=(nav2.measured_velocity() if nav2 is not None and commands.feedback is not None else
                              velocity_reader.get() if velocity_reader is not None else None)
                    if measured is not None:measured=np.asarray(measured,dtype=float)
                    if nav2 is not None and (not nav2.enabled or nav2.stop_immediately or (commands.feedback is not None and measured is None)):
                        commands.reset();desired=np.zeros(3)
                    adapted=commands.update(np.zeros(3) if desired is None else desired,measured)
                    requested=policy.step(s,[*adapted,cfg.height])
                    command_fields={'desired_velocity':commands.desired.tolist(),
                        'measured_velocity':None if measured is None else measured.tolist(),
                        'command_dt_s':commands.command_dt,
                        'policy_velocity':adapted.tolist()}
                    if joystick is not None:command_fields['joystick_axes']=joystick.axes.tolist()
                else:requested=policy.step(s)
                duration=time.monotonic()-t0
                inference_count+=1;inference_sum_ms+=1000*duration
                inference_max_ms=max(inference_max_ms,1000*duration)
                if duration>.05:
                    inference_over_50ms+=1
                    if commands is None:raise DeployFault('Policy inference exceeded 50ms')
                if commands is None and time.monotonic()-s.stamp>.10: raise DeployFault('Inference input became stale')
                try:
                    limited=bounded_target(requested,current_target,cfg)
                except DeployFault as exc:
                    failure={'phase':phase,'tick':s.tick,'reason':str(exc),
                             'violations':getattr(exc,'violations',[])}
                    status['policy_failure']=failure
                    record_state(s,phase,observation=policy.history[-1].tolist(),raw_action=policy.action.tolist(),
                                 requested_target=requested.tolist(),limited_target=None,
                                 inference_ms=1000*duration,guard_failure=failure,**command_fields)
                    raise
                record_state(s,phase,observation=policy.history[-1].tolist(),raw_action=policy.action.tolist(),
                             requested_target=requested.tolist(),limited_target=limited.tolist(),
                             inference_ms=1000*duration,**command_fields)
                status['frames']+=1
                return limited
            duration=2. if args.arm else args.seconds
            deadline=time.monotonic()
            while within_duration(time.monotonic()-preflight,duration):
                s=state();desired=desired_velocity(s,shadow=True)
                if terminal is not None and terminal.quit:raise DeployFault('Operator terminal exit')
                current_target=infer_record(s,'shadow',desired)
                if discovery is not None and discovery.count_lowcmd():
                    raise DeployFault('Another rt/lowcmd publisher exists; refusing takeover')
                deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))
            save('shadow_passed',inference_ms=inference_stats())
            if not args.arm:
                status['result']='read-only live inference passed'
            else:
                if nav2 is not None:
                    save('waiting_nav2_enable')
                    while not nav2.requested:
                        s=state()
                        record_state(s,'waiting_nav2_enable')
                        time.sleep(.02)
                else:
                    print('[student-deploy] Release and press L1+Start. Keep L1 held through posture transition and policy. Select stops.',flush=True)
                    wait_button(START,'waiting_L1_Start')
                if discovery.count_lowcmd(): raise DeployFault('Competing LowCmd publisher before takeover')
                save('checking_original_mode')
                def create_switcher():
                    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
                    # ClientBase constructs DDS channels (0.2s) and ClientStub
                    # waits another 0.5s, before the first RPC is possible.
                    switcher=MotionSwitcherClient();switcher.SetTimeout(1.);switcher.Init()
                    return switcher
                switcher=mode_rpc(create_switcher)
                code,mode=mode_rpc(switcher.CheckMode)
                if code!=0 or not isinstance(mode,dict) or 'name' not in mode:
                    raise DeployFault(f'MotionSwitcher CheckMode failed: {code}, {mode}')
                status['original_mode']=mode
                s=state()
                if nav2 is None and not s.buttons&L1: raise DeployFault('L1 released before control ownership change')
                if mode['name']:
                    release_attempted=True;save('releasing_original_mode')
                    code,_=mode_rpc(switcher.ReleaseMode)
                    if code!=0: raise DeployFault(f'MotionSwitcher ReleaseMode failed: {code}')
                code,mode=mode_rpc(switcher.CheckMode)
                if code!=0 or not isinstance(mode,dict) or mode.get('name')!='':
                    raise DeployFault(f'Original controller not released: {code}, {mode}')
                released=True
                s=state()
                if nav2 is None and not s.buttons&L1: raise DeployFault('L1 released before LowCmd startup')
                writer=mode_rpc(lambda:MotorWriter(reader,cfg))
                s=state();start_q=s.q.copy();current_target=start_q.copy()
                if commands is not None:
                    plan=writer.move_to_posture(start_q,cfg.default,args.posture_speed)
                    transition_seconds=plan.duration
                else:
                    plan=None;transition_seconds=4.
                    writer.update(current_target,s.stamp)
                writer.thread.start()
                save('moving_to_default' if commands is not None else 'moving_to_default_4s',
                     posture_transition_seconds=transition_seconds)
                begin=time.monotonic();deadline=begin
                while time.monotonic()-begin<transition_seconds:
                    s=state()
                    if plan is not None:current_target=plan.at(time.monotonic()-begin)
                    else:
                        ratio=min(1.,(time.monotonic()-begin)/transition_seconds)
                        current_target=bounded_target(start_q*(1-ratio)+cfg.default*ratio,current_target,cfg)
                        writer.update(current_target,s.stamp)
                    record_state(s,'moving_to_default',limited_target=current_target.tolist())
                    if discovery.count_lowcmd()>1: raise DeployFault('Competing LowCmd publisher during posture transition')
                    deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))
                if commands is not None:
                    s=state();current_target=bounded_target(cfg.default,current_target,cfg)
                    writer.hold_posture(current_target)
                    record_state(s,'default_held',limited_target=current_target.tolist())
                if nav2 is None:
                    print('[student-deploy] Default posture held. Release then press A while holding L1 to run '
                          +('OpenHomie with the selected velocity input.' if commands is not None else 'ZERO-speed Student for a bounded duration.'),flush=True)
                    wait_button(A,'waiting_L1_A',hold_deadman=True)
                policy.reset()
                if commands is not None:commands.reset()
                if commands is not None:
                    print('[student-deploy] Upper body holds default standing targets: waist yaw and both arms = 0 rad. '
                          'Policy duration: '+('unlimited' if args.seconds==0 else f'{args.seconds:g}s')+'.',flush=True)
                save('homie_active' if commands is not None else 'student_zero_active')
                if nav2 is not None:
                    print('[command] Nav2 input active. enable_motion=false requests zero velocity; '
                          'Select or process exit ends the policy through damping.',flush=True)
                    print('[command] feedback='+args.feedback+', source='+str(status['velocity_input']['topic'])+
                          ', yaw feedforward='+str(args.yaw_feedforward)+' rad/s.',flush=True)
                if joystick is not None:
                    print('[command] Unitree joystick active. Left stick forward/left = vx/vy; '
                          'right stick left = positive yaw. Center sticks requests zero velocity. '
                          'Keep L1 held; Select or release L1 exits through damping. '
                          'Full-stick speeds: '+str(joystick.scale.tolist())+
                          ', center deadzone: '+str(joystick.deadzone)+'.',flush=True)
                if terminal is not None:
                    print('[command] Enter vx vy wz in THIS terminal, then Enter. '
                          'Example: 0.1 0 0 | 0 0 0.15 | stop | quit. Command persists until changed. '
                          'stop requests zero velocity; quit exits through damping.',flush=True)
                begin=time.monotonic();deadline=begin
                while within_duration(time.monotonic()-begin,args.seconds):
                    s=state();desired=desired_velocity(s)
                    if terminal is not None and terminal.quit:break
                    current_target=infer_record(s,'homie_active' if commands is not None else 'student_zero',desired)
                    writer.update(current_target,s.stamp)
                    if discovery.count_lowcmd()>1: raise DeployFault('Competing LowCmd publisher during policy')
                    deadline+=.02;time.sleep(max(0,deadline-time.monotonic()))
                status['result']=('operator terminal exit' if terminal is not None and terminal.quit else
                    'bounded OpenHomie policy completed' if commands is not None else 'bounded suspended zero-speed policy completed')
    except Exception as exc:
        exit_code=1;status['result']='stopped';status['reason']=str(exc)
        status['lowstate_receive_at_stop']=reader.diagnostics()
        print('[student-deploy] STOP: '+str(exc),flush=True)
    finally:
        if writer is not None:
            save('exit_damping')
            writer.close()
            if writer.fault and 'reason' not in status:
                exit_code=1;status['result']='stopped';status['reason']=writer.fault
        status['finished']=True
        status['inference_ms']=inference_stats()
        save('finished')
        if nav2 is not None:
            nav2.poll(active=False,ready=False,phase='finished')
            nav2.close()
        reader.close()
        if velocity_reader is not None:velocity_reader.subscriber.Close()
        if pid_file.exists() and pid_file.read_text().strip()==str(os.getpid()): pid_file.unlink()
        lock_file.close()
        if (released or release_attempted) and nav2 is not None:
            print('[student-deploy] OpenHomie exited through damping. Restore factory control with the official recovery procedure.',flush=True)
        elif released or release_attempted:
            print('[student-deploy] Robot remains supported. No automatic restore of the factory controller; use the official recovery procedure after verifying posture.',flush=True)
    return exit_code


if __name__=='__main__':
    raise SystemExit(main())
