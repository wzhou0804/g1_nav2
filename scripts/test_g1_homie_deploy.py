#!/usr/bin/env python3
"""Exercise command/odometry boundaries and the deployment CLI without robot I/O."""
from contextlib import ExitStack
import io
import json
import os
from pathlib import Path
import runpy
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch,MagicMock

import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from navigation.homie import Configuration,Policy,observation
from navigation.homie_deploy import HomieDeployCommand,OdomVelocityReader,yaw_velocity,PostureTransition
from navigation.locomotion_commands import TerminalCommands,UnitreeJoystickCommands
from navigation.student_deploy_core import Snapshot,L1,SELECT,DeployFault,validate_state,snapshot_from_message
from navigation.velocity_feedback import VelocityFeedback
from scripts import deploy_g1_student as deploy


def odom(stamp=1,velocity=(.2,.1,0),yaw=np.pi/2):
    return SimpleNamespace(error_code=0,velocity=velocity,yaw_speed=.3,
        stamp=SimpleNamespace(sec=stamp,nanosec=0),
        imu_state=SimpleNamespace(quaternion=[np.cos(yaw/2),0,0,np.sin(yaw/2)]))


def remote(lx=0.,ly=0.,rx=0.,ry=0.,buttons=L1):
    data=bytearray(40)
    struct.pack_into('<H',data,2,buttons)
    for offset,value in ((4,lx),(8,rx),(12,ry),(20,ly)):
        struct.pack_into('<f',data,offset,value)
    return bytes(data)


class DeployCommandTests(unittest.TestCase):
    def test_joystick_packet_matches_sdk_and_snapshot_preserves_same_frame(self):
        data=remote(lx=-.25,ly=.75,rx=.5,ry=-.125,buttons=L1|SELECT)
        path=ROOT/'.cache-g1/unitree_sdk2_python/example/wireless_controller/wireless_controller.py'
        reference=runpy.run_path(str(path))['unitreeRemoteController']()
        reference.parse_key(data);reference.parse_botton(data[2],data[3])
        commands=UnitreeJoystickCommands();commands.update(data)
        np.testing.assert_array_equal(commands.axes,[reference.Lx,reference.Ly,reference.Rx,reference.Ry])
        self.assertEqual((reference.L1,reference.Select),(1,1))
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        message=unitree_hg_msg_dds__LowState_();message.wireless_remote=list(data)
        s=snapshot_from_message(message,123.)
        self.assertEqual(s.wireless_remote,data);self.assertEqual(s.buttons,L1|SELECT)
        message.wireless_remote[4]=255
        self.assertEqual(s.wireless_remote,data)  # The snapshot owns its immutable bytes.

    def test_joystick_directions_deadzone_and_center_zero(self):
        commands=UnitreeJoystickCommands()
        for data,expected in ((remote(ly=1),[.3,0,0]),(remote(ly=-1),[-.3,0,0]),
                              (remote(lx=-1),[0,.15,0]),(remote(lx=1),[0,-.15,0]),
                              (remote(rx=-1),[0,0,.4]),(remote(rx=1),[0,0,-.4]),
                              (remote(lx=.04,ly=-.04,rx=.04),[0,0,0]),(remote(),[0,0,0])):
            np.testing.assert_allclose(commands.update(data),expected,atol=1e-7)
        np.testing.assert_allclose(commands.update(remote(ly=.54)),[.15,0,0],atol=1e-7)
        # Mapping scales are configurable; no former terminal speed ceiling applies.
        np.testing.assert_allclose(UnitreeJoystickCommands((1.,0.,1.2),0).update(remote(ly=1,lx=-1,rx=-1)),[1,0,1.2])

    def test_joystick_rejects_invalid_packet_and_mapping(self):
        commands=UnitreeJoystickCommands()
        for data in (None,b'',bytes(39),remote(ly=float('nan')),remote(rx=float('inf')),remote(ly=1.1)):
            with self.subTest(data=data),self.assertRaises(ValueError):commands.update(data)
        for scale,deadzone in (((1,2),.08),((1,-1,1),.08),((1,1,float('nan')),.08),
                               ((1,1,1),-1),((1,1,1),1),((1,1,1),float('nan'))):
            with self.subTest(scale=scale,deadzone=deadzone),self.assertRaises(ValueError):
                UnitreeJoystickCommands(scale,deadzone)

    def test_entry_duration_follows_displacement_with_smooth_endpoints(self):
        start=np.zeros(27);end=np.zeros(27);end[0]=.5
        plan=PostureTransition(start,end,.5)
        self.assertAlmostEqual(plan.duration,1.5)
        np.testing.assert_array_equal(plan.at(0),start)
        np.testing.assert_array_equal(plan.at(plan.duration),end)
        steps=np.linspace(0,plan.duration,1001)
        samples=np.array([plan.at(t) for t in steps])
        speed=np.diff(samples[:,0])/np.diff(steps)
        self.assertLessEqual(float(np.max(speed)),.5+1e-10)
        self.assertLess(speed[0],.002);self.assertLess(speed[-1],.002)
        self.assertEqual(PostureTransition(start,start).duration,0)

    def test_static_hold_continues_without_main_loop_refresh(self):
        cfg=Configuration()
        s=Snapshot(0,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),L1)
        def get_state():s.stamp=time.monotonic();return s
        reader=SimpleNamespace(get=get_state,latest=s,lock=threading.Lock())
        writes=[]
        def write(command):
            writes.append((time.monotonic(),command.motor_cmd[12].kp));return True
        publisher=SimpleNamespace(Init=lambda:None,Close=lambda:None,Write=write)
        with patch('unitree_sdk2py.core.channel.ChannelPublisher',return_value=publisher):
            writer=deploy.MotorWriter(reader,cfg)
        try:
            writer.hold_posture(cfg.default);writer.thread.start()
            time.sleep(.20)  # No update(), longer than the dynamic target TTL.
            self.assertIsNone(writer.fault)
            active=[stamp for stamp,kp in writes if kp>0]
            self.assertGreater(active[-1]-active[0],.15)
            # Dynamic targets no longer expire, while explicit stops still work.
            requested=cfg.default.copy();requested[0]+=.01
            writer.update(requested,time.monotonic()-.2)
            time.sleep(.20)
            self.assertIsNone(writer.fault)
            np.testing.assert_array_equal(writer.last_target,requested)
            self.assertGreater(writer.target_age_max_ms,100)
            self.assertGreater(writer.source_age_max_ms,200)
            s.buttons=0
            writer.thread.join(2)
            self.assertIn('L1 deadman released',writer.fault)
        finally:writer.close()

    def test_scheduling_gap_is_recorded_without_custom_exit(self):
        cfg=Configuration()
        s=Snapshot(0,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),L1)
        def get_state():s.stamp=time.monotonic();return s
        reader=SimpleNamespace(get=get_state,latest=s,lock=threading.Lock())
        writes=[];resumed=threading.Event()
        def write(command):
            writes.append(command.motor_cmd[12].kp)
            if len(writes)==1:time.sleep(.07)
            elif len(writes)==2:resumed.set()
            return True
        publisher=SimpleNamespace(Init=lambda:None,Close=lambda:None,Write=write)
        with patch('unitree_sdk2py.core.channel.ChannelPublisher',return_value=publisher):
            writer=deploy.MotorWriter(reader,cfg)
        try:
            writer.update(cfg.default,time.monotonic());writer.thread.start()
            self.assertTrue(resumed.wait(2))
            self.assertIsNone(writer.fault);self.assertGreater(writer.max_gap,.05)
            self.assertGreater(writes[1],0)
        finally:writer.close()

    def test_nav2_writer_runs_without_l1_and_select_still_stops(self):
        cfg=Configuration();cfg.require_l1=False
        s=Snapshot(0,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),0)
        def get_state():s.stamp=time.monotonic();return s
        reader=SimpleNamespace(get=get_state,latest=s,lock=threading.Lock())
        sent=threading.Event()
        publisher=SimpleNamespace(Init=lambda:None,Close=lambda:None,
            Write=lambda c:sent.set() or True)
        with patch('unitree_sdk2py.core.channel.ChannelPublisher',return_value=publisher):
            writer=deploy.MotorWriter(reader,cfg)
        try:
            writer.hold_posture(cfg.default);writer.thread.start()
            self.assertTrue(sent.wait(2));self.assertIsNone(writer.fault)
            self.assertIsNotNone(writer.last_target)
            s.buttons=SELECT;writer.thread.join(2)
            self.assertIn('Select stop',writer.fault)
        finally:writer.close()

    def test_custom_state_thresholds_removed_but_invalid_data_and_stop_rejected(self):
        cfg=Configuration()
        q=2*np.array([np.cos(.45),np.sin(.45),0,0])
        s=Snapshot(time.monotonic(),0,6,cfg.upper+.2,2*cfg.velocity,np.zeros(27),np.full(27,90.),
            np.zeros(27),np.zeros(27,int),q,np.zeros(3),L1)
        validate_state(s,time.monotonic(),cfg)
        from navigation.student_deploy_core import gravity
        np.testing.assert_allclose(observation(s,np.zeros(12),cfg)[7:10],gravity(q/np.linalg.norm(q)),atol=1e-7)
        s.motor_flags[0]=1
        with self.assertRaisesRegex(DeployFault,'motor fault'):validate_state(s,time.monotonic(),cfg)
        s.motor_flags[0]=0;s.buttons|=SELECT
        with self.assertRaisesRegex(DeployFault,'Select stop'):validate_state(s,time.monotonic(),cfg)
        s.buttons=L1;s.quaternion[:]=0
        with self.assertRaisesRegex(DeployFault,'quaternion'):validate_state(s,time.monotonic(),cfg)
        s.quaternion[:]=[1,0,0,0];s.q[0]=np.nan
        with self.assertRaisesRegex(DeployFault,'Nonfinite'):validate_state(s,time.monotonic(),cfg)

    def test_static_hold_still_rejects_stale_lowstate(self):
        cfg=Configuration()
        s=Snapshot(time.monotonic()-.2,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),L1)
        reader=SimpleNamespace(get=lambda:s,latest=s,lock=threading.Lock())
        stiffness=[]
        publisher=SimpleNamespace(Init=lambda:None,Close=lambda:None,
            Write=lambda c:stiffness.append(c.motor_cmd[12].kp) or True)
        with patch('unitree_sdk2py.core.channel.ChannelPublisher',return_value=publisher):
            writer=deploy.MotorWriter(reader,cfg)
        try:
            writer.hold_posture(cfg.default);writer.thread.start();writer.thread.join(2)
            self.assertIn('LowState stale',writer.fault)
            self.assertTrue(stiffness);self.assertTrue(all(kp==0 for kp in stiffness))
        finally:writer.close()

    def test_unlimited_defaults_and_optional_durations(self):
        self.assertEqual(deploy.deployment_durations('homie'),(0,0))
        self.assertEqual(deploy.deployment_durations('homie',3600,7200),(3600,7200))
        self.assertTrue(deploy.within_duration(1e9,0))
        self.assertFalse(deploy.within_duration(60,60))
        self.assertEqual(deploy.deployment_durations('htd'),(10,180))
        for value in (-1,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                deploy.deployment_durations('homie',value,0)
            with self.subTest(wait=value),self.assertRaises(ValueError):
                deploy.deployment_durations('homie',0,value)

    def test_upper_body_stays_at_default_for_all_velocity_commands(self):
        from navigation.student_deploy_core import fill_command,ACTIVE_SLOTS
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        cfg=Configuration();policy=Policy(cfg)
        s=Snapshot(0,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),L1)
        s.q[12:]+=.1
        for velocity in ([0,0,0],[.3,0,0],[0,0,.4],[0,0,-.4],[0,0,0]):
            target=policy.step(s,[*velocity,cfg.height])
            np.testing.assert_array_equal(target[12:],cfg.default[12:])
            message=fill_command(unitree_hg_msg_dds__LowCmd_(),target,cfg)
            for joint,slot in enumerate(ACTIVE_SLOTS[12:],12):
                motor=message.motor_cmd[slot]
                self.assertEqual(motor.mode,1)
                self.assertEqual(motor.q,cfg.default[joint])
                self.assertEqual(motor.kp,cfg.kp[joint])
                self.assertEqual(motor.kd,cfg.kd[joint])

    def test_pasted_stop_is_not_hidden_in_stdin_buffer(self):
        read_fd,write_fd=os.pipe()
        try:
            os.write(write_fd,b'0.3 0 0\nstop\n')
            with os.fdopen(read_fd,'r') as stream,patch('sys.stdin',stream):
                commands=TerminalCommands()
                np.testing.assert_array_equal(commands.poll(),0)
        finally:os.close(write_fd)

    def test_deploy_commands_have_no_custom_velocity_ceiling(self):
        commands=TerminalCommands(limits=None)
        np.testing.assert_array_equal(commands.accept('1 .5 1.2'),[1,.5,1.2])
        np.testing.assert_array_equal(HomieDeployCommand().update(commands.command),[1,.5,1.2])
        with self.assertRaises(ValueError):commands.accept('nan 0 0')
        # MuJoCo's existing interactive defaults are independent of this change.
        with self.assertRaises(ValueError):TerminalCommands().accept('1 .5 1.2')

    def test_feedback_matches_simulation_and_zero_clears_integrator(self):
        actual=HomieDeployCommand(True);reference=VelocityFeedback()
        reference.limit=np.array([.8,.5,.8])
        for desired in ([.1,0,.15],)*100+([0,0,-.4],)*100+([0,0,0],):
            measured=[.02,-.01,.03]
            np.testing.assert_array_equal(actual.update(desired,measured),reference.update(desired,measured))
        np.testing.assert_array_equal(actual.command,0)
        np.testing.assert_array_equal(actual.feedback.integral,0)
        with self.assertRaises(DeployFault):actual.update([.1,0,0])
        with self.assertRaises(DeployFault):actual.update([.1,0,0],[float('nan'),0,0])
        np.testing.assert_array_equal(HomieDeployCommand().update([.3,0,.4]),[.3,0,.4])

    def test_yaw_feedforward_preserves_zero_stop_direction_and_output_limit(self):
        commands=HomieDeployCommand(True)
        commands.feedback.feedforward[2]=.4
        np.testing.assert_array_equal(commands.update([0,0,0],[0,0,0]),0)
        for _ in range(100):command=commands.update([0,0,.25],[0,0,0])
        self.assertGreater(command[2],.4)
        self.assertLessEqual(command[2],.8)
        np.testing.assert_array_equal(command[:2],0)
        np.testing.assert_array_equal(commands.desired,[0,0,.25])
        np.testing.assert_array_equal(commands.update([0,0,0],[0,0,.1]),0)
        np.testing.assert_array_equal(commands.feedback.integral,0)
        self.assertLess(commands.update([0,0,-.05],[0,0,0])[2],0)

    def test_velocity_frame_conversion_and_invalid_orientation(self):
        m=odom()
        np.testing.assert_allclose(yaw_velocity(m,'world'),[.1,-.2,.3],atol=1e-12)
        np.testing.assert_array_equal(yaw_velocity(m,'yaw'),[.2,.1,.3])
        m.imu_state.quaternion=[0,0,0,0]
        with self.assertRaises(DeployFault):yaw_velocity(m,'world')
        m=odom();m.velocity=[float('nan'),0,0]
        with self.assertRaises(DeployFault):yaw_velocity(m,'yaw')

    def reader(self):
        with patch('unitree_sdk2py.core.channel.ChannelSubscriber'):
            return OdomVelocityReader('yaw')

    def test_duplicate_odometry_does_not_refresh_watchdog(self):
        r=self.reader();r.receive(odom())
        r.latest=(time.monotonic()-.2,r.latest[1]);r.receive(odom())
        self.assertEqual(r.count,1)
        with self.assertRaisesRegex(DeployFault,'stale'):r.get()
        r.receive(odom(2));np.testing.assert_allclose(r.get(),[.2,.1,.3])
        r.receive(odom(1))
        with self.assertRaisesRegex(DeployFault,'clock reset'):r.get()

    def test_zero_stamp_receipt_mode_and_source_fault(self):
        r=self.reader();r.receive(odom(0));self.assertFalse(r.source_clock_available)
        np.testing.assert_allclose(r.get(),[.2,.1,.3])
        r.receive(odom(1));self.assertTrue(r.source_clock_available)
        r.receive(odom(0))
        with self.assertRaisesRegex(DeployFault,'clock disappeared'):r.get()
        r=self.reader();m=odom();m.error_code=1;r.receive(m)
        with self.assertRaisesRegex(DeployFault,'error_code'):r.get()

    def test_cli_rejects_missing_frame_and_htd_velocity_without_io(self):
        for argv in (['--policy','homie','--feedback','closed-loop'],
                     ['--command','.1','0','0'],['--policy','homie','--command','nan','0','0'],
                     ['--joystick'],['--policy','homie','--joystick','--interactive'],
                     ['--nav2'],['--policy','homie','--nav2','--joystick'],
                     ['--policy','homie','--nav2','--interactive'],
                     ['--policy','homie','--nav2','--command','.1','0','0'],
                     ['--policy','homie','--joystick','--command','.1','0','0'],
                     ['--policy','homie','--joystick-scale','.3','.15','.4'],
                     ['--policy','homie','--joystick','--joystick-deadzone','1']):
            with self.subTest(argv=argv),patch.object(sys,'argv',['deploy',*argv]),\
                 patch.object(deploy,'initialize_channel') as initialize,\
                 patch('sys.stderr',new=io.StringIO()),self.assertRaises(SystemExit) as raised:
                deploy.main()
            self.assertEqual(raised.exception.code,2);initialize.assert_not_called()

    def run_mock_cli(self,armed=False,interactive=False,policy_delay=0.,joystick=False,nav2=False,nav2_feedback=False):
        cfg=Configuration()
        s=Snapshot(0,0,6,cfg.default.copy(),np.zeros(27),np.zeros(27),np.full(27,30.),
            np.full(27,48.),np.zeros(27,int),np.array([1.,0,0,0]),np.zeros(3),L1)
        s.wireless_remote=remote(ly=1,lx=-1,rx=-1,ry=.2)
        if nav2:s.buttons=0;s.wireless_remote=remote(buttons=0)
        def get_state():s.stamp=time.monotonic();s.tick+=1;return s
        reader=SimpleNamespace(get=get_state,count=1,bad_crc=0,subscriber=MagicMock(),close=MagicMock(),diagnostics=lambda:{})
        velocity=SimpleNamespace(get=lambda:np.zeros(3),ready=lambda:True,count=1,
            source_clock_available=False,subscriber=MagicMock())
        writer=MagicMock(count=0,damping_count=0,max_gap=0.,overruns=0,fault=None,last_target=None,
            target_age_max_ms=0.,source_age_max_ms=0.)
        def update(target,stamp):
            writer.last_target=target.copy()
            if joystick:s.wireless_remote=remote()
        writer.update.side_effect=update
        writer.move_to_posture.side_effect=lambda start,target,speed:PostureTransition(start,target,speed)
        writer.hold_posture.side_effect=lambda target:setattr(writer,'last_target',target.copy())
        switcher=MagicMock()
        switcher.CheckMode.side_effect=[(0,{'name':'factory'}),(0,{'name':''})]
        switcher.ReleaseMode.return_value=(0,{})
        constructing_writer=threading.Event()
        def construct_writer(*args):
            if nav2:
                constructing_writer.set();time.sleep(.7);constructing_writer.clear()
            return writer
        with tempfile.TemporaryDirectory() as temp,ExitStack() as stack:
            output=Path(temp)/'output'
            argv=['deploy','--policy','homie','--interface','lo','--address','127.0.0.1',
                '--output',str(output)]
            if not (armed and interactive):argv+=['--seconds','.08']
            if nav2:
                argv+=['--nav2']
                if nav2_feedback:argv+=['--feedback','closed-loop','--yaw-feedforward','.4']
                class FakeNav2:
                    command=(0.,0.,0.)
                    requested=False
                    enabled=False
                    stop_immediately=False
                    release_polls=0
                    constructor_polls=0
                    writer_polls=0
                    last_poll=None
                    maximum_poll_gap=0.
                    def __init__(self,port,require_feedback=False):pass
                    def measured_velocity(self):return (0.,0.,0.)
                    def poll(self,active=False,ready=False,phase='waiting'):
                        now=time.monotonic()
                        if self.last_poll is not None:
                            FakeNav2.maximum_poll_gap=max(FakeNav2.maximum_poll_gap,now-self.last_poll)
                        self.last_poll=now
                        self.requested=ready
                        self.enabled=active and ready
                        self.command=((0.,0.,.05) if nav2_feedback else (.1,0.,0.)) if active else (0.,0.,0.)
                        if phase=='releasing_original_mode':FakeNav2.release_polls+=1
                        if phase=='checking_original_mode':FakeNav2.constructor_polls+=1
                        if constructing_writer.is_set():FakeNav2.writer_polls+=1
                    def close(self):pass
                stack.enter_context(patch.object(deploy,'HomieNavCommands',FakeNav2))
                switcher.ReleaseMode.side_effect=lambda:(time.sleep(.7) or (0,{}))
                stack.enter_context(patch.object(TerminalCommands,'poll',side_effect=AssertionError('Terminal input in Nav2 mode')))
            elif joystick:
                argv+=['--joystick']
                stack.enter_context(patch.object(sys.stdin,'isatty',return_value=False))
                stack.enter_context(patch.object(TerminalCommands,'poll',side_effect=AssertionError('Terminal input in joystick mode')))
            elif interactive:
                argv+=['--interactive']
                class FakeTerminal(TerminalCommands):
                    def __init__(self,*args,**kwargs):
                        super().__init__(*args,**kwargs);self.inputs=iter(['.1 0 0','stop','quit'])
                    def poll(self):return self.accept(next(self.inputs))
                stack.enter_context(patch.object(deploy,'TerminalCommands',FakeTerminal))
                stack.enter_context(patch.object(sys.stdin,'isatty',return_value=True))
            else:argv+=['--command','.1','0','0']
            if armed:
                argv+=['--arm']
                if not nav2:argv+=['--support-confirmed','--operator-ready',
                    '--feedback','closed-loop','--velocity-frame','yaw']
            def check_output(args,**kwargs):
                return ('[{"addr_info":[{"local":"127.0.0.1"}]}]' if args[0]=='ip' else
                    '814556d15970dd2ecf1c9984e845ca02ab07e206')
            for target,value in [('ROOT',Path(temp)),('StateReader',lambda:reader),
                                 ('initialize_channel',MagicMock()),('OdomVelocityReader',lambda *args:velocity),
                                 ('MotorWriter',construct_writer),('FreshButton',lambda *args:SimpleNamespace(press=lambda _:True)),
                                 ('Discovery',lambda:SimpleNamespace(count_lowcmd=lambda:0))]:
                stack.enter_context(patch.object(deploy,target,value))
            if nav2:
                stack.enter_context(patch.object(deploy,'OdomVelocityReader',side_effect=AssertionError('Extra DDS odometry reader in Nav2 mode')))
            if nav2:
                stack.enter_context(patch.object(deploy,'FreshButton',side_effect=AssertionError('Button handshake in Nav2 mode')))
            stack.enter_context(patch.object(sys,'argv',argv))
            stack.enter_context(patch.object(deploy.subprocess,'check_output',side_effect=check_output))
            stack.enter_context(patch.object(deploy.signal,'signal'))
            stack.enter_context(patch('navigation.homie.Configuration',return_value=cfg))
            if policy_delay:
                class SlowPolicy(Policy):
                    def step(self,*args,**kwargs):
                        time.sleep(policy_delay);return super().step(*args,**kwargs)
                stack.enter_context(patch('navigation.homie.Policy',SlowPolicy))
            def construct_switcher():
                if nav2:time.sleep(.7)  # Actual SDK: writer initialization + ClientStub wait.
                return switcher
            stack.enter_context(patch('unitree_sdk2py.comm.motion_switcher.motion_switcher_client.MotionSwitcherClient',side_effect=construct_switcher))
            # Any accidentally introduced SDK channel is a test failure.
            stack.enter_context(patch('unitree_sdk2py.core.channel.ChannelPublisher',side_effect=AssertionError('Robot publisher in offline test')))
            stack.enter_context(patch('unitree_sdk2py.core.channel.ChannelSubscriber',side_effect=AssertionError('Robot subscriber in offline test')))
            self.assertEqual(deploy.main(),0)
            if nav2:
                self.assertGreater(FakeNav2.release_polls,10)
                self.assertGreater(FakeNav2.constructor_polls,10)
                self.assertGreater(FakeNav2.writer_polls,10)
                self.assertLess(FakeNav2.maximum_poll_gap,.6)
            status=json.loads((output/'status.json').read_text())
            rows=[json.loads(line) for line in (output/'telemetry.jsonl').read_text().splitlines()]
        return status,rows,writer,switcher

    def test_nav2_service_replaces_manual_handshake_with_same_policy_and_posture(self):
        status,rows,writer,switcher=self.run_mock_cli(armed=True,nav2=True)
        self.assertEqual(status['velocity_input']['mode'],'nav2')
        self.assertEqual(status['velocity_input']['feedback'],'raw')
        self.assertFalse(status['support_confirmed']);self.assertFalse(status['operator_ready'])
        self.assertEqual(status['timing']['remote_wait_seconds'],0)
        self.assertFalse(any('waiting_L1' in r['phase'] for r in rows))
        shadow=[r for r in rows if r['phase']=='shadow']
        active=[r for r in rows if r['phase']=='homie_active']
        self.assertTrue(shadow and active)
        for row in shadow:np.testing.assert_array_equal(row['policy_velocity'],0)
        for row in active:
            np.testing.assert_array_equal(row['desired_velocity'],[.1,0,0])
            np.testing.assert_allclose(row['observation'][:4],[.2,0,0,.74],atol=1e-7)
            np.testing.assert_array_equal(row['limited_target'][12:],np.zeros(15))
        switcher.ReleaseMode.assert_called_once()
        writer.move_to_posture.assert_called_once();writer.close.assert_called_once()

    def test_nav2_feedback_reaches_real_onnx_without_extra_sdk_velocity_subscription(self):
        status,rows,writer,switcher=self.run_mock_cli(armed=True,nav2=True,nav2_feedback=True)
        self.assertEqual(status['velocity_input']['topic'],'/dog_odom')
        self.assertEqual(status['velocity_input']['yaw_feedforward_rad_s'],.4)
        self.assertEqual(status['velocity_input']['feedback_limit'],[.6,.6,.6])
        active=[r for r in rows if r['phase']=='homie_active']
        self.assertGreater(len(active),1)
        self.assertAlmostEqual(active[0]['policy_velocity'][2],.008)
        for before,after in zip(active,active[1:]):
            self.assertLessEqual(abs(after['policy_velocity'][2]-before['policy_velocity'][2]),.4*after['command_dt_s']+1e-12)
        for row in active:
            np.testing.assert_array_equal(row['desired_velocity'],[0,0,.05])
            np.testing.assert_allclose(row['observation'][:3],np.array(row['policy_velocity'])*[2,2,.25],atol=1e-7)
            np.testing.assert_array_equal(row['limited_target'][12:],np.zeros(15))

    def test_navigation_profile_matches_tested_mujoco_controller_and_zero_resets(self):
        control=HomieDeployCommand(True,yaw_feedforward=.4,limits=(.6,.6,.6),navigation=True)
        reference=VelocityFeedback();reference.limit=np.array([.6,.6,.6]);reference.feedforward[2]=.4
        reference.acceleration=np.array([.2,.15,.4]);reference.smooth_stop=True
        rng=np.random.default_rng(42)
        tick=0
        for desired in ([.05,0,0],[0,0,.05],[0,0,-.05],[.6,.6,.6],[-.6,-.6,-.6],[0,0,0]):
            for _ in range(60):
                measured=rng.normal(0,.03,3)
                np.testing.assert_allclose(control.update(desired,measured,now=tick*.02),reference.update(desired,measured),atol=1e-12)
                tick+=1
        np.testing.assert_array_equal(control.update([0,0,0],None),np.zeros(3))
        np.testing.assert_array_equal(control.feedback.integral,np.zeros(3))

    def test_navigation_slew_stopping_and_reversal_with_irregular_steps(self):
        control=HomieDeployCommand(True,yaw_feedforward=.4,limits=(.6,.6,.6),navigation=True)
        now=0.;previous=np.zeros(3)
        for desired in ([.6,.6,.6],[0,0,0],[-.6,-.6,-.6],[0,0,0]):
            for i in range(400):
                dt=(.013,.027,.02)[i%3];now+=dt
                command=control.update(desired,[0,0,0],now=now)
                allowed=np.array([.2,.15,.4])*control.command_dt
                self.assertTrue(np.all(np.abs(command-previous)<=allowed+1e-12))
                self.assertTrue(np.all(np.abs(command)<=.6+1e-12))
                if not np.any(desired):
                    self.assertTrue(np.all(np.abs(command)<=np.abs(previous)+1e-12))
                previous=command
            np.testing.assert_allclose(command,desired,atol=1e-12)
        # Disarming/invalid feedback resets the output rather than slewing a stale command.
        control.update([.6,.6,.6],[0,0,0],now=now+.02)
        control.reset()
        np.testing.assert_array_equal(control.command,[0,0,0])

    def test_readonly_joystick_cli_maps_current_remote_to_real_onnx_without_tty(self):
        status,rows,writer,switcher=self.run_mock_cli(joystick=True)
        self.assertEqual(status['velocity_input']['mode'],'joystick')
        self.assertEqual(status['velocity_input']['joystick']['scale'],[.3,.15,.4])
        self.assertEqual(status['hardware_command_count'],0)
        writer.update.assert_not_called();switcher.CheckMode.assert_not_called()
        for row in rows:
            np.testing.assert_allclose(row['desired_velocity'],[.3,.15,.4])
            np.testing.assert_allclose(row['observation'][:4],[.6,.3,.1,.74],atol=1e-7)
            np.testing.assert_allclose(row['joystick_axes'],[-1,1,-1,.2])

    def test_mock_armed_joystick_preflight_zero_and_recenter_zero(self):
        status,rows,writer,switcher=self.run_mock_cli(armed=True,joystick=True)
        shadow=[r for r in rows if r['phase']=='shadow']
        active=[r for r in rows if r['phase']=='homie_active']
        self.assertTrue(shadow);self.assertGreater(len(active),1)
        for row in shadow:np.testing.assert_array_equal(row['desired_velocity'],0)
        np.testing.assert_allclose(active[0]['desired_velocity'],[.3,.15,.4])
        np.testing.assert_array_equal(active[1]['desired_velocity'],0)
        np.testing.assert_array_equal(active[1]['policy_velocity'],0)
        self.assertEqual(status['velocity_input']['mode'],'joystick')
        switcher.ReleaseMode.assert_called_once();writer.close.assert_called_once()

    def test_readonly_cli_passes_requested_velocity_to_real_onnx(self):
        status,rows,writer,switcher=self.run_mock_cli()
        self.assertEqual(status['hardware_command_count'],0)
        self.assertFalse(status['release_attempted']);switcher.CheckMode.assert_not_called()
        writer.update.assert_not_called()
        self.assertTrue(rows)
        for row in rows:
            np.testing.assert_array_equal(row['desired_velocity'],[.1,0,0])
            np.testing.assert_allclose(row['observation'][:4],[.2,0,0,.74],atol=1e-7)

    def test_homie_inference_over_100ms_is_recorded_without_timeout_stop(self):
        status,rows,writer,switcher=self.run_mock_cli(policy_delay=.12)
        self.assertTrue(status['finished'])
        self.assertEqual(status['result'],'read-only live inference passed')
        self.assertIsNone(status['timing']['inference_timeout_ms'])
        for name in ('target_timeout_ms','inference_input_timeout_ms','publisher_gap_timeout_ms'):
            self.assertIsNone(status['timing'][name])
        self.assertEqual(status['timing']['lowstate_timeout_ms'],100)
        self.assertGreater(status['inference_ms']['max'],100)
        self.assertGreater(status['inference_ms']['over_50ms_count'],0)
        self.assertTrue(any(r['inference_ms']>50 for r in rows))

    def test_mock_armed_cli_keeps_preflight_zero_and_adapts_active_command(self):
        status,rows,writer,switcher=self.run_mock_cli(True,interactive=True)
        shadow=[r for r in rows if r['phase']=='shadow']
        active=[r for r in rows if r['phase']=='homie_active']
        self.assertTrue(shadow and active)
        for r in shadow:np.testing.assert_array_equal(r['policy_velocity'],0)
        self.assertEqual(active[0]['desired_velocity'],[.1,0,0])
        self.assertAlmostEqual(active[0]['policy_velocity'][0],.016)
        np.testing.assert_allclose(active[0]['observation'][:4],[.032,0,0,.74],atol=1e-7)
        np.testing.assert_array_equal(active[1]['policy_velocity'],0)
        self.assertEqual(status['result'],'operator terminal exit')
        self.assertEqual(status['timing']['policy_seconds'],0)
        self.assertEqual(status['timing']['remote_wait_seconds'],0)
        self.assertEqual(status['posture_transition_seconds'],0)
        writer.move_to_posture.assert_called_once()
        writer.hold_posture.assert_called_once()
        held=[r for r in rows if r['phase']=='default_held']
        self.assertTrue(held)
        np.testing.assert_array_equal(held[0]['limited_target'][12:],np.zeros(15))
        for row in active:
            np.testing.assert_array_equal(row['requested_target'][12:],np.zeros(15))
            np.testing.assert_array_equal(row['limited_target'][12:],np.zeros(15))
        self.assertTrue(status['original_controller_released'])
        switcher.ReleaseMode.assert_called_once();writer.close.assert_called_once()


if __name__=='__main__':unittest.main(verbosity=2)
