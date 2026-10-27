"""Transport-free mapping and guards for suspended G1 mode_machine=6 tests."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

CHECKPOINT_SHA = 'c09f87a81a49f1e8ca4826f0db15251973e63222c685c7ea3d2fcac7cdffcc35'
ACTIVE_SLOTS = tuple(range(13)) + tuple(range(15,29))
LOWER_NAMES = [f'{side}_{joint}_joint' for side in ('left','right') for joint in
               ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')] + ['waist_yaw_joint']
ARM_NAMES = [f'{side}_{joint}_joint' for side in ('left','right') for joint in
             ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]
JOINT_NAMES = LOWER_NAMES + ARM_NAMES
L1, START, SELECT, A = 1 << 1, 1 << 2, 1 << 3, 1 << 8


class DeployFault(RuntimeError):
    pass


class TargetRangeFault(DeployFault):
    def __init__(self, requested, cfg, indices):
        self.violations = [dict(joint=JOINT_NAMES[j], motor_slot=ACTIVE_SLOTS[j],
            target_rad=float(requested[j]), lower_rad=float(cfg.lower[j]),
            upper_rad=float(cfg.upper[j])) for j in indices]
        detail = '; '.join(f"{v['joint']} (slot {v['motor_slot']}): target={v['target_rad']:.4f} rad, "
            f"official=[{v['lower_rad']:.4f}, {v['upper_rad']:.4f}] rad" for v in self.violations)
        super().__init__('Policy target grossly outside official joint range: '+detail)


@dataclass
class Snapshot:
    stamp: float
    tick: int
    machine: int
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    temperature: np.ndarray
    voltage: np.ndarray
    motor_flags: np.ndarray
    quaternion: np.ndarray
    gyro: np.ndarray
    buttons: int
    wireless_remote: bytes | None = None


def snapshot_from_message(message, now):
    motors = [message.motor_state[i] for i in ACTIVE_SLOTS]
    remote=bytes(message.wireless_remote)
    return Snapshot(now,int(message.tick),int(message.mode_machine),
        np.array([m.q for m in motors],dtype=np.float64),
        np.array([m.dq for m in motors],dtype=np.float64),
        np.array([m.tau_est for m in motors],dtype=np.float64),
        np.array([max(m.temperature) for m in motors],dtype=np.float64),
        np.array([m.vol for m in motors],dtype=np.float64),
        np.array([m.motorstate for m in motors],dtype=np.uint32),
        np.asarray(message.imu_state.quaternion,dtype=np.float64),
        np.asarray(message.imu_state.gyroscope,dtype=np.float64),
        struct.unpack_from('<H',remote,2)[0],remote)


class Configuration:
    def __init__(self, root, policy_target_margin=.5):
        if not np.isfinite(policy_target_margin) or not 0<=policy_target_margin<=1.:
            raise DeployFault('Policy target rejection margin must be within [0, 1] rad')
        self.policy_target_margin = float(policy_target_margin)
        self.root = Path(root)
        repo = self.root/'.cache-g1/htd/IsaacLab-Decoupled-WBC'
        self.checkpoint = repo/'example/student_checkpoints/student_policy_jit.pt'
        digest = hashlib.sha256(self.checkpoint.read_bytes()).hexdigest()
        if digest != CHECKPOINT_SHA: raise DeployFault('Student checkpoint checksum mismatch')
        source = self.root/'.cache-g1/locked-g1/source'
        manifest = json.loads((source/'manifest.json').read_text())
        if manifest['commit']!='15ebe53f4cd77e7d745ae83b9d7ff90e685e8998':
            raise DeployFault('Unrecognized official model commit')
        urdf = source/manifest['urdf']
        if hashlib.sha256(urdf.read_bytes()).hexdigest()!=manifest['sha256'][manifest['urdf']]:
            raise DeployFault('Locked-waist URDF checksum mismatch')
        xml = ET.parse(urdf).getroot()
        joints = {j.attrib['name']:j for j in xml.findall('joint')}
        if any(joints[n].attrib['type']!='fixed' for n in ('waist_roll_joint','waist_pitch_joint')):
            raise DeployFault('Waist roll/pitch must be physically fixed')
        caps = [joints[n].find('limit').attrib for n in JOINT_NAMES]
        self.lower = np.array([float(c['lower']) for c in caps])
        self.upper = np.array([float(c['upper']) for c in caps])
        self.effort = np.array([float(c['effort']) for c in caps])
        self.velocity = np.array([float(c['velocity']) for c in caps])
        self.default = np.array([-.20,0,0,.42,-.23,0]*2+[0]+
                                [.35,.18,0,.87,0,0,0]+[.35,-.18,0,.87,0,0,0])
        saved_path = repo/'example/params/env.yaml'
        saved = yaml.load(saved_path.read_text(),Loader=yaml.BaseLoader)['scene']['robot']['actuators']
        def gain(name,field):
            groups = [v for v in saved.values() if any(re.fullmatch(p,name) for p in v['joint_names_expr'])]
            if len(groups)!=1: raise DeployFault(f'Ambiguous actuator for {name}')
            value = groups[0][field]
            matches = [v for p,v in value.items() if re.fullmatch(p,name)] if isinstance(value,dict) else [value]
            if len(matches)!=1: raise DeployFault(f'Ambiguous {field}: {name}')
            return float(matches[0])
        self.kp = np.array([gain(n,'stiffness') for n in JOINT_NAMES])
        self.kd = np.array([gain(n,'damping') for n in JOINT_NAMES])
        # Passive exit damping stays small relative to rated speed/effort.
        self.exit_kd = np.minimum(2.,.5*self.effort/self.velocity)
        self.position_target_rate_limit = 2.
        self.pd_estimate_budget_fraction = .8
        self.metadata = {'checkpoint_sha256':digest,'joint_names':JOINT_NAMES,'motor_slots':ACTIVE_SLOTS,
            'default_q_rad':self.default.tolist(),'kp':self.kp.tolist(),'kd':self.kd.tolist(),
            'position_lower_rad':self.lower.tolist(),'position_upper_rad':self.upper.tolist(),
            'official_effort_limits_nm':self.effort.tolist(),'official_velocity_limits_rad_s':self.velocity.tolist(),
            'pd_gains_sha256':hashlib.sha256(saved_path.read_bytes()).hexdigest(),
            'command':[0,0,0,.72,0,0,0], 'action_scale':.25, 'policy_shape':[1,2,58],
            'position_target_rate_limit_rad_s':self.position_target_rate_limit,
            'pd_estimate_budget_fraction':self.pd_estimate_budget_fraction,
            'raw_policy_target_rejection_margin_rad':self.policy_target_margin,
            'torque_caveat':'Dynamic target bounding uses an approximate PD law; it does not set a firmware torque limit.',
            'locked_action_history':'raw 15 outputs retained; physical slots 13,14 remain disabled',
            'scope':'suspended zero-velocity commissioning only; no walking or Nav2 command input'}


def gravity(quaternion):
    w,x,y,z = quaternion
    return np.array([2*(-z*x+w*y),-2*(z*y+w*x),1-2*(w*w+z*z)],dtype=np.float32)


def validate_state(s, now, cfg, max_age=.10):
    if s is None or now-s.stamp>max_age or now<s.stamp:
        raise DeployFault('LowState stale or missing')
    if s.machine!=6: raise DeployFault(f'mode_machine changed/unsupported: {s.machine}')
    if not all(np.isfinite(a).all() for a in (s.q,s.dq,s.tau,s.temperature,s.voltage,s.quaternion,s.gyro)):
        raise DeployFault('Nonfinite telemetry')
    norm = np.linalg.norm(s.quaternion)
    extra_limits=getattr(cfg,'commissioning_state_limits',True)
    if not np.isfinite(norm) or norm==0 or (extra_limits and abs(norm-1.)>.03):
        raise DeployFault('Invalid IMU quaternion norm')
    if np.any(s.motor_flags): raise DeployFault('Nonzero motor fault flags')
    if extra_limits:
        tilt = math.acos(float(np.clip(-gravity(s.quaternion)[2],-1,1)))
        if tilt>.50: raise DeployFault(f'Body tilt exceeds commissioning limit: {tilt:.3f} rad')
        if np.any(s.voltage<1.): raise DeployFault('Active motor power missing')
        if np.max(s.temperature)>=70.: raise DeployFault('Commissioning motor temperature limit 70C')
        if np.any(s.q<cfg.lower-.08) or np.any(s.q>cfg.upper+.08):
            raise DeployFault('Measured joint position outside official limits')
        if np.any(np.abs(s.dq)>1.05*cfg.velocity): raise DeployFault('Joint velocity exceeds official limit')
    if s.buttons & SELECT: raise DeployFault('Remote Select stop')


def observation(s, previous_action, cfg):
    q = np.zeros(15,dtype=np.float32);dq = np.zeros(15,dtype=np.float32)
    q[:13] = s.q[:13]-cfg.default[:13];dq[:13] = s.dq[:13]
    obs = np.concatenate((s.gyro,gravity(s.quaternion),[0,0,0,.72,0,0,0],q,dq,previous_action)).astype(np.float32)
    if obs.shape!=(58,) or not np.isfinite(obs).all(): raise DeployFault('Invalid policy observation')
    return np.clip(obs,-100,100)


class Policy:
    def __init__(self,cfg):
        import torch
        self.torch,self.cfg = torch,cfg
        torch.set_num_threads(1)
        self.model = torch.jit.load(str(cfg.checkpoint),map_location='cpu').eval()
        self.reset()
        with torch.inference_mode():
            for _ in range(30): self.model(torch.zeros((1,2,58)))

    def reset(self):
        self.history = np.zeros((2,58),dtype=np.float32)
        self.action = np.zeros(15,dtype=np.float32)

    def step(self,s):
        self.history[0] = self.history[1].copy()
        self.history[1] = observation(s,self.action,self.cfg)
        with self.torch.inference_mode():
            result = self.model(self.torch.from_numpy(self.history[None].copy())).cpu().numpy()
        if result.shape!=(1,15) or not np.isfinite(result).all():
            raise DeployFault('Nonfinite or wrong-shaped Student action')
        self.action = np.clip(result[0],-100,100)
        target = self.cfg.default.copy()
        target[:13] += .25*self.action[:13]
        return target


def bounded_target(requested, previous, cfg, dt=.02):
    requested = np.asarray(requested,dtype=np.float64)
    previous = np.asarray(previous,dtype=np.float64)
    if requested.shape!=(27,) or previous.shape!=(27,) or not np.isfinite(requested).all() or not np.isfinite(previous).all():
        raise DeployFault('Invalid position target')
    margin = cfg.policy_target_margin
    if margin is not None:
        violations = np.flatnonzero((requested<cfg.lower-margin)|(requested>cfg.upper+margin))
        if len(violations):raise TargetRangeFault(requested,cfg,violations)
    position_margin=getattr(cfg,'position_target_margin',.02)
    target = np.clip(requested,cfg.lower+position_margin,cfg.upper-position_margin)
    rate = cfg.position_target_rate_limit
    return target if rate is None else np.clip(target,previous-rate*dt,previous+rate*dt)


def pd_bounded_target(target,s,cfg,previous=None,dt=.002):
    target = np.asarray(target,dtype=np.float64)
    if target.shape!=(27,) or not np.isfinite(target).all():
        raise DeployFault('Invalid position target')
    fraction = cfg.pd_estimate_budget_fraction
    position_margin=getattr(cfg,'position_target_margin',.02)
    lower,upper = cfg.lower+position_margin,cfg.upper-position_margin
    if fraction is not None:
        budget = fraction*cfg.effort
        lower = np.maximum(lower,s.q+(-budget+cfg.kd*s.dq)/cfg.kp)
        upper = np.minimum(upper,s.q+(budget+cfg.kd*s.dq)/cfg.kp)
    rate = cfg.position_target_rate_limit
    if previous is not None and rate is not None:
        lower=np.maximum(lower,previous-rate*dt)
        upper=np.minimum(upper,previous+rate*dt)
    if np.any(lower>upper): raise DeployFault('No joint target satisfies position and PD estimate bounds')
    return np.clip(target,lower,upper)


def fill_command(command,target,cfg,damping=False):
    command.mode_machine=6;command.mode_pr=0
    for motor in command.motor_cmd:
        motor.mode=0;motor.q=0.;motor.dq=0.;motor.kp=0.;motor.kd=0.;motor.tau=0.
    for j,slot in enumerate(ACTIVE_SLOTS):
        motor = command.motor_cmd[slot]
        motor.mode=1
        motor.q=0. if damping else float(target[j])
        motor.kp=0. if damping else float(cfg.kp[j])
        motor.kd=float(cfg.exit_kd[j] if damping else cfg.kd[j])
    return command


class FreshButton:
    """Require a release after startup before accepting an operator press."""
    def __init__(self,mask): self.mask,self.released,self.last = mask,False,False

    def press(self,buttons):
        down = bool(buttons & self.mask)
        if not down: self.released=True
        edge = down and not self.last and self.released
        self.last=down
        return edge
