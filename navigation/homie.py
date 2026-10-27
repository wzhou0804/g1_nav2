"""OpenHomie 27-DoF ONNX contract shared by simulation and hardware readers.

Upstream weights and source remain under their original license in .cache-g1/OpenHomie.
No DDS, LCM, ROS, or motor publisher is created by this module.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from navigation.student_deploy_core import ACTIVE_SLOTS, JOINT_NAMES, DeployFault, gravity

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '33071f79f1884bb122bd45631c8c60d61dac1eb8'
CHECKPOINT_SHA = 'afe6363c83c8b9ab8b1a2fed598fce7de35ff045eee1a95b336fd6720e3d3d49'
MODEL_RELATIVE = 'HomieRL/legged_gym/resources/robots/g1_description/g1.urdf'


def model_joints(path):
    xml=ET.parse(path).getroot()
    movable={j.attrib['name']:j for j in xml.findall('joint') if j.attrib['type']!='fixed'}
    if list(movable)!=JOINT_NAMES:
        raise DeployFault('OpenHomie requires the exact locked-waist 27-joint order')
    joints={j.attrib['name']:j for j in xml.findall('joint')}
    for name in ('waist_roll_joint','waist_pitch_joint'):
        if joints[name].attrib['type']!='fixed':
            raise DeployFault('OpenHomie requires physically fixed waist roll/pitch')
    caps=[movable[name].find('limit').attrib for name in JOINT_NAMES]
    return xml,{key:np.array([float(c[key]) for c in caps]) for key in ('lower','upper','effort','velocity')}


class Configuration:
    def __init__(self, root=ROOT, policy_target_margin=.5, height=.74, gains='deploy', model='official27'):
        self.root=Path(root)
        if gains not in ('deploy','training') or model not in ('official27','homie27'):
            raise DeployFault('Unrecognized OpenHomie model or gain profile')
        if not np.isfinite(height):raise DeployFault('OpenHomie height command must be finite')
        # Keep the old argument accepted for callers; OpenHomie no longer uses
        # the integration's raw-target rejection threshold.
        self.height=float(height);self.policy_target_margin=None
        self.repo=self.root/'.cache-g1/OpenHomie'
        self.checkpoint=self.repo/'HomieDeploy/deploy.onnx'
        if hashlib.sha256(self.checkpoint.read_bytes()).hexdigest()!=CHECKPOINT_SHA:
            raise DeployFault('OpenHomie checkpoint checksum mismatch')
        manifest=json.loads((self.repo/'integration-manifest.json').read_text())
        if manifest['commit']!=COMMIT:
            raise DeployFault('OpenHomie source commit mismatch')
        for name,expected in manifest['sha256'].items():
            if hashlib.sha256((self.repo/name).read_bytes()).hexdigest()!=expected:
                raise DeployFault('OpenHomie asset checksum mismatch: '+name)
        self.urdf=self.repo/MODEL_RELATIVE
        if model=='official27':
            folder=self.root/'.cache-g1/locked-g1/source'
            official=json.loads((folder/'manifest.json').read_text())
            if official['commit']!='15ebe53f4cd77e7d745ae83b9d7ff90e685e8998':
                raise DeployFault('Unrecognized official model commit')
            for name,expected in official['sha256'].items():
                if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=expected:
                    raise DeployFault('Official model asset checksum mismatch: '+name)
            self.urdf=folder/official['urdf']
        xml,caps=model_joints(self.urdf)
        self.lower,self.upper,self.effort,self.velocity=(caps[k] for k in ('lower','upper','effort','velocity'))
        self.default=np.array([-.1,0,0,.3,-.2,0]*2+[0]*15,dtype=np.float64)
        # Deployment values from HomieDeploy/unitree_sdk2/g1_control.cpp.
        # Training differs at hip/knee stiffness. Do not silently mix profiles.
        self.kp=np.array(([150,150,150,300,40,40] if gains=='deploy' else [100,100,100,150,40,40])*2+
                         [300]+([150,150,150,100,10,10,5] if gains=='deploy' else [200,200,200,100,20,20,20])*2,dtype=float)
        self.kd=np.array([2,2,2,4,2,2]*2+[5]+[4,4,4,1,.5,.5,.5]*2,dtype=float)
        self.exit_kd=np.minimum(2.,.5*self.effort/self.velocity)
        # OpenHomie uses position bounds without the integration's custom
        # target slew limit or approximate PD budget projection.
        self.position_target_rate_limit=None
        self.pd_estimate_budget_fraction=None
        # Integration-added timing exits are not part of the upstream policy.
        self.policy_timing_checks=False
        self.commissioning_state_limits=False
        self.position_target_margin=0.
        self.metadata={'policy':'openhomie','upstream':'https://github.com/InternRobotics/OpenHomie',
            'commit':COMMIT,'checkpoint_sha256':CHECKPOINT_SHA,'joint_names':JOINT_NAMES,
            'motor_slots':ACTIVE_SLOTS,'joints':27,'controlled_joints':JOINT_NAMES[:12],
            'policy_shape':[1,456],'observation_frame':76,'history_frames':6,'actions':12,
            'command':[0,0,0,self.height],'command_scale':[2,2,.25,1],
            'gyro_scale':.5,'joint_velocity_scale':.05,'action_scale':.25,
            'default_q_rad':self.default.tolist(),'kp':self.kp.tolist(),'kd':self.kd.tolist(),
            'gain_profile':gains,'model_variant':model,'urdf':str(self.urdf),
            'model_sha256':hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
            'urdf_mass_kg':sum(float(m.attrib['value']) for m in xml.findall('./link/inertial/mass')),
            'official_effort_limits_nm':self.effort.tolist(),'official_velocity_limits_rad_s':self.velocity.tolist(),
            'position_lower_rad':self.lower.tolist(),'position_upper_rad':self.upper.tolist(),
            'raw_policy_target_rejection_margin_rad':self.policy_target_margin,
            'position_target_rate_limit_rad_s':self.position_target_rate_limit,
            'pd_estimate_budget_fraction':self.pd_estimate_budget_fraction,
            'position_target_margin_rad':self.position_target_margin,
            'commissioning_state_limits':self.commissioning_state_limits,
            'policy_timing_checks':self.policy_timing_checks,
            'torque_caveat':'No software PD budget projection; URDF effort values do not configure a firmware torque clamp.',
            'upper_body':'waist yaw and fourteen arm joints held at zero; all 27 positions/velocities observed',
            'scope':'simulation or explicit gated hardware deployment; velocity input optional; no automatic takeover'}


def observation(s, previous_action, cfg, command=None):
    command=np.asarray([0,0,0,cfg.height] if command is None else command,dtype=np.float32)
    if command.shape!=(4,) or s.q.shape!=(27,) or s.dq.shape!=(27,) or np.shape(previous_action)!=(12,):
        raise DeployFault('Invalid OpenHomie observation shape')
    orientation=np.asarray(s.quaternion,dtype=float)
    norm=np.linalg.norm(orientation)
    if orientation.shape!=(4,) or not np.isfinite(norm) or norm==0:
        raise DeployFault('Invalid OpenHomie orientation')
    obs=np.concatenate((command*np.array([2,2,.25,1]),s.gyro*.5,gravity(orientation/norm),
                        s.q-cfg.default,s.dq*.05,previous_action)).astype(np.float32)
    if obs.shape!=(76,) or not np.isfinite(obs).all():
        raise DeployFault('Invalid OpenHomie observation')
    return np.clip(obs,-100,100)


class Policy:
    def __init__(self,cfg):
        import onnxruntime as ort
        self.cfg=cfg
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        self.model=ort.InferenceSession(str(cfg.checkpoint),sess_options=options,providers=['CPUExecutionProvider'])
        inputs=self.model.get_inputs();outputs=self.model.get_outputs()
        if len(inputs)!=1 or inputs[0].shape[1:]!=[456] or len(outputs)!=1 or outputs[0].shape[1:]!=[12]:
            raise DeployFault('OpenHomie ONNX interface must be 456 inputs / 12 outputs')
        self.input_name=inputs[0].name
        self.runtime={'onnxruntime':ort.__version__,'device':'cpu'}
        self.reset()
        for _ in range(30):self.model.run(None,{self.input_name:np.zeros((1,456),np.float32)})

    def reset(self):
        self.history=np.zeros((6,76),np.float32);self.action=np.zeros(12,np.float32)

    def step(self,s,command=None):
        self.history[:-1]=self.history[1:].copy()
        self.history[-1]=observation(s,self.action,self.cfg,command)
        result=self.model.run(None,{self.input_name:self.history.reshape(1,456).copy()})[0]
        if result.shape!=(1,12) or not np.isfinite(result).all():
            raise DeployFault('Nonfinite or wrong-shaped OpenHomie action')
        self.action=np.clip(result[0],-100,100)
        target=self.cfg.default.copy();target[:12]+=.25*self.action
        return target
