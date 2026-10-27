"""The existing OpenHomie ONNX/URDF contract in MuJoCo, without ROS or DDS."""
import hashlib
import copy
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from navigation.homie import Configuration,Policy
from navigation.student_deploy_core import JOINT_NAMES,Snapshot,bounded_target,gravity
from navigation.velocity_feedback import VelocityFeedback


class DisplaySnapshot:
    """Give the viewer private physics state so its Reset/sliders cannot edit control."""
    def __init__(self,simulation):
        self.simulation=simulation
        self.model=copy.copy(simulation.model)
        self.data=mujoco.MjData(self.model)
        self.spec=mujoco.mjtState.mjSTATE_INTEGRATION
        self.state=np.empty(mujoco.mj_stateSize(self.model,self.spec))
        self.update()

    def update(self):
        mujoco.mj_getState(self.simulation.model,self.simulation.data,self.state,self.spec)
        mujoco.mj_setState(self.model,self.data,self.state,self.spec)
        mujoco.mj_forward(self.model,self.data)


def quaternion_rpy(q):
    w,x,y,z=q
    return np.array([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def convert_model(cfg,physics_dt):
    """Import the same checked URDF, preserving inertias and fixed waist bodies."""
    cache=cfg.root/'.cache-g1/homie-mujoco'/cfg.metadata['model_variant']
    cache.mkdir(parents=True,exist_ok=True)
    # Separate builds prevent concurrent viewers/tests from reading partial XML.
    folder=Path(tempfile.mkdtemp(prefix='build-',dir=cache))
    xml=ET.parse(cfg.urdf).getroot()
    compiler=xml.find('./mujoco/compiler')
    compiler.set('meshdir',str(cfg.urdf.parent/'meshes'))
    compiler.set('strippath','true');compiler.set('fusestatic','false')
    compiler.set('discardvisual','false')
    for link in xml.findall('link'):
        for kind in ('visual','collision'):
            for i,geom in enumerate(link.findall(kind)):
                geom.set('name',f"{link.attrib['name']}_{kind}_{i}")
    ET.SubElement(xml,'link',name='world')
    joint=ET.SubElement(xml,'joint',name='floating_base_joint',type='floating')
    ET.SubElement(joint,'parent',link='world');ET.SubElement(joint,'child',link='pelvis')
    source=folder/'floating.urdf';ET.ElementTree(xml).write(source)
    model=mujoco.MjModel.from_xml_path(str(source))
    converted=folder/'imported.xml';mujoco.mj_saveLastXML(str(converted),model)
    scene=ET.parse(converted).getroot()
    # mj_saveLastXML rounds values. Restore the compiled URDF inertias and body
    # transforms at full precision before using the serialized model.
    def numbers(values):return ' '.join(format(float(x),'.17g') for x in np.atleast_1d(values))
    for body in scene.findall('.//worldbody//body'):
        i=model.body(body.attrib['name']).id
        body.set('pos',numbers(model.body_pos[i]));body.set('quat',numbers(model.body_quat[i]))
        inertial=body.find('inertial')
        if inertial is not None:
            for key,value in (('mass',model.body_mass[i]),('pos',model.body_ipos[i]),
                              ('quat',model.body_iquat[i]),('diaginertia',model.body_inertia[i])):
                inertial.set(key,numbers(value))
    for name,lo,hi in zip(JOINT_NAMES,cfg.lower,cfg.upper):
        scene.find(f".//worldbody//joint[@name='{name}']").set('range',numbers([lo,hi]))
    option=ET.SubElement(scene,'option',timestep=str(physics_dt),integrator='implicitfast',
                         gravity='0 0 -9.81',iterations='50')
    ET.SubElement(option,'flag',autoreset='disable')
    default=ET.SubElement(scene,'default')
    ET.SubElement(default,'joint',damping='0',armature='0.01',frictionloss='0')
    ET.SubElement(default,'geom',friction='1 0.005 0.0001',condim='3')
    world=scene.find('worldbody')
    ET.SubElement(world,'geom',name='ground',type='plane',size='200 200 0.1',
                  contype='2',conaffinity='1',rgba='0.32 0.36 0.40 1')
    ET.SubElement(world,'light',pos='0 0 5',dir='0 0 -1',diffuse='0.8 0.8 0.8')
    # Robot-to-ground contacts only, matching the self-collision-disabled Isaac test.
    for geom in world.findall('.//body/geom'):
        if geom.get('contype','1')!='0' or geom.get('conaffinity','1')!='0':
            geom.set('contype','1');geom.set('conaffinity','2')
    actuators=ET.SubElement(scene,'actuator')
    for name,effort in zip(JOINT_NAMES,cfg.effort):
        ET.SubElement(actuators,'motor',name=name+'_motor',joint=name,gear='1',
                      ctrllimited='true',ctrlrange=f'{-effort} {effort}')
    visual=ET.SubElement(scene,'visual');ET.SubElement(visual,'global',offwidth='640',offheight='480')
    path=folder/f'g1-{physics_dt:g}.xml';ET.indent(scene);ET.ElementTree(scene).write(path)
    return path,mujoco.MjModel.from_xml_path(str(path))


class MuJoCoSimulation:
    def __init__(self,model='official27',feedback=False,execution='author',physics_dt=.005,gains='deploy'):
        if physics_dt<=0 or not np.isfinite(physics_dt) or not np.isclose(.02/physics_dt,round(.02/physics_dt)):
            raise ValueError('physics_dt must divide the 0.02s policy period')
        self.cfg=Configuration(model=model,gains=gains)
        self.path,self.model=convert_model(self.cfg,physics_dt)
        self.data=mujoco.MjData(self.model);self.policy=Policy(self.cfg)
        self.feedback=VelocityFeedback() if feedback else None
        if self.feedback is not None:self.feedback.limit=np.array([.8,.5,.8])
        self.execution=execution;self.step_dt=.02;self.decimation=round(.02/physics_dt)
        self.name=f'mujoco_{model}_{gains}_{execution}_'+('closedloop' if feedback else 'raw')
        self.joints=np.array([self.model.joint(n).id for n in JOINT_NAMES])
        self.qadr=self.model.jnt_qposadr[self.joints];self.vadr=self.model.jnt_dofadr[self.joints]
        self.pelvis=self.model.body('pelvis').id;self.torso=self.model.body('torso_link').id
        if (self.model.nq,self.model.nv,self.model.nu)!=(34,33,27):
            raise ValueError('MuJoCo model must have 27 hinge joints plus one free base')
        for name in ('waist_roll_joint','waist_pitch_joint'):
            if mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,name)!=-1:
                raise ValueError('Waist roll/pitch must remain fixed')
        np.testing.assert_allclose(self.model.jnt_range[self.joints],np.column_stack((self.cfg.lower,self.cfg.upper)))
        np.testing.assert_allclose(self.model.body_mass.sum(),self.cfg.metadata['urdf_mass_kg'],atol=1e-8)
        self.metadata={**self.cfg.metadata,'engine':'MuJoCo','mujoco_version':mujoco.__version__,
            'physics_dt_s':physics_dt,'policy_dt_s':.02,'execution':execution,'velocity_feedback':feedback,
            'model_xml':str(self.path),'model_xml_sha256':hashlib.sha256(self.path.read_bytes()).hexdigest(),
            'physics_mass_kg':float(self.model.body_mass.sum()),'initial_root_height_m':.8,
            'converted_fixed_waist_verified':True,'hardware_interface':'none',
            'contact':'URDF convex mesh/sphere contacts; floor friction 1; self collision disabled',
            'integrator':'implicitfast','joint_armature':.01,'joint_passive_damping':0,
            'velocity_limit_enforcement':'monitored only; MuJoCo has no matching hard URDF joint speed clamp',
            'runtime':self.policy.runtime}
        self.diagnostics={'physics_steps':0,'policy_frames':0,'frames_with_raw_position_outside_limits':0,
            'max_abs_torque_nm_per_joint':np.zeros(27),'max_abs_velocity_rad_s_per_joint':np.zeros(27),
            'max_position_limit_excursion_rad_per_joint':np.zeros(27),'torque_saturation_steps_per_joint':np.zeros(27,dtype=int)}

    def reset(self,trial):
        mujoco.mj_resetData(self.model,self.data);self.policy.reset()
        if self.feedback is not None:self.feedback.reset()
        self.data.qpos[:3]=[0,0,.8];self.data.qpos[3:7]=[1,0,0,0]
        self.data.qpos[self.qadr]=self.cfg.default+np.random.default_rng(42+trial).uniform(-.005,.005,27)
        mujoco.mj_forward(self.model,self.data)
        self.previous_target=self.data.qpos[self.qadr].copy()

    def snapshot(self):
        spatial=np.zeros(6)
        # XBODY uses the link frame; BODY's local frame follows its inertial axes.
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_XBODY,self.pelvis,spatial,1)
        return Snapshot(time.monotonic(),0,6,self.data.qpos[self.qadr].copy(),self.data.qvel[self.vadr].copy(),
            self.data.actuator_force.copy(),np.full(27,30.),np.full(27,48.),np.zeros(27,int),
            self.data.xquat[self.pelvis].copy(),spatial[:3].copy(),0)

    def velocity(self):
        yaw=quaternion_rpy(self.data.xquat[self.pelvis])[2]
        c,s=np.cos(yaw),np.sin(yaw);vx,vy=self.data.qvel[:2]
        # Free-joint angular qvel is local; transform to world for the yaw rate metric.
        world_omega=self.data.xmat[self.pelvis].reshape(3,3)@self.data.qvel[3:6]
        return np.array([c*vx+s*vy,-s*vx+c*vy,world_omega[2]])

    def step(self,command):
        self.last_policy_command=(self.feedback.update(command,self.velocity()) if self.feedback is not None
                                  else np.asarray(command,dtype=float))
        requested=self.policy.step(self.snapshot(),[*self.last_policy_command,self.cfg.height])
        self.diagnostics['policy_frames']+=1
        self.diagnostics['frames_with_raw_position_outside_limits']+=int(
            np.any(requested<self.cfg.lower)|np.any(requested>self.cfg.upper))
        target=(bounded_target(requested,self.previous_target,self.cfg) if self.execution=='position' else requested)
        self.previous_target=target.copy()
        for _ in range(self.decimation):
            q=self.data.qpos[self.qadr];dq=self.data.qvel[self.vadr]
            torque=self.cfg.kp*(target-q)-self.cfg.kd*dq
            self.data.ctrl[:]=np.clip(torque,-self.cfg.effort,self.cfg.effort)
            mujoco.mj_step(self.model,self.data)
            diag=self.diagnostics;diag['physics_steps']+=1
            diag['max_abs_torque_nm_per_joint']=np.maximum(diag['max_abs_torque_nm_per_joint'],np.abs(self.data.actuator_force))
            diag['max_abs_velocity_rad_s_per_joint']=np.maximum(diag['max_abs_velocity_rad_s_per_joint'],np.abs(self.data.qvel[self.vadr]))
            excursion=np.maximum(0,np.maximum(self.cfg.lower-self.data.qpos[self.qadr],self.data.qpos[self.qadr]-self.cfg.upper))
            diag['max_position_limit_excursion_rad_per_joint']=np.maximum(diag['max_position_limit_excursion_rad_per_joint'],excursion)
            diag['torque_saturation_steps_per_joint']+=(np.abs(torque)>=self.cfg.effort).astype(int)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise RuntimeError('Nonfinite MuJoCo state')
            if np.any(self.data.warning.number):raise RuntimeError('MuJoCo warning; numerical run stopped')
        mujoco.mj_forward(self.model,self.data)
        roll,pitch,_=quaternion_rpy(self.data.xquat[self.torso])
        return bool(self.data.qpos[2]<.4 or abs(roll)>1 or abs(pitch)>1)

    def diagnostics_json(self):
        return {k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in self.diagnostics.items()}
