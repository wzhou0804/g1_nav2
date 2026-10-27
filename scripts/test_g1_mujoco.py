#!/usr/bin/env python3
"""Engine-boundary checks for the URDF, local IMU frame, and torque execution."""
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np
from navigation.homie import observation
from navigation.homie_mujoco import MuJoCoSimulation,DisplaySnapshot
from navigation.student_deploy_core import JOINT_NAMES


class MuJoCoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.sim=MuJoCoSimulation()

    def setUp(self):self.sim.reset(0)

    def test_model_preserves_mass_fixed_waist_and_joint_order(self):
        s=self.sim
        self.assertEqual((s.model.nq,s.model.nv,s.model.nu),(34,33,27))
        self.assertAlmostEqual(float(s.model.body_mass.sum()),33.34114202,places=8)
        self.assertEqual([s.model.joint(int(i)).name for i in s.joints],JOINT_NAMES)
        np.testing.assert_allclose(s.model.actuator_ctrlrange,np.column_stack((-s.cfg.effort,s.cfg.effort)))
        np.testing.assert_allclose(s.model.jnt_range[s.joints],np.column_stack((s.cfg.lower,s.cfg.upper)))
        for name in ('waist_roll_joint','waist_pitch_joint'):
            self.assertEqual(mujoco.mj_name2id(s.model,mujoco.mjtObj.mjOBJ_JOINT,name),-1)

    def test_body_gyro_and_gravity_at_rotated_base(self):
        s=self.sim
        s.data.qpos[3:7]=[np.sqrt(.5),np.sqrt(.5),0,0]
        s.data.qvel[3:6]=[.1,.2,.3]
        mujoco.mj_forward(s.model,s.data)
        state=s.snapshot()
        np.testing.assert_allclose(state.gyro,[.1,.2,.3],atol=1e-12)
        frame=observation(state,np.zeros(12),s.cfg)
        np.testing.assert_allclose(frame[4:7],[.05,.1,.15],atol=1e-7)
        np.testing.assert_allclose(frame[7:10],[0,-1,0],atol=1e-7)
        self.assertAlmostEqual(s.velocity()[2],.2)

    def test_ground_masks_and_torque_limits_during_closed_loop(self):
        s=self.sim;ground=s.model.geom('ground').id
        self.assertEqual((s.model.geom_contype[ground],s.model.geom_conaffinity[ground]),(2,1))
        for i in range(s.model.ngeom):
            if i!=ground and s.model.geom_contype[i]:
                self.assertEqual((s.model.geom_contype[i],s.model.geom_conaffinity[i]),(1,2))
        for _ in range(100):
            self.assertFalse(s.step([0,0,0]))
            self.assertTrue(np.all(np.abs(s.data.actuator_force)<=s.cfg.effort+1e-9))
        self.assertEqual(s.diagnostics['physics_steps'],400)
        self.assertAlmostEqual(s.data.time,2.)
        self.assertIsNone(s.cfg.position_target_rate_limit)
        self.assertIsNone(s.cfg.pd_estimate_budget_fraction)

    def test_viewer_reset_cannot_modify_controller_state_or_options(self):
        s=self.sim
        for _ in range(10):s.step([0,0,0])
        display=DisplaySnapshot(s)
        q=s.data.qpos.copy();t=s.data.time
        mujoco.mj_resetData(display.model,display.data)
        display.model.opt.gravity[:]=0
        display.data.ctrl[:]=35
        np.testing.assert_array_equal(s.data.qpos,q)
        np.testing.assert_array_equal(s.model.opt.gravity,[0,0,-9.81])
        self.assertEqual(s.data.time,t)
        display.update()
        np.testing.assert_array_equal(display.data.qpos,q)
        self.assertEqual(display.data.time,t)

    def test_terminal_velocity_commands_and_stop(self):
        from scripts.benchmark_g1_mujoco import TerminalCommands
        commands=TerminalCommands()
        np.testing.assert_array_equal(commands.accept('0.3 0 0'),[.3,0,0])
        np.testing.assert_array_equal(commands.accept('0 0 -0.4'),[0,0,-.4])
        np.testing.assert_array_equal(commands.accept('stop'),[0,0,0])
        for invalid in ('1 0 0','nan 0 0','0 0','hello'):
            with self.subTest(command=invalid),self.assertRaises(ValueError):commands.accept(invalid)
        commands.accept('quit');self.assertTrue(commands.quit)


if __name__=='__main__':unittest.main(verbosity=2)
