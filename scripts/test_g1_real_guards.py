"""Offline tests: no ROS participant and no robot connection."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.real_g1_core import RobotClock, VelocityGate


class Guards(unittest.TestCase):
    def test_disabled_never_emits_even_zero(self):
        gate = VelocityGate()
        gate.update((1., 1., 1.), 10)
        self.assertFalse(gate.arm(True, 10))
        for healthy in (True, False):
            self.assertIsNone(gate.step(10.1, healthy))

    def test_arm_does_not_replay_old_command(self):
        gate = VelocityGate(permitted=True)
        gate.update((.1, 0., 0.), 10)
        self.assertFalse(gate.arm(False, 10))
        self.assertTrue(gate.arm(True, 10))
        self.assertEqual(gate.step(10.1, True), (0., 0., 0.))

    def test_limits_slew_and_watchdog(self):
        gate = VelocityGate(permitted=True)
        gate.arm(True, 10)
        gate.update((100., -100., 100.), 10)
        self.assertEqual(gate.command, (.15, -.06, .25))
        for actual, expected in zip(gate.step(10.1, True), (.02, -.015, .04)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(gate.step(10.36, True), (0., 0., 0.))

    def test_fault_disarms_and_cannot_auto_resume(self):
        gate = VelocityGate(permitted=True)
        gate.arm(True, 10)
        gate.update((.1, 0., 0.), 10)
        self.assertEqual(gate.step(10.1, False), (0., 0., 0.))
        gate.update((.1, 0., 0.), 10.2)
        self.assertIsNone(gate.step(10.2, True))

    def test_nonfinite_command_disarms(self):
        for invalid in (math.nan, math.inf, -math.inf):
            gate = VelocityGate(permitted=True)
            gate.arm(True, 10)
            gate.update((invalid, 0., 0.), 10)
            self.assertIsNone(gate.step(10.1, True))

    def test_clock_preserves_sensor_intervals(self):
        clock = RobotClock()
        clock.observe(10_000_000_000, 1600_000_000_000)
        self.assertEqual(clock.convert(9_800_000_000), 1599_800_000_000)
        clock.observe(10_100_000_000, 1600_100_000_000)
        self.assertFalse(clock.fault)

    def test_clock_reset_and_drift_latch_fault(self):
        for robot, host in ((1_000_000_000, 1600_100_000_000),
                            (10_100_000_000, 1601_100_000_000)):
            clock = RobotClock()
            clock.observe(10_000_000_000, 1600_000_000_000)
            clock.observe(robot, host)
            self.assertTrue(clock.fault)
            clock.observe(10_200_000_000, 1600_200_000_000)
            self.assertIsNone(clock.convert(10_200_000_000))


if __name__ == '__main__':
    unittest.main()
