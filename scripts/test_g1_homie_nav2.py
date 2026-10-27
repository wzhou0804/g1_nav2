"""Loopback-only checks for the ROS-to-policy velocity connection."""
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
import math

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.homie_nav2_transport import HomieNavClient, HomieNavCommands
from navigation.odom_velocity import body_twist_to_yaw


class PolicyVelocityLink(unittest.TestCase):
    def setUp(self):
        self.policy = HomieNavCommands(port=0)
        self.nav = HomieNavClient(self.policy.port)

    def tearDown(self):
        self.nav.close()
        self.policy.close()

    def exchange(self, velocity=(0., 0., 0.), enabled=False, active=True, ready=True):
        self.nav.send(velocity, enabled)
        self.policy.poll(active=active, ready=ready)
        self.nav.poll()
        return self.policy.command

    def test_enable_starts_policy_without_passing_velocity_during_transition(self):
        self.assertEqual(self.exchange(active=False), (0., 0., 0.))
        self.assertTrue(self.nav.ready())
        self.assertFalse(self.nav.status['active'])
        self.assertEqual(self.exchange((.1, -.02, .15), enabled=True, active=False), (0., 0., 0.))
        self.assertTrue(self.policy.requested)
        self.assertEqual(self.exchange((.1, -.02, .15), enabled=True), (.1, -.02, .15))
        self.assertEqual(self.exchange(enabled=False), (0., 0., 0.))
        self.assertFalse(self.policy.requested)

    def test_normal_zero_and_watchdog_stop_are_distinct(self):
        self.exchange()
        self.exchange((.1,0,.1),enabled=True)
        self.exchange(enabled=True)
        self.assertTrue(self.policy.enabled)
        self.assertFalse(self.policy.stop_immediately)
        self.nav.send((.1,0,.1),enabled=True,stop_immediately=True)
        self.policy.poll(active=True,ready=True)
        self.assertTrue(self.policy.enabled)
        self.assertTrue(self.policy.stop_immediately)
        self.assertEqual(self.policy.command,(0.,0.,0.))
        self.exchange((.1,0,.1),enabled=True)
        self.assertFalse(self.policy.stop_immediately)

    def test_new_client_cannot_start_with_an_enabled_packet(self):
        self.assertEqual(self.exchange((.1, 0., 0.), enabled=True), (0., 0., 0.))
        self.assertFalse(self.nav.ready())
        self.exchange()
        self.assertEqual(self.exchange((.1, 0., 0.), enabled=True), (.1, 0., 0.))

    def test_expired_input_zeroes_and_requires_new_packet(self):
        self.exchange()
        self.exchange((.1, 0., .1), enabled=True)
        self.policy.last_received -= 1
        self.assertEqual(self.policy.poll(active=True), (0., 0., 0.))
        self.assertFalse(self.policy.requested)
        self.nav.last_ack -= 1
        self.assertFalse(self.nav.ready())

    def test_old_or_nonfinite_packet_does_not_replace_command(self):
        self.exchange()
        self.exchange((.1, 0., 0.), enabled=True)
        packet = dict(version=1, session=self.nav.session, sequence=self.nav.sequence,
                      stamp=time.monotonic(), enabled=True, velocity=[.15, 0., 0.])
        for field, value in (('sequence', self.nav.sequence - 1), ('stamp', time.monotonic() - 1),
                             ('velocity', [float('nan'), 0., 0.])):
            invalid = {**packet, field: value}
            self.nav.socket.sendto(json.dumps(invalid).encode(), self.nav.destination)
            self.assertEqual(self.policy.poll(active=True), (.1, 0., 0.))

    def test_feedback_is_required_and_old_measurement_cannot_be_refreshed_by_packets(self):
        self.policy.require_feedback = True
        self.exchange()
        self.assertFalse(self.nav.ready())
        self.assertEqual(self.exchange((.1, 0., 0.), enabled=True), (0., 0., 0.))
        stamp = time.monotonic()
        self.nav.send((.1, 0., .05), enabled=True,
                      measured_velocity=[.02, -.01, .03], measurement_stamp=stamp)
        self.policy.poll(active=True, ready=True)
        self.nav.poll()
        self.assertTrue(self.nav.ready())
        self.assertEqual(self.policy.measured_velocity(), (.02, -.01, .03))
        self.assertEqual(self.policy.command, (.1, 0., .05))
        with patch('navigation.homie_nav2_transport.time.monotonic', return_value=stamp+.51):
            self.nav.send((.1, 0., .05), enabled=True,
                          measured_velocity=[.02, -.01, .03], measurement_stamp=stamp)
            self.policy.poll(active=True, ready=True)
            self.nav.poll()
            self.assertFalse(self.nav.ready())
            self.assertIsNone(self.policy.measured_velocity())
            self.assertEqual(self.policy.command, (0., 0., 0.))
            self.assertFalse(self.policy.requested)

    def test_invalid_feedback_zeroes_motion_and_readiness(self):
        self.policy.require_feedback = True
        self.exchange()
        for measured, stamp in (([float('nan'), 0, 0], time.monotonic()),
                                ([0, 0], time.monotonic()), ([0, 0, 0], float('inf')),
                                ([0, 0, 0], time.monotonic()+10)):
            packet = dict(version=1, session=self.nav.session, sequence=self.nav.sequence+1,
                          stamp=time.monotonic(), enabled=True, velocity=[.1, 0, 0],
                          measured_velocity=measured, measurement_stamp=stamp)
            self.nav.sequence += 1
            self.nav.socket.sendto(json.dumps(packet).encode(), self.nav.destination)
            self.policy.poll(active=True, ready=True)
            self.nav.poll()
            self.assertFalse(self.nav.ready())
            self.assertEqual(self.policy.command, (0., 0., 0.))


class OdometryProjection(unittest.TestCase):
    def test_heading_rotation_does_not_change_forward_body_velocity(self):
        values = body_twist_to_yaw((0, 0, math.sqrt(.5), math.sqrt(.5)), (.1, -.02, 0), (0, 0, .15))
        for actual, expected in zip(values, (.1, -.02, .15)):
            self.assertAlmostEqual(actual, expected)

    def test_tilt_projection_uses_all_linear_and_angular_components(self):
        roll = math.pi/6
        values = body_twist_to_yaw((math.sin(roll/2), 0, 0, math.cos(roll/2)),
                                  (.1, .2, .3), (0, .4, .5))
        self.assertAlmostEqual(values[0], .1)
        self.assertAlmostEqual(values[1], math.cos(roll)*.2-math.sin(roll)*.3)
        self.assertAlmostEqual(values[2], math.sin(roll)*.4+math.cos(roll)*.5)

    def test_invalid_twist_and_quaternion(self):
        for q, linear in (((0, 0, 0, 0), (0, 0, 0)),
                          ((0, 0, 0, 1), (float('nan'), 0, 0))):
            with self.assertRaises(ValueError):body_twist_to_yaw(q, linear, (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
