"""Loopback-only simulated ROS endpoints; never communicates with the robot.

Run after sourcing scripts/activate_g1_real.sh. This process overrides the DDS
domain and interface before importing rclpy, including in a robot terminal.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

os.environ['ROS_DOMAIN_ID'] = '119'
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface address="127.0.0.1"/></Interfaces>'
    '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import rclpy
import yaml
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowState

from navigation.real_g1 import RealG1Bridge
from navigation.homie_nav2_transport import HomieNavCommands


class FactoryNavigation(unittest.TestCase):
    def test_rviz_goal_modes(self):
        path = ROOT / 'navigation/g1_real_navigation.launch.py'
        spec = importlib.util.spec_from_file_location('real_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = ROOT / 'navigation/config/g1_real.rviz'
        before = source.read_text()
        original = yaml.safe_load(before)
        execute = module.execution_rviz(source)
        preview_tool = next(t for t in original['Visualization Manager']['Tools']
                            if t['Class'] == 'rviz_default_plugins/SetGoal')
        execution_tool = next(t for t in execute['Visualization Manager']['Tools']
                              if t['Class'] == 'rviz_default_plugins/SetGoal')
        self.assertEqual(preview_tool['Topic']['Value'], '/g1_real/preview_goal')
        self.assertEqual(execution_tool['Topic']['Value'], '/g1_real/goal_pose')
        execution_tool['Topic']['Value'] = '/g1_real/preview_goal'
        self.assertEqual(original, execute)
        self.assertEqual(source.read_text(), before)

    def test_velocity_rpc_arming_and_cancel(self):
        self.check_velocity_rpc_arming_and_cancel(500)

    def test_velocity_rpc_arming_and_cancel_fsm801(self):
        self.check_velocity_rpc_arming_and_cancel(801)

    def check_velocity_rpc_arming_and_cancel(self, fsm):
        rclpy.init(args=['--ros-args', '-p', 'allow_motion:=true'])
        bridge = RealG1Bridge()
        peer = Node('fake_factory_and_nav2')
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(peer)
        requests, cancellations = [], []
        response_pub = peer.create_publisher(Response, '/api/sport/response', 10)

        def on_rpc(request):
            requests.append(request)
            response = Response()
            response.header.identity.id = request.header.identity.id
            response.header.identity.api_id = request.header.identity.api_id
            response.header.status.code = 0
            response.data = json.dumps({'data': fsm})
            response_pub.publish(response)

        def on_cancel(request, response):
            cancellations.append(request)
            response.return_code = 0
            return response

        peer.create_subscription(Request, '/api/sport/request', on_rpc, 10)
        peer.create_service(CancelGoal, '/g1_real/navigate_to_pose/_action/cancel_goal', on_cancel)
        arm_client = peer.create_client(SetBool, '/g1_real/enable_motion')
        odom_pub = peer.create_publisher(Odometry, '/dog_odom', qos_profile_sensor_data)
        cloud_pub = peer.create_publisher(PointCloud2, '/utlidar/cloud_livox_mid360', qos_profile_sensor_data)
        low_pub = peer.create_publisher(LowState, '/lowstate', qos_profile_sensor_data)
        command_pub = peer.create_publisher(Twist, '/g1_real/cmd_vel', 1)

        def pump(seconds, command=False):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                odom = Odometry()
                odom.header.stamp = peer.get_clock().now().to_msg()
                odom.pose.pose.position.z = 0.8
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                cloud = PointCloud2()
                cloud.header.stamp = odom.header.stamp
                cloud_pub.publish(cloud)
                low = LowState()
                low.mode_machine = 6
                low_pub.publish(low)
                if command:
                    twist = Twist()
                    twist.linear.x, twist.linear.y, twist.angular.z = 0.1, 0.02, 0.1
                    command_pub.publish(twist)
                executor.spin_once(timeout_sec=0.01)

        def set_armed(value):
            future = arm_client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic() + 2
            while not future.done() and time.monotonic() < deadline:
                pump(0.02)
            self.assertTrue(future.done(), 'Enable motion service did not respond')
            return future.result()

        def velocities():
            return [json.loads(r.parameter) for r in requests if r.header.identity.api_id == 7105]

        try:
            pump(1.0, command=True)
            self.assertTrue(bridge.healthy())
            self.assertEqual(bridge.fsm, fsm)
            self.assertEqual(bridge.get_parameter('allowed_fsm_ids').value, [500, 801])
            for invalid in (0, 1, 4, 999):
                bridge.fsm = invalid
                rejected = bridge.on_arm(SetBool.Request(data=True), SetBool.Response())
                self.assertFalse(rejected.success)
                self.assertIn(f'fsm={invalid}', rejected.message)
                self.assertFalse(bridge.gate.armed)
            bridge.fsm = fsm
            self.assertTrue(bridge.cancel_client.service_is_ready())
            self.assertFalse(bridge.gate.armed)
            self.assertEqual(velocities(), [])
            self.assertTrue(set_armed(True).success)
            pump(0.15)
            self.assertTrue(all(v['velocity'] == [0., 0., 0.] for v in velocities()),
                            'Arming replayed a command sent before arming')
            pump(0.4, command=True)
            self.assertTrue(any(v['velocity'][0] > 0 for v in velocities()))
            self.assertTrue(all(v['duration'] == 0.25 for v in velocities()))
            self.assertTrue(set_armed(False).success)
            pump(0.2)
            self.assertFalse(bridge.gate.armed)
            self.assertEqual(velocities()[-1]['velocity'], [0., 0., 0.])
            self.assertEqual(len(cancellations), 1)
            self.assertEqual(list(cancellations[0].goal_info.goal_id.uuid), [0] * 16)
            self.assertEqual(cancellations[0].goal_info.stamp.sec, 0)
            self.assertEqual(cancellations[0].goal_info.stamp.nanosec, 0)
            count = len(velocities())
            pump(0.2, command=True)
            self.assertEqual(len(velocities()), count)
            self.assertLessEqual({r.header.identity.api_id for r in requests}, {7001, 7105})
        finally:
            executor.shutdown()
            bridge.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()


class HomieNavigation(unittest.TestCase):
    def test_feedback_uses_body_twist_and_does_not_refresh_duplicate_source_samples(self):
        worker = HomieNavCommands(0, require_feedback=True)
        rclpy.init(args=['--ros-args', '-p', 'locomotion:=homie', '-p', f'homie_port:={worker.port}'])
        bridge = RealG1Bridge()
        try:
            odom = Odometry()
            odom.header.stamp = bridge.get_clock().now().to_msg()
            odom.child_frame_id = 'robot_center'
            odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = 2**-.5
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = .1, -.02
            odom.twist.twist.angular.z = .15
            bridge.on_odom(odom)
            for actual, expected in zip(bridge.measured_velocity, (.1, -.02, .15)):
                self.assertAlmostEqual(actual, expected)
            bridge.control()
            worker.poll(active=True, ready=True)
            self.assertIsNotNone(worker.measured_velocity())
            bridge.measurement_stamp -= 1.
            original = bridge.measurement_stamp
            bridge.on_odom(odom)
            self.assertEqual(bridge.measurement_stamp, original)
            bridge.control()
            worker.poll(active=True, ready=True)
            self.assertIsNone(worker.measured_velocity())
            self.assertEqual(worker.command, (0., 0., 0.))
            odom.header.stamp = bridge.get_clock().now().to_msg()
            odom.twist.twist.linear.x = float('nan')
            bridge.on_odom(odom)
            self.assertIsNone(bridge.measured_velocity)
        finally:
            bridge.destroy_node()
            worker.close()
            rclpy.shutdown()

    def test_same_service_and_velocity_gate_without_factory_velocity_rpc(self):
        worker = HomieNavCommands(0, require_feedback=True)
        rclpy.init(args=['--ros-args', '-p', 'allow_motion:=true',
                        '-p', 'locomotion:=homie', '-p', f'homie_port:={worker.port}'])
        bridge = RealG1Bridge()
        peer = Node('fake_homie_sensors_and_nav2')
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(peer)
        requests, cancellations, commands = [], [], []
        peer.create_subscription(Request, '/api/sport/request', requests.append, 10)

        def cancel(request, response):
            cancellations.append(request)
            return response

        peer.create_service(CancelGoal, '/g1_real/navigate_to_pose/_action/cancel_goal', cancel)
        arm_client = peer.create_client(SetBool, '/g1_real/enable_motion')
        odom_pub = peer.create_publisher(Odometry, '/dog_odom', qos_profile_sensor_data)
        cloud_pub = peer.create_publisher(PointCloud2, '/utlidar/cloud_livox_mid360', qos_profile_sensor_data)
        low_pub = peer.create_publisher(LowState, '/lowstate', qos_profile_sensor_data)
        command_pub = peer.create_publisher(Twist, '/g1_real/cmd_vel', 1)
        ready, active = False, False

        def pump(seconds, command=False, sensors=True):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if sensors:
                    odom = Odometry()
                    odom.header.stamp = peer.get_clock().now().to_msg()
                    odom.pose.pose.position.z = 0.8
                    odom.pose.pose.orientation.w = 1.
                    odom_pub.publish(odom)
                    cloud = PointCloud2()
                    cloud.header.stamp = odom.header.stamp
                    cloud_pub.publish(cloud)
                    low = LowState()
                    low.mode_machine = 6
                    low_pub.publish(low)
                if command:
                    twist = Twist()
                    twist.linear.x, twist.linear.y, twist.angular.z = .1, .02, .1
                    command_pub.publish(twist)
                worker.poll(ready=ready, active=active,
                            phase='homie_active' if active else 'waiting_nav2_enable')
                commands.append(worker.command)
                executor.spin_once(timeout_sec=.01)

        def set_armed(value):
            future = arm_client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic() + 2
            while not future.done() and time.monotonic() < deadline:
                pump(.02)
            self.assertTrue(future.done())
            return future.result()

        try:
            pump(.5)
            self.assertFalse(set_armed(True).success)
            ready = True
            pump(.3, command=True)
            self.assertTrue(bridge.healthy())
            self.assertEqual(worker.measured_velocity(), (0., 0., 0.))
            self.assertIsNone(bridge.fsm)  # Low-level ownership does not use factory FSMs.
            self.assertTrue(set_armed(True).success)
            pump(.3, command=True)
            self.assertTrue(worker.requested)
            self.assertFalse(worker.enabled)
            self.assertTrue(all(v == (0., 0., 0.) for v in commands))
            active = True
            pump(.2)
            self.assertEqual(worker.command, (0., 0., 0.))
            pump(.4, command=True)
            self.assertGreater(worker.command[0], 0)
            self.assertLessEqual(worker.command[0], bridge.gate.limits[0])
            self.assertTrue(set_armed(False).success)
            pump(.2, command=True)
            self.assertFalse(worker.enabled)
            self.assertEqual(worker.command, (0., 0., 0.))
            self.assertEqual(len(cancellations), 1)
            self.assertTrue(set_armed(True).success)
            pump(.3, command=True)
            self.assertGreater(worker.command[0], 0)
            pump(.6)  # Existing Nav2 command timeout still requests zero velocity.
            self.assertEqual(worker.command, (0., 0., 0.))
            self.assertTrue(worker.stop_immediately)
            self.assertTrue(bridge.gate.armed)
            bridge.homie.last_ack -= .7  # Reproduce the old SDK constructor's response gap.
            with patch.object(bridge.homie,'poll'):
                bridge.control()
            self.assertFalse(bridge.gate.armed)
            self.assertIn('ack_age=',bridge.last_disarm['reason'])
            self.assertIn('OpenHomie unavailable',bridge.last_disarm['reason'])
            first_disarm=bridge.last_disarm.copy()
            pump(.3)
            self.assertTrue(bridge.healthy())
            self.assertFalse(bridge.gate.armed)  # Link recovery never silently re-arms.
            self.assertEqual(bridge.last_disarm,first_disarm)
            self.assertTrue(set_armed(True).success)
            pump(.3,command=True)
            self.assertGreater(worker.command[0],0)
            pump(.7, command=True, sensors=False)
            self.assertFalse(bridge.gate.armed)
            self.assertEqual(worker.command, (0., 0., 0.))
            self.assertIn('sensors unavailable',bridge.last_disarm['reason'])
            self.assertEqual(requests, [])
        finally:
            executor.shutdown()
            bridge.destroy_node()
            worker.poll(active=active, ready=ready)
            self.assertFalse(worker.requested)
            worker.close()
            peer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
