"""Real sensors -> TF/SLAM/Nav2; gated velocity -> factory service or OpenHomie.

Never publishes LowCmd, changes FSM, releases controllers or activates a locomotion mode.
All outgoing RPCs are GET 7001 unless explicit motion permission AND arming.
"""
import json
import math
import time
from collections import Counter
from copy import deepcopy
from pathlib import Path

import rclpy
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowState
from visualization_msgs.msg import Marker

from navigation.real_g1_core import RobotClock, VelocityGate
from navigation.homie_nav2_transport import HomieNavClient
from navigation.odom_velocity import body_twist_to_yaw


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def stamp(value):
    return Time(sec=int(value // 1_000_000_000), nanosec=int(value % 1_000_000_000))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def transform(parent, child, when, xyz, quat):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id, t.header.stamp = parent, child, when
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, quat)
    return t


class RealG1Bridge(Node):
    def __init__(self):
        super().__init__('real_bridge', namespace='g1_real')
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('locomotion', 'factory')
        self.declare_parameter('homie_port', 17651)
        self.locomotion = self.get_parameter('locomotion').value
        if self.locomotion not in ('factory', 'homie'):
            raise ValueError('locomotion must be factory or homie')
        self.homie = (HomieNavClient(self.get_parameter('homie_port').value)
                      if self.locomotion == 'homie' else None)
        self.homie_was_active = False
        self.measured_velocity = None
        self.measurement_stamp = None
        self.velocity_source_ns = 0
        self.last_disarm = None
        self.last_sent_velocity = (0., 0., 0.)
        # Explicit compatibility with the factory FSMs used on this robot.
        # This allowlist does not change the robot's FSM or validate its gait.
        self.declare_parameter('allowed_fsm_ids', [500, 801])
        self.declare_parameter('status_file', '')
        # Nominal official locked-waist URDF. Verify robot_center == pelvis and
        # ground datum before enabling locomotion; allow measured correction.
        self.declare_parameter('odom_to_pelvis_z_offset', 0.0)
        self.declare_parameter('lidar_xyz', [0.0002835, 0.00003, 0.428434])
        self.declare_parameter('lidar_pitch', 0.05112069379091391)
        self.clock_bridge = RobotClock()
        self.gate = VelocityGate(permitted=self.get_parameter('allow_motion').value)
        self.count = Counter()
        self.seen = {}
        self.mode = None
        self.waist = 0.0
        self.fsm = None
        self.pending = {}
        self.rpc_fault = False
        self.cloud_time_valid = False
        self.last_tf = 0
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        p = self.get_parameter('lidar_pitch').value
        self.static_tf.sendTransform(transform('g1_real/torso', 'g1_real/lidar', self.get_clock().now().to_msg(),
            self.get_parameter('lidar_xyz').value, (math.cos(p/2), 0.0, -math.sin(p/2), 0.0)))
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'status', 1)
        self.marker_pub = self.create_publisher(Marker, 'state_marker', 1)
        self.rpc_pub = None
        if self.locomotion == 'factory':
            self.rpc_pub = self.create_publisher(Request, '/api/sport/request', 10)
            self.create_subscription(Response, '/api/sport/response', self.on_response, 20)
        self.create_subscription(Odometry, '/dog_odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_livox_mid360', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/utlidar/imu_livox_mid360', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LowState, '/lowstate', self.on_lowstate, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 1)
        self.create_service(SetBool, 'enable_motion', self.on_arm)
        self.cancel_client = self.create_client(CancelGoal, '/g1_real/navigate_to_pose/_action/cancel_goal')
        self.create_timer(1.0, self.status)
        self.create_timer(0.1, self.control)
        if self.locomotion == 'factory':
            self.create_timer(0.5, lambda: self.rpc(7001, {}))
        self.get_logger().info(f'Velocity destination: {self.locomotion}. Motion DISARMED. No FSM changes or LowCmd from this bridge.')

    def record(self, name):
        self.seen[name] = time.monotonic()
        self.count[name] += 1

    def age(self, name):
        return time.monotonic() - self.seen.get(name, -math.inf)

    def healthy(self):
        if self.homie is not None:
            return self.homie_health_reason() is None
        return (self.mode == 6 and self.sensors_ready() and not self.rpc_fault
                and self.age('fsm') < 1.5
                and self.fsm in self.get_parameter('allowed_fsm_ids').value)

    def homie_health_reason(self):
        if self.mode != 6:
            return f'mode_machine={self.mode}, expected 6'
        if not self.sensors_ready():
            return (f'sensors unavailable: odom_age={self.age("odom"):.3f}s, '
                    f'cloud_age={self.age("cloud"):.3f}s, lowstate_age={self.age("lowstate"):.3f}s, '
                    f'cloud_time_valid={self.cloud_time_valid}, clock_fault={self.clock_bridge.fault}')
        if not self.homie.ready():
            return (f'OpenHomie unavailable: phase={self.homie.status.get("phase")}, '
                    f'ready={self.homie.status.get("ready")}, '
                    f'ack_age={time.monotonic()-self.homie.last_ack:.3f}s')
        return None

    def record_disarm(self, was_armed):
        if was_armed and not self.gate.armed:
            self.last_disarm = {'host_monotonic': time.monotonic(), 'reason': self.gate.reason}
            self.get_logger().warning('Motion DISARMED: '+self.gate.reason)

    def sensors_ready(self):
        return (not self.clock_bridge.fault and self.clock_bridge.offset_ns is not None
                and self.age('odom') < 0.5 and self.age('cloud') < 1.0
                and self.cloud_time_valid and self.age('lowstate') < 0.5)

    def on_lowstate(self, msg):
        self.record('lowstate')
        self.mode, self.waist = msg.mode_machine, float(msg.motor_state[12].q)

    def on_odom(self, msg):
        robot_ns = ns(msg.header.stamp)
        self.clock_bridge.observe(robot_ns, self.get_clock().now().nanoseconds)
        self.record('odom')
        when = self.clock_bridge.convert(robot_ns)
        # Use the existing robot_center/body twist, projected into the same
        # gravity-aligned heading frame as the MuJoCo feedback measurement.
        # Duplicate source timestamps must not refresh feedback freshness.
        if self.homie is not None and when is not None and robot_ns > self.velocity_source_ns:
            self.velocity_source_ns = robot_ns
            twist, q = msg.twist.twist, msg.pose.pose.orientation
            try:
                self.measured_velocity = body_twist_to_yaw(
                    (q.x, q.y, q.z, q.w),
                    (twist.linear.x, twist.linear.y, twist.linear.z),
                    (twist.angular.x, twist.angular.y, twist.angular.z))
                self.measurement_stamp = time.monotonic()
            except ValueError:
                self.measured_velocity = self.measurement_stamp = None
        if when is None or robot_ns - self.last_tf < 20_000_000:
            return
        pos, orientation = msg.pose.pose.position, msg.pose.pose.orientation
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        norm = math.sqrt(sum(v*v for v in q))
        if not math.isfinite(norm) or norm < 0.5:
            self.clock_bridge.fault = True
            return
        q = tuple(v / norm for v in q)
        yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        planar = yaw_quat(yaw)
        tilted = multiply(yaw_quat(-yaw), q)
        z = pos.z + self.get_parameter('odom_to_pelvis_z_offset').value
        when = stamp(when)
        self.tf.sendTransform([
            transform('g1_real/odom', 'g1_real/base_footprint', when, (pos.x, pos.y, 0), planar),
            transform('g1_real/base_footprint', 'g1_real/base_link', when, (0, 0, z), tilted),
            transform('g1_real/base_link', 'g1_real/waist_yaw', when, (0, 0, 0), yaw_quat(self.waist)),
            transform('g1_real/waist_yaw', 'g1_real/torso', when, (-0.0039635, 0, 0.044), (0, 0, 0, 1)),
        ])
        out = deepcopy(msg)
        out.header.stamp, out.header.frame_id, out.child_frame_id = when, 'g1_real/odom', 'g1_real/base_footprint'
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = 0.0, 0.0
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = planar[2:]
        # /dog_odom twist is in robot_center; only its planar components are used.
        out.twist.twist.linear.z = out.twist.twist.angular.x = out.twist.twist.angular.y = 0.0
        self.odom_pub.publish(out)
        self.last_tf = robot_ns
        self.count['tf'] += 1

    def on_cloud(self, msg):
        self.record('cloud')
        when = self.clock_bridge.convert(ns(msg.header.stamp))
        age = None if when is None else (self.get_clock().now().nanoseconds - when) / 1e9
        self.cloud_time_valid = age is not None and -0.1 <= age <= 1.0
        if not self.cloud_time_valid:
            return
        # Preserve vendor fields and the actual 22-byte stride, including ring/time.
        msg.header.stamp, msg.header.frame_id = stamp(when), 'g1_real/lidar'
        self.cloud_pub.publish(msg)

    def on_imu(self, msg):
        self.record('imu')
        when = self.clock_bridge.convert(ns(msg.header.stamp))
        if when is not None:
            msg.header.stamp, msg.header.frame_id = stamp(when), 'g1_real/lidar_imu'
            self.imu_pub.publish(msg)  # IMU extrinsic uncalibrated; not used by SLAM.

    def rpc(self, api, parameters):
        if self.rpc_pub is None:
            raise RuntimeError('Factory RPC is disabled for OpenHomie locomotion')
        assert api in (7001, 7105), 'Only GET_FSM_ID and SET_VELOCITY are allowed'
        msg = Request()
        msg.header.identity.id = time.monotonic_ns()
        msg.header.identity.api_id = api
        msg.parameter = json.dumps(parameters)
        self.pending[msg.header.identity.id] = (api, time.monotonic())
        self.rpc_pub.publish(msg)
        self.count[f'rpc_{api}'] += 1

    def on_response(self, msg):
        request = self.pending.pop(msg.header.identity.id, None)
        if request is None:
            return
        api, _ = request
        if api != msg.header.identity.api_id or msg.header.status.code != 0:
            if api == 7105:
                self.rpc_fault = True
            return
        self.count[f'ack_{api}'] += 1
        if api == 7001:
            try:
                self.fsm = int(json.loads(msg.data)['data'])
                self.record('fsm')
            except (KeyError, ValueError, TypeError):
                pass

    def on_command(self, msg):
        self.record('nav_commands')
        self.gate.update((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def on_arm(self, request, response):
        if not request.data:
            was_armed = self.gate.armed
            self.gate.disarm('operator disarmed')
            self.record_disarm(was_armed)
            if self.homie is not None:
                self.homie.send(enabled=False)
            elif was_armed:
                self.rpc(7105, {'velocity': [0.0, 0.0, 0.0], 'duration': 0.25})
            cancelling = self.cancel_client.service_is_ready()
            if cancelling:
                # All-zero goal UUID and stamp request cancellation of all goals.
                future = self.cancel_client.call_async(CancelGoal.Request())
                future.add_done_callback(self.on_cancelled)
            response.success, response.message = True, ('Disarmed; navigation cancellation requested' if cancelling else
                'Disarmed; navigation cancellation service unavailable')
        else:
            response.success = self.gate.arm(self.healthy(), time.monotonic())
            response.message = ('Armed; awaiting a new command' if response.success else
                f'Rejected: allow_motion={self.gate.permitted}, mode_machine={self.mode}, '
                f'locomotion={self.locomotion}, policy_ready={self.homie.ready() if self.homie else None}, '
                f'sensors_ready={self.sensors_ready()}, fsm={self.fsm}, '
                f'allowed_fsm_ids={self.get_parameter("allowed_fsm_ids").value}, '
                f'fsm_age_s={self.age("fsm"):.3f}, rpc_fault={self.rpc_fault}')
        return response

    def on_cancelled(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Navigation cancel response={result.return_code}, goals={len(result.goals_canceling)}')
        except Exception as exc:
            self.get_logger().warning(f'Navigation cancel failed; motion remains disarmed: {exc}')

    def control(self):
        now = time.monotonic()
        if self.homie is not None:
            self.homie.poll()
            was_armed = self.gate.armed
            health_reason = self.homie_health_reason()
            if self.gate.armed and health_reason is not None:
                self.gate.disarm(health_reason)
            active = self.homie.status.get('active') is True
            if active and not self.homie_was_active and self.gate.armed:
                self.gate.arm(self.healthy(), now)
            self.homie_was_active = active
            velocity = self.gate.step(now, self.healthy())
            self.last_sent_velocity = velocity if active and velocity is not None else (0., 0., 0.)
            self.record_disarm(was_armed)
            self.homie.send(self.last_sent_velocity,
                            enabled=self.gate.armed, measured_velocity=self.measured_velocity,
                            measurement_stamp=self.measurement_stamp,
                            stop_immediately=self.gate.reason=='command watchdog: stop')
            self.count['homie_packets'] += 1
            return
        for identity, (api, sent) in list(self.pending.items()):
            if now - sent > 0.6:
                self.pending.pop(identity, None)
                if api == 7105:
                    self.rpc_fault = True
        velocity = self.gate.step(now, self.healthy())
        if velocity is not None:
            self.rpc(7105, {'velocity': list(velocity), 'duration': 0.25})

    def status(self):
        data = {'motion_permitted': self.gate.permitted, 'armed': self.gate.armed,
                'locomotion': self.locomotion,
                'homie_status': None if self.homie is None else self.homie.status,
                'homie_measured_velocity': self.measured_velocity,
                'homie_velocity_source': '/dog_odom twist: body -> heading',
                'sensors_ready': self.sensors_ready(), 'motion_ready': self.healthy(),
                'motion_reason': self.gate.reason, 'mode_machine': self.mode, 'fsm': self.fsm,
                'last_disarm': self.last_disarm,
                'nav_command_velocity': self.gate.command,
                'homie_sent_velocity': self.last_sent_velocity,
                'nav_command_age_seconds': round(self.age('nav_commands'), 3) if 'nav_commands' in self.seen else None,
                'allowed_fsm_ids': self.get_parameter('allowed_fsm_ids').value,
                'clock_offset_seconds': None if self.clock_bridge.offset_ns is None else self.clock_bridge.offset_ns/1e9,
                'clock_fault': self.clock_bridge.fault, 'rpc_fault': self.rpc_fault,
                'cloud_time_valid': self.cloud_time_valid,
                'counts': dict(self.count), 'sensor_age_seconds': {
                    key: round(self.age(key), 3) if math.isfinite(self.age(key)) else None
                    for key in ('odom', 'cloud', 'imu', 'lowstate', 'fsm')},
                'calibration': 'nominal URDF; robot_center/pelvis and floor datum need field validation'}
        encoded = json.dumps(data, ensure_ascii=False, indent=2)
        self.status_pub.publish(String(data=encoded))
        file = self.get_parameter('status_file').value
        if file:
            target = Path(file)
            tmp = target.with_suffix('.tmp')
            tmp.write_text(encoded)
            tmp.replace(target)
        marker = Marker()
        marker.header.frame_id = 'g1_real/base_footprint'
        # Latest transform keeps the stale-data label visible after a disconnect.
        marker.header.stamp = Time()
        marker.ns, marker.id, marker.type = 'state', 0, Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 1.7
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.a = 1.0, 0.4, 1.0
        if not self.sensors_ready():
            marker.text = 'SENSORS STALE / WAITING\nMOTION DISABLED'
            marker.color.g = 0.1
        else:
            marker.text = 'MAPPING LIVE\n' + ('MOTION ENABLED' if self.gate.armed else 'MOTION DISABLED')
        self.marker_pub.publish(marker)

    def destroy_node(self):
        if self.homie is not None:
            self.homie.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = RealG1Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gate.armed and node.homie is None and rclpy.ok():
            node.rpc(7105, {'velocity': [0.0, 0.0, 0.0], 'duration': 0.25})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
