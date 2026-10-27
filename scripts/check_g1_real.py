#!/usr/bin/env python3
"""Stationary integration check; requests paths only, never a navigation goal.

Reads live sensors, TF, lifecycle, original GET RPCs, and saves map + report.
"""
import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=12)
    p.add_argument('--output', type=Path, default=Path('docs/g1_real_navigation_validation.json'))
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('g1_stationary_validation')
    counts, rpc_ids, latest = Counter(), Counter(), {}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subs = []

    def receive(key, msg):
        counts[key] += 1
        latest[key] = msg

    for key, cls, topic in [('points', PointCloud2, '/g1_real/points'),
                            ('scan', LaserScan, '/g1_real/scan'),
                            ('odom', Odometry, '/g1_real/odom'),
                            ('status', String, '/g1_real/status')]:
        subs.append(node.create_subscription(cls, topic, lambda m, k=key: receive(k, m), qos_profile_sensor_data))
    for key, topic in [('map', '/g1_real/map'), ('costmap', '/g1_real/global_costmap/costmap')]:
        subs.append(node.create_subscription(OccupancyGrid, topic, lambda m, k=key: receive(k, m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subs.append(node.create_subscription(Request, '/api/sport/request',
        lambda m: rpc_ids.update([str(m.header.identity.api_id)]), 100))
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    duration = time.monotonic() - start
    report = {'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'stationary_only': True,
              'duration_seconds': round(duration, 3), 'received': dict(counts),
              'rates_hz': {k: round(v / duration, 2) for k, v in counts.items()},
              'observed_sport_request_api_ids': dict(rpc_ids), 'lifecycle': {}}
    if 'status' in latest:
        report['bridge'] = json.loads(latest['status'].data)
    for name in ['controller_server', 'planner_server', 'smoother_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower', 'velocity_smoother']:
        client = node.create_client(GetState, f'/g1_real/{name}/get_state')
        if client.wait_for_service(timeout_sec=1):
            f = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=2)
            report['lifecycle'][name] = f.result().current_state.label if f.done() and f.result() else 'timeout'
        else:
            report['lifecycle'][name] = 'missing'
    path = None
    position = None
    try:
        tf = buffer.lookup_transform('g1_real/map', 'g1_real/base_footprint', Time())
        position = (tf.transform.translation.x, tf.transform.translation.y)
        report['robot_map_xy'] = position
        buffer.lookup_transform('g1_real/map', 'g1_real/lidar', Time())
        report['tf_map_to_lidar'] = True
    except Exception as e:
        report['tf_error'] = str(e)
    if 'points' in latest:
        m = latest['points']
        report['cloud'] = {'points': m.width*m.height, 'point_step': m.point_step,
                           'frame': m.header.frame_id}
    if 'scan' in latest:
        m = latest['scan']
        report['scan'] = {'rays': len(m.ranges), 'finite': sum(math.isfinite(x) for x in m.ranges),
                          'frame': m.header.frame_id}
    if 'map' in latest:
        m = latest['map']
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        report['map'] = {'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
                         'known_cells': int((grid >= 0).sum()), 'occupied_cells': int((grid >= 65).sum()),
                         'frame': m.header.frame_id}
        from PIL import Image
        prefix = args.output.parent / 'g1_real_local_map'
        image = np.where(grid < 0, 205, np.where(grid >= 65, 0, 254)).astype(np.uint8)
        Image.fromarray(np.flipud(image)).save(str(prefix) + '.pgm')
        Image.fromarray(np.flipud(image)).save(str(prefix) + '.png')
        Path(str(prefix) + '.yaml').write_text(yaml.safe_dump({
            'image': prefix.name + '.pgm', 'mode': 'trinary', 'resolution': m.info.resolution,
            'origin': [m.info.origin.position.x, m.info.origin.position.y, 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    if position and 'costmap' in latest:
        m = latest['costmap']
        grid = np.array(m.data).reshape(m.info.height, m.info.width)
        ys, xs = np.nonzero(grid == 0)
        xy = np.column_stack([m.info.origin.position.x+(xs+.5)*m.info.resolution,
                              m.info.origin.position.y+(ys+.5)*m.info.resolution])
        distances = np.linalg.norm(xy-np.array(position), axis=1)
        candidates = xy[(distances >= 0.8) & (distances <= 2.0)]
        action = ActionClient(node, ComputePathToPose, '/g1_real/compute_path_to_pose')
        report['planning_attempts'] = []
        if len(candidates) and action.wait_for_server(timeout_sec=2):
            # Nearby uninflated free cells. This requests a path, never FollowPath.
            indices = np.linspace(0, len(candidates)-1, min(8, len(candidates))).astype(int)
            for x, y in candidates[indices]:
                goal = ComputePathToPose.Goal()
                goal.goal = PoseStamped()
                goal.goal.header.frame_id = 'g1_real/map'
                goal.goal.header.stamp = node.get_clock().now().to_msg()
                goal.goal.pose.position.x, goal.goal.pose.position.y = float(x), float(y)
                goal.goal.pose.orientation.w = 1.0
                goal.planner_id = 'GridBased'
                f = action.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, f, timeout_sec=3)
                if not f.done() or not f.result() or not f.result().accepted:
                    continue
                result = f.result().get_result_async()
                rclpy.spin_until_future_complete(node, result, timeout_sec=3)
                outcome = result.result() if result.done() else None
                size = len(outcome.result.path.poses) if outcome else 0
                report['planning_attempts'].append({'goal_xy': [float(x), float(y)], 'poses': size,
                                                    'action_status': outcome.status if outcome else None})
                if outcome and outcome.status == 4 and size > 1:
                    path = outcome.result.path
                    report['planned_path_xy'] = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
                    break
    report['passed'] = (all(counts.get(k, 0) > 0 for k in ('points', 'scan', 'odom', 'map', 'status'))
        and len(report['lifecycle']) == 7 and all(x == 'active' for x in report['lifecycle'].values())
        and report.get('tf_map_to_lidar', False) and path is not None
        and not report.get('bridge', {}).get('motion_permitted', True)
        and not report.get('bridge', {}).get('armed', True)
        and not report.get('bridge', {}).get('clock_fault', True)
        and not report.get('bridge', {}).get('rpc_fault', True)
        and report.get('bridge', {}).get('cloud_time_valid', False)
        and report.get('bridge', {}).get('sensors_ready', False)
        and report.get('bridge', {}).get('mode_machine') == 6
        and (report.get('bridge', {}).get('locomotion', 'factory') == 'homie' or
             report.get('bridge', {}).get('counts', {}).get('ack_7001', 0) > 0)
        and report.get('bridge', {}).get('counts', {}).get('rpc_7105', 0) == 0
        and rpc_ids.get('7105', 0) == 0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    # Compact report; path coordinates remain in JSON.
    print(json.dumps({k: v for k, v in report.items() if k != 'planned_path_xy'}, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
