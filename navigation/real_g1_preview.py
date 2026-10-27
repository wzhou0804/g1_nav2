"""RViz goal poses -> ComputePathToPose -> preview path; never executes motion."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class PlanPreview(Node):
    def __init__(self):
        super().__init__('plan_preview', namespace='g1_real')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'preview_path', qos)
        self.status_pub = self.create_publisher(String, 'preview_status', qos)
        self.client = ActionClient(self, ComputePathToPose, '/g1_real/compute_path_to_pose')
        self.create_subscription(PoseStamped, 'preview_goal', self.on_goal, 10)
        self.sequence = 0
        self.pending = False
        self.started = 0.
        self.create_timer(1., self.watchdog)
        self.get_logger().info('2D Goal Pose previews paths only; no NavigateToPose or velocity output')

    def status(self, state, **extra):
        self.status_pub.publish(String(data=json.dumps({'state': state, **extra})))

    def watchdog(self):
        if self.pending and time.monotonic()-self.started > 15.:
            self.pending = False
            self.sequence += 1  # Ignore any late response from the old request.
            self.status('timeout')

    def on_goal(self, pose):
        p, q = pose.pose.position, pose.pose.orientation
        if pose.header.frame_id != 'g1_real/map' or not all(
                math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.status('invalid_goal', reason='Expected finite coordinates in g1_real/map')
            return
        if self.pending:
            self.get_logger().info('Planning in progress; select the next goal after the result')
            return
        self.path_pub.publish(Path(header=pose.header))
        if not self.client.server_is_ready():
            self.status('not_ready')
            return
        self.sequence += 1
        sequence = self.sequence
        self.pending, self.started = True, time.monotonic()
        self.status('planning', goal_xy=[p.x, p.y])
        goal = ComputePathToPose.Goal(goal=pose, planner_id='GridBased')
        future = self.client.send_goal_async(goal)

        def accepted(request):
            if sequence != self.sequence:
                return
            try:
                handle = request.result()
                if handle is None or not handle.accepted:
                    raise RuntimeError('Planner rejected the preview goal')
                handle.get_result_async().add_done_callback(completed)
            except Exception as exc:
                self.pending = False
                self.status('failed', reason=str(exc))

        def completed(result):
            if sequence != self.sequence:
                return
            self.pending = False
            try:
                outcome = result.result()
                path = outcome.result.path
                success = outcome.status == 4 and len(path.poses) > 1
                if success:
                    self.path_pub.publish(path)
                self.status('succeeded' if success else 'failed',
                            action_status=outcome.status, poses=len(path.poses), goal_xy=[p.x, p.y])
            except Exception as exc:
                self.status('failed', reason=str(exc))

        future.add_done_callback(accepted)


def main():
    rclpy.init()
    node = PlanPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
