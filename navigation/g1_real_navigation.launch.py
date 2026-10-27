"""Physical G1: wall-clock mapping, conservative Nav2 and opt-in velocity gate."""
from pathlib import Path
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def execution_rviz(source):
    config = yaml.safe_load(Path(source).read_text())
    for tool in config['Visualization Manager']['Tools']:
        if tool['Class'] == 'rviz_default_plugins/SetGoal':
            tool['Topic']['Value'] = '/g1_real/goal_pose'
    return config


def generate_launch_description():
    root = Path(__file__).resolve().parent
    runtime = root.parent / '.cache-g1' / 'real-runtime'
    runtime.mkdir(parents=True, exist_ok=True)
    execute_rviz = runtime / 'g1_real_execute.rviz'
    execute_rviz.write_text(yaml.safe_dump(execution_rviz(root/'config/g1_real.rviz')))
    params = yaml.safe_load((root / 'config/nav2.yaml').read_text())
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(root / 'config/g1_real_navigate.xml')
    params['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = str(root / 'config/g1_real_through_poses.xml')
    params_file = runtime / 'nav2.yaml'
    # Absolute node names are necessary for all nested costmap nodes in namespace.
    params_file.write_text(yaml.safe_dump({'/g1_real': params}))
    slam = yaml.safe_load((root / 'config/slam.yaml').read_text())
    slam_file = runtime / 'slam.yaml'
    slam_file.write_text(yaml.safe_dump({'/g1_real': slam}))
    nodes = []
    executables = (
        ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
        ('nav2_smoother', 'smoother_server'), ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'), ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    )
    for package, executable in executables:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', '/g1_real/cmd_vel_nav')]
        if executable == 'velocity_smoother':
            remaps = [('cmd_vel', '/g1_real/cmd_vel_nav'), ('cmd_vel_smoothed', '/g1_real/cmd_vel')]
        nodes.append(Node(package=package, executable=executable, name=executable,
            namespace='g1_real', parameters=[str(params_file)], remappings=remaps, output='screen'))
    def rviz_node(context):
        config = (execute_rviz if LaunchConfiguration('goal_mode').perform(context) == 'navigate'
                  else root / 'config/g1_real.rviz')
        return [Node(package='rviz2', executable='rviz2', name='rviz2', namespace='g1_real',
            arguments=['-d', str(config)], parameters=[{'use_sim_time': False}], output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')))]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('goal_mode', default_value='preview', choices=['preview', 'navigate'],
            description='preview: compute path only; navigate: RViz goal sends NavigateToPose'),
        ExecuteProcess(cmd=[sys.executable, '-m', 'navigation.real_g1', '--ros-args',
            '-p', ['allow_motion:=', LaunchConfiguration('allow_motion')],
            '-p', f'status_file:={runtime / "status.json"}'], cwd=str(root.parent), output='screen'),
        ExecuteProcess(cmd=[sys.executable, '-m', 'navigation.real_g1_preview'],
                       cwd=str(root.parent), output='screen'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', namespace='g1_real', output='screen',
            parameters=[{'use_sim_time': False, 'target_frame': 'g1_real/base_footprint',
                'transform_tolerance': 0.3, 'min_height': 0.25, 'max_height': 1.8,
                'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                'angle_increment': 0.00872664626, 'scan_time': 0.1,
                'range_min': 0.5, 'range_max': 20.0, 'use_inf': True}],
            remappings=[('cloud_in', '/g1_real/points'), ('scan', '/g1_real/scan')]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace='g1_real', parameters=[str(slam_file)], output='screen'),
        *nodes,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace='g1_real', output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True,
                         'node_names': [name for _, name in executables]}]),
        OpaqueFunction(function=rviz_node),
    ])
