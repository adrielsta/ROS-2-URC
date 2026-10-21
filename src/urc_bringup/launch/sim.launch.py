"""Hardware-free integration course. Fixed map->odom is a SIMULATION shortcut."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('urc_bringup')
    use_sim = {'use_sim_time': True}
    params = LaunchConfiguration('params')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    nodes = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    actions = [
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('bridge', default_value=os.path.join(share, 'config', 'bridge.yaml')),
        DeclareLaunchArgument('target_timeout', default_value='180.0'),
        DeclareLaunchArgument('mission_budget', default_value='2400.0'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'training.sdf')),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'training.yaml')),
        DeclareLaunchArgument('mission', default_value=os.path.join(share, 'config', 'mission.yaml')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=lambda context: [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ('-r ' + ('' if gui.perform(context) == 'true' else '-s --headless-rendering ') +
                                           '"' + world.perform(context) + '"')}.items())]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
             parameters=[{'config_file': LaunchConfiguration('bridge')}]),
        # Gazebo wheel odometry starts at (0,0). Only valid for this flat demo!
        Node(package='tf2_ros', executable='static_transform_publisher', name='sim_map_origin',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'], parameters=[use_sim]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_frame',
             arguments=['--x', '0.2', '--z', '0.3', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'], parameters=[use_sim]),
        Node(package='urc_brain', executable='supervisor', parameters=[use_sim], output='screen'),
        Node(package='urc_brain', executable='mission', parameters=[use_sim, {
            'mission_file': LaunchConfiguration('mission'),
            'target_timeout': LaunchConfiguration('target_timeout'),
            'mission_budget': LaunchConfiguration('mission_budget')}], output='screen'),
    ]
    for package, executable, name in [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ]:
        extras = {}
        if name == 'map_server':
            extras['yaml_filename'] = LaunchConfiguration('map')
        if name == 'bt_navigator':
            extras['default_nav_to_pose_bt_xml'] = os.path.join(share, 'config', 'navigate.xml')
        actions.append(Node(package=package, executable=executable, name=name,
                            parameters=[params, use_sim, extras], output='screen',
                            remappings=[('cmd_vel', '/cmd_vel_nav')]))
    actions += [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager',
             parameters=[use_sim, {'autostart': True, 'node_names': nodes}], output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[use_sim], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    return LaunchDescription(actions)
