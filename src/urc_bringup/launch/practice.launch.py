"""Real Utah practice scenarios; deliberately ideal simulation localization."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=get_package_share_directory('urc_bringup')
    name=LaunchConfiguration('scenario').perform(context)
    allowed=('utah','utah-route','astronaut-approach','delivery-navigation','science-navigation','equipment-approach')
    if name not in allowed:raise ValueError('Unknown scenario; choose '+', '.join(allowed))
    scenario=os.path.join(share,'config','scenarios',name+'.yaml')
    with open(scenario) as f:spec=yaml.safe_load(f)
    common={'use_sim_time':True,'terrain_directory':os.path.join(share,'models','utah_site')}
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','sim.launch.py')),
            launch_arguments={'gui':LaunchConfiguration('gui'),'rviz':LaunchConfiguration('rviz'),
                'world':os.path.join(share,'worlds',name+'.sdf'),'map':os.path.join(share,'maps','utah.yaml'),
                'mission':scenario,'params':os.path.join(share,'config','utah_nav2.yaml'),
                'bridge':os.path.join(share,'config','utah_bridge.yaml'),
                'target_timeout':'7200.0','mission_budget':str(float(spec['budget_s']))}.items()),
        Node(package='urc_brain',executable='terrain_scan',parameters=[common],output='screen'),
        Node(package='urc_brain',executable='practice_referee',parameters=[common,{
            'scenario_file':scenario,'report_file':os.path.abspath('artifacts/'+name+'-practice.json')}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scenario',default_value='utah'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='false'),
        OpaqueFunction(function=setup)])
