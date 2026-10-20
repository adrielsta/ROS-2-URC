#!/usr/bin/env python3
"""Catch ROS-only node construction/parameter errors before launching physics."""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from urc_brain.mission import Mission
from urc_brain.supervisor import Supervisor

mission_file = Path(get_package_share_directory('urc_bringup'))/'config/mission.yaml'
rclpy.init(args=['--ros-args', '-p', f'mission_file:={mission_file}'])
nodes = []
try:
    nodes.append(Supervisor())
    nodes.append(Mission())
    print('PASS: both ROS nodes construct successfully')
finally:
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()
