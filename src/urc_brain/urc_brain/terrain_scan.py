"""SIMULATION ONLY: remove known DEM ground hits using ideal simulator pose.

This is a controlled navigation experiment, not deployable terrain perception.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from urc_brain.terrain_model import Terrain, rotate


class TerrainScan(Node):
    def __init__(self):
        super().__init__('simulation_terrain_scan')
        self.terrain = Terrain(self.declare_parameter('terrain_directory','').value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_raw',self.scan,qos_profile_sensor_data)
        self.get_logger().warning('SIMULATION ONLY: scan ground filter uses the known DEM and ideal simulator pose')

    def scan(self,msg):
        try:
            t = self.tf.lookup_transform('map',msg.header.frame_id,rclpy.time.Time())
        except TransformException:
            return
        age = abs((rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds-
                   rclpy.time.Time.from_msg(t.header.stamp).nanoseconds)/1e9)
        if age > 0.25:
            return
        p,q = t.transform.translation,t.transform.rotation
        quat = [q.x,q.y,q.z,q.w]
        values = list(msg.ranges)
        for i,distance in enumerate(values):
            if not math.isfinite(distance) or not msg.range_min<=distance<=msg.range_max:
                continue
            angle = msg.angle_min+i*msg.angle_increment
            xyz = rotate(quat,[distance*math.cos(angle),distance*math.sin(angle),0])+np.array([p.x,p.y,p.z])
            ground = self.terrain.height(xyz[0],xyz[1])
            if math.isfinite(ground) and xyz[2]-ground < 0.20:
                values[i] = math.inf
        msg.ranges = values
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
