"""Simulation truth -> planar Nav2 frame plus actual tilted sensor/body frame."""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from urc_brain.terrain_model import rotate, planar_rotation


class SimulationFrames(Node):
    def __init__(self):
        super().__init__('simulation_frames')
        self.pub=self.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,'/simulation/ground_truth',self.odom,qos_profile_sensor_data)
        self.get_logger().warning('SIMULATION ONLY: ideal pose publishes navigation TF; GNSS is not being fused')

    def odom(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        full=[q.x,q.y,q.z,q.w]
        planar,tilt=planar_rotation(full)
        output=copy.deepcopy(msg)
        output.header.frame_id='odom';output.child_frame_id='base_footprint'
        output.pose.pose.position.z=0.
        oq=output.pose.pose.orientation
        oq.x,oq.y,oq.z,oq.w=map(float,planar)
        v=msg.twist.twist.linear
        # Body-frame velocity -> gravity-aligned footprint coordinates.
        linear=rotate(tilt,[v.x,v.y,v.z])
        output.twist.twist.linear.x=float(linear[0]);output.twist.twist.linear.y=float(linear[1]);output.twist.twist.linear.z=0.
        output.twist.twist.angular.x=output.twist.twist.angular.y=0.
        self.pub.publish(output)
        base=TransformStamped();base.header=copy.deepcopy(msg.header);base.header.frame_id='odom';base.child_frame_id='base_footprint'
        base.transform.translation.x=p.x;base.transform.translation.y=p.y
        base.transform.rotation=copy.deepcopy(output.pose.pose.orientation)
        body=TransformStamped();body.header=copy.deepcopy(msg.header);body.header.frame_id='base_footprint';body.child_frame_id='base_link'
        body.transform.translation.z=p.z
        body.transform.rotation.x,body.transform.rotation.y,body.transform.rotation.z,body.transform.rotation.w=map(float,tilt)
        self.tf.sendTransform([base,body])


def main(args=None):
    rclpy.init(args=args);node=SimulationFrames()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
