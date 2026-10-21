"""The only application publisher to the simulator's drive command topic."""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from urc_brain.safety import CommandGate, Limits, Mode


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        defaults = Limits()
        values = {key: self.declare_parameter(key, value).value
                  for key, value in vars(defaults).items()}
        self.gate = CommandGate(Limits(**values))
        self.arrived = False
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Empty, '/c2/heartbeat', self.heartbeat, 1)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav, 1)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop, 1)
        self.create_subscription(Bool, '/mission/arrival', self.arrival, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status = self.create_publisher(String, '/rover/status', 1)
        self.led = self.create_publisher(String, '/rover/led', 1)
        self.create_service(Trigger, '/safety/arm_autonomy', self.arm_auto)
        self.create_service(Trigger, '/safety/arm_teleop', self.arm_teleop)
        self.create_service(Trigger, '/safety/stop', self.stop)
        self.create_service(Trigger, '/safety/estop', self.estop)
        self.create_service(Trigger, '/safety/reset_estop', self.reset)
        # A paused /clock must not freeze the command watchdog.
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.ticks = 0

    def odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                         t.linear.x, t.linear.y, t.angular.z)):
            self.gate.last_odom = time.monotonic()
            self.gate.stopped = (math.hypot(t.linear.x, t.linear.y) < 0.03
                                 and abs(t.angular.z) < 0.05)

    def scan(self, msg):
        # +inf is a valid no-return; all-NaN or empty scans are not healthy.
        if (msg.ranges and math.isfinite(msg.range_min)
                and math.isfinite(msg.range_max) and msg.range_max > msg.range_min >= 0
                and any((math.isfinite(v) and v >= msg.range_min) or v == math.inf
                        for v in msg.ranges)):
            self.gate.last_scan = time.monotonic()

    def heartbeat(self, _):
        self.gate.last_heartbeat = time.monotonic()

    def nav(self, msg):
        self.gate.receive(Mode.AUTONOMY, msg.linear.x, msg.angular.z, time.monotonic())

    def teleop(self, msg):
        self.gate.receive(Mode.TELEOP, msg.linear.x, msg.angular.z, time.monotonic())

    def arrival(self, msg):
        self.arrived = msg.data

    def arm_auto(self, _, response):
        self.arrived = False
        response.success, response.message = self.gate.arm(Mode.AUTONOMY, time.monotonic())
        return response

    def arm_teleop(self, _, response):
        self.arrived = False
        response.success, response.message = self.gate.arm(Mode.TELEOP, time.monotonic())
        return response

    def stop(self, _, response):
        self.gate.stop()
        self.arrived = False
        response.success, response.message = True, self.gate.reason
        return response

    def estop(self, _, response):
        self.gate.estop()
        self.arrived = False
        response.success, response.message = True, self.gate.reason
        return response

    def reset(self, _, response):
        response.success, response.message = self.gate.reset_estop(time.monotonic())
        return response

    def tick(self):
        linear, angular = self.gate.output(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub.publish(msg)
        self.ticks += 1
        if self.ticks % 5 == 0:
            # HOLD/ESTOP aren't specified by the rules: off, not a false autonomy claim.
            color = {Mode.AUTONOMY: 'red', Mode.TELEOP: 'blue'}.get(self.gate.mode, 'off')
            if self.arrived and self.gate.mode == Mode.AUTONOMY and self.gate.stopped:
                color = 'flashing_green'
            self.led.publish(String(data=color))
            self.status.publish(String(data=json.dumps({
                'mode': self.gate.mode.value, 'reason': self.gate.reason,
                'healthy': self.gate.healthy(time.monotonic()),
                'measured_stopped': self.gate.stopped, 'led': color,
            })))


def main(args=None):
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

