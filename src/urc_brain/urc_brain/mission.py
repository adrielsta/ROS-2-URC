"""Sequential Nav2 goals with explicit start/cancel and observed arrival checks.

No gesture recognition or manipulation is claimed by this route demonstrator.
"""
import json
import math
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from urc_brain.mission_plan import validate_plan


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        path = self.declare_parameter('mission_file', '').value
        with open(path, encoding='utf-8') as f:
            self.targets = validate_plan(yaml.safe_load(f))
        self.tolerance = self.declare_parameter('arrival_tolerance', 0.75).value
        self.timeout = self.declare_parameter('target_timeout', 180.0).value
        self.budget = self.declare_parameter('mission_budget', 2400.0).value
        if not 0 < self.tolerance <= 1.0 or self.timeout <= 0 or self.budget <= 0:
            raise ValueError('Invalid arrival tolerance or target timeout')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.arm = self.create_client(Trigger, '/safety/arm_autonomy')
        self.stop_client = self.create_client(Trigger, '/safety/stop')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.nav_active = False
        self.lifecycle_future = None
        self.create_timer(1.0, self.check_nav_state)
        self.create_service(Trigger, '/mission/start', self.start)
        self.create_service(Trigger, '/mission/cancel', self.cancel)
        self.status_pub = self.create_publisher(String, '/mission/status', 1)
        self.arrival_pub = self.create_publisher(Bool, '/mission/arrival', 1)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(String, '/rover/status', self.rover_status, 1)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.state, self.detail = 'idle', 'Call /mission/start when ready'
        self.index, self.epoch = 0, 0
        self.goal_handle = None
        self.last_odom = -math.inf
        self.is_stopped = False
        self.deadline = math.inf
        self.verify_since = None
        self.started_sim = None
        self.finished_sim = None
        self.create_timer(0.1, self.tick)

    def odom(self, msg):
        v = msg.twist.twist
        self.is_stopped = math.hypot(v.linear.x, v.linear.y) < 0.03 and abs(v.angular.z) < 0.05
        self.last_odom = time.monotonic()

    def rover_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if self.state == 'awaiting_autonomy' and status.get('mode') == 'autonomy':
            self.send_goal()
        elif self.state in ('navigating', 'verifying', 'arrival') and status.get('mode') != 'autonomy':
            self.abort('Supervisor disarmed: ' + str(status.get('reason')))

    def start(self, _, response):
        if self.state not in ('idle', 'complete', 'canceled', 'failed'):
            response.success, response.message = False, 'Mission already active'
            return response
        if not self.nav_active or not self.nav.server_is_ready() or not self.arm.service_is_ready():
            response.success, response.message = False, 'Nav2 or supervisor is not ready'
            return response
        if not self.is_stopped or time.monotonic() - self.last_odom > 1.0:
            response.success, response.message = False, 'Wait for fresh odometry and measured zero velocity'
            return response
        self.epoch += 1
        self.index, self.state = 0, 'arming'
        self.started_sim = self.get_clock().now().nanoseconds / 1e9
        self.finished_sim = None
        self.deadline = time.monotonic() + 10.0
        self.arrival_pub.publish(Bool(data=False))
        epoch = self.epoch
        self.arm.call_async(Trigger.Request()).add_done_callback(lambda f: self.armed(f, epoch))
        response.success, response.message = True, 'Start accepted; watch /mission/status'
        return response

    def check_nav_state(self):
        if self.lifecycle_future is not None and not self.lifecycle_future.done():
            return
        self.nav_active = False
        if self.lifecycle.service_is_ready():
            self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
            self.lifecycle_future.add_done_callback(self.nav_state)

    def nav_state(self, future):
        try:
            self.nav_active = future.result().current_state.id == 3
        except Exception:
            self.nav_active = False

    def armed(self, future, epoch):
        if epoch != self.epoch:
            # A late arm response after cancellation must not leave the gate open.
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
            return
        try:
            response = future.result()
            if not response.success:
                self.abort(response.message)
                return
            # Observe the gate's new mode before navigation, so an older HOLD
            # telemetry message cannot race the arm service response.
            self.state, self.detail = 'awaiting_autonomy', 'Waiting for armed supervisor state'
            self.deadline = time.monotonic() + 5.0
        except Exception as exc:
            self.abort(str(exc))

    def send_goal(self):
        target = self.targets[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target['x'], target['y']
        goal.pose.pose.orientation.z = math.sin(target['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(target['yaw'] / 2)
        self.state, self.detail = 'navigating', f'Target {self.index + 1}'
        self.deadline = time.monotonic() + self.timeout
        epoch = self.epoch
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.accepted(f, epoch))

    def accepted(self, future, epoch):
        try:
            handle = future.result()
            if epoch != self.epoch:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.abort('Nav2 rejected goal')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.finished(f, epoch))
        except Exception as exc:
            if epoch == self.epoch:
                self.abort(str(exc))

    def finished(self, future, epoch):
        if epoch != self.epoch:
            return
        try:
            result = future.result()
            self.goal_handle = None
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.abort(f'Nav2 status {result.status}')
                return
            self.state, self.detail = 'verifying', 'Checking measured stop and target distance'
            self.deadline = time.monotonic() + 10.0
            self.verify_since = None
        except Exception as exc:
            self.abort(str(exc))

    def abort(self, reason, state='failed'):
        self.epoch += 1
        self.state, self.detail = state, reason
        self.finished_sim = self.get_clock().now().nanoseconds / 1e9
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        self.arrival_pub.publish(Bool(data=False))

    def cancel(self, _, response):
        self.abort('Operator canceled', 'canceled')
        response.success, response.message = True, 'Stopped and cancellation requested'
        return response

    def tick(self):
        now = time.monotonic()
        sim_now = self.get_clock().now().nanoseconds / 1e9
        elapsed = max(0.0, (self.finished_sim if self.finished_sim is not None else sim_now) - self.started_sim) if self.started_sim is not None else 0.0
        if self.state in ('arming', 'awaiting_autonomy', 'navigating', 'verifying', 'arrival') and elapsed >= self.budget:
            self.abort('Mission simulation-time budget exceeded')
        if self.state == 'arrival' and sim_now >= self.next_target_at:
            self.send_goal()
        if self.state in ('arming', 'awaiting_autonomy', 'navigating', 'verifying') and now > self.deadline:
            self.abort('Mission step timed out')
        if self.state == 'verifying':
            try:
                transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = transform.transform.translation
                target = self.targets[self.index]
                distance = math.hypot(p.x - target['x'], p.y - target['y'])
                age = (self.get_clock().now().nanoseconds -
                       rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
                good = (distance <= self.tolerance and self.is_stopped
                        and now - self.last_odom < 1.0 and 0 <= age < 1.0)
                if good:
                    if self.verify_since is None:
                        self.verify_since = now
                    if now - self.verify_since >= 0.5:
                        self.index += 1
                        if self.index == len(self.targets):
                            self.state, self.detail = 'complete', 'All navigation targets reached and stopped'
                            self.finished_sim = sim_now
                        else:
                            self.state, self.detail = 'arrival', 'Stopped arrival; indicating before next target'
                            self.next_target_at = sim_now + 3.0
                else:
                    self.verify_since = None
            except TransformException:
                self.verify_since = None
        self.arrival_pub.publish(Bool(data=self.state in ('arrival', 'complete')))
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'detail': self.detail,
            'completed_targets': self.index, 'total_targets': len(self.targets),
            'elapsed_sim_s': round(elapsed, 2), 'budget_sim_s': self.budget,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.abort('Mission process exiting', 'canceled')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
