#!/usr/bin/env python3
"""End-to-end check against actual ROS nodes; requires simulation running.

Checks disarmed startup, two Nav2 goals, stopped arrival, and a latched e-stop.
Does not certify real-world traversal, GNSS localization, or radio hardware.
"""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('integration_probe')
        self.mission = {}
        self.rover = {}
        self.speed = None
        self.create_subscription(String, '/mission/status', self.mission_cb, 10)
        self.create_subscription(String, '/rover/status', self.rover_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.speed_cb, 10)

    def mission_cb(self, msg):
        self.mission = json.loads(msg.data)

    def rover_cb(self, msg):
        self.rover = json.loads(msg.data)

    def speed_cb(self, msg):
        self.speed = (msg.linear.x, msg.angular.z)

    def until(self, test, seconds, description):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if test():
                print('PASS:', description, flush=True)
                return
        raise AssertionError(f'{description}: timed out. Rover={self.rover}, mission={self.mission}, speed={self.speed}')

    def call(self, service):
        client = self.create_client(Trigger, service)
        assert client.wait_for_service(timeout_sec=30), service + ' unavailable'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=15)
        assert future.done(), service + ' timed out'
        result = future.result()
        self.destroy_client(client)
        return result


def main():
    rclpy.init()
    node = Probe()
    report = {'status': 'failed', 'checks': []}
    try:
        node.until(lambda: node.rover.get('healthy') and node.speed == (0.0, 0.0), 120,
                   'Sensors active, rover starts disarmed at zero speed')
        assert node.rover['mode'] == 'hold'
        report['checks'].append('startup_disarmed')
        # Lifecycle activation may finish after the sensors become ready.
        deadline = time.monotonic() + 60
        while True:
            start = node.call('/mission/start')
            if start.success:
                break
            if time.monotonic() > deadline:
                raise AssertionError(start.message)
            rclpy.spin_once(node, timeout_sec=0.5)
        node.until(lambda: node.mission.get('state') in ('complete', 'failed'), 240, 'Mission terminates')
        assert node.mission['state'] == 'complete', node.mission
        assert node.mission['completed_targets'] == 2
        node.until(lambda: node.speed == (0.0, 0.0) and node.rover.get('led') == 'flashing_green',
                   10, 'Stopped arrival and LED indication')
        report['checks'].append('nav2_two_targets_around_obstacle')
        result = node.call('/safety/estop')
        assert result.success
        node.until(lambda: node.rover.get('mode') == 'estop' and node.speed == (0.0, 0.0),
                   5, 'Emergency stop latches zero output')
        assert not node.call('/safety/arm_autonomy').success
        report['checks'].append('estop_blocks_rearming')
        report['status'] = 'passed'
    finally:
        report['mission'] = node.mission
        report['rover'] = node.rover
        Path('artifacts').mkdir(exist_ok=True)
        Path('artifacts/integration.json').write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
