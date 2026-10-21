#!/usr/bin/env python3
"""Check actual Gazebo navigation on the bundled real Utah heightfield."""
import json
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from ros_integration import Probe


def main():
    rclpy.init()
    node=Probe()
    node.practice={}
    node.create_subscription(String,'/practice/status',lambda msg:setattr(node,'practice',json.loads(msg.data)),10)
    try:
        node.until(lambda:node.rover.get('healthy') and node.speed==(0.,0.) and bool(node.practice),180,
                   'Real Utah terrain sensors and simulator ground truth active; rover disarmed')
        assert abs(node.practice['surface_clearance_m']-.28)<.20,node.practice
        deadline=time.monotonic()+60
        while True:
            result=node.call('/mission/start')
            if result.success:break
            if time.monotonic()>deadline:raise AssertionError(result.message)
            rclpy.spin_once(node,timeout_sec=.5)
        node.until(lambda:node.mission.get('state') in ('complete','failed'),400,'Real Utah route terminates')
        assert node.mission['state']=='complete',node.mission
        node.until(lambda:node.practice.get('navigation_checks_passed'),10,
                   'Two measured stopped arrivals and C2-heartbeat dropout pass on real Utah terrain')
        assert node.practice['heartbeat_blackout_observed']
        assert node.practice['max_tilt_deg']<35
        print(json.dumps(node.practice,indent=2),flush=True)
    finally:
        Path('artifacts').mkdir(exist_ok=True)
        Path('artifacts/utah-integration.json').write_text(json.dumps(node.practice,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
