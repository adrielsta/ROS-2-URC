"""Simulation-only assessment and geometric C2-heartbeat fault model.

Never returns official competition scores; no arm or science task is faked.
"""
import json
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from urc_brain.terrain_model import Terrain, stopped_arrival


class Referee(Node):
    def __init__(self):
        super().__init__('practice_referee')
        self.scenario = yaml.safe_load(Path(self.declare_parameter('scenario_file','').value).read_text())
        self.terrain = Terrain(self.declare_parameter('terrain_directory','').value)
        self.report = Path(self.declare_parameter('report_file','artifacts/practice.json').value)
        self.pub = self.create_publisher(String,'/practice/status',10)
        self.heartbeat = self.create_publisher(Empty,'/c2/heartbeat',1)
        self.estop = self.create_client(Trigger,'/safety/estop')
        self.create_subscription(Odometry,'/simulation/ground_truth',self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(String,'/mission/status',self.mission_cb,10)
        self.create_subscription(String,'/rover/status',self.rover_cb,10)
        self.pose = None
        self.last_pose = -math.inf
        self.mission = {}
        self.rover = {}
        self.arrivals = {}
        self.failures = []
        self.blackout_samples = 0
        self.max_tilt = 0.0
        self.create_timer(0.2,self.tick)
        self.get_logger().warning('PRACTICE: known terrain + ideal simulator localization; heartbeat model is not RF propagation')

    def odom_cb(self,msg):
        self.pose = msg
        self.last_pose = time.monotonic()

    def mission_cb(self,msg):
        data = json.loads(msg.data)
        if data['state']=='arming' and self.mission.get('state')!='arming':
            self.arrivals,self.failures,self.blackout_samples = {},[],0
        self.mission = data

    def rover_cb(self,msg):
        self.rover = json.loads(msg.data)

    def fault(self,reason):
        if reason not in self.failures:
            self.failures.append(reason)
            self.get_logger().error(reason)
            if self.estop.service_is_ready():
                self.estop.call_async(Trigger.Request())

    def tick(self):
        if self.pose is None or time.monotonic()-self.last_pose>1.0:
            return
        p,q = self.pose.pose.pose.position,self.pose.pose.pose.orientation
        v = self.pose.twist.twist
        xy = [p.x,p.y]
        tilt = math.degrees(math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y)))))
        self.max_tilt = max(self.max_tilt,tilt)
        ground = self.terrain.height(*xy)
        if not math.isfinite(ground): self.fault('Rover left the terrain patch')
        elif p.z-ground < -0.5: self.fault('Rover is below expected DEM surface')
        if tilt>35: self.fault('Simulation tilt stop: greater than 35 degrees (assumed limit)')
        connected = self.terrain.line_of_sight([0,0],xy)
        # Optional deterministic outage tests the same heartbeat interface.
        forced = self.scenario.get('heartbeat_outage_after_first',False) and self.mission.get('completed_targets',0)>=1
        connected = connected and not forced
        if connected:
            self.heartbeat.publish(Empty())
        elif self.mission.get('state') in ('arrival','navigating','verifying','complete'):
            self.blackout_samples += 1
        index = self.mission.get('completed_targets',0)-1
        if self.mission.get('state') in ('arrival','complete') and 0<=index<len(self.scenario['checks']):
            target = self.scenario['checks'][index]
            if stopped_arrival(xy,target['xy'],math.hypot(v.linear.x,v.linear.y),v.angular.z,
                               self.rover.get('led'),target['radius_m']):
                self.arrivals[target['id']] = {'distance_m':round(math.dist(xy,target['xy']),3),
                    'elapsed_sim_s':self.mission.get('elapsed_sim_s'),'indication':'flashing_green'}
        complete = self.mission.get('state')=='complete'
        checked = len(self.arrivals)==len(self.scenario['checks'])
        passed = complete and checked and not self.failures
        if self.scenario.get('require_heartbeat_outage',False):
            passed = passed and self.blackout_samples>0
        status = {
            'scenario':self.scenario['id'],'scope':self.scenario['scope'],
            'state':self.mission.get('state','waiting'),'elapsed_sim_s':self.mission.get('elapsed_sim_s',0),
            'budget_sim_s':self.scenario['budget_s'],'completed_navigation_targets':len(self.arrivals),
            'total_navigation_targets':len(self.scenario['checks']),'arrivals':self.arrivals,
            'ideal_pose_xy_m':[round(p.x,2),round(p.y,2)],'surface_clearance_m':round(p.z-ground,3) if math.isfinite(ground) else None,
            'max_tilt_deg':round(self.max_tilt,2),'c2_heartbeat_available':connected,
            'heartbeat_blackout_observed':self.blackout_samples>0,
            'navigation_checks_passed':passed,'failures':self.failures,
            'unimplemented_tasks':self.scenario['unimplemented'],
            'official_score':None,'localization':'ideal simulator pose, not a tested GNSS estimator',
            'communications':'terrain line-of-sight / forced heartbeat dropout only, not full network or RF simulation'}
        payload = json.dumps(status,allow_nan=False)
        self.pub.publish(String(data=payload))
        self.report.parent.mkdir(parents=True,exist_ok=True)
        self.report.with_suffix('.tmp').write_text(payload+'\n')
        self.report.with_suffix('.tmp').replace(self.report)


def main(args=None):
    rclpy.init(args=args)
    node=Referee()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
