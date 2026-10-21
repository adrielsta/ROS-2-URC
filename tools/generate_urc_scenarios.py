#!/usr/bin/env python3
"""Generate real-terrain practice assets from the versioned Utah patch.

Targets/fixtures are invented practice locations, never organizer coordinates.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from pyproj import Transformer
import yaml
from generate_training import write_xml
from route_preview import astar, inflate

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/urc_brain'))
from urc_brain.terrain_model import Terrain
SHARE=ROOT/'src/urc_bringup'
SITE=SHARE/'models/utah_site'


def box(name,x,y,z,size,color,collision=True):
    geo=f'<geometry><box><size>{size}</size></box></geometry>'
    hit=f'<collision name="collision">{geo}</collision>' if collision else ''
    return f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name="link">{hit}<visual name="visual">{geo}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'


def marker(tag,x,y,ground):
    bits=json.loads((SITE/'aruco_bits.json').read_text())[str(tag)]
    faces=''
    for face in range(3):
        angle=face*2*math.pi/3
        faces+=f'<link name="face_{face}"><pose>{0.058*math.cos(angle)} {0.058*math.sin(angle)} 1.1 0 0 {angle}</pose>'
        faces+='<visual name="white"><geometry><box><size>0.004 0.2 0.2</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>'
        for r in range(6):
            for c in range(6):
                if bits[r][c]: continue
                faces+=f'<visual name="cell_{r}_{c}"><pose>0.003 {(2.5-c)*.025} {(2.5-r)*.025} 0 0 0</pose><geometry><box><size>0.002 0.025 0.025</size></box></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual>'
        faces+='</link>'
    return f'<model name="target_{tag}"><static>true</static><pose>{x} {y} {ground} 0 0 0</pose><link name="post"><pose>0 0 .5 0 0 0</pose><collision name="post"><geometry><cylinder><radius>.025</radius><length>1</length></cylinder></geometry></collision><visual name="post"><geometry><cylinder><radius>.025</radius><length>1</length></cylinder></geometry><material><diffuse>.3 .3 .3 1</diffuse></material></visual></link>{faces}</model>'


def main():
    terrain=Terrain(SITE); m=terrain.meta; step=terrain.step; half=terrain.center
    slope=np.asarray(Image.open(SITE/'slope.png'),dtype=float)/1000
    cost=np.where(slope<=18,1+4*(slope/18)**2,np.inf)
    cost[[0,-1],:]=np.inf;cost[:,[0,-1]]=np.inf
    pixels=np.where(np.isfinite(cost),254-np.minimum(100,slope/18*100),0).astype('uint8')
    Image.fromarray(pixels).save(SITE/'prior.png')
    (SHARE/'maps/utah.yaml').write_text(yaml.safe_dump({'image':'../models/utah_site/prior.png',
        'mode':'scale','resolution':step,'origin':[-(half+.5)*step,-(half+.5)*step,0],
        'negate':0,'occupied_thresh':.99,'free_thresh':.01},sort_keys=False))

    model=ET.parse(SHARE/'models/urc_rover/model.sdf').getroot()
    base=model.find('model')
    # Keep encoder output separate; exactly one pose source drives navigation TF.
    drive=base.find('plugin')
    drive.find('tf_topic').text='/simulation/wheel_tf_unused'
    drive.find('odom_topic').text='/simulation/wheel_odometry'
    base.append(ET.fromstring('''<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><dimensions>3</dimensions><odom_frame>odom</odom_frame><robot_base_frame>base_link</robot_base_frame><odom_topic>/simulation/ground_truth/odometry</odom_topic><tf_topic>/simulation/ground_truth/tf</tf_topic><odom_publish_frequency>30</odom_publish_frequency></plugin>'''))
    rover=SHARE/'models/urc_rover_utah';rover.mkdir(exist_ok=True)
    write_xml(rover/'model.sdf',ET.tostring(model,encoding='unicode'))
    write_xml(rover/'model.config','<model><name>URC simulation truth base</name><version>0.2</version><sdf version="1.9">model.sdf</sdf><description>Placeholder chassis with ideal simulation pose; never a production localization configuration.</description></model>')

    bridge=yaml.safe_load((SHARE/'config/bridge.yaml').read_text())
    for item in bridge:
        if item['ros_topic_name']=='/odom':item['gz_topic_name']='/simulation/ground_truth/odometry'
        if item['ros_topic_name']=='/tf':item['gz_topic_name']='/simulation/ground_truth/tf'
        if item['ros_topic_name']=='/scan':item['ros_topic_name']='/scan_raw'
    (SHARE/'config/utah_bridge.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
    nav=yaml.safe_load((SHARE/'config/nav2.yaml').read_text())
    nav['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']=.12
    nav['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']=.35
    nav['controller_server']['ros__parameters']['FollowPath']['use_cost_regulated_linear_velocity_scaling']=False
    for layer_name in ('local_costmap','global_costmap'):
        cfg=nav[layer_name][layer_name]['ros__parameters']
        cfg['obstacle_layer']['scan']['min_obstacle_height']=-100.0
        cfg['obstacle_layer']['scan']['max_obstacle_height']=100.0
        cfg['obstacle_layer']['min_obstacle_height']=-100.0
        cfg['obstacle_layer']['max_obstacle_height']=100.0
        cfg['inflation_layer']['inflation_radius']=.75
    local=nav['local_costmap']['local_costmap']['ros__parameters']
    local['plugins']=['static_layer','obstacle_layer','inflation_layer']
    local['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True,'trinary_costmap':False}
    nav['global_costmap']['global_costmap']['ros__parameters']['resolution']=step
    (SHARE/'config/utah_nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))

    # The route targets are natural hill / far-side locations selected from the
    # DEM, not a modified or fabricated hill. Approach paths are prior-only checks.
    hill=[-39*step,44*step]; far=[-50*step,65*step]
    definitions={
      'utah':('Short actual-terrain navigation check',2400,[[6,0],[9,4]],.75,
          ['Outdoor localization','Full hill route','Astronaut assistance']),
      'utah-route':('Autonomy route-finding practice, rules 1.e.xii–xviii',2400,[hill,far],1.0,
          ['GNSS/IMU fusion','ArUco detection/refinement','Penalty-bearing breadcrumb retreat','Astronaut assistance']),
      'astronaut-approach':('Initial astronaut rendezvous only, rule 1.e.v',2400,[[12,2]],3.0,
          ['Suit camera/microphone','Follow gesture/voice perception','Stay while astronaut moves >20m','Fetch rock hammer','Come to moving astronaut','Give hammer']),
      'delivery-navigation':('Pickup/drop navigation checkpoints only, rules 1.c.i–v',1800,[[12,-2],[22,6],[6,12]],1.0,
          ['Official stage script and stage score gate','Grasping/carrying/delivering objects','Opening toolboxes','Reading signs','Search and real radio link']),
      'science-navigation':('Two selected site visits and return only, rules 1.b.ii–vii',1200,[[10,-4],[-8,-10],[0,0]],1.0,
          ['Site selection reasoning','Panorama/close-up evidence','Two life detection capabilities including bioassay','At least 10cm-deep and 5g sample','Seal and remove cache','Judges debrief']),
      'equipment-approach':('Lander approach only, rule 1.d.i',1800,[[12,0]],.75,
          ['Sample tube/cache/lock/drawer','Hinged panel latch','Autonomous 3–6 letter keyboard typing','Transfer and turn key','Cam-lock hose fitting','Valve/buttons/switches/knobs'])}
    scenario_dir=SHARE/'config/scenarios';scenario_dir.mkdir(exist_ok=True)
    transformer=Transformer.from_crs(m['source_crs'],4326,always_xy=True)
    inflated=inflate(cost,.6/step)
    for name,(scope,budget,points,radius,missing) in definitions.items():
        checks=[];targets=[];fixtures='';route_evidence=[];previous=[0.,0.]
        for i,point in enumerate(points):
            pos=list(map(float,point));approach=pos[:];yaw=0.
            if name=='utah-route':
                start=(round(half-previous[1]/step),round(half+previous[0]/step))
                goal=(round(half-pos[1]/step),round(half+pos[0]/step))
                path=astar(inflated,start,goal)
                a,b=path[-2:];direction=np.array([b[1]-a[1],a[0]-b[0]],dtype=float);direction/=np.linalg.norm(direction)
                approach=(np.array(pos)-.8*direction).tolist();yaw=math.atan2(direction[1],direction[0])
                route_evidence.append({'length_m':sum(math.dist(a,b)*step for a,b in zip(path,path[1:])),
                    'max_slope_deg':max(float(slope[r,c]) for r,c in path)})
                fixtures+=marker(i,*pos,terrain.height(*pos))
            lon,lat=transformer.transform(m['local_origin_projected_m'][0]+pos[0],m['local_origin_projected_m'][1]+pos[1])
            checks.append({'id':f'target_{i+1}','xy':pos,'radius_m':radius,'latitude':lat,'longitude':lon})
            targets.append({'x':approach[0],'y':approach[1],'yaw':yaw})
            previous=pos
            if name not in ('utah-route','equipment-approach','astronaut-approach'):
                fixtures+=box(f'checkpoint_{i}',*pos,terrain.height(*pos)+.015,'1.0 1.0 .02','.1 .5 .8 1',False)
        if name=='equipment-approach':
            fixtures+=box('practice_lander',14,0,terrain.height(14,0)+.75,'1 2 1.5','.55 .55 .6 1')
        if name=='astronaut-approach':
            fixtures+=box('astronaut_placeholder',14.5,2,terrain.height(14.5,2)+.85,'.4 .4 1.7','.9 .9 .9 1')
        scenario={'id':name,'scope':scope,'budget_s':budget,'frame':'map','targets':targets,
            'checks':checks,'unimplemented':missing,'heartbeat_outage_after_first':name=='utah',
            'require_heartbeat_outage':name in ('utah','utah-route'),
            'route_prior_evidence':route_evidence,'practice_locations_not_official':True,
            'terrain_los_to_targets':[terrain.line_of_sight([0,0],p) for p in points]}
        (scenario_dir/f'{name}.yaml').write_text(yaml.safe_dump(scenario,sort_keys=False))
        xml=ET.parse(SHARE/'worlds/training.sdf').getroot();world=xml.find('world');world.set('name','urc_practice')
        for element in list(world):
            if element.tag in ('model','include'):world.remove(element)
        physics=world.find('physics');physics.append(ET.fromstring('<dart><collision_detector>bullet</collision_detector></dart>'))
        coords=world.find('spherical_coordinates');coords.find('latitude_deg').text=str(m['local_origin_wgs84'][1]);coords.find('longitude_deg').text=str(m['local_origin_wgs84'][0]);coords.find('elevation').text=str(m['spawn_elevation_m'])
        shift=m['baseline_elevation_m']-m['spawn_elevation_m'];size=m['physical_size_m']
        shape=f'<uri>model://utah_site/height.png</uri><size>{size} {size} {m["height_scale_m"]}</size><pos>0 0 0</pos><sampling>1</sampling>'
        tex=f'<texture><size>{size}</size><diffuse>model://utah_site/aerial.jpg</diffuse><normal>model://utah_site/normal.png</normal></texture>'
        # Apply datum translation at the model frame. The DART heightfield
        # backend does not consistently apply the heightmap-specific <pos>.
        world.append(ET.fromstring(f'<model name="real_utah_terrain"><static>true</static><pose>0 0 {shift} 0 0 0</pose><link name="terrain"><collision name="terrain"><geometry><heightmap>{shape}</heightmap></geometry></collision><visual name="terrain"><geometry><heightmap>{shape}{tex}<use_terrain_paging>false</use_terrain_paging></heightmap></geometry></visual></link></model>'))
        world.append(ET.fromstring('<include><uri>model://urc_rover_utah</uri><name>urc_rover</name><pose>0 0 .55 0 0 0</pose></include>'))
        for fixture in ET.fromstring('<fixtures>'+fixtures+'</fixtures>'):world.append(fixture)
        write_xml(SHARE/f'worlds/{name}.sdf',ET.tostring(xml,encoding='unicode'))
    print('Generated six real-terrain practice profiles. Coordinates are illustrative; no official scores.')


if __name__=='__main__':main()
