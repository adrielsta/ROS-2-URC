#!/usr/bin/env python3
"""Generate the checked-in SDF model/world and occupancy map from one description."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / 'src/urc_bringup'
OBSTACLES = [(3.0, 0.0, 1.2, 1.6, 1.2), (2.0, 4.5, 1.0, 1.0, 0.8)]


def write_xml(path, xml):
    root = ET.fromstring(xml)
    ET.indent(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)


def generate():
    wheels = ''
    for name, y in [('left', 0.34), ('right', -0.34)]:
        wheels += f'''
        <link name="{name}_wheel"><pose>0 {y} -0.12 {math.pi/2} 0 0</pose>
          <inertial><mass>1</mass><inertia><ixx>0.007</ixx><iyy>0.007</iyy><izz>0.013</izz></inertia></inertial>
          <collision name="collision"><geometry><cylinder><radius>0.16</radius><length>0.08</length></cylinder></geometry></collision>
          <visual name="visual"><geometry><cylinder><radius>0.16</radius><length>0.08</length></cylinder></geometry><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
        </link>
        <joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child><axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'''
    for name, x in [('front', 0.32), ('rear', -0.32)]:
        wheels += f'''
        <link name="{name}_caster"><pose>{x} 0 -0.21 0 0 0</pose>
          <inertial><mass>0.1</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz></inertia></inertial>
          <collision name="collision"><geometry><sphere><radius>0.07</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
          <visual name="visual"><geometry><sphere><radius>0.07</radius></sphere></geometry></visual>
        </link><joint name="{name}_caster_joint" type="fixed"><parent>base_link</parent><child>{name}_caster</child></joint>'''
    model = f'''<sdf version="1.9"><model name="urc_rover">
      <link name="base_link">
        <inertial><mass>20</mass><inertia><ixx>0.67</ixx><iyy>1.13</iyy><izz>1.67</izz></inertia></inertial>
        <collision name="body"><geometry><box><size>0.8 0.6 0.2</size></box></geometry></collision>
        <visual name="body"><geometry><box><size>0.8 0.6 0.2</size></box></geometry><material><diffuse>0.85 0.35 0.08 1</diffuse></material></visual>
        <sensor name="lidar" type="gpu_lidar"><pose>0.2 0 0.3 0 0 0</pose><topic>/scan</topic><gz_frame_id>lidar_link</gz_frame_id><update_rate>10</update_rate><always_on>true</always_on>
          <lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal></scan><range><min>0.15</min><max>20</max><resolution>0.01</resolution></range><noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></lidar>
        </sensor>
        <sensor name="imu" type="imu"><topic>/imu</topic><gz_frame_id>base_link</gz_frame_id><update_rate>50</update_rate><always_on>true</always_on></sensor>
        <sensor name="gps" type="navsat"><topic>/gps</topic><gz_frame_id>base_link</gz_frame_id><update_rate>5</update_rate><always_on>true</always_on></sensor>
        <sensor name="camera" type="camera"><pose>0.41 0 0.15 0 0 0</pose><topic>/camera</topic><update_rate>5</update_rate><always_on>true</always_on><camera><horizontal_fov>1.4</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.1</near><far>100</far></clip></camera></sensor>
      </link>{wheels}
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_joint</left_joint><right_joint>right_joint</right_joint>
        <wheel_separation>0.68</wheel_separation><wheel_radius>0.16</wheel_radius>
        <topic>/model/urc_rover/cmd_vel</topic><odom_topic>/model/urc_rover/odometry</odom_topic><tf_topic>/model/urc_rover/tf</tf_topic>
        <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id><odom_publish_frequency>30</odom_publish_frequency>
        <max_linear_velocity>0.5</max_linear_velocity><max_angular_velocity>0.8</max_angular_velocity>
        <max_linear_acceleration>0.5</max_linear_acceleration><max_angular_acceleration>1.0</max_angular_acceleration>
      </plugin>
    </model></sdf>'''
    write_xml(SHARE / 'models/urc_rover/model.sdf', model)
    write_xml(SHARE / 'models/urc_rover/model.config', '''<model><name>URC placeholder base</name><version>0.1</version><sdf version="1.9">model.sdf</sdf><description>Two drive wheels and passive casters; replace with measured team hardware.</description></model>''')
    obstacles = ''
    for i, (x, y, sx, sy, sz) in enumerate(OBSTACLES):
        geometry = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        obstacles += f'<model name="rock_{i}"><static>true</static><pose>{x} {y} {sz/2} 0 0 0</pose><link name="link"><collision name="collision">{geometry}</collision><visual name="visual">{geometry}<material><diffuse>0.4 0.3 0.2 1</diffuse></material></visual></link></model>'
    world = f'''<sdf version="1.9"><world name="training">
      <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
      <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
      <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
      <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>38.418</latitude_deg><longitude_deg>-110.777</longitude_deg><elevation>1370</elevation><heading_deg>0</heading_deg></spherical_coordinates>
      <light type="directional" name="sun"><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>
      <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry><material><diffuse>0.72 0.56 0.37 1</diffuse></material></visual></link></model>
      <include><uri>model://urc_rover</uri><pose>0 0 0.3 0 0 0</pose></include>{obstacles}
    </world></sdf>'''
    write_xml(SHARE / 'worlds/training.sdf', world)
    # ASCII PGM keeps this generated fixture readable and diffable in Git.
    size, resolution, origin = 150, 0.2, -10.0
    rows = []
    for row in range(size):
        y = origin + (size - row - 0.5) * resolution
        values = []
        for col in range(size):
            x = origin + (col + 0.5) * resolution
            blocked = any(abs(x-ox) <= sx/2 and abs(y-oy) <= sy/2
                          for ox, oy, sx, sy, _ in OBSTACLES)
            values.append('0' if blocked else '254')
        rows.append(' '.join(values))
    maps = SHARE / 'maps'
    maps.mkdir(exist_ok=True)
    (maps / 'training.pgm').write_text(f'P2\n{size} {size}\n255\n' + '\n'.join(rows) + '\n')
    (maps / 'training.yaml').write_text('image: training.pgm\nmode: trinary\nresolution: 0.2\norigin: [-10.0, -10.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


if __name__ == '__main__':
    generate()
