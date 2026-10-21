import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from PIL import Image
from urc_brain.terrain_model import Terrain, rotate, stopped_arrival, planar_rotation
from urc_brain.mission_plan import validate_plan

SHARE=Path(__file__).resolve().parents[1]/'src/urc_bringup'


def test_bundled_coordinate_origin_and_height():
    t=Terrain(SHARE/'models/utah_site')
    assert abs(t.height(0,0))<.001
    assert math.isnan(t.height(10000,0))
    assert math.isnan(t.height(float('nan'),0))
    # The chosen natural hill is above the start, and its far side is occluded.
    assert t.height(-39*t.step,44*t.step)>5
    assert not t.line_of_sight([0,0],[-50*t.step,65*t.step])


def test_rotation_and_stopped_arrival_checks():
    assert rotate([0,0,math.sqrt(.5),math.sqrt(.5)],[1,0,0])==pytest.approx([0,1,0])
    assert rotate([math.sqrt(.5),0,0,math.sqrt(.5)],[0,1,0])==pytest.approx([0,0,1])
    with pytest.raises(ValueError):rotate([0,0,0,0],[1,0,0])
    assert stopped_arrival([.8,0],[0,0],0,0,'flashing_green',1)
    assert not stopped_arrival([1.01,0],[0,0],0,0,'flashing_green',1)
    assert not stopped_arrival([.8,0],[0,0],.1,0,'flashing_green',1)
    assert not stopped_arrival([.8,0],[0,0],0,0,'red',1)


def test_profiles_are_explicit_subsets_and_have_safe_start():
    t=Terrain(SHARE/'models/utah_site')
    files=list((SHARE/'config/scenarios').glob('*.yaml'))
    assert len(files)==6
    for file in files:
        s=yaml.safe_load(file.read_text())
        goals=validate_plan(s)
        assert s['practice_locations_not_official']
        assert s['unimplemented']
        assert len(goals)==len(s['checks'])
        for target in s['checks']:
            assert math.isfinite(t.height(*target['xy']))
            assert 38.411<target['latitude']<38.425
            assert -110.786<target['longitude']<-110.768
        ET.parse(SHARE/'worlds'/f'{s["id"]}.sdf')
    s=yaml.safe_load((SHARE/'config/scenarios/utah-route.yaml').read_text())
    assert s['budget_s']==2400
    assert len(s['checks'])==2
    assert all(x['radius_m']==1 for x in s['checks'])
    assert s['terrain_los_to_targets'][1] is False
    assert all(x['max_slope_deg']<=18 for x in s['route_prior_evidence'])
    grid=yaml.safe_load((SHARE/'maps/utah.yaml').read_text())
    origin=np.array(grid['origin'][:2]);resolution=grid['resolution']
    for target,check in zip(s['targets'],s['checks']):
        xy=np.array([target['x'],target['y']])
        endpoint=origin+np.floor((xy-origin)/resolution)*resolution
        assert np.linalg.norm(endpoint-np.array(check['xy']))+.12<check['radius_m']
    astronaut=yaml.safe_load((SHARE/'config/scenarios/astronaut-approach.yaml').read_text())
    fixture=ET.parse(SHARE/'worlds/astronaut-approach.sdf').find(".//model[@name='astronaut_placeholder']/pose")
    assert astronaut['checks'][0]['xy']==list(map(float,fixture.text.split()[:2]))
    assert astronaut['targets'][0]['x']==12.0


def test_ideal_pose_does_not_conflict_with_encoder_tf():
    model=ET.parse(SHARE/'models/urc_rover_utah/model.sdf').getroot()
    plugins=model.findall('.//plugin')
    assert len(plugins)==2
    assert plugins[0].find('tf_topic').text!=plugins[1].find('tf_topic').text
    assert plugins[1].find('dimensions').text=='3'
    bridge=yaml.safe_load((SHARE/'config/utah_bridge.yaml').read_text())
    assert sum(x['ros_topic_name']=='/tf' for x in bridge)==0
    assert sum(x['ros_topic_name']=='/simulation/ground_truth' for x in bridge)==1
    assert any(x['ros_topic_name']=='/scan_raw' for x in bridge)


def test_level_navigation_frame_preserves_tilted_body_rotation():
    q=[.15,-.2,.3,.92]
    q=np.array(q)/np.linalg.norm(q)
    planar,tilt=planar_rotation(q)
    for vector in np.eye(3):
        assert rotate(planar,rotate(tilt,vector))==pytest.approx(rotate(q,vector))
    assert rotate(planar,[0,0,1])==pytest.approx([0,0,1])


def test_harmonic_heightfield_datum_and_sampling_alignment():
    t=Terrain(SHARE/'models/utah_site')
    world=ET.parse(SHARE/'worlds/utah.sdf')
    model=world.find(".//model[@name='real_utah_terrain']")
    assert float(model.find('pose').text.split()[2])==pytest.approx(t.meta['baseline_elevation_m']-t.meta['spawn_elevation_m'])
    collision=model.find('.//collision/geometry/heightmap')
    visual=model.find('.//visual/geometry/heightmap')
    assert collision.find('pos').text=='0 0 0'
    assert float(collision.find('size').text.split()[0])/t.meta['grid_size']==pytest.approx(t.step)
    assert float(visual.find('size').text.split()[0])/(t.meta['grid_size']-1)==pytest.approx(t.step)


def test_tag_geometry_matches_rule_dimensions():
    world=ET.parse(SHARE/'worlds/utah-route.sdf')
    for model in world.findall('.//model'):
        if not model.attrib['name'].startswith('target_'):continue
        faces=[x for x in model.findall('link') if x.attrib['name'].startswith('face_')]
        assert len(faces)==3
        for face in faces:
            assert face.find("visual[@name='white']/geometry/box/size").text=='0.004 0.2 0.2'
            assert float(face.find('pose').text.split()[2])==1.1


def test_navigation_grid_preserves_prior_and_fits_controller_lookahead():
    coarse=np.asarray(Image.open(SHARE/'models/utah_site/prior.png'))
    fine=np.asarray(Image.open(SHARE/'models/utah_site/navigation.png'))
    assert fine.shape==(4*coarse.shape[0],4*coarse.shape[1])
    assert np.array_equal(fine,np.repeat(np.repeat(coarse,4,axis=0),4,axis=1))
    spec=yaml.safe_load((SHARE/'maps/utah.yaml').read_text())
    nav=yaml.safe_load((SHARE/'config/utah_nav2.yaml').read_text())
    lookahead=nav['controller_server']['ros__parameters']['FollowPath']['min_lookahead_dist']
    tolerance=nav['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
    # RPP deliberately retains the final two path poses, even after the
    # nearer pose becomes the endpoint. A diagonal segment must fit too.
    assert math.sqrt(2)*spec['resolution']+tolerance<lookahead
    assert len(fine)*spec['resolution']==pytest.approx(len(coarse)*1.52)
