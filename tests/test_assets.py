from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_all_xml_and_yaml_parse():
    for suffix in ('*.sdf', '*.xml', '*.config'):
        for path in (ROOT/'src').rglob(suffix):
            ET.parse(path)
    for path in (ROOT/'src').rglob('*.yaml'):
        assert yaml.safe_load(path.read_text()) is not None


def test_only_supervisor_output_is_bridged_to_motors():
    bridge = yaml.safe_load((ROOT/'src/urc_bringup/config/bridge.yaml').read_text())
    commands = [entry for entry in bridge if entry['direction'] == 'ROS_TO_GZ']
    assert len(commands) == 1 and commands[0]['ros_topic_name'] == '/cmd_vel'
