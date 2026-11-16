import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sim_model_test_helper', ROOT / 'scripts/sim_model.py')
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def description():
    return xacro.process_file(str(ROOT.parent / 'scoutfetch_description/urdf/scoutfetch.urdf.xacro'),
                              mappings={'sim_mode': 'true'}).toxml()


def test_contact_model_keeps_cad_and_arm_physics_but_models_roller_slip(tmp_path):
    sdf = ET.parse(HELPER.prepare_sim_model(description(), tmp_path)).getroot()
    for name, (_, direction) in HELPER.WHEELS.items():
        link = sdf.find(f"model/link[@name='{name}']")
        assert link.find('visual/geometry/mesh/uri').text.endswith(name + '.STL')
        assert link.find('inertial/mass') is not None
        assert float(link.findtext('collision/geometry/sphere/radius')) == HELPER.RADIUS
        friction = link.find('collision/surface/friction/ode')
        assert float(friction.findtext('mu')) > 0.0
        assert float(friction.findtext('mu2')) == 0.0
        assert friction.findtext('fdir1') == direction
        assert friction.find('fdir1').attrib['{http://ignitionrobotics.org/schema}expressed_in'] == 'base_link'
    # The actual arm and contact gripper remain physical meshes, not teleported.
    assert sdf.find("model/link[@name='link6']/collision/geometry/mesh") is not None
    assert sdf.find("model/link[@name='gripper_link1']/collision/geometry/mesh") is not None
    assert all(v.findtext('visibility_flags') == '2' for v in sdf.findall('.//visual'))
    lidar = sdf.find(".//sensor[@type='gpu_lidar']/lidar")
    assert lidar.findtext('visibility_mask') == '1'
    assert float(lidar.findtext('range/min')) <= .05
    for link in sdf.findall('model/link'):
        if link.findall('collision'):
            sensor = link.find("sensor[@type='contact']")
            assert sensor.findtext('contact/topic') == '/mc_arm/physical_contacts'
            assert {e.text for e in sensor.findall('contact/collision')} == {
                e.get('name') for e in link.findall('collision')}


def test_drive_geometry_matches_effective_wheel_contacts():
    urdf = ET.fromstring(description())
    centers = []
    for link, (cy, _) in HELPER.WHEELS.items():
        joint = urdf.find(f"joint[@name='{link[:-5]}']")
        x, y, _ = map(float, joint.find('origin').attrib['xyz'].split())
        yaw = float(joint.find('origin').attrib['rpy'].split()[2])
        centers.append((x - math.sin(yaw)*cy, y + math.cos(yaw)*cy))
    kinematics = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())['base_controller']['ros__parameters']['kinematics']
    expected = sum(abs(x) + abs(y) for x, y in centers) / 4
    assert abs(kinematics['sum_of_robot_center_projection_on_X_Y_axis'] - expected) < 1e-5
    assert kinematics['wheels_radius'] == HELPER.RADIUS
