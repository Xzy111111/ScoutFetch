"""Derive Gazebo contact physics from the canonical URDF, preserving visuals.

The CAD wheel is one rigid mesh, not independent freely rolling rollers. Use
the Fortress example's sphere + chassis-fixed diagonal friction approximation:
https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/mecanum_drive.sdf
This changes simulation contact physics only, not the shared robot geometry.
"""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


RADIUS = 0.07687545  # measured CAD radial envelope, metres
WHEELS = {
    "F_F_L_Link": (-0.0387764, "1 -1 0"),
    "F_F_R_Link": (0.0387767, "1 1 0"),
    "F_B_L_Link": (-0.0387767, "1 1 0"),
    "F_B_R_Link": (-0.0387764, "1 -1 0"),
}


def prepare_sim_model(robot_description, directory):
    path = Path(directory)
    urdf = path / "mc_arm.urdf"
    urdf.write_text(robot_description, encoding="utf-8")
    result = subprocess.run(["ign", "sdf", "-p", str(urdf)],
                            check=True, text=True, capture_output=True)
    root = ET.fromstring(result.stdout)
    model = root.find("model")
    # GPU lidar renders visuals. Give the robot its own visibility bit so the
    # near clip can be 5 cm without the scanner being blinded by its own CAD
    # housing/wheels. GUI and RGB camera retain the default all-bits mask.
    for visual in model.findall('.//visual'):
        flag = visual.find('visibility_flags')
        if flag is None:
            flag = ET.SubElement(visual, 'visibility_flags')
        flag.text = '2'
    for sensor in model.findall('.//sensor'):
        if sensor.get('type') == 'gpu_lidar':
            lidar = sensor.find('lidar')
            if lidar is None:
                lidar = sensor.find('ray')
            if lidar is None:
                raise ValueError('GPU lidar geometry missing')
            ET.SubElement(lidar, 'visibility_mask').text = '1'
    # Independent physical acceptance telemetry; never an input to autonomy.
    # All links share one Gazebo contact topic, including wheel/floor contacts
    # so a silent sensor cannot be mistaken for a collision-free run.
    for link in model.findall('link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        sensor = ET.SubElement(link, 'sensor', name='physical_contacts', type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '20'
        ET.SubElement(sensor, 'topic').text = '/mc_arm/physical_contacts'
        contact = ET.SubElement(sensor, 'contact')
        # Fortress Contact reads contact/topic (not sensor/topic).
        ET.SubElement(contact, 'topic').text = '/mc_arm/physical_contacts'
        for collision in collisions:
            ET.SubElement(contact, 'collision').text = collision.get('name')
    for name, (center_y, direction) in WHEELS.items():
        link = model.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError(f"canonical wheel link missing: {name}")
        collisions = link.findall("collision")
        if len(collisions) != 1:
            raise ValueError(f"expected one wheel collision: {name}")
        collision = collisions[0]
        for child in list(collision):
            collision.remove(child)
        ET.SubElement(collision, "pose").text = f"0 {center_y} 0 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = str(RADIUS)
        ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, "surface"), "friction"), "ode")
        ET.SubElement(ode, "mu").text = "1.0"
        ET.SubElement(ode, "mu2").text = "0.0"
        fdir = ET.SubElement(ode, "fdir1")
        fdir.set("{http://ignitionrobotics.org/schema}expressed_in", "base_link")
        fdir.text = direction
    ET.register_namespace("ignition", "http://ignitionrobotics.org/schema")
    sdf = path / "mc_arm.sdf"
    root.set("version", "1.9")
    ET.ElementTree(root).write(sdf, encoding="utf-8", xml_declaration=True)
    return str(sdf)
