"""Gazebo fixture generation. No generated object pose enters the robot pipeline."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET


def generate_scene(source, destination, seed, radius_min=0.95, radius_max=1.4,
                   include_bottle=True):
    """Move the existing chair and dynamic bottle together, preserving geometry.

    The chair faces the robot, with the bottle on its near seat edge. Sampling
    a ring leaves a clear rotation area around the robot and avoids the walls.
    """
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    chair = world.find("model[@name='chair']")
    bottle = world.find("model[@name='water_bottle']")
    if chair is None or bottle is None:
        raise ValueError("scene must contain chair and water_bottle")
    rng = random.Random(seed)
    if (not all(math.isfinite(v) for v in (radius_min, radius_max)) or
            not 0.90 <= radius_min <= radius_max <= 2.4):
        raise ValueError("bottle radii must satisfy 0.90 <= min <= max <= 2.40 metres")
    for _ in range(1000):
        angle = rng.uniform(-math.pi, math.pi)
        radius = math.sqrt(rng.uniform(radius_min**2, radius_max**2))
        c, s = math.cos(angle), math.sin(angle)
        cx, cy = (radius + 0.12)*c, (radius + 0.12)*s
        # Reject chair corners that intersect the room walls or leave the floor.
        corners = [(cx+dx*c-dy*s, cy+dx*s+dy*c)
                   for dx in (-0.275, 0.275) for dy in (-0.30, 0.30)]
        if all(-2.95 < x < 2.90 and abs(y) < 2.40 for x, y in corners):
            break
    else:
        raise ValueError("no supported bottle placement fits the room")
    chair.find("pose").text = f"{(radius + 0.12)*c:.6f} {(radius + 0.12)*s:.6f} 0 0 0 {angle:.6f}"
    bottle.find("pose").text = f"{radius*c:.6f} {radius*s:.6f} 0.46 0 0 {angle:.6f}"
    # Separate fixture for delayed-target acceptance; it is never read by robot nodes.
    fixture = ET.Element("sdf", version="1.8")
    fixture.append(bottle)
    ET.ElementTree(fixture).write(Path(destination).with_name("bottle.sdf"),
                                  encoding="utf-8", xml_declaration=True)
    if not include_bottle:
        world.remove(bottle)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def generate_room_map(source, destination):
    """Rasterize only the unchanged room shell, excluding movable fixtures.

    This is a known simulation-room map, not object localization. The random
    chair must be observed by the existing LaserScan costmap obstacle layers.
    Avoid retaining the original chair as a permanent phantom map obstacle.
    """
    room = ET.parse(source).getroot().find("world/model[@name='laboratory_room']/link")
    boxes = {}
    for collision in room.findall("collision"):
        size = collision.find("geometry/box/size")
        if size is not None:
            pose = [float(v) for v in collision.findtext("pose").split()]
            sx, sy, _ = [float(v) for v in size.text.split()]
            boxes[collision.attrib["name"]] = (pose[0], pose[1], sx / 2, sy / 2)
    floor = boxes.pop("floor_collision")
    resolution = 0.05
    ox, oy = floor[0] - floor[2] - 0.2, floor[1] - floor[3] - 0.2
    width = round((2 * floor[2] + 0.4) / resolution)
    height = round((2 * floor[3] + 0.4) / resolution)
    cells = bytearray()
    for row in range(height):
        y = oy + (height - row - 0.5) * resolution
        for col in range(width):
            x = ox + (col + 0.5) * resolution
            occupied = any(abs(x - bx) <= hx and abs(y - by) <= hy
                           for bx, by, hx, hy in boxes.values())
            free = abs(x - floor[0]) <= floor[2] and abs(y - floor[1]) <= floor[3]
            cells.append(0 if occupied else (254 if free else 205))
    path = Path(destination)
    image = path.with_suffix(".pgm")
    image.write_bytes(f"P5\n{width} {height}\n255\n".encode() + cells)
    path.write_text(
        f"image: {image.name}\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
