import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

from scoutfetch_mission.random_scene import generate_room_map, generate_scene


SOURCE = Path(__file__).resolve().parents[2] / "scoutfetch_sim/worlds/pick_place.sdf"


def test_random_scene_is_repeatable_supported_and_clear_of_robot(tmp_path):
    locations = set()
    for seed in range(40):
        path = tmp_path / "scene.sdf"
        generate_scene(SOURCE, path, seed)
        content = path.read_bytes()
        generate_scene(SOURCE, path, seed)
        assert path.read_bytes() == content
        world = ET.parse(path).getroot().find("world")
        chair = world.find("model[@name='chair']")
        bottle = world.find("model[@name='water_bottle']")
        cx, cy, _, _, _, yaw = map(float, chair.findtext("pose").split())
        bx, by, bz, _, _, _ = map(float, bottle.findtext("pose").split())
        assert 0.949 < math.hypot(bx, by) < 1.401
        assert bz == 0.46  # seat upper surface
        assert math.hypot(bx - (cx - 0.12 * math.cos(yaw)),
                          by - (cy - 0.12 * math.sin(yaw))) < 2e-6
        assert math.hypot(cx, cy) - 0.42 > 0.60
        assert max(abs(cx), abs(cy)) + 0.42 < 2.45
        assert bottle.findtext("static") == "false"
        assert bottle.find("link/collision") is not None
        locations.add((bx, by))
    assert len(locations) == 40
    assert any(x < 0 for x, _ in locations) and any(x > 0 for x, _ in locations)
    assert any(y < 0 for _, y in locations) and any(y > 0 for _, y in locations)


def test_room_map_keeps_walls_and_excludes_old_chair(tmp_path):
    path = tmp_path / "room.yaml"
    generate_room_map(SOURCE, path)
    header, shape, maximum, cells = path.with_suffix(".pgm").read_bytes().split(b"\n", 3)
    assert header == b"P5" and maximum == b"255"
    width, height = map(int, shape.split())
    assert len(cells) == width * height

    def cell(x, y):
        col = int((x + 3.2) / 0.05)
        row = height - 1 - int((y + 2.7) / 0.05)
        return cells[row * width + col]

    assert cell(0.0, 0.0) == 254
    assert cell(1.0, 0.0) == 254  # no fixed chair ground truth
    assert cell(3.0, 0.0) == 0
    assert cell(0.0, 2.5) == 0
    assert cell(-3.1, 0.0) == 205


def test_far_random_layouts_stay_supported_and_inside_walls(tmp_path):
    for seed in range(100):
        path = tmp_path / 'far.sdf'
        generate_scene(SOURCE, path, seed, radius_min=1.6, radius_max=2.4)
        world = ET.parse(path).getroot().find('world')
        bottle = world.find("model[@name='water_bottle']")
        x, y, *_ = map(float, bottle.findtext('pose').split())
        assert 1.599 < math.hypot(x, y) < 2.401
        chair = world.find("model[@name='chair']")
        cx, cy, _, _, _, yaw = map(float, chair.findtext('pose').split())
        c, s = math.cos(yaw), math.sin(yaw)
        for dx in (-0.275, 0.275):
            for dy in (-0.30, 0.30):
                assert -2.951 < cx+dx*c-dy*s < 2.901
                assert abs(cy+dx*s+dy*c) < 2.401


def test_empty_scene_retains_separate_delayed_bottle_fixture(tmp_path):
    path = tmp_path / 'empty.sdf'
    generate_scene(SOURCE, path, 1, radius_min=2.2, radius_max=2.4, include_bottle=False)
    world = ET.parse(path).getroot().find('world')
    assert world.find("model[@name='water_bottle']") is None
    assert world.find("model[@name='chair']") is not None
    assert ET.parse(tmp_path / 'bottle.sdf').getroot().find("model[@name='water_bottle']") is not None


@pytest.mark.parametrize('low,high', [(0.1, 1.0), (2.0, 1.5), (1.0, 3.0), (float('nan'), 2.0)])
def test_invalid_search_radius_is_rejected(tmp_path, low, high):
    with pytest.raises(ValueError):
        generate_scene(SOURCE, tmp_path / 'bad.sdf', 1, low, high)
