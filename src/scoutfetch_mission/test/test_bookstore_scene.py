"""Bookstore fixtures must have support, clearance and connected approaches."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import cv2

from scoutfetch_mission.bookstore_scene import (SUPPORT_HEIGHT, STATIONS, base_free,
                                         generate_bookstore, mesh_triangles, surface_below)

GAZEBO = Path(__file__).resolve().parents[2]/'scoutfetch_sim'
ASSETS = GAZEBO/'models/aws_bookstore'
LAB = GAZEBO/'worlds/pick_place.sdf'


@pytest.mark.parametrize('station', range(6))
def test_random_bottle_stays_supported_with_robust_access(tmp_path, station):
    for seed in range(20):
        audit = generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', seed, station)
        assert audit['surface_edge_margin_m'] >= .09
        assert audit['bottle_xyz'][2] == SUPPORT_HEIGHT == .46
        assert audit['connected_robust_approaches']
        assert all(.5 <= a[3] <= .6 for a in audit['connected_robust_approaches'])
        assert len(audit['retained_models']) >= 60
        assert sum(n.startswith('Bookshelf') for n in audit['retained_models']) >= 12
        world = ET.parse(tmp_path/'scene.sdf').find('world')
        assert world.get('name') == 'pick_place'
        assert len(world.findall("model[@name='water_bottle']")) == 1
        assert len([m for m in world.findall('model') if m.get('name').startswith('pickup_table_')]) == 6
        assert world.find("model[@name='water_bottle']/static").text == 'false'
        for uri in world.findall('.//geometry/mesh/uri'):
            assert Path(uri.text.removeprefix('file://')).is_file()


def test_navigation_map_includes_overhangs_without_changing_amcl_slice(tmp_path):
    generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', 7, 2)
    scan = cv2.imread(str(tmp_path/'bookstore.pgm'), 0)
    navigation = cv2.imread(str(tmp_path/'bookstore_navigation.pgm'), 0)
    assert np.all(navigation[scan == 0] == 0)
    assert np.count_nonzero((scan == 254) & (navigation == 0)) > 1000
    assert base_free(navigation, 0., 0., 0.)


def test_seed_reproducibility_and_random_station_coverage(tmp_path):
    stations = set()
    for seed in range(40):
        audit = generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', seed)
        stations.add(audit['station'])
        if seed == 7:
            same = generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', seed)
            assert audit == same
    assert stations == set(range(6))


def test_empty_bookstore_has_no_target_and_unsupported_station_is_rejected(tmp_path):
    generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', 3, include_bottle=False)
    assert ET.parse(tmp_path/'scene.sdf').find("world/model[@name='water_bottle']") is None
    assert ET.parse(tmp_path/'bottle.sdf').find("model[@name='water_bottle']") is not None
    with pytest.raises(ValueError, match='station'):
        generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', 3, station=6)


def test_collada_units_and_scene_transforms_are_used_for_map_geometry():
    floor = next(ASSETS.glob('models/*Floor*/meshes/*collision.DAE'))
    triangles = mesh_triangles(str(floor))
    size = np.ptp(triangles.reshape(-1, 3), axis=0)
    assert 15 < size[0] < 16 and 14 < size[1] < 15
    assert size[2] < .3  # source is centimetres, not metres


def test_footprint_check_rejects_obstacles_and_unknown_cells():
    grid = np.full((400, 400), 254, dtype=np.uint8)
    assert base_free(grid, 0, 0, 0)
    grid[199, 200] = 205
    assert not base_free(grid, 0, 0, 0)
    grid[199, 200] = 0
    assert not base_free(grid, 0, 0, 0)


def test_removed_tables_take_all_their_props_including_outside_clearance_zone(tmp_path):
    audit = generate_bookstore(ASSETS, LAB, tmp_path/'scene.sdf', 1)
    kept = set(audit['retained_models'])
    for name in ('DeskA_01_001', 'DeskA_01_002', 'DeskA_01_004',
                 'BookE_01_006', 'BookE_01_020', 'Tablet_01_004', 'Holder_01_008',
                 'BookE_01_009'):  # original source contains an impossible z=343582 m
        assert name not in kept
    for name, support in audit['decor_supports'].items():
        if name in kept:
            assert support['support_retained'] and support['support'] in kept


def test_collada_translation_is_in_the_last_column(tmp_path):
    path = tmp_path/'translated.dae'
    path.write_text('''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
      <asset><unit meter="1"/></asset><library_geometries><geometry id="g"><mesh>
      <source id="p"><float_array>0 0 0 1 0 0 0 1 0</float_array>
      <technique_common><accessor stride="3"/></technique_common></source>
      <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
      <triangles><input semantic="VERTEX" source="#v"/><p>0 1 2</p></triangles>
      </mesh></geometry></library_geometries><library_visual_scenes><visual_scene>
      <node><matrix>1 0 0 2 0 1 0 3 0 0 1 4 0 0 0 1</matrix>
      <instance_geometry url="#g"/></node></visual_scene></library_visual_scenes></COLLADA>''')
    triangle = mesh_triangles(str(path))[0]
    np.testing.assert_allclose(triangle, [[2, 3, 4], [3, 3, 4], [2, 4, 4]])


def test_support_ray_rejects_other_locations_and_finds_actual_surface():
    triangles = np.array([[[0, 0, 1], [2, 0, 1], [0, 2, 1]]], dtype=float)
    assert surface_below(triangles, .5, .5, 1.1) == 1.0
    assert surface_below(triangles, 3, 3, 1.1) is None
    assert surface_below(triangles, .5, .5, .8) is None
