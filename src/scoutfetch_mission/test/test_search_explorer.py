import math

import numpy as np

from scoutfetch_mission.search_explorer import SearchExplorer


def room():
    grid = np.zeros((120, 120), dtype=np.int16)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    return grid


def test_new_view_is_clear_connected_and_different_from_scanned_location():
    explorer = SearchExplorer()
    grid = room()
    # A complete wall separates a region which must never be selected.
    grid[:, 70] = 100
    explorer.update(grid, .1, (-6, -6, 0))
    explorer.mark_scanned(-3, -3)
    goal = explorer.choose(-3, -3)
    assert goal is not None
    assert .9 <= math.dist(goal[:2], (-3, -3)) <= 3.0
    assert explorer.cell(*goal[:2])[1] < 70
    explorer.reject_last()
    other = explorer.choose(-3, -3)
    assert other is not None and math.dist(other[:2], goal[:2]) >= .8


def test_unknown_and_disconnected_islands_are_not_exploration_destinations():
    explorer = SearchExplorer()
    grid = np.full((100, 100), -1, dtype=np.int16)
    grid[10:30, 10:30] = 0
    grid[60:90, 60:90] = 0
    explorer.update(grid, .1, (0, 0, 0))
    explorer.mark_scanned(2, 2)
    assert explorer.choose(2, 2) is None


def test_visibility_stops_at_occluding_wall():
    explorer = SearchExplorer()
    grid = room(); grid[:, 60] = 100
    explorer.update(grid, .1, (0, 0, 0))
    explorer.mark_scanned(5, 5)
    assert np.any(explorer.seen[:, :60])
    assert not np.any(explorer.seen[:, 61:])


def test_view_limit_and_reset_do_not_reuse_old_task_coverage():
    explorer = SearchExplorer(max_views=1)
    explorer.update(room(), .1, (-6, -6, 0))
    explorer.mark_scanned(-3, -3)
    assert explorer.choose(-3, -3)
    assert explorer.choose(-3, -3) is None
    explorer.reset()
    assert not explorer.visited and not np.any(explorer.seen)
    assert explorer.choose(-3, -3)


def test_rotated_map_origin_round_trips_cell_coordinates():
    explorer = SearchExplorer()
    explorer.update(room(), .1, (4, -2, math.pi/2))
    assert explorer.cell(*explorer.world(20, 30)) == (20, 30)


def test_distant_detection_gets_clear_standoff_view_facing_the_target():
    explorer = SearchExplorer()
    explorer.update(room(), .1, (-6, -6, 0))
    target = (3.5, 0.)
    goal = explorer.choose(0., 0., target=target)
    assert goal is not None
    assert 1.3 <= math.dist(goal[:2], target) <= 2.2
    assert math.dist(goal[:2], (0., 0.)) <= explorer.step
    assert explorer.pose_free(*goal)
    assert goal[2] == math.atan2(target[1]-goal[1], target[0]-goal[0])


def test_bearing_only_hint_uses_bounded_connected_steps_without_crossing_wall():
    explorer = SearchExplorer()
    grid = room()
    grid[:, 75] = 100
    explorer.update(grid, .1, (-6, -6, 0))
    goal = explorer.choose(0., 0., bearing=0.)
    assert goal is not None
    assert .6 < goal[0] < 1.2
    assert math.hypot(*goal[:2]) <= 1.5
    assert explorer.pose_free(*goal)
    grid[:, 65] = 100  # No safe progress toward the ray remains.
    explorer.update(grid, .1, (-6, -6, 0))
    assert explorer.choose(0., 0., bearing=0.) is None
