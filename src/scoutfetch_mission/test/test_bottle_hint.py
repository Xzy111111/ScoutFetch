import math

import pytest

from scoutfetch_mission.bottle_hint import BottleHint, optical_bearing


def test_camera_projection_accounts_for_pixel_offset_and_camera_orientation():
    optical_to_base = (-.5, .5, -.5, .5)
    k = (500., 500., 320., 240.)
    assert optical_bearing(320., 240., k, optical_to_base) == pytest.approx(0.)
    assert optical_bearing(420., 240., k, optical_to_base) == pytest.approx(-math.atan(.2))
    # Same optical camera rotated 90 degrees in map: centre faces map +Y.
    s = math.sqrt(.5)
    assert optical_bearing(320., 240., k, (0., s, 0., s)) == pytest.approx(0.)
    assert optical_bearing(320., 240., k, (-s, 0., 0., s)) == pytest.approx(math.pi/2)


def test_hint_requires_distinct_consistent_frames_expires_and_is_consumed_once():
    h = BottleHint()
    assert not h.observe(3.13, 1, 0.)
    assert not h.observe(3.13, 1, .2)  # DDS replay is not a second frame.
    assert h.observe(-3.13, 2, .3)  # Wrapped headings agree.
    assert h.ready(4.)
    assert h.take(4.) == -3.13
    assert h.take(4.) is None
    h.observe(0., 3, 5.);h.observe(0., 4, 5.2)
    assert h.take(26.) is None


def test_inconsistent_bearings_and_time_gaps_restart_hint_evidence():
    h = BottleHint()
    h.observe(0., 1, 0.)
    assert not h.observe(1., 2, .3)
    assert h.observe(1.1, 3, .5)
    assert not h.observe(1.1, 4, 3.)
