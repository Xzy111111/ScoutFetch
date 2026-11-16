import math

import pytest

from scoutfetch_mission.bottle_search import BottleSearch


def ready_search(**kwargs):
    search = BottleSearch(**kwargs)
    search.feedback(0.0, 0.0, 0.0, 0.0)
    search.start(0.0)
    return search


def feed(search, now, yaw=0.0, x=0.0):
    search.feedback(x, 0.0, yaw, now)
    return search.tick(now)


def test_no_motion_before_start_and_stale_start_rejected():
    search = BottleSearch()
    assert search.tick(1.0) == 0.0
    with pytest.raises(ValueError, match="odometry"):
        search.start(1.0)
    search.feedback(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="odometry"):
        search.start(1.0)


def test_step_rotation_uses_measured_angle_then_dwells():
    search = ready_search()
    assert feed(search, 0.5) == 0.0
    assert feed(search, 1.0) == 0.0
    assert feed(search, 1.5) == search.speed
    # Time passing with a stuck robot is not progress.
    assert feed(search, 2.0) == search.speed
    assert feed(search, 2.5, math.pi / 6) == 0.0
    assert search.mode == "DWELL"


def test_uniform_rotation_has_no_step_or_full_turn_pauses():
    search = ready_search(continuous_rotation=True, max_turns=0, timeout=0.)
    yaw = 0.
    for index in range(1, 1801):
        now = index*.05
        speed = feed(search, now, math.atan2(math.sin(yaw), math.cos(yaw)))
        assert speed == .25
        assert search.mode == 'ROTATE'
        yaw += speed*.05
    assert search.completed_turns == 3
    assert search.error is None


def test_uniform_rotation_stops_for_confirmation_and_resumes_after_lost_candidate():
    search = ready_search(continuous_rotation=True)
    assert feed(search, .1) == .25
    assert not search.observe((1., 0., .4), 1, .1)
    assert feed(search, .3) == 0.
    assert search.mode == 'CONFIRM'
    assert feed(search, 4.1) == .25
    assert search.mode == 'ROTATE'


def test_uniform_rotation_finite_turn_uses_feedback_and_stops():
    search = ready_search(continuous_rotation=True)
    for i in range(1, 51):
        assert feed(search, i*.1) == .25  # no movement despite elapsed time
    assert search.completed_turns == 0
    yaw = 0.
    for i in range(1, 600):
        speed = feed(search, 5.+i*.05, math.atan2(math.sin(yaw), math.cos(yaw)))
        yaw += speed*.05
        if search.error:
            break
    assert search.completed_turns == 1
    assert speed == 0. and 'full turn' in search.error


def test_full_turn_handles_yaw_wrap_and_stops_without_target():
    search = ready_search()
    yaw = 0.0
    for index in range(1, 1801):
        now = index * 0.05
        speed = feed(search, now, math.atan2(math.sin(yaw), math.cos(yaw)))
        yaw += speed * 0.05
        if search.error:
            break
    assert "full turn" in search.error
    assert 6.2 < search.travel < 6.4
    assert speed == 0.0


def test_continuous_search_survives_multiple_empty_turns_then_confirms_target():
    search = ready_search(max_turns=0, timeout=0.0)
    yaw = 0.0
    for index in range(1, 2601):
        now = index * 0.05
        speed = feed(search, now, math.atan2(math.sin(yaw), math.cos(yaw)))
        yaw += speed * 0.05
    assert search.completed_turns >= 2
    assert search.active and search.error is None
    assert not search.observe((1.8, 0.3, 0.4), 1, now)
    assert not search.observe((1.8, 0.3, 0.4), 2, now + 1.0)
    assert not search.observe((1.8, 0.3, 0.4), 3, now + 1.1)
    assert search.observe((1.8, 0.3, 0.4), 4, now + 1.2)


@pytest.mark.parametrize('continuous_rotation', [False, True])
def test_feedback_loss_translation_and_total_timeout_stop_motion(continuous_rotation):
    search = ready_search(continuous_rotation=continuous_rotation)
    assert search.tick(1.0) == 0.0
    assert search.error == "odometry stopped"
    search = ready_search(continuous_rotation=continuous_rotation)
    assert feed(search, 0.2, x=0.11) == 0.0
    assert "translated" in search.error
    search = ready_search(timeout=2.0, continuous_rotation=continuous_rotation)
    for i in range(1, 5):
        speed = feed(search, i * 0.5)
    assert speed == 0.0
    assert "timeout" in search.error


def test_target_stops_rotation_and_requires_fresh_distinct_confirmations():
    search = ready_search()
    assert not search.observe((1.0, 0.0, 0.4), 1, 0.1)
    assert search.tick(0.2) == 0.0
    assert not search.observe((1.0, 0.0, 0.4), 2, 0.3)  # still settling
    assert not search.observe((1.0, 0.0, 0.4), 3, 1.0)
    assert not search.observe((1.0, 0.0, 0.4), 3, 1.1)  # duplicate image
    assert not search.observe((1.01, 0.0, 0.4), 4, 1.2)
    assert search.observe((0.99, 0.0, 0.4), 5, 1.3)
    assert search.confirmed
    assert search.tick(1.4) == 0.0


def test_inconsistent_detections_do_not_confirm_and_search_resumes():
    search = ready_search()
    search.observe((1.0, 0.0, 0.4), 1, 0.1)
    for i in range(3):
        assert not search.observe((float(i), 0.0, 0.4), i + 2, 1.0 + i * 0.1)
    assert feed(search, 4.2) == 0.0
    assert search.mode == "DWELL"
    assert feed(search, 4.7) == 0.0
    assert feed(search, 5.2) == 0.0
    assert feed(search, 5.7) == search.speed


def test_motion_during_confirmation_restarts_settle_period():
    search = ready_search()
    search.observe((1.0, 0.0, 0.4), 1, 0.1)
    search.feedback(0.0, 0.0, 0.01, 0.8)
    assert not search.observe((1.0, 0.0, 0.4), 2, 1.0)
    assert not search.points


@pytest.mark.parametrize("kwargs", [
    {"angular_speed": float("nan")}, {"angular_speed": 0.8},
    {"step_angle": math.pi}, {"settle": 0.0},
    {"confirmation_frames": 1}, {"confirmation_timeout": 0.1},
    {"max_turns": -1}, {"timeout": -1.0},
])
def test_invalid_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        BottleSearch(**kwargs)
