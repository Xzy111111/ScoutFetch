"""Exercise the OS lock and detection of pre-fix orphan servers."""

import os
from pathlib import Path
import subprocess
import sys

import pytest

from scoutfetch_mission.simulation_guard import acquire_simulation_lock


def test_lock_blocks_a_second_start_and_releases_on_process_exit(tmp_path):
    lock = tmp_path / 'demo.lock'
    proc = tmp_path / 'proc'
    proc.mkdir()
    script = (
        'import time\n'
        'from scoutfetch_mission.simulation_guard import acquire_simulation_lock\n'
        f'acquire_simulation_lock({str(lock)!r}, {str(proc)!r})\n'
        'print("locked", flush=True)\n'
        'time.sleep(30)\n'
    )
    child = subprocess.Popen([sys.executable, '-c', script], stdout=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == 'locked'
        with pytest.raises(RuntimeError, match='已有 MC ARM 仿真'):
            acquire_simulation_lock(lock, proc)
    finally:
        child.terminate()
        child.wait(timeout=5)
        child.stdout.close()
    fd = acquire_simulation_lock(lock, proc)
    os.close(fd)


@pytest.mark.parametrize('args', [
    ['python3', '/opt/ros/humble/bin/ros2', 'launch', 'scoutfetch_mission', 'sim_bottle_search.launch.py'],
    ['ign', 'gazebo', '-r', '-s', '/tmp/scoutfetch_bottle_search_old/random_bottle.sdf'],
])
def test_pre_guard_launch_or_orphan_blocks_start_before_spawning(tmp_path, monkeypatch, args):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    proc = tmp_path / 'proc'
    entry = proc / '12345678'
    entry.mkdir(parents=True)
    (entry / 'cmdline').write_bytes(b'\0'.join(x.encode() for x in args) + b'\0')
    (entry / 'environ').write_bytes(b'ROS_DOMAIN_ID=0\0')
    lock = tmp_path / 'demo.lock'
    with pytest.raises(RuntimeError, match='12345678'):
        acquire_simulation_lock(lock, proc)
    # Rejecting an old process must not retain the new lock.
    (entry / 'cmdline').write_bytes(b'')
    fd = acquire_simulation_lock(lock, proc)
    os.close(fd)


def test_different_ros_domain_still_shares_gazebo_world(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '4')
    entry = tmp_path / 'proc/12345678'
    entry.mkdir(parents=True)
    (entry / 'cmdline').write_bytes(
        b'ign\0gazebo\0-r\0-s\0/tmp/scoutfetch_bottle_search_old/random_bottle.sdf\0')
    (entry / 'environ').write_bytes(b'ROS_DOMAIN_ID=5\0')
    with pytest.raises(RuntimeError, match='12345678'):
        acquire_simulation_lock(tmp_path / 'demo.lock', entry.parent)
