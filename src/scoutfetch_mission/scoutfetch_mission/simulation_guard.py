"""Keep a second local demo from sharing the first demo's ROS graph."""

import atexit
import fcntl
import os
from pathlib import Path
import tempfile

from launch import Action


def acquire_simulation_lock(lock_path=None, process_root=Path('/proc')):
    if lock_path is None:
        # Changing ROS_DOMAIN_ID alone does not isolate Gazebo Transport's
        # world/create services. This single-robot demo permits one per user.
        lock_path = Path(tempfile.gettempdir()) / f'mc_arm_sim_{os.getuid()}.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = os.pread(fd, 128, 0).decode(errors='replace').strip()
            raise RuntimeError(
                f'已有 MC ARM 仿真正在运行（PID {owner or "unknown"}）。'
                '请在原仿真终端按 Ctrl+C，等待退出后再启动。') from None

        # Also catch demos started before this guard was installed, and the
        # orphan server left by the former shell-based Gazebo launcher.
        for entry in Path(process_root).glob('[0-9]*'):
            try:
                if int(entry.name) == os.getpid() or entry.stat().st_uid != os.getuid():
                    continue
                args = (entry / 'cmdline').read_bytes().split(b'\0')
                is_demo = (b'launch' in args and b'scoutfetch_mission' in args and any(
                    name in args for name in (b'sim_bottle_search.launch.py', b'sim_bookstore_search.launch.py')))
                is_server = (b'gazebo' in args and b'-s' in args and any(
                    b'/scoutfetch_bottle_search_' in arg for arg in args))
                if not (is_demo or is_server):
                    continue
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
            raise RuntimeError(
                f'检测到已有仿真或残留 Gazebo（PID {entry.name}），已取消重复启动。'
                '请先退出原仿真；只停止 ros2 launch 入口可能仍留有旧版 Gazebo 子进程。')

        os.ftruncate(fd, 0)
        os.write(fd, str(os.getpid()).encode())
        return fd
    except BaseException:
        os.close(fd)
        raise


class SimulationGuard(Action):
    def execute(self, context):
        fd = acquire_simulation_lock()
        # Keep the lock until the launch interpreter exits, including while
        # OnShutdown is still waiting for its child processes to terminate.
        # Python descriptors are non-inheritable, so children cannot retain it.
        atexit.register(os.close, fd)
