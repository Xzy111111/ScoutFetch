import importlib.util
import math
from pathlib import Path

from sensor_msgs.msg import LaserScan


spec = importlib.util.spec_from_file_location('scan_self_filter',
    Path(__file__).parents[1]/'scripts/scan_self_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_self_returns_are_unknown_and_close_external_obstacles_are_preserved():
    # Lidar sits forward of the base origin and points toward -X. A .1 m
    # return is inside the chassis; .45 m behind and .25 m ahead are outside.
    scan = LaserScan(angle_min=0., angle_increment=math.pi,
                     ranges=[.1, .25, .45, float('inf')], range_min=.05)
    result = module.filter_self(scan, .1325, 0., math.pi)
    assert math.isnan(result.ranges[0])
    assert abs(result.ranges[1]-.25) < 1e-6
    assert abs(result.ranges[2]-.45) < 1e-6
    assert math.isinf(result.ranges[3])
    assert result.range_min == scan.range_min
    assert not math.isnan(scan.ranges[0])
