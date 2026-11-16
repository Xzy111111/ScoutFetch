"""Expand the complete launch graph without starting external processes.

This catches cross-include argument leaks (notably config_file) and verifies
the actual resolved execution/time settings of the simulation pipeline.
"""

from pathlib import Path

import pytest
import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch_ros.actions import Node
from launch.utilities import perform_substitutions


LAUNCH = Path(__file__).resolve().parents[1] / "launch/sim_bottle_search.launch.py"


@pytest.mark.parametrize("execute", ["true", "false"])
@pytest.mark.parametrize("auto_start", ["true", "false"])
@pytest.mark.parametrize("scene", ["laboratory", "bookstore"])
def test_complete_launch_resolves_isolated_configs_and_velocity_mux(execute, auto_start, scene):
    context = LaunchContext()
    context.launch_configurations.update({
        "seed": "42", "gui": "false", "rviz": "false", "execute_pick": execute,
        "ros_namespace": "/", "auto_start": auto_start, "scene": scene,
    })
    nodes = {}
    gazebo_processes = []
    safe_actions = {
        "DeclareLaunchArgument", "GroupAction", "IncludeLaunchDescription", "OpaqueFunction",
        "PushLaunchConfigurations", "PopLaunchConfigurations", "SetLaunchConfiguration",
        "PushEnvironment", "PopEnvironment", "SetEnvironmentVariable", "SetRemap",
    }

    def visit(entity):
        condition = getattr(entity, "condition", None)
        if condition is not None and not condition.evaluate(context):
            return
        if isinstance(entity, LaunchDescription):
            children = entity.entities
        elif isinstance(entity, TimerAction):
            children = entity.actions
        elif isinstance(entity, Node):
            entity._perform_substitutions(context)
            params = {}
            for filename, is_file in entity._Node__expanded_parameter_arguments or []:
                if is_file:
                    # launch_ros serializes normalized string arrays as tuples.
                    data = yaml.full_load(Path(filename).read_text()) or {}
                    for node_name, config in data.items():
                        if node_name in ("/**", entity.node_name, entity.node_name.lstrip("/")):
                            params.update(config.get("ros__parameters", {}))
            # Node is an ExecuteProcess subclass: always handle it before that guard.
            nodes[entity.node_executable] = (params, entity)
            return
        elif isinstance(entity, ExecuteProcess):
            command = [perform_substitutions(context, part) for part in entity.cmd]
            if 'gazebo' in command:
                gazebo_processes.append((entity, command))
            return
        elif type(entity).__name__ in ("RegisterEventHandler", "LogInfo", "SimulationGuard"):
            return
        elif type(entity).__name__ in safe_actions:
            children = entity.execute(context) or []
        else:
            raise AssertionError(f"Unreviewed launch action: {type(entity).__name__}")
        for child in children:
            visit(child)

    visit(get_launch_description_from_python_launch_file(str(LAUNCH)))
    assert len(gazebo_processes) == 1  # gui=false in this launch context
    server, command = gazebo_processes[0]
    assert server.shell is False
    assert command[:5] == ['ruby', '/usr/bin/ign', 'gazebo', '-r', '-s']
    plugin_env = {perform_substitutions(context, key): perform_substitutions(context, value)
                  for key, value in server.additional_env}
    assert '/opt/ros/humble/lib' in plugin_env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'].split(':')
    detector = nodes["yolo_detector.py"][0]
    localizer = nodes["localization_node"][0]
    pick = nodes["pick_node"][0]
    task = nodes["task_fsm"][0]
    assert detector.get("target_classes") == ["bottle"], (
        nodes["yolo_detector.py"][1].node_name,
        nodes["yolo_detector.py"][1]._Node__expanded_parameter_arguments, detector)
    assert Path(detector["model_path"]).is_file()
    assert Path(detector["model_path"]).name == "yolov8s.onnx"
    assert detector["confidence_threshold"] == 0.20
    assert localizer["pixel_source"] == "yolo"
    assert localizer["max_detection_depth_delta"] == 0.12
    assert pick["gripper"]["grasp_width_min"] == 0.06
    assert pick["execute"] is (execute == "true")
    assert pick["allow_real_execution"] is (execute == "true")
    assert pick["return_to_stow"] is True
    assert task["search_enabled"] is True
    assert task['exploration_enabled'] is True
    assert task["search_max_turns"] == 0
    assert task['search_continuous_rotation'] is True
    assert task['search_angular_speed'] == .25
    assert task["search_timeout_sec"] == 0.0
    if scene == 'bookstore':
        angles = nodes['scoutfetch_reach_node'][0]['angle_offsets_deg']
        assert angles[0] == -180.0 and angles[-1] == 180.0
        assert max(b-a for a, b in zip(angles, angles[1:])) <= 10.0
    assert nodes["nav_goal_executor"][0]["enable_navigation"] is True
    for name in ("yolo_detector.py", "localization_node", "pick_node", "task_fsm",
                 "nav_goal_executor", "target_pose_resolver", "move_group", "twist_mux"):
        assert nodes[name][0]["use_sim_time"] is True, name
    assert nodes["twist_mux"][0]["topics"]["search"]["topic"] == "/cmd_vel_search"
    assert ("start_sim_task" in nodes) is (auto_start == "true")
    if auto_start == "true":
        assert nodes["start_sim_task"][0]["use_sim_time"] is True
    remaps = nodes["velocity_smoother"][1]._Node__expanded_remappings
    assert ("velocity_smoother:cmd_vel", "/cmd_vel_nav_raw") in remaps
    assert ("velocity_smoother:cmd_vel_smoothed", "/cmd_vel_nav") in remaps
    assert ("controller_server:cmd_vel", "/cmd_vel_nav_raw") in nodes["controller_server"][1]._Node__expanded_remappings
