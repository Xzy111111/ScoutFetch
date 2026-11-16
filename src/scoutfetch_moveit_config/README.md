# scoutfetch_moveit_config

Shared MoveIt configuration for simulation and hardware. It uses the canonical
`scoutfetch_description/urdf/scoutfetch.urdf.xacro`; this package contains no duplicate
URDF geometry.

The robot backend must be running first and must provide joint states, TF, and:

```text
/arm_controller/follow_joint_trajectory
```

Simulation:

```bash
ros2 launch scoutfetch_sim sim.launch.py
ros2 launch scoutfetch_moveit_config moveit.launch.py backend:=sim
```

Hardware (after the hardware bringup):

```bash
ros2 launch scoutfetch_moveit_config moveit.launch.py backend:=real
```

Defaults:

- planning group: `arm`
- planning frame: `base_link`
- tip link: `gripper_tcp`
- sim joint states: `/joint_states`
- real joint states: `/feedback/joint_states`
- controller: `/arm_controller/follow_joint_trajectory`

This launch deliberately does not start robot_state_publisher, ros2_control, a
controller spawner, or a hardware driver. Each of those must have exactly one
owner in the selected backend.

