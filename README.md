# antsy_description

Robot description package for ANTSY. This repo does not provide custom C++ or Python runtime nodes; it provides launch files that configure standard ROS nodes such as `robot_state_publisher`, `joint_state_publisher`, and `rviz2`.

## Launch files

### `description.launch.py`

Typical usage:

```bash
ros2 launch antsy_description description.launch.py
```

This launch file starts `robot_state_publisher` with a Xacro-generated robot description.

Launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `true` | Passed to `robot_state_publisher` as the standard ROS simulation-clock parameter. |
| `xacro_path` | `share/antsy_description/urdf/antsy_description.xacro` | Xacro file to evaluate and publish as `robot_description`. |
| `servo_velocity_limit` | `5.6` | Xacro argument used to set the URDF joint velocity limits in rad/s. |
| `servo_effort_limit` | `4.25` | Xacro argument used to set the URDF joint effort limits. |
| `servo_static_friction` | `0.3` | Xacro argument used to set the URDF joint static friction. |
| `servo_damping_friction` | `0.1` | Xacro argument used to set the URDF joint damping friction. |

Effective node parameters:

- `robot_state_publisher.use_sim_time`: fed from `use_sim_time`
- `robot_state_publisher.robot_description`: generated from the selected Xacro and the four servo arguments above
- `robot_state_publisher.publish_frequency`: fixed at `50.0`

### `display.launch.py`

Typical usage:

```bash
ros2 launch antsy_description display.launch.py
```

This is a debugging launch file. It starts:

- `robot_state_publisher`
- either `joint_state_publisher` or `joint_state_publisher_gui`
- `rviz2`

Launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `gui` | `True` | If true, start `joint_state_publisher_gui`; otherwise start `joint_state_publisher`. |

Effective node parameters:

- `robot_state_publisher.robot_description`: generated directly from `antsy_description.xacro`
- the GUI and RViz nodes use their package defaults here

Warning: this launch file continuously publishes joint states for manual visualization, so it will fight the controller if both are active.

### `rviz.launch.py`

Typical usage:

```bash
ros2 launch antsy_description rviz.launch.py
```

This launch file starts only `rviz2`. It does not declare launch arguments or set custom ROS parameters.

## Notes

- This package is mostly a launch-and-model package. The important tunables live in the launch arguments, not in custom node parameters.
- The launch file currently hardcodes the RViz config path in `rviz.launch.py`; if that path changes, update the launch file accordingly.

### Credits

- [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2): used as the starting point for converting the Fusion360 model into this ROS 2 package.
