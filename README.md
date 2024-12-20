# antsy_description

Launches the robot description of our beloved Yetti.
`ros2 launch antsy_description description.launch.py`

For debug purposes, we can open RVIZ2 with a GUI joint state publisher.
`ros2 launch antsy_description display.launch.py`

This debug tool will keep publishing joint states, so it will mess up our controller. If you only want to see our beautifull Yetti, there is another launch file with only the RVIZ2 part.
`ros2 launch antsy_description rviz.launch.py`

### Credits to
- [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2): This repository was used to turn a Fusion360 model into the starting point for this ROS2 package.