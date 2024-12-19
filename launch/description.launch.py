import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_path = LaunchConfiguration('xacro_path')

    share_dir = get_package_share_directory('antsy_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'antsy_description.xacro')

    # Convert xacro to urdf
    urdf = ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=xacro_file,
            description='path to urdf.xacro file to publish'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': urdf,
                'publish_frequency': 50.0,
                }],
            ),
    ])
