from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Ros2_for_Waveshare_Alphabot2',  # Replace with your actual ROS 2 package name
            executable='test',  # Replace with your actual ROS 2 node name
            name='test_node',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([Path(__file__).resolve().parent, 'Alphabot2_standard_launch.py']),
        ),
    ])