from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='cube_detection',
             executable='cube_publisher.py',
             output='screen',
             parameters=[{"use_sim_time": True}]),
    ])
