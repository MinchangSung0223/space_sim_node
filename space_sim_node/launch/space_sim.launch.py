from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='space_sim_node',
            executable='space_sim_node',
            name='space_sim_node',
            output='screen'
        )
    ])
