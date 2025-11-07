import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('landmark_color', default_value='cyan',
                              description='vision processor of the selected color'),
        DeclareLaunchArgument('landmark_height', default_value='0.5',
                              description='true height of the estimated landmark' ),
        Node(
            package='prob_rob_labs',
            executable='vision_processor',
            name='vision_processor',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'landmark_color' : LaunchConfiguration('landmark_color')}]
        )
    ])
