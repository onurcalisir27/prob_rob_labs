import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('desired_speed', default_value='0.5', description='Desired Forward Speed for the Bot'
    ),                 
        Node(
            package='prob_rob_labs',
            executable='lab2',
            name='lab2',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'desired_speed': LaunchConfiguration('desired_speed')}]
        )
    ])
