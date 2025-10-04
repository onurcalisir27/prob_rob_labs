import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('horizon', default_value='1000', description="Averaging Horizon"),
        DeclareLaunchArgument('threshold', default_value='235.0', description='Trigger Threshold'),
        Node(
            package='prob_rob_labs',
            executable='lab3',
            name='lab3',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'threshold': LaunchConfiguration('threshold'),
                         'horizon' : LaunchConfiguration('horizon')
                         }]
        )
    ])
