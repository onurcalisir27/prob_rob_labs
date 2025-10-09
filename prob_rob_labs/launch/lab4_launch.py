import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('reference_frame', default_value='odom', description='Frame of Reference Interested'
    ),                 
        Node(
            package='prob_rob_labs',
            executable='lab4',
            name='lab4',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'reference_frame': LaunchConfiguration('reference_frame')}]
        )
    ])
