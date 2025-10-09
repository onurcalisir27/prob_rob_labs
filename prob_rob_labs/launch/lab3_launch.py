import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('horizon', default_value='500', description="Data Sampling Horizon"),
        DeclareLaunchArgument('threshold', default_value='235.0', description='Door State Open/Close Trigger Threshold'),
        DeclareLaunchArgument('flaky_door', default_value='0', description='Boolean flag to set the uncertainty of the door opener'),
        DeclareLaunchArgument('collect_data', default_value='0', description='Boolean flag to set the robot state to "measure" or "control"'),
        Node(
            package='prob_rob_labs',
            executable='lab3',
            name='lab3',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'threshold': LaunchConfiguration('threshold'),
                         'horizon' : LaunchConfiguration('horizon'),
                         'collect_data' : LaunchConfiguration('collect_data'),
                         'flaky_door': LaunchConfiguration('flaky_door')
                         }]
        )
    ])
