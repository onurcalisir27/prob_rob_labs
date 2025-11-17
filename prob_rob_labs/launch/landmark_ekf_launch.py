import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

path = "/run/host/workdir/local_oc2356/ros2_ws/src/prob_rob_labs_ros_2/landmark_map.json"
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('map_path', default_value=path,
                              description='File path to the landmark map'),
        Node(
            package='prob_rob_labs',
            executable='landmark_ekf',
            name='landmark_ekf',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'map_path': LaunchConfiguration('map_path')}]
        )
    ])
 