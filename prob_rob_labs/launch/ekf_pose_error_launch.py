import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('prob_rob_labs')
    launch_path = os.path.join(pkg_dir, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        # Robot base ground truth publisher
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'lab4_launch.py')
        ),
        ),
        Node(
            package='prob_rob_labs',
            executable='ekf_pose_error',
            name='ekf_pose_error',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
